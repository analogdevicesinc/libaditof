/*
 * MIT License
 *
 * Copyright (c) 2025 Analog Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "camera_itof.h"
#include "aditof/frame.h"
#include "aditof/frame_operations.h"
#include "hardware/adsd3500_registers.h"
#include "helpers/depth_parameter_mapper.h"
#include "utils_ini.h"

#include "../../platform/platform_impl.h"
#include "aditof/adsd3500_hardware_interface.h"
#include "aditof/playback_interface.h"
#include "aditof/utils.h"
#include "crc.h"
#include "tofi/algorithms.h"
#include "tofi/floatTolin.h"
#include "tofi/tofi_config.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <json.h>
#include <limits.h>
#include <sstream>
#include <unistd.h>

#include <aditof/log.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <list>
#include <math.h>
#include <string>
#include <thread>
#include <vector>

#undef NDEBUG
#include <cassert>

/**
 * @brief Constructor for CameraItof.
 *
 * Initializes a camera instance with a depth sensor interface and version information.
 * Sets up default camera parameters, XYZ table pointers, and determines sensor type
 * (ADSD3500, offline, etc.). Detects imager type from sensor name.
 *
 * @param[in] depthSensor Shared pointer to DepthSensorInterface for V4L2 communication.
 * @param[in] ubootVersion U-Boot bootloader version string.
 * @param[in] kernelVersion Linux kernel version string.
 * @param[in] sdCardImageVersion SD card image/firmware version string.
 * @param[in] netLinkTest Optional network link test flag.
 *
 * @note Calls FloatToLinGenerateTable() and checks sensor name to determine
 *       if ADSD3500 or offline mode should be used.
 */
CameraItof::CameraItof(
    std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
    const std::string &ubootVersion, const std::string &kernelVersion,
    const std::string &sdCardImageVersion, const std::string &netLinkTest)
    : m_config(std::make_unique<aditof::CameraConfiguration>()),
      m_calibrationMgr(
          std::make_unique<aditof::CalibrationManager>(depthSensor)),
      m_adsd3500Ctrl(std::make_unique<aditof::Adsd3500Controller>(depthSensor)),
      m_recordingMgr(std::make_unique<aditof::RecordingManager>(
          depthSensor, m_calibrationMgr.get(), m_config.get())),
      m_sensorConfigHelper(std::make_unique<aditof::SensorConfigHelper>(
          depthSensor, m_config.get())),
      m_initManager(std::make_unique<aditof::CameraInitializationManager>(
          depthSensor, m_calibrationMgr.get(), m_adsd3500Ctrl.get(),
          m_config.get())),
      m_firmwareManager(
          std::make_unique<aditof::CameraFirmwareManager>(depthSensor)),
      m_frameAcqManager(std::make_unique<aditof::CameraFrameAcquisitionManager>(
          depthSensor, m_calibrationMgr.get(), m_config.get())),
      m_depthSensor(depthSensor), m_adsd3500Hardware(nullptr),
      m_devStarted(false), m_devStreaming(false), m_adsd3500Enabled(false),
      m_isOffline(false), m_xyzEnabled(true), m_xyzSetViaApi(false),
      m_cameraFps(0), m_modesVersion(0),
      m_imagerType(aditof::ImagerType::UNSET), m_dropFrameOnce(true) {

    m_adsd3500Hardware =
        std::dynamic_pointer_cast<aditof::Adsd3500HardwareInterface>(
            depthSensor);

    if (!m_adsd3500Hardware) {
        LOG(WARNING) << "Sensor does not support ADSD3500 hardware interface";
    }

    // Initialize DepthParameterMapper after camera is constructed (avoids circular dependency)
    m_depthParamMapper = std::make_unique<aditof::DepthParameterMapper>(this);

    FloatToLinGenerateTable();
    m_details.mode = -1;
    m_details.uBootVersion = ubootVersion;
    m_details.kernelVersion = kernelVersion;
    m_details.sdCardImageVersion = sdCardImageVersion;
    m_netLinkTest = netLinkTest;
    m_isOffline = false;

    if (!depthSensor) {
        LOG(WARNING) << "Invalid instance of a depth sensor";
        return;
    }

    aditof::SensorDetails sDetails;
    m_depthSensor->getDetails(sDetails);
    m_details.cameraId = sDetails.id;
    m_details.connection = sDetails.connectionType;

    std::string sensorName;
    m_depthSensor->getName(sensorName);
    LOG(INFO) << "Sensor name = " << sensorName;
    if (sensorName == "adsd3500") {
        m_adsd3500Enabled = true;
        m_isOffline = false;
    } else if (sensorName == "offline") {
        m_adsd3500Enabled = false;
        m_isOffline = true;
    }

    m_adsd3500_master = true;
}

/**
 * @brief Destructor for CameraItof.
 *
 * Cleans up resources (XYZ tables, etc.) when camera object is destroyed.
 * Calibration manager cleanup is automatic via unique_ptr.
 */
CameraItof::~CameraItof() {
    // Cleanup is handled automatically by CalibrationManager destructor
}

/**
 * @brief Initializes the camera and sets up depth computation parameters.
 *
 * In offline mode, prepares for playback from a recorded file. In online mode,
 * opens the V4L2 sensor interface, detects imager type, reads intrinsic and
 * dealias parameters from ADSD3500, retrieves available modes and their details,
 * loads or generates depth processing parameters, and configures MIPI, deskew,
 * temperature compensation, and edge confidence settings.\n
 * Also reads serial number if supported by firmware.
 *
 * @param[in] configFilepath Path to a JSON configuration file with depth parameters.
 *                            If empty, uses firmware defaults.\n
 *
 * @return aditof::Status::OK on successful initialization;\n
 *         aditof::Status::UNAVAILABLE if online mode is misconfigured;\n
 *         error codes if sensor operations fail (open, control, parameter reads).\n
 *
 * @note This function is called automatically during camera setup before streaming.\n
 *       In online mode, requires a valid depthSensor interface.\n
 *       Creates and caches intrinsic parameters and dealias data for each mode.\n
 */
aditof::Status CameraItof::initialize(const std::string &configFilepath) {
    using namespace aditof;
    Status status = Status::OK;

    if (m_isOffline) {
        status = m_initManager->initializeOfflineMode();
        return status;
    }

    if (!m_adsd3500Enabled) {
        LOG(ERROR) << "This usecase is no longer supported.";
        return Status::UNAVAILABLE;
    }

    m_initConfigFilePath = configFilepath;

    if (!m_netLinkTest.empty()) {
        m_depthSensor->setControl("netlinktest", "1");
    }

    status = m_initManager->initializeOnlineMode(
        m_details, m_availableModes, m_availableSensorModeDetails, m_imagerType,
        m_adsd3500FwGitHash, configFilepath);

    if (status != Status::OK) {
        return status;
    }

    m_devStarted = true;

    // Load depth process parameters from config or firmware
    status = retrieveDepthProcessParams();
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to load process parameters!";
        return status;
    }

    return Status::OK;
}

/**
 * @brief Starts the camera and begins frame streaming.
 *
 * Initiates the underlying depth sensor to start capturing and streaming
 * depth frames. Sets the internal streaming flag to indicate the camera is active.
 *
 * @return aditof::Status::OK on successful start;
 *         error codes from depth sensor if start operation fails.
 */
aditof::Status CameraItof::start() {
    using namespace aditof;

    Status status = m_depthSensor->start();
    if (Status::OK != status) {
        LOG(ERROR) << "Error starting adsd3500.";
        return status;
    }
    m_devStreaming = true;

    return aditof::Status::OK;
}

/**
 * @brief Stops the camera and halts frame streaming.
 *
 * Terminates frame capture from the underlying depth sensor. In offline mode,
 * stops playback of recorded frames. Clears the streaming flag to indicate
 * the camera is no longer active.
 *
 * @return aditof::Status::OK on successful stop;
 *         status from depth sensor operations if issues occur during shutdown.
 */
aditof::Status CameraItof::stop() {
    aditof::Status status = aditof::Status::OK;

    if (m_isOffline) {
        auto playbackInterface =
            std::dynamic_pointer_cast<aditof::PlaybackInterface>(m_depthSensor);
        if (playbackInterface) {
            status = playbackInterface->stopPlayback();
            if (status != aditof::Status::OK) {
                LOG(INFO) << "Failed to stop playback of offline file!";
            }
        }
    }

    status = m_depthSensor->stop();
    if (status != aditof::Status::OK) {
        LOG(INFO) << "Failed to stop camera!";
    }

    m_devStreaming = false;

    return status;
}

/**
 * @brief Retrieves depth processing parameters for a specific mode.
 *
 * Looks up and returns the depth computation INI parameters (such as thresholds,
 * filter settings, etc.) associated with the given frame mode.
 *
 * @param[in] mode Frame mode number to retrieve parameters for.
 * @param[out] params Map populated with parameter name-value pairs for the mode.
 *
 * @return aditof::Status::OK if parameters found and returned;
 *         aditof::Status::INVALID_ARGUMENT if the mode is not in the parameter map.
 */
aditof::Status
CameraItof::getDepthParamtersMap(uint16_t mode,
                                 std::map<std::string, std::string> &params) {
    using namespace aditof;

    return m_config->getDepthParamsForMode(mode, params);

    return Status::INVALID_ARGUMENT;
}

/**
 * @brief Set the camera operating mode (offline or online path).
 *
 * In offline mode, this initializes the camera state and internal caches
 * from the playback file header obtained via the sensor interface. The
 * input parameter is ignored by design in offline mode, as the recorded
 * file fully specifies the mode and frame layout.
 *
 * In online mode, this validates the requested mode against the list of
 * available sensor modes, applies the corresponding depth-compute INI
 * parameters, commands the ADSD3500 to switch modes, refreshes the
 * `m_modeDetailsCache`, prepares depth/AB/conf output layout in
 * `m_details.frameType`, and (for non-PCM modes) initializes target depth
 * compute and regenerates XYZ tables if enabled.
 *
 * @param mode Requested mode number (used only in online mode).
 * @return Status::OK on success;
 *         Status::INVALID_ARGUMENT if the mode is not supported (online);
 *         other error codes if sensor or compute configuration fails.
 *
 * @note This function updates internal flags (`m_depthEnabled`,
 *       `m_abEnabled`, `m_confEnabled`, `m_xyzEnabled`) via
 *       configureSensorModeDetails(), and may allocate/free XYZ tables.
 *       It also updates `m_details.mode` and `m_details.frameType` to
 *       reflect the active configuration.
 */
aditof::Status CameraItof::setMode(const uint8_t &mode) {
    using namespace aditof;
    Status status = Status::OK;

    if (m_isOffline) {
        status =
            m_recordingMgr->loadPlaybackHeader(m_modeDetailsCache, m_details);
        if (status != Status::OK) {
            return status;
        }

        m_pcmFrame = m_modeDetailsCache.isPCM;
        configureSensorModeDetails();

    } else {

        auto modeIt = std::find_if(m_availableSensorModeDetails.begin(),
                                   m_availableSensorModeDetails.end(),
                                   [&mode](const DepthSensorModeDetails &d) {
                                       return (d.modeNumber == mode);
                                   });

        if (modeIt == m_availableSensorModeDetails.end()) {
            LOG(WARNING) << "Mode: " << (int)mode << " not supported by camera";
            return Status::INVALID_ARGUMENT;
        }

        // Set target mode on sensor BEFORE configureSensorModeDetails() so that
        // runtime config bit depths (abBits, confidenceBits) update the correct
        // mode's m_bitsInAB[]/m_bitsInConf[] arrays for buffer allocation
        m_depthSensor->setControl("targetModeNumber", std::to_string(mode));
        std::map<std::string, std::string> iniKeyValPairs;
        if (m_config->getDepthParamsForMode(mode, iniKeyValPairs) !=
            Status::OK) {
            LOG(ERROR) << "Failed to get depth parameters for mode "
                       << (int)mode;
            return Status::GENERIC_ERROR;
        }
        m_config->setIniKeyValPairs(iniKeyValPairs);
        configureSensorModeDetails();

        // Apply raw bypass mode setting before configuring V4L2 driver
        // Raw bypass is enabled when lensScatterCompensationEnabled is set to "1"
        auto lensScatteringIt =
            iniKeyValPairs.find("lensScatterCompensationEnabled");
        bool rawBypassEnabled = (lensScatteringIt != iniKeyValPairs.end() &&
                                 lensScatteringIt->second == "1");
        status = adsd3500SetRawBypassMode(rawBypassEnabled);
        if (status != Status::OK) {
            LOG(WARNING) << "Failed to set raw bypass mode";
            return status;
        }

        // Set lens scatter compensation flag for buffer allocation
        if (rawBypassEnabled) {
            LOG(INFO) << "Setting lens scatter compensation flag to enabled";
            m_depthSensor->setControl("lensScatterCompensationEnabled", "1");
        } else {
            LOG(INFO) << "Setting lens scatter compensation flag to disabled";
            m_depthSensor->setControl("lensScatterCompensationEnabled", "0");
        }

        // Declare rotationValue here for use in both rotation logic and frame details
        std::string rotationValue;

        // Apply VGA rotation setting from JSON config if present
        auto rotationIt = iniKeyValPairs.find("enableRotation");

        // ADTF3066: rotation enabled by default; override only if user loaded JSON
        // Other imagers: rotation disabled by default; respect JSON if present
        if (m_imagerType == aditof::ImagerType::ADTF3066) {
            if (m_userJsonLoaded && rotationIt != iniKeyValPairs.end()) {
                // User explicitly loaded JSON - respect their setting
                rotationValue = rotationIt->second;
                LOG(INFO) << "ADTF3066 rotation overridden by user JSON: "
                          << rotationValue;
            } else {
                // Default: enabled for ADTF3066
                rotationValue = "1";
                LOG(INFO) << "ADTF3066 rotation enabled by default";
            }
        } else {
            if (rotationIt != iniKeyValPairs.end()) {
                rotationValue = rotationIt->second;
                LOG(INFO) << "Rotation from JSON config: " << rotationValue;
            } else {
                rotationValue = "0";
                LOG(INFO) << "Rotation using default: disabled";
            }
        }

        // Apply the rotation control before setMode
        m_depthSensor->setControl("enableRotation", rotationValue);

        status = m_depthSensor->setMode(mode);
        if (status != Status::OK) {
            LOG(WARNING) << "Failed to set frame type";
            return status;
        }

        status = m_depthSensor->getModeDetails(mode, m_modeDetailsCache);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to get frame type details!";
            return status;
        }

        // For lens scatter mode: ToFi processes raw input → depth+AB output
        // Frame buffers are sized for processed OUTPUT (1024×1024), not raw input (2048×4608)
        if (rawBypassEnabled) {
            // Frame content reflects processed OUTPUT based on bit configuration
            m_modeDetailsCache.frameContent.clear();

            // Check bitsInPhaseOrDepth (depth is typically always present)
            auto depthBitsIt = iniKeyValPairs.find("bitsInPhaseOrDepth");
            if (depthBitsIt != iniKeyValPairs.end() &&
                depthBitsIt->second != "0") {
                m_modeDetailsCache.frameContent.push_back("depth");
            }

            // Check bitsInAB
            auto abBitsIt = iniKeyValPairs.find("bitsInAB");
            if (abBitsIt != iniKeyValPairs.end() && abBitsIt->second != "0") {
                m_modeDetailsCache.frameContent.push_back("ab");
            }

            // Check bitsInConf
            auto confBitsIt = iniKeyValPairs.find("bitsInConf");
            if (confBitsIt != iniKeyValPairs.end() &&
                confBitsIt->second != "0") {
                m_modeDetailsCache.frameContent.push_back("conf");
            }
        }

        m_pcmFrame = m_modeDetailsCache.isPCM;

        uint16_t chipCmd = ADSD3500_CMD_MODE_SWITCH_BASE;
        chipCmd += mode;
        status = m_adsd3500Hardware->adsd3500_write_cmd(
            chipCmd, ADSD3500_CMD_MODE_SWITCH_PAYLOAD, 200000);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to switch mode in chip using host commands!";
            return status;
        }

        setDepthIniParams(m_config->getIniKeyValPairs(), false);
        configureSensorModeDetails();
        m_details.mode = mode;

        const auto &paramsForLogging = m_config->getIniKeyValPairs();
        LOG(INFO) << "Using the following configuration parameters for mode "
                  << int(mode);
        for (auto param : paramsForLogging) {
            LOG(INFO) << param.first << " : " << param.second;
        }

        if (m_config->getMetadataInAB() > 0) {
            if (!m_pcmFrame) {
                status =
                    adsd3500SetEnableMetadatainAB(m_config->getMetadataInAB());
                if (status != Status::OK) {
                    LOG(ERROR) << "Failed to set enableMetaDatainAB.";
                    return status;
                }
                LOG(INFO)
                    << "Metadata in AB is enabled and it is stored in the "
                       "first 128 bytes.";

            } else {
                status = adsd3500SetEnableMetadatainAB(0);
                if (status != Status::OK) {
                    LOG(ERROR) << "Failed to disable enableMetaDatainAB.";
                    return status;
                }
                LOG(INFO) << "Metadata in AB is disabled for this frame type.";
            }

        } else {
            status = adsd3500SetEnableMetadatainAB(m_config->getMetadataInAB());
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to set enableMetaDatainAB.";
                return status;
            }

            LOG(WARNING) << "Metadata in AB is disabled.";
        }

        // Store the frame details in camera details
        // When 90° rotation is enabled, the pixel data is transposed:
        // the SDK buffer layout becomes H×W, so report swapped dimensions.
        bool rotationEnabled = (rotationValue == "1");
        uint32_t reportedWidth = m_modeDetailsCache.baseResolutionWidth;
        uint32_t reportedHeight = m_modeDetailsCache.baseResolutionHeight;
        if (rotationEnabled) {
            std::swap(reportedWidth, reportedHeight);
        }

        m_details.mode = mode;
        m_details.frameType.width = reportedWidth;
        m_details.frameType.height = reportedHeight;
        m_details.frameType.totalCaptures = 1;
        m_details.frameType.dataDetails.clear();
        for (const auto &item : m_modeDetailsCache.frameContent) {
            if (item == "xyz" && !m_xyzEnabled) {
                continue;
            }

            FrameDataDetails fDataDetails;
            fDataDetails.type = item;
            fDataDetails.width = reportedWidth;
            fDataDetails.height = reportedHeight;
            fDataDetails.subelementSize = sizeof(uint16_t);
            fDataDetails.subelementsPerElement = 1;

            if (item == "xyz") {
                fDataDetails.subelementsPerElement = 3;
            } else if (item == "raw") {
                fDataDetails.width = m_modeDetailsCache.frameWidthInBytes;
                fDataDetails.height = m_modeDetailsCache.frameHeightInBytes;
                fDataDetails.subelementSize = 1;
                fDataDetails.subelementsPerElement = 1;
            } else if (item == "metadata") {
                fDataDetails.subelementSize = 1;
                fDataDetails.width = 128;
                fDataDetails.height = 1;
            } else if (item == "conf") {
                fDataDetails.subelementSize = sizeof(float);
            }
            fDataDetails.bytesCount = fDataDetails.width * fDataDetails.height *
                                      fDataDetails.subelementSize *
                                      fDataDetails.subelementsPerElement;

            m_details.frameType.dataDetails.emplace_back(fDataDetails);
        }

        // We want computed frames (Depth & AB). Tell target to initialize depth compute
        // Skip this for raw bypass mode (no ToF processing)
        // Exception: lens scatter mode needs TofiCompute to process raw Bayer input
        // Reuse iniKeyValPairs from line 660 (already set to current mode's params)
        auto lensScatterIt =
            iniKeyValPairs.find("lensScatterCompensationEnabled");
        bool lensScatterEnabled = (lensScatterIt != iniKeyValPairs.end() &&
                                   lensScatterIt->second == "1");

        if (!m_pcmFrame &&
            (!m_modeDetailsCache.isRawBypass || lensScatterEnabled)) {
            size_t dataSize = 0;
            std::string s;

            std::map<std::string, std::string> currentModeParams;
            if (m_config->getDepthParamsForMode(mode, currentModeParams) !=
                Status::OK) {
                LOG(ERROR) << "Mode " << (int)mode
                           << " not found in depth params map";
                return Status::INVALID_ARGUMENT;
            }

            // Create a string from the ini parameters
            for (auto &param : currentModeParams) {
                s += param.first + "=" + param.second + "\n";
            }
            dataSize = s.size();

            aditof::Status localStatus;

            // Check if ISP depth compute is enabled for this mode
            bool ispEnabled =
                (currentModeParams.find("depthComputeIspEnable") !=
                     currentModeParams.end() &&
                 currentModeParams["depthComputeIspEnable"] == "1");

            if (ispEnabled) {
                // ISP enabled: pass parsed TofiXYZDealiasData
                LOG(INFO) << "Initializing depth compute with ISP (parsed "
                             "dealias data)";
                // Copy all dealias data to a buffer for transmission
                TofiXYZDealiasData dealiasBuffer[10];
                for (int i = 0; i < 10; i++) {
                    m_calibrationMgr->getXYZDealiasData(i, dealiasBuffer[i]);
                }
                localStatus = m_depthSensor->initTargetDepthCompute(
                    (uint8_t *)s.c_str(), dataSize, (uint8_t *)dealiasBuffer,
                    sizeof(TofiXYZDealiasData) * 10);
            } else {
                // ISP disabled: pass raw CCB data
                LOG(INFO)
                    << "Initializing depth compute without ISP (raw CCB data)";
                const std::string &rawCCB = m_calibrationMgr->getRawCCBData();
                if (rawCCB.empty()) {
                    LOG(ERROR)
                        << "Raw CCB data not available for non-ISP mode!";
                    return Status::GENERIC_ERROR;
                }
                localStatus = m_depthSensor->initTargetDepthCompute(
                    (uint8_t *)s.c_str(), dataSize, (uint8_t *)rawCCB.c_str(),
                    rawCCB.size());
            }
            if (localStatus != aditof::Status::OK) {
                LOG(ERROR) << "Failed to initialize depth compute on target!";
                return localStatus;
            }

            if (!m_isOffline) {
                std::string depthComputeStatus;
                localStatus = m_depthSensor->getControl(
                    "depthComputeOpenSource", depthComputeStatus);
                if (localStatus == aditof::Status::OK) {
                    if (depthComputeStatus == "0") {
                        LOG(INFO)
                            << "Using closed source depth compute library.";
                    } else {
                        LOG(INFO) << "Using open source depth compute library.";
                    }
                } else {
                    LOG(ERROR)
                        << "Failed to get depth compute version from target!";
                }
            }
        }

        // If we compute XYZ then prepare the XYZ tables which depend on the mode
        if (m_xyzEnabled && !m_pcmFrame) {
            uint8_t _mode = m_modeDetailsCache.modeNumber;

            const int GEN_XYZ_ITERATIONS = 20;
            TofiXYZDealiasData dealias;
            m_calibrationMgr->getXYZDealiasData(_mode, dealias);
            TofiXYZDealiasData *pDealias = &dealias;

            m_calibrationMgr->cleanupXYZtables();
            XYZTable xyzTable = m_calibrationMgr->getXYZTable();
            int ret = Algorithms::GenerateXYZTables(
                &xyzTable.p_x_table, &xyzTable.p_y_table, &xyzTable.p_z_table,
                &(pDealias->camera_intrinsics), pDealias->n_sensor_rows,
                pDealias->n_sensor_cols,
                m_modeDetailsCache.baseResolutionHeight,
                m_modeDetailsCache.baseResolutionWidth, pDealias->n_offset_rows,
                pDealias->n_offset_cols, pDealias->row_bin_factor,
                pDealias->col_bin_factor, GEN_XYZ_ITERATIONS);
            m_calibrationMgr->setXYZTable(xyzTable);
            if (ret != 0 || !xyzTable.p_x_table || !xyzTable.p_y_table ||
                !xyzTable.p_z_table) {
                LOG(ERROR) << "Failed to generate the XYZ tables";
                return Status::GENERIC_ERROR;
            }
        }

        // If a Dynamic Mode Switching sequences has been loaded from config file then configure ADSD3500
        const auto &dmsSequence = m_config->getDmsSequence();
        if (dmsSequence.size() > 0) {
            status = this->adsd3500setEnableDynamicModeSwitching(true);
            if (status != Status::OK) {
                LOG(WARNING) << "Could not enable 'Dynamic Mode Switching.";
                return status;
            }

            status =
                this->adsds3500setDynamicModeSwitchingSequence(dmsSequence);
            // Note: sequence will be reloaded from config on next setMode if needed
            if (status != Status::OK) {
                LOG(WARNING) << "Could not set a sequence for the 'Dynamic "
                                "Mode Switching'.";
                return status;
            }
        }
    }

    return status;
}

/**
 * @brief Retrieves the current depth processing parameters from the sensor.
 *
 * Queries the underlying depth sensor for the frame processing parameters
 * currently active (only valid in online mode).
 *
 * @param[out] params Map to be populated with the current processing parameters.
 *
 * @return aditof::Status::OK if parameters retrieved successfully;
 *         aditof::Status::GENERIC_ERROR if called in offline mode or sensor fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status
CameraItof::getFrameProcessParams(std::map<std::string, std::string> &params) {

    aditof::Status status = aditof::Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->getDepthComputeParams(params);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "get ini parameters failed.";
    }

    return status;
}

/**
 * @brief Sets depth processing parameters on the sensor (online mode only).
 *
 * Applies depth computation parameter overrides to the underlying depth sensor.
 * Parameter changes take effect immediately; applying changes while streaming
 * is not recommended.
 *
 * @param[in] params Map of parameter name-value pairs to apply to the sensor.
 * @param[in] mode Reserved parameter for future multi-mode support (currently unused).
 *
 * @return aditof::Status::OK if parameters applied successfully;
 *         aditof::Status::GENERIC_ERROR if called in offline mode or sensor fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status
CameraItof::setFrameProcessParams(std::map<std::string, std::string> &params,
                                  int32_t mode) {
    aditof::Status status = aditof::Status::OK;

    if (m_isOffline) {
        status = aditof::Status::GENERIC_ERROR; // Invalid call
    } else {
        if (m_devStreaming)
            LOG(WARNING)
                << "Setting camera parameters while streaming is one is "
                   "not recommended";
        status = setDepthIniParams(params);
        if (status != aditof::Status::OK) {
            LOG(ERROR) << "Failed to set ini parameters on ADSD3500";
        }
    }
    return status;
}

/**
 * @brief Sets the offline playback file (offline mode only).
 *
 * Configures the file path for frame playback in offline mode. This must be
 * called before calling setMode() in offline operation.
 *
 * @param[in] filePath Path to the recorded frame file to play back.
 *
 * @return aditof::Status::OK if file set successfully;
 *         aditof::Status::GENERIC_ERROR if called in online mode.
 *
 * @note This function is only valid in offline mode.
 */
aditof::Status CameraItof::setPlaybackFile(std::string &filePath) {
    if (!m_isOffline) {
        return aditof::Status::GENERIC_ERROR; // Invalid call
    }
    return m_recordingMgr->setPlaybackFile(filePath);
}

/**
 * @brief Starts recording frames to a file (online mode only).
 *
 * Begins capturing and writing depth frames to a file in the specified location.
 * The file header contains frame metadata (mode details, XYZ parameters, etc.)
 * followed by raw frame data. Recording continues until stopRecording() is called.
 *
 * @param[in] filePath Destination file path for the recorded frame data.
 *
 * @return aditof::Status::OK if recording started successfully;
 *         aditof::Status::GENERIC_ERROR if called in offline mode or file I/O fails;
 *         aditof::Status::INVALID_ARGUMENT if mode is not supported.
 *
 * @note This function is only valid in online mode.
 * @note The file format includes a header with camera configuration and frame details.
 */
aditof::Status CameraItof::startRecording(std::string &filePath) {
    if (m_isOffline) {
        return aditof::Status::GENERIC_ERROR; // Invalid call
    }

    return m_recordingMgr->startRecording(
        filePath, m_cameraFps, m_details, m_modeDetailsCache,
        m_availableSensorModeDetails, m_depthEnabled, m_abEnabled,
        m_confEnabled, m_xyzEnabled);
}

/**
 * @brief Stops recording frames to file (online mode only).
 *
 * Terminates the ongoing recording session and closes the output file.
 *
 * @return aditof::Status::OK if recording stopped successfully;
 *         aditof::Status::GENERIC_ERROR if called in offline mode or sensor fails.
 *
 * @note This function is only valid in online mode.
 */
aditof::Status CameraItof::stopRecording() {
    if (m_isOffline) {
        return aditof::Status::GENERIC_ERROR; // Invalid call
    }
    return m_recordingMgr->stopRecording();
}

/**
 * @brief Resets INI parameters to defaults for a specific mode.
 *
 * Sends a command to the ADSD3500 to reset all depth computation parameters
 * for the given mode back to firmware defaults.
 *
 * @param[in] mode Frame mode number to reset parameters for.
 *
 * @return aditof::Status::OK if reset command sent successfully;
 *         error codes from depth sensor if command fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500ResetIniParamsForMode(const uint16_t mode) {
    aditof::Status status = aditof::Status::OK;

    assert(!m_isOffline);

    status =
        m_adsd3500Hardware->adsd3500_write_cmd(ADSD3500_REG_MODE_SELECT, mode);

    return status;
}

/**
 * @brief Retrieves the list of supported frame modes.
 *
 * Returns the list of frame modes (resolutions and configurations) supported
 * by the camera hardware and firmware.
 *
 * @param[out] availableModes Vector to be populated with supported mode numbers.
 *
 * @return aditof::Status::OK on success;
 *         aditof::Status::OK (with empty list) in offline mode.
 */
aditof::Status
CameraItof::getAvailableModes(std::vector<uint8_t> &availableModes) const {
    using namespace aditof;
    Status status = Status::OK;

    if (m_isOffline) {
        availableModes.clear();
    } else {
        availableModes.clear();

        for (const auto &mode : m_availableModes) {
            availableModes.emplace_back(mode);
        }
    }

    return status;
}

/**
 * @brief Retrieves and processes a depth frame from the sensor.
 *
 * Acquires a frame from the underlying depth sensor and populates the provided
 * Frame object with depth, AB, XYZ, and metadata components based on current
 * mode settings and enabled features.
 *
 * @details
 * The function performs the following operations:
 * - Validates and initializes frame buffer from either "frameData" (MP modes) or
 *   "ab" (QMP modes) based on PCM frame flag
 * - Optionally drops the first frame on initial startup (configurable via
 *   m_dropFirstFrame flag) to stabilize sensor output; caller controls retry via
 *   return status
 * - Retrieves actual frame data from depth sensor via V4L2/ISP pipeline
 * - Computes XYZ coordinates from depth data if enabled and buffers allocated
 * - Clears disabled data channels (depth/AB) to ensure clean output
 * - Extracts or generates metadata (128-byte header) and populates frame
 *
 * All buffer pointers are validated before use to prevent memory corruption.
 * Metadata struct size is verified at compile-time to fit within AB frame header.
 *
 * @param[in,out] frame Pointer to Frame object to populate. Must be non-null.
 *                       On return, contains depth, AB, XYZ, and metadata based
 *                       on enabled channels and current mode settings.
 * @param[in] index Reserved parameter for multi-frame operations (default: 0).
 *                  Currently unused; provided for API compatibility.
 *
 * @return aditof::Status::OK on successful frame acquisition and processing.
 *         aditof::Status::INVALID_ARGUMENT if frame pointer is null.
 *         aditof::Status::GENERIC_ERROR if frame buffer allocation fails or
 *                                        frame data location is null despite
 *                                        successful getData() call.
 *         Sensor error status codes from depth sensor operations (getFrame calls).
 *
 * @note Single-threaded access only. Caller must ensure exclusive access to
 *       requestFrame during camera streaming. Frame drop retry logic is
 *       intentional; caller determines when to stop retrying based on status.
 *
 * @note Drop-first-frame behavior: On initial startup with m_dropFirstFrame=true,
 *       the function performs two getFrame calls: one to discard initial frame
 *       and one to return actual data. This increases latency on first call.
 *       Failures during drop phase reset m_dropFrameOnce flag for retry.
 *
 * @note Memory safety: This function is hardened with comprehensive pointer
 *       validation and bounds checking. Metadata struct size is enforced at
 *       compile-time to prevent buffer overread from AB frame header.
 *
 * @see CameraItof::getDetails() for frame type information
 * @see buffer_processor.cpp for V4L2 frame acquisition details
 * @see Algorithms::ComputeXYZ() for XYZ coordinate generation
 */
aditof::Status CameraItof::requestFrame(aditof::Frame *frame, uint32_t index) {
    using namespace aditof;

    return m_frameAcqManager->requestFrame(
        frame, index, m_details.frameType, m_modeDetailsCache, m_isOffline,
        m_pcmFrame, m_depthEnabled, m_abEnabled, m_confEnabled, m_xyzEnabled,
        m_confBitsPerPixel, m_abBitsPerPixel, m_depthBitsPerPixel,
        m_dropFrameOnce);
}

/**
 * @brief Retrieves detailed camera information and configuration.
 *
 * Returns camera details including mode, resolution, frame type information,
 * intrinsics, serial number, firmware versions, and other metadata.
 *
 * @param[out] details CameraDetails struct to be populated with camera information.
 *
 * @return aditof::Status::OK.
 */
aditof::Status CameraItof::getDetails(aditof::CameraDetails &details) const {
    using namespace aditof;
    Status status = Status::OK;

    details = m_details;

    return status;
}

/**
 * @brief Retrieves the underlying depth sensor interface.
 *
 * Returns a shared pointer to the DepthSensorInterface used by this camera
 * for V4L2 communication and frame acquisition.
 *
 * @return Shared pointer to the depth sensor interface.
 */
std::shared_ptr<aditof::DepthSensorInterface> CameraItof::getSensor() {
    return m_depthSensor;
}

/**
 * @brief Retrieves the list of available camera control strings.
 *
 * Returns the names of all camera controls that can be queried or modified
 * via setControl() and getControl().
 *
 * @param[out] controls Vector to be populated with control name strings.
 *
 * @return aditof::Status::OK.
 */
aditof::Status
CameraItof::getAvailableControls(std::vector<std::string> &controls) const {
    using namespace aditof;
    Status status = Status::OK;

    controls.clear();
    controls.reserve(m_controls.size());
    for (const auto &item : m_controls) {
        controls.emplace_back(item.first);
    }

    return status;
}

/**
 * @brief Sets a camera control to a specified value.
 *
 * Modifies a camera control setting. If the value is "call", invokes the
 * control as a callable function. Otherwise, stores the value for the control.
 *
 * @param[in] control Name of the control to set.
 * @param[in] value Value to set (or "call" to invoke as a function).
 *
 * @return aditof::Status::OK if control set successfully;
 *         aditof::Status::INVALID_ARGUMENT if control is unsupported.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::setControl(const std::string &control,
                                      const std::string &value) {
    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    if (m_controls.count(control) > 0) {
        if (value == "call") {
            status = m_noArgCallables.at(control)();
        } else {
            m_controls[control] = value;
        }
    } else {
        LOG(WARNING) << "Unsupported control";
        status = Status::INVALID_ARGUMENT;
    }

    return status;
}

/**
 * @brief Retrieves the current value of a camera control.
 *
 * Queries the current setting of a named camera control.
 *
 * @param[in] control Name of the control to query.
 * @param[out] value Current value of the control.
 *
 * @return aditof::Status::OK if control retrieved successfully;
 *         aditof::Status::INVALID_ARGUMENT if control is unsupported.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::getControl(const std::string &control,
                                      std::string &value) const {
    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    if (m_controls.count(control) > 0) {
        value = m_controls.at(control);
    } else {
        LOG(WARNING) << "Unsupported control";
        return Status::INVALID_ARGUMENT;
    }

    return status;
}

/**
 * @brief Reads the module serial number from the sensor.
 *
 * Retrieves the 32-byte serial number programmed in the ADSD3500 module.
 * Supports caching to avoid repeated sensor reads. Requires firmware version 4710+.
 *
 * @param[out] serialNumber String populated with the 32-byte serial number.
 * @param[in] useCacheValue If true, returns cached value if available; defaults to true.
 *
 * @return aditof::Status::OK if serial read successfully;
 *         aditof::Status::UNAVAILABLE if firmware does not support serial read;
 *         aditof::Status::GENERIC_ERROR if sensor read fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::readSerialNumber(std::string &serialNumber,
                                            bool useCacheValue) {
    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    if (m_adsd3500FwVersionInt < 4710) {
        LOG(WARNING) << "Serial read is not supported in this firmware!";
        return Status::UNAVAILABLE;
    }

    if (useCacheValue) {
        if (m_details.serialNumber.empty()) {
            LOG(INFO)
                << "No serial number stored in cache. Reading from memory.";
        } else {
            serialNumber = m_details.serialNumber;
            return status;
        }
    }

    uint8_t serial[32] = {0};

    status = m_adsd3500Hardware->adsd3500_read_payload_cmd(0x19, serial, 32);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to read serial number!";
        return status;
    }

    m_details.serialNumber = std::string(reinterpret_cast<char *>(serial), 32);
    serialNumber = m_details.serialNumber;

    return status;
}

/**
 * @brief Saves the ADSD3500 CCB (Configuration Calibration Block) to a file.
 *
 * Reads the configuration calibration block from the ADSD3500 memory and saves
 * it to the specified file path for backup or analysis. Includes CRC validation
 * to ensure data integrity. Delegates to CalibrationManager.
 *
 * @param[in] filepath Path where the CCB file should be written.
 *
 * @return aditof::Status::OK if CCB saved successfully;
 *         aditof::Status::INVALID_ARGUMENT if filepath is empty;
 *         aditof::Status::GENERIC_ERROR if read fails after retries or CRC invalid.
 *
 * @note This function asserts that the camera is not in offline mode.
 * @note Includes automatic retry logic (NR_READADSD3500CCB attempts) for reliability.
 */
aditof::Status CameraItof::saveModuleCCB(const std::string &filepath) {
    assert(!m_isOffline);
    return m_calibrationMgr->saveCCBToFile(filepath);
}

/**
 * @brief Saves module configuration (CFG) to a file.
 *
 * Placeholder for future configuration file export functionality.
 * Currently not implemented.
 *
 * @param[in] filepath Path where the CFG file should be written.
 *
 * @return aditof::Status::UNAVAILABLE (not currently supported);
 *         aditof::Status::INVALID_ARGUMENT if filepath is empty.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::saveModuleCFG(const std::string &filepath) const {

    assert(!m_isOffline);

    if (filepath.empty()) {
        LOG(ERROR) << "File path where CFG should be written is empty.";
        return aditof::Status::INVALID_ARGUMENT;
    }

    LOG(ERROR) << "CFG files is unavailable";

    return aditof::Status::UNAVAILABLE;
}

/**
 * @brief Enables or disables XYZ coordinate computation.
 *
 * Controls whether XYZ (3D world coordinates) data is computed and included
 * in output frames. When disabled, only depth and AB data are provided.
 * This setting takes precedence over the INI parameter configuration.
 *
 * @param[in] enable True to enable XYZ computation; false to disable.
 *
 * @return aditof::Status::OK.
 *
 * @note Calling this function sets m_xyzSetViaApi to true, preventing INI
 *       parameters from overriding the user setting.
 */
aditof::Status CameraItof::enableXYZframe(bool enable) {
    m_xyzEnabled = enable;
    m_xyzSetViaApi = true;

    return aditof::Status::OK;
}

/**
 * @brief Placeholder for depth compute enable control.
 *
 * Currently not supported by the camera implementation.
 *
 * @param[in] enable Requested enable state (currently ignored).
 *
 * @return aditof::Status::UNAVAILABLE.
 */
aditof::Status CameraItof::enableDepthCompute(bool enable) {
    return aditof::Status::UNAVAILABLE;
}

/**
 * @brief Updates the ADSD3500 firmware from a binary file.
 *
 * Delegates firmware update operation to CameraFirmwareManager, which handles:
 * - Firmware file validation and loading
 * - Mode switching (Standard → Burst → Standard)
 * - Chunked transmission with CRC validation
 * - Interrupt-based completion detection
 *
 * @param[in] fwFilePath Path to the firmware binary file (.bin).
 *
 * @return aditof::Status::OK if firmware updated successfully;
 *         aditof::Status::GENERIC_ERROR if update fails or times out.
 *
 * @note This function asserts that the camera is not in offline mode.
 * @note Timeout is 60 seconds for firmware reprogramming. Without interrupt
 *       support, the function waits 60 seconds unconditionally.
 * @note The chip is automatically switched back to standard mode after update.
 */
aditof::Status
CameraItof::adsd3500UpdateFirmware(const std::string &fwFilePath) {
    using namespace aditof;
    assert(!m_isOffline);

    return m_firmwareManager->updateFirmware(fwFilePath);
}

/**
 * @brief Configures sensor mode details for online or offline operation.
 *
 * Delegates to SensorConfigHelper to handle bit depth configuration,
 * lens scatter compensation, and frame type enables.
 *
 * @note Updates m_depthEnabled, m_abEnabled, m_confEnabled, and m_xyzEnabled flags
 *       based on sensor configuration or recorded content in offline mode.
 */
void CameraItof::configureSensorModeDetails() {
    m_sensorConfigHelper->configureModeDetails(
        m_isOffline, m_modeDetailsCache, m_depthEnabled, m_abEnabled,
        m_confEnabled, m_xyzEnabled, m_xyzSetViaApi, m_depthBitsPerPixel,
        m_abBitsPerPixel, m_confBitsPerPixel);
}

/**
 * @brief Extracts an integer value from a JSON object.
 *
 * Looks up a key in the JSON object and retrieves its value as a string,
 * then converts it to an integer. Used for parsing depth computation configuration
 * parameters from JSON format.
 *
 * @param[in] config_json Pointer to JSON object to query
 * @param[in] key String key to look up in the JSON object
 *
 * @return Integer value if key found and string is valid, -1 otherwise
 *
 * @note Currently unused but retained for potential future use in JSON configuration parsing.
 */
/* // Not currently used, but leaving in case it is needed later.
static int16_t getValueFromJSON(json_object *config_json, std::string key) {
    int16_t value = -1;
    json_object *json_value = NULL;
    if (json_object_object_get_ex(config_json, key.c_str(), &json_value)) {
        if (json_object_is_type(json_value, json_type_string)) {
            const char *valuestring = json_object_get_string(json_value);
            if (valuestring != NULL) {
                value = atoi(valuestring);
            }
        }
    }
    return value;
}
*/

/**
 * @brief Retrieves depth processing parameters for all modes.
 *
 * Loads depth computation INI parameters from either a configuration file
 * (if provided during initialization) or from the sensor firmware defaults.
 * Populates the internal parameter map for all available modes.
 *
 * @return aditof::Status::OK on success.
 *
 * @note This function asserts that the camera is not in offline mode.
 * @note Creates m_depth_params_map_reset backup on first call.
 */
aditof::Status CameraItof::retrieveDepthProcessParams() {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    if (m_initConfigFilePath == "") {

        for (const auto &mode : m_availableModes) {

            std::string iniArray;
            m_depthSensor->getIniParamsArrayForMode(mode, iniArray);

            std::map<std::string, std::string> paramsMap;
            UtilsIni::getKeyValuePairsFromString(iniArray, paramsMap);
            m_config->setDepthParamsForMode(mode, paramsMap);
        }
        m_config->cacheDepthParams();
    } else {
        loadDepthParamsFromJsonFile(m_initConfigFilePath);
    }
    return status;
}

/**
 * @brief Resets all depth parameters to the original cached values.
 *
 * Restores the depth parameter map to the state captured during the first
 * call to retrieveDepthProcessParams(). Useful for reverting custom modifications.
 *
 * @return aditof::Status::OK if parameters reset successfully;
 *         aditof::Status::GENERIC_ERROR if no cached parameters available.
 */
aditof::Status CameraItof::resetDepthProcessParams() {
    return m_config->restoreCachedDepthParams();
}

aditof::Status CameraItof::setRotationEnabled(bool enable) {
    return m_depthSensor->setControl("enableRotation", enable ? "1" : "0");
}

/**
 * @brief Auto-discovers JSON configuration file from binary path or environment variable.
 *
 * Searches for a configuration file in the following order:
 * 1. Binary directory (where the executable is located): adcam_config.json
 * 2. Environment variable: ADCAM_CONFIG_PATH
 * 3. Returns empty string if not found (proceed with factory defaults)
 *
 * @return Path to discovered JSON file, or empty string if not found.
 */
std::string CameraItof::autoDiscoverConfigFile() {
    return m_config->autoDiscoverConfigFile();
}

/**
 * @brief Saves current depth parameters to a JSON configuration file.
 *
 * Delegates to CameraConfiguration for JSON serialization.
 *
 * @param[in] savePathFile Path where the JSON configuration file should be written.
 *
 * @return aditof::Status::OK if file saved successfully;
 *         aditof::Status::GENERIC_ERROR if file I/O fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status
CameraItof::saveDepthParamsToJsonFile(const std::string &savePathFile) {
    assert(!m_isOffline);
    return m_config->saveDepthParamsToJsonFile(savePathFile);
}

/**
 * @brief Loads depth parameters from a JSON configuration file.
 *
 * Parses a JSON configuration file and populates the depth parameter map with
 * mode-specific settings. Also extracts global settings (MIPI speed, deskew,
 * temp compensation, dynamic mode switching sequences, etc.).
 * Optionally loads parameters for a specific mode only.
 *
 * @param[in] pathFile Path to the JSON configuration file.
 * @param[in] mode_in_use If >= 0, load parameters only for this specific mode;
 *                         if < 0, load parameters for all available modes.
 *
 * @return aditof::Status::OK on successful parse;
 *         aditof::Status::OK (with partial data) if file malformed but parseable.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status
CameraItof::loadDepthParamsFromJsonFile(const std::string &pathFile,
                                        const int16_t mode_in_use) {
    assert(!m_isOffline);
    return m_config->loadDepthParamsFromJsonFile(pathFile, mode_in_use);
}

/**
 * @brief Enables or disables the first-frame drop behavior.
 *
 * Controls whether requestFrame() drops the first sensor frame to allow the
 * imager to stabilize. Useful for mitigating sensor startup transients.
 *
 * @param[in] dropFrame True to drop first frame; false to use all frames.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
void CameraItof::dropFirstFrame(bool dropFrame) {
    assert(!m_isOffline);
    m_config->setDropFirstFrame(dropFrame);
}

/**
 * @brief Sets the FSYNC toggle mode (master vs. slave operation).
 *
 * Configures how the ADSD3500 handles the FSYNC signal for frame timing:
 * - Mode 0: FSYNC does not automatically toggle
 * - Mode 1: FSYNC automatically toggles at user-specified frame rate (master)
 * - Mode 2: FSYNC is driven by external source; pin acts as input (slave)
 *
 * @param[in] mode FSYNC toggle mode (0, 1, or 2).
 *
 * @return aditof::Status::OK if mode set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetToggleMode(int mode) {
    assert(!m_isOffline);

    aditof::Status status = m_adsd3500Ctrl->setToggleMode(mode);
    if (status != aditof::Status::OK) {
        return status;
    }

    if (mode == 2) {
        m_adsd3500_master = false;
    }

    return status;
}

/**
 * @brief Manually toggles the FSYNC signal (master mode only).
 *
 * Sends a command to toggle the FSYNC output. Only valid when ADSD3500 is
 * configured as the timing master (m_adsd3500_master = true).
 *
 * @return aditof::Status::OK if FSYNC toggled successfully;
 *         aditof::Status error codes if not master or sensor fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500ToggleFsync() {
    assert(!m_isOffline);

    if (!m_adsd3500_master) {
        LOG(ERROR) << "ADSD3500 not set as master - cannot toggle FSYNC";
        return aditof::Status::GENERIC_ERROR;
    }

    return m_adsd3500Ctrl->toggleFsync();
}

/**
 * @brief Retrieves ADSD3500 firmware version and Git commit hash.
 *
 * Reads the 44-byte firmware version structure from the ADSD3500 ROM,
 * extracting the version number (4 bytes) and Git hash (40 bytes).
 * Caches results in m_adsd3500FwGitHash and m_adsd3500FwVersionInt.
 *
 * @param[out] fwVersion Formatted version string (e.g., "X.Y.Z.W").
 * @param[out] fwHash 40-character Git commit hash string.
 *
 * @return aditof::Status::OK if version read successfully;
 *         aditof::Status error codes if sensor read fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500GetFirmwareVersion(std::string &fwVersion,
                                                      std::string &fwHash) {
    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    uint8_t fwData[44] = {0};
    fwData[0] = uint8_t(1);

    status = m_adsd3500Hardware->adsd3500_read_payload_cmd(0x05, fwData, 44);
    if (status != Status::OK) {
        LOG(INFO) << "Failed to retrieve fw version and git hash for "
                     "adsd3500!";
        return status;
    }

    std::string fwv;

    fwv = std::to_string(fwData[0]) + '.' + std::to_string(fwData[1]) + '.' +
          std::to_string(fwData[2]) + '.' + std::to_string(fwData[3]);

    m_adsd3500FwGitHash =
        std::make_pair(fwv, std::string((char *)(fwData + 4), 40));

    m_adsd3500FwVersionInt = 0;
    for (int i = 0; i < 4; i++) {
        m_adsd3500FwVersionInt = m_adsd3500FwVersionInt * 10 + fwData[i];
    }

    fwVersion = m_adsd3500FwGitHash.first;
    fwHash = m_adsd3500FwGitHash.second;

    return status;
}

/**
 * @brief Sets the AB (amplitude) invalidation threshold.
 *
 * Configures the minimum AB value below which depth pixels are marked invalid.
 * Prevents low-confidence measurements from being used.
 *
 * @param[in] threshold Threshold value (sensor-specific units).
 *
 * @return aditof::Status::OK if threshold set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetABinvalidationThreshold(int threshold) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setABinvalidationThreshold(threshold);
}

/**
 * @brief Retrieves the current AB invalidation threshold.
 *
 * @param[out] threshold Current threshold value.
 *
 * @return aditof::Status::OK if threshold read successfully;
 *         aditof::Status error codes if sensor read fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500GetABinvalidationThreshold(int &threshold) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->getABinvalidationThreshold(threshold);
}

/**
 * @brief Sets the confidence validity threshold.
 *
 * Configures the minimum confidence level required for depth pixels to be
 * considered valid. Lower confidence pixels are invalidated.
 *
 * @param[in] threshold Confidence threshold value.
 *
 * @return aditof::Status::OK if threshold set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetConfidenceThreshold(int threshold) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setConfidenceThreshold(threshold);
}
/**
 * @brief Retrieves the current confidence threshold.
 *
 * @param[out] threshold Current threshold value.
 *
 * @return aditof::Status::OK if threshold read successfully;
 *         aditof::Status error codes if sensor read fails.
 */
aditof::Status CameraItof::adsd3500GetConfidenceThreshold(int &threshold) {
    if (m_isOffline) {
        return aditof::Status::OK;
    }
    return m_adsd3500Ctrl->getConfidenceThreshold(threshold);
}

/**
 * @brief Enables or disables the JBLF (Joint Bilateral Laplacian Filter).
 *
 * Controls whether bilateral filtering is applied to the depth map to reduce
 * noise while preserving depth edges.
 *
 * @param[in] enable True to enable JBLF; false to disable.
 *
 * @return aditof::Status::OK if state set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetJBLFfilterEnableState(bool enable) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setJBLFfilterEnableState(enable);
}
/**
 * @brief Retrieves the JBLF filter enable state.
 *
 * @param[out] enabled True if JBLF is enabled; false otherwise.
 *
 * @return aditof::Status::OK if state read successfully;
 *         aditof::Status error codes if sensor read fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500GetJBLFfilterEnableState(bool &enabled) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->getJBLFfilterEnableState(enabled);
}

/**
 * @brief Sets the JBLF filter kernel size.
 *
 * Configures the spatial window size for the bilateral filter. Larger sizes
 * increase smoothing but reduce edge definition.
 *
 * @param[in] size Filter kernel size (e.g., 3, 5, 7).
 *
 * @return aditof::Status::OK if size set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetJBLFfilterSize(int size) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setJBLFfilterSize(size);
}
/**
 * @brief Retrieves the current JBLF filter kernel size.
 *
 * @param[out] size Current filter kernel size.
 *
 * @return aditof::Status::OK if size read successfully;
 *         aditof::Status error codes if sensor read fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500GetJBLFfilterSize(int &size) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->getJBLFfilterSize(size);
}

/**
 * @brief Sets the minimum radial distance validity threshold.
 *
 * Configures the closest distance at which depth measurements are considered valid.
 * Pixels closer than this threshold are marked invalid (blind zone).
 *
 * @param[in] threshold Minimum radial distance threshold.
 *
 * @return aditof::Status::OK if threshold set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetRadialThresholdMin(int threshold) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setRadialThresholdMin(threshold);
}
/**
 * @brief Retrieves the minimum radial distance threshold.
 *
 * @param[out] threshold Current minimum threshold value.
 *
 * @return aditof::Status::OK if threshold read successfully;
 *         aditof::Status error codes if sensor read fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500GetRadialThresholdMin(int &threshold) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->getRadialThresholdMin(threshold);
}

/**
 * @brief Sets the maximum radial distance validity threshold.
 *
 * Configures the farthest distance at which depth measurements are considered valid.
 * Pixels farther than this threshold are marked invalid (range limit).
 *
 * @param[in] threshold Maximum radial distance threshold.
 *
 * @return aditof::Status::OK if threshold set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetRadialThresholdMax(int threshold) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setRadialThresholdMax(threshold);
}

/**
 * @brief Retrieves the maximum radial distance threshold.
 *
 * @param[out] threshold Current maximum threshold value.
 *
 * @return aditof::Status::OK if threshold read successfully;
 *         aditof::Status error codes if sensor read fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500GetRadialThresholdMax(int &threshold) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->getRadialThresholdMax(threshold);
}

/**
 * @brief Sets the MIPI camera serial interface output speed.
 *
 * Configures the data rate of the MIPI CSI-2 interface used to stream depth
 * frames to the host. Higher speeds support higher frame rates and resolutions.
 *
 * @param[in] speed MIPI output speed setting (device-specific encoding).
 *
 * @return aditof::Status::OK if speed set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetMIPIOutputSpeed(uint16_t speed) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setMIPIOutputSpeed(speed);
}

/**
 * @brief Enables or disables raw bypass mode on ADSD3500.
 *
 * Writes to register 0x0033 to control raw bypass mode. When enabled (0x0001),
 * the ISP outputs unprocessed raw Bayer sensor data instead of computing
 * depth/AB/confidence frames. When disabled (0x0000), standard depth computation
 * occurs.
 *
 * @param[in] enable true to enable raw bypass mode; false for standard depth mode.
 *
 * @return aditof::Status::OK if register write succeeds;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 * @note Register 0x0033 must be written before V4L2 driver configuration.
 */
aditof::Status CameraItof::adsd3500SetRawBypassMode(bool enable) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setRawBypassMode(enable);
}

/**
 * @brief Enables or disables hardware deskew on stream start.
 *
 * Configures whether the ADSD3500 applies image deskew correction when
 * streaming begins (useful for rolling shutter compensation).
 *
 * @param[in] value 1 to enable deskew; 0 to disable.
 *
 * @return aditof::Status::OK if value set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetEnableDeskewAtStreamOn(uint16_t value) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setEnableDeskewAtStreamOn(value);
}

/**
 * @brief Retrieves the current MIPI output speed setting.
 *
 * @param[out] speed Current MIPI speed value.
 *
 * @return aditof::Status::OK if speed read successfully;
 *         aditof::Status error codes if sensor read fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500GetMIPIOutputSpeed(uint16_t &speed) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->getMIPIOutputSpeed(speed);
}

/**
 * @brief Retrieves the imager error status code.
 *
 * Reads the error register from the paired imager (ADTF3080, ADTF3066, ADSD3100, etc.)
 * to diagnose hardware issues.
 *
 * @param[out] errcode Error code from the imager.
 *
 * @return aditof::Status::OK if error code read successfully;
 *         aditof::Status error codes if sensor read fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500GetImagerErrorCode(uint16_t &errcode) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->getImagerErrorCode(errcode);
}

/**
 * @brief Sets the VCSEL laser pulse timing delay.
 *
 * Configures the delay between the modulation clock and VCSEL firing for
 * phase adjustment. Used for depth calibration and mode-specific optimization.
 *
 * @param[in] delay VCSEL timing delay value.
 *
 * @return aditof::Status::OK if delay set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetVCSELDelay(uint16_t delay) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setVCSELDelay(delay);
}

/**
 * @brief Retrieves the current VCSEL timing delay.
 *
 * @param[out] delay Current VCSEL delay value.
 *
 * @return aditof::Status::OK if delay read successfully;
 *         aditof::Status error codes if sensor read fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500GetVCSELDelay(uint16_t &delay) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->getVCSELDelay(delay);
}

/**
 * @brief Sets the JBLF maximum edge threshold.
 *
 * Configures the edge detection threshold for the bilateral filter.
 * Transitions exceeding this threshold are treated as edges and preserved.
 *
 * @param[in] threshold Maximum edge threshold value.
 *
 * @return aditof::Status::OK if threshold set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetJBLFMaxEdgeThreshold(uint16_t threshold) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setJBLFMaxEdgeThreshold(threshold);
}

/**
 * @brief Sets the JBLF AB (amplitude) threshold for filtering.
 *
 * Configures the minimum amplitude required for a pixel to participate in
 * the bilateral filtering process.
 *
 * @param[in] threshold Minimum AB threshold for filtering.
 *
 * @return aditof::Status::OK if threshold set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetJBLFABThreshold(uint16_t threshold) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setJBLFABThreshold(threshold);
}

/**
 * @brief Sets the JBLF Gaussian smoothing parameter (sigma).
 *
 * Configures the standard deviation of the Gaussian kernel for spatial
 * weighting in the bilateral filter. Higher values increase smoothing range.
 *
 * @param[in] value Gaussian sigma parameter value.
 *
 * @return aditof::Status::OK if parameter set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetJBLFGaussianSigma(uint16_t value) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setJBLFGaussianSigma(value);
}

/**
 * @brief Retrieves the JBLF Gaussian sigma parameter.
 *
 * @param[out] value Current Gaussian sigma value.
 *
 * @return aditof::Status::OK if parameter read successfully;
 *         aditof::Status error codes if sensor read fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500GetJBLFGaussianSigma(uint16_t &value) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->getJBLFGaussianSigma(value);
}

/**
 * @brief Sets the JBLF exponential decay parameter.
 *
 * Configures the rate of spatial weighting decay in the bilateral filter.
 * Controls how quickly the filter influence drops with distance.
 *
 * @param[in] value Exponential decay parameter.
 *
 * @return aditof::Status::OK if parameter set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetJBLFExponentialTerm(uint16_t value) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setJBLFExponentialTerm(value);
}

/**
 * @brief Retrieves the JBLF exponential decay parameter.
 *
 * @param[out] value Current exponential term value.
 *
 * @return aditof::Status::OK if parameter read successfully;
 *         aditof::Status error codes if sensor read fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500GetJBLFExponentialTerm(uint16_t &value) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->getJBLFExponentialTerm(value);
}

/**
 * @brief Retrieves the current frame rate setting.
 *
 * Returns the frame rate in offline mode from cached parameters, or reads
 * from the ADSD3500 in online mode.
 *
 * @param[out] fps Current frame rate in frames per second.
 *
 * @return aditof::Status::OK if frame rate read successfully;
 *         aditof::Status error codes if sensor read fails.
 */
aditof::Status CameraItof::adsd3500GetFrameRate(uint16_t &fps) {
    if (m_isOffline) {
        fps = m_recordingMgr->getFrameRate();
        return aditof::Status::OK;
    }
    return m_adsd3500Ctrl->getFrameRate(fps);
}

/**
 * @brief Sets the camera frame rate.
 *
 * Configures the frame acquisition rate via the sensor control interface.
 * If fps is 0, defaults to 10 FPS with a warning.
 *
 * @param[in] fps Desired frame rate in frames per second.
 *
 * @return aditof::Status::OK if frame rate set successfully;
 *         aditof::Status error codes if sensor fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetFrameRate(uint16_t fps) {
    assert(!m_isOffline);

    aditof::Status status = m_adsd3500Ctrl->setFrameRate(fps);
    if (status == aditof::Status::OK) {
        m_cameraFps = fps;
        LOG(INFO) << "Camera FPS set from parameter list at: " << m_cameraFps;
    }
    return status;
}

/**
 * @brief Enables or disables edge-based confidence adjustment.
 *
 * Controls whether the ISP adjusts confidence scores based on detected edges
 * to improve depth reliability at object boundaries.
 *
 * @param[in] value 1 to enable edge confidence; 0 to disable.
 *
 * @return aditof::Status::OK if value set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetEnableEdgeConfidence(uint16_t value) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setEnableEdgeConfidence(value);
}

/**
 * @brief Retrieves the temperature compensation enable status.
 *
 * @param[out] value Temperature compensation status (0=disabled, 1=enabled).
 *
 * @return aditof::Status::OK if status read successfully;
 *         aditof::Status error codes if sensor read fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status
CameraItof::adsd3500GetTemperatureCompensationStatus(uint16_t &value) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->getTemperatureCompensationStatus(value);
}

/**
 * @brief Enables or disables phase invalidation filtering.
 *
 * Controls whether the ISP marks pixels with invalid phase as invalid depth.
 * Helps prevent artifacts from poor signal-to-noise phases.
 *
 * @param[in] value 1 to enable phase invalidation; 0 to disable.
 *
 * @return aditof::Status::OK if value set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetEnablePhaseInvalidation(uint16_t value) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setEnablePhaseInvalidation(value);
}

/**
 * @brief Enables or disables temperature compensation in the ADSD3500.
 *
 * Configures the ADSD3500 ISP to apply or disable temperature compensation
 * algorithms to account for thermal effects on sensor performance and depth accuracy.
 *
 * @param[in] value 1 to enable temperature compensation; 0 to disable.
 *
 * @return aditof::Status::OK if value set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 * @note Uses command 0x0021 to configure the temperature compensation setting.
 */
aditof::Status
CameraItof::adsd3500SetEnableTemperatureCompensation(uint16_t value) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setEnableTemperatureCompensation(value);
}

/**
 * @brief Enables or disables metadata embedding in the AB frame.
 *
 * Controls whether the ADSD3500 embeds 128-byte metadata (frame info, status, etc.)
 * at the start of the AB (amplitude) frame data.
 *
 * @param[in] value 1 to enable metadata embedding; 0 to disable.
 *
 * @return aditof::Status::OK if value set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetEnableMetadatainAB(uint16_t value) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setEnableMetadatainAB(value);
}

/**
 * @brief Retrieves the metadata embedding enable status.
 *
 * @param[out] value Metadata embedding status (0=disabled, 1=enabled).
 *
 * @return aditof::Status::OK if status read successfully;
 *         aditof::Status error codes if sensor read fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500GetEnableMetadatainAB(uint16_t &value) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->getEnableMetadatainAB(value);
}

/**
 * @brief Writes a generic 16-bit register value to the ADSD3500.
 *
 * Generic register write interface for setting arbitrary ADSD3500 registers
 * not explicitly handled by dedicated functions.
 *
 * @param[in] reg Register address.
 * @param[in] value Value to write.
 *
 * @return aditof::Status::OK if register written successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500SetGenericTemplate(uint16_t reg,
                                                      uint16_t value) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->setGenericTemplate(reg, value);
}

/**
 * @brief Reads a generic 16-bit register value from the ADSD3500.
 *
 * Generic register read interface for reading arbitrary ADSD3500 registers
 * not explicitly handled by dedicated functions.
 *
 * @param[in] reg Register address.
 * @param[out] value Register value to be populated.
 *
 * @return aditof::Status::OK if register read successfully;
 *         aditof::Status error codes if sensor read fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500GetGenericTemplate(uint16_t reg,
                                                      uint16_t &value) {
    assert(!m_isOffline);
    return m_adsd3500Ctrl->getGenericTemplate(reg, value);
}

/**
 * @brief Disables or enables the CCBM (Calibration Calibration Block Manager).
 *
 * Controls whether the ADSD3500 applies stored calibration data to depth computation.
 *
 * @param[in] disable True to disable CCBM (bypass calibration); false to enable.
 *
 * @return aditof::Status::OK if control applied successfully;
 *         aditof::Status error codes if sensor fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500DisableCCBM(bool disable) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->setControl("disableCCBM", std::to_string(disable));

    return status;
}

/**
 * @brief Checks if CCBM (calibration support) is available in firmware.
 *
 * Queries the sensor for CCBM capability and translates the control value
 * to a boolean.
 *
 * @param[out] supported True if CCBM is supported; false otherwise.
 *
 * @return aditof::Status::OK if support status determined;
 *         aditof::Status::INVALID_ARGUMENT if control value is invalid.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500IsCCBMsupported(bool &supported) {

    aditof::Status status = aditof::Status::OK;

    assert(!m_isOffline);

    std::string availableCCMB;

    status = m_depthSensor->getControl("availableCCBM", availableCCMB);
    if (status == aditof::Status::OK) {
        if (availableCCMB == "1") {
            supported = true;
        } else if (availableCCMB == "0") {
            supported = false;
        } else {
            LOG(ERROR) << "Invalid value for control availableCCBM: "
                       << availableCCMB;
            return aditof::Status::INVALID_ARGUMENT;
        }
    }

    return status;
}

/**
 * @brief Reads ADSD3500 and imager error status.
 *
 * Queries the chip and imager status registers. Logs detailed error messages
 * if errors are detected, with error code translation based on imager type.
 *
 * @param[out] chipStatus ADSD3500 chip status code (0 or 41 = healthy).
 * @param[out] imagerStatus Imager-specific status code (interpretation depends on type).
 *
 * @return aditof::Status::OK if status read successfully;
 *         aditof::Status::GENERIC_ERROR if sensor communication fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500GetStatus(int &chipStatus,
                                             int &imagerStatus) {
    aditof::Status status = aditof::Status::OK;

    assert(!m_isOffline);

    status = m_adsd3500Hardware->adsd3500_get_status(chipStatus, imagerStatus);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to read chip/imager status!";
        return status;
    }

    if (chipStatus != 0 && chipStatus != 41) {
        LOG(ERROR) << "ADSD3500 error detected: "
                   << m_adsdErrors.GetStringADSD3500(chipStatus);

        if (chipStatus == m_adsdErrors.ADSD3500_STATUS_IMAGER_ERROR) {
            if (m_imagerType == aditof::ImagerType::ADSD3100) {
                LOG(ERROR) << "ADSD3100 imager error detected: "
                           << m_adsdErrors.GetStringADSD3100(imagerStatus);
            } else if (m_imagerType == aditof::ImagerType::ADSD3030) {
                LOG(ERROR) << "ADSD3030 imager error detected: "
                           << m_adsdErrors.GetStringADSD3030(imagerStatus);
            } else if (m_imagerType == aditof::ImagerType::ADTF3080) {
                LOG(ERROR) << "ADTF3080 imager error detected: "
                           << m_adsdErrors.GetStringADSD3080(imagerStatus);
            } else if (m_imagerType == aditof::ImagerType::ADTF3066) {
                LOG(ERROR) << "ADTF3066 imager error detected: "
                           << m_adsdErrors.GetStringADTF3066(imagerStatus);
            } else {
                LOG(ERROR) << "Imager error detected. Cannot be displayed "
                              "because imager type is unknown";
            }
        }
    } else {
        LOG(INFO) << "No chip/imager errors detected.";
    }

    return status;
}

/**
 * @brief Reads the current sensor temperature.
 *
 * Retrieves the temperature of the ADTF imager from the ADSD3500 sensor bridge.
 * Uses frame-rate-based timing delays to allow sensor settling.
 *
 * @param[out] tmpValue Temperature value (sensor-specific units).
 *
 * @return aditof::Status::OK if temperature read successfully;
 *         aditof::Status::GENERIC_ERROR if sensor read fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500GetSensorTemperature(uint16_t &tmpValue) {
    assert(!m_isOffline);

    unsigned int usDelay = 0;
    if (m_cameraFps > 0) {
        usDelay =
            static_cast<unsigned int>((1 / (double)m_cameraFps) * 1000000);
    }

    aditof::Status status =
        m_adsd3500Ctrl->getSensorTemperature(tmpValue, usDelay);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Can not read sensor temperature";
        return aditof::Status::GENERIC_ERROR;
    }
    return status;
}
/**
 * @brief Reads the current laser temperature.
 *
 * Retrieves the temperature of the VCSEL laser module from the ADSD3500 sensor bridge.
 * Uses frame-rate-based timing delays to allow sensor settling.
 *
 * @param[out] tmpValue Temperature value (sensor-specific units).
 *
 * @return aditof::Status::OK if temperature read successfully;
 *         aditof::Status::GENERIC_ERROR if sensor read fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::adsd3500GetLaserTemperature(uint16_t &tmpValue) {
    assert(!m_isOffline);

    unsigned int usDelay = 0;
    if (m_cameraFps > 0) {
        usDelay =
            static_cast<unsigned int>((1 / (double)m_cameraFps) * 1000000);
    }

    aditof::Status status =
        m_adsd3500Ctrl->getLaserTemperature(tmpValue, usDelay);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Can not read laser temperature";
        return aditof::Status::GENERIC_ERROR;
    }
    return status;
}

/**
 * @brief Optionally updates a parameter in the depth parameters map.
 *
 * Helper function to conditionally update a single depth parameter in the
 * internal parameter map for the current mode.
 *
 * @param[in] update If false, function returns without action; if true, performs update.
 * @param[in] index Parameter name/key to update.
 * @param[in] value New value to store for the parameter.
 */
void CameraItof::UpdateDepthParamsMap(bool update, const char *index,
                                      std::string value) {

    if (update) {
        std::map<std::string, std::string> currentModeParams;
        if (m_config->getDepthParamsForMode(
                m_details.mode, currentModeParams) == aditof::Status::OK) {
            auto indexIt = currentModeParams.find(index);
            if (indexIt != currentModeParams.end()) {
                currentModeParams[index] = value;
                m_config->setDepthParamsForMode(m_details.mode,
                                                currentModeParams);
            }
        }
    }
}

/**
 * @brief Applies depth computation INI parameters to the ADSD3500.
 *
 * Delegates to DepthParameterMapper for table-driven parameter application.
 * Parameters are matched against known keys; missing parameters generate warnings.
 *
 * @param[in] iniKeyValPairs Map of parameter name-value pairs to apply.
 * @param[in] updateDepthMap If true, updates internal parameter cache;
 *                            if false, applies to hardware only.
 *
 * @return aditof::Status::OK (always, with per-parameter warnings on failures).
 *
 * @note This function asserts that the camera is not in offline mode.
 */
aditof::Status CameraItof::setDepthIniParams(
    const std::map<std::string, std::string> &iniKeyValPairs,
    bool updateDepthMap) {

    assert(!m_isOffline);

    return m_depthParamMapper->applyParameters(
        iniKeyValPairs, updateDepthMap,
        [this](bool update, const char *key, std::string value) {
            UpdateDepthParamsMap(update, key, value);
        });
}

/**
 * @brief Retrieves the imager type (sensor model) in use with ADSD3500.
 *
 * Returns the specific depth imager model paired with the ADSD3500 ISP.
 * Common imagers: ADSD3100, ADSD3030, ADTF3080, ADTF3066.
 *
 * @param[out] imagerType ImagerType enum value representing the sensor.
 *
 * @return aditof::Status::OK.
 */
aditof::Status CameraItof::getImagerType(aditof::ImagerType &imagerType) const {
    imagerType = m_imagerType;

    return aditof::Status::OK;
}

/**
 * @brief Enables or disables dynamic mode switching.
 *
 * Controls whether the ADSD3500 automatically switches between frame modes
 * according to a pre-configured sequence.
 *
 * @param[in] en True to enable dynamic mode switching; false to disable.
 *
 * @return aditof::Status::OK if mode set successfully;
 *         aditof::Status error codes if sensor write fails.
 *
 * @note This function asserts that the camera is not in offline mode.
 * @note Requires a switching sequence to be configured via adsds3500setDynamicModeSwitchingSequence().
 */
aditof::Status CameraItof::adsd3500setEnableDynamicModeSwitching(bool en) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_adsd3500Hardware->adsd3500_write_cmd(
        ADSD3500_REG_ENABLE_PHASE_INVALIDATION, en ? 0x0001 : 0x0000);

    return status;
}

/**
 * @brief Configures the dynamic mode switching sequence.
 *
 * Programs the ADSD3500 to automatically cycle through a sequence of frame modes
 * with specified repetition counts. Supports up to 8 mode entries.
 *
 * @param[in] sequence Vector of (mode, repeat_count) pairs defining the switching sequence.
 *                     For example: [(mode0, 10), (mode2, 5)] repeats mode0 10 times,
 *                     then mode2 5 times, then loops back to mode0.
 *
 * @return aditof::Status::OK if sequence configured successfully;
 *         aditof::Status error codes if sensor write fails at any step.
 *
 * @note This function asserts that the camera is not in offline mode.
 * @note Maximum 8 entries per sequence; excess entries are silently ignored with warning.
 */
aditof::Status CameraItof::adsds3500setDynamicModeSwitchingSequence(
    const std::vector<std::pair<uint8_t, uint8_t>> &sequence) {
    using namespace aditof;

    Status status = Status::OK;

    assert(!m_isOffline);

    uint32_t entireSequence = 0xFFFFFFFF;
    uint32_t entireRepCount = 0x00000000;
    uint8_t *bytePtrSq = reinterpret_cast<uint8_t *>(&entireSequence);
    uint8_t *bytePtrRc = reinterpret_cast<uint8_t *>(&entireRepCount);

    for (size_t i = 0; i < sequence.size(); ++i) {
        if (i < 8) {
            if (i % 2) {
                *bytePtrSq = (*bytePtrSq & 0x0F) | (sequence[i].first << 4);
                *bytePtrRc = (*bytePtrRc & 0x0F) | (sequence[i].second << 4);
            } else {
                *bytePtrSq = (*bytePtrSq & 0xF0) | (sequence[i].first << 0);
                *bytePtrRc = (*bytePtrRc & 0xF0) | (sequence[i].second << 0);
            }
            bytePtrSq += i % 2;
            bytePtrRc += i % 2;
        } else {
            LOG(WARNING) << "More than 8 entries have been provided. Ignoring "
                            "all entries starting from the 9th.";
            break;
        }
    }

    uint16_t *sequence0 = reinterpret_cast<uint16_t *>(&entireSequence);
    uint16_t *sequence1 = reinterpret_cast<uint16_t *>(&entireSequence) + 1;
    status = m_adsd3500Hardware->adsd3500_write_cmd(ADSD3500_REG_DMS_SEQUENCE_0,
                                                    *sequence0);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to set sequence 0 for the Dynamic Mode Switching";
        return status;
    }
    status = m_adsd3500Hardware->adsd3500_write_cmd(ADSD3500_REG_DMS_SEQUENCE_1,
                                                    *sequence1);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to set sequence 1 for the Dynamic Mode Switching";
        return status;
    }

    uint16_t *repCount0 = reinterpret_cast<uint16_t *>(&entireRepCount);
    uint16_t *repCount1 = reinterpret_cast<uint16_t *>(&entireRepCount) + 1;
    status = m_adsd3500Hardware->adsd3500_write_cmd(
        ADSD3500_REG_DMS_REPEAT_COUNT_0, *repCount0);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to set mode repeat count 0 for the Dynamic Mode "
                      "Switching";
        return status;
    }
    status = m_adsd3500Hardware->adsd3500_write_cmd(
        ADSD3500_REG_DMS_REPEAT_COUNT_1, *repCount1);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to set mode repeat count 0 for the Dynamic Mode "
                      "Switching";
        return status;
    }

    return Status::OK;
}
