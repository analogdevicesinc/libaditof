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
#include "utils_ini.h"

#include "aditof/utils.h"
#include "crc.h"
#include "tofi/algorithms.h"
#include "tofi/floatTolin.h"
#include "tofi/tofi_config.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <json.h>
#include <omp.h>
#include <sstream>

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

static const int skMetaDataBytesCount = 128;

CameraItof::CameraItof(
    std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
    const std::string &ubootVersion, const std::string &kernelVersion,
    const std::string &sdCardImageVersion, const std::string &netLinkTest)
    : m_depthSensor(depthSensor), m_devStarted(false), m_devStreaming(false),
      m_adsd3500Enabled(false), m_isOffline(false), m_xyzEnabled(true),
      m_xyzSetViaApi(false), m_cameraFps(0), m_fsyncMode(-1),
      m_mipiOutputSpeed(1), m_isdeskewEnabled(1), m_enableTempCompenstation(-1),
      m_enableMetaDatainAB(-1), m_enableEdgeConfidence(-1), m_modesVersion(0),
      m_xyzTable({nullptr, nullptr, nullptr}),
      m_imagerType(aditof::ImagerType::UNSET), m_dropFirstFrame(true),
      m_dropFrameOnce(true) {

    FloatToLinGenerateTable();
    memset(&m_xyzTable, 0, sizeof(m_xyzTable));
    m_details.mode = -1;
    m_details.uBootVersion = ubootVersion;
    m_details.kernelVersion = kernelVersion;
    m_details.sdCardImageVersion = sdCardImageVersion;
    m_netLinkTest = netLinkTest;
    m_isOffline = false;

    // Define some of the controls of this camera
    // For now there are none. To add one use: m_controls.emplace("your_control_name", "default_control_value");
    // And handle the control action in setControl()

    // Check Depth Sensor
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

CameraItof::~CameraItof() { cleanupXYZtables(); }

aditof::Status CameraItof::initialize(const std::string &configFilepath) {
    using namespace aditof;
    Status status = Status::OK;

    if (m_isOffline) {

        LOG(INFO) << "Initializing camera: Offline";

        return status;

    } else {

        LOG(INFO) << "Initializing camera: Online";
        if (!m_adsd3500Enabled && !m_isOffline) {
            LOG(ERROR) << "This usecase is no longer supported.";
            return aditof::Status::UNAVAILABLE;
        }

        m_initConfigFilePath = configFilepath;

        // Setting up the UVC filters, samplegrabber interface, Video renderer and filters
        // Setting UVC mediaformat and Running the stream is done once mode is set
        if (!m_devStarted) {
            status = m_depthSensor->open();
            if (status != Status::OK) {
                LOG(WARNING) << "Failed to open device";
                return status;
            }
            m_devStarted = true;
        }

        if (!m_netLinkTest.empty()) {
            m_depthSensor->setControl("netlinktest", "1");
        }

        // get imager type that is used toghether with ADSD3500
        std::string controlValue;
        status = m_depthSensor->getControl("imagerType", controlValue);
        if (status == Status::OK) {
            if (controlValue == ControlValue.at(ImagerType::ADSD3100)) {
                m_imagerType = ImagerType::ADSD3100;
            } else if (controlValue == ControlValue.at(ImagerType::ADSD3030)) {
                m_imagerType = ImagerType::ADSD3030;
            } else if (controlValue == ControlValue.at(ImagerType::ADTF3080)) {
                m_imagerType = ImagerType::ADTF3080;
            } else {
                m_imagerType = ImagerType::UNSET;
                LOG(ERROR) << "Unkown imager type: " << controlValue;
                return Status::UNAVAILABLE;
            }

            status = m_depthSensor->getAvailableModes(m_availableModes);
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to get available frame types name!";
                return status;
            }

            for (auto availablemodes : m_availableModes) {
                DepthSensorModeDetails modeDetails;
                status =
                    m_depthSensor->getModeDetails(availablemodes, modeDetails);
                if (status != Status::OK) {
                    LOG(ERROR)
                        << "Failed to get available frame types details!";
                    return status;
                }
                m_availableSensorModeDetails.emplace_back(modeDetails);

                uint8_t intrinsics[56] = {0};
                uint8_t dealiasParams[32] = {0};
                TofiXYZDealiasData dealiasStruct;
                //the first element of readback_data for adsd3500_read_payload is used for the custom command
                //it will be overwritten by the returned data
                uint8_t mode = modeDetails.modeNumber;

                intrinsics[0] = mode;
                dealiasParams[0] = mode;
                //hardcoded function values to return intrinsics
                status = m_depthSensor->adsd3500_read_payload_cmd(
                    0x01, intrinsics, 56);
                if (status != Status::OK) {
                    LOG(ERROR) << "Failed to read intrinsics for adsd3500!";
                    return status;
                }

                //hardcoded function values to return dealias parameters
                status = m_depthSensor->adsd3500_read_payload_cmd(
                    0x02, dealiasParams, 32);
                if (status != Status::OK) {
                    LOG(ERROR)
                        << "Failed to read dealias parameters for adsd3500!";
                    return status;
                }

                memcpy(&dealiasStruct, dealiasParams,
                       sizeof(TofiXYZDealiasData) - sizeof(CameraIntrinsics));
                memcpy(&dealiasStruct.camera_intrinsics, intrinsics,
                       sizeof(CameraIntrinsics));

                m_xyz_dealias_data[mode] = dealiasStruct;
                memcpy(&m_details.intrinsics, &dealiasStruct.camera_intrinsics,
                       sizeof(CameraIntrinsics));
            }

            std::string fwVersion;
            std::string fwHash;

            status = adsd3500GetFirmwareVersion(fwVersion, fwHash);

            if (status == Status::OK) {
                LOG(INFO) << "Current adsd3500 firmware version is: "
                          << m_adsd3500FwGitHash.first;
                LOG(INFO) << "Current adsd3500 firmware git hash is: "
                          << m_adsd3500FwGitHash.second;
            } else {
                return status;
            }
        }

        //Note: m_depth_params_map is created by retrieveDepthProcessParams
        aditof::Status paramsStatus = retrieveDepthProcessParams();
        if (paramsStatus != Status::OK) {
            LOG(ERROR) << "Failed to load process parameters!";
            return status;
        }

        if (m_fsyncMode >= 0) {
            status = adsd3500SetToggleMode(m_fsyncMode);
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to set fsyncMode.";
                return status;
            }
        } else {
            LOG(WARNING) << "fsyncMode is not being set by SDK.";
        }

        if (m_mipiOutputSpeed != 1) {
            status = adsd3500SetMIPIOutputSpeed(m_mipiOutputSpeed);
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to set mipiOutputSpeed.";
                return status;
            }
        } else {
            status = adsd3500SetMIPIOutputSpeed(m_mipiOutputSpeed);
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to set mipiOutputSpeed.";
                return status;
            }
            LOG(WARNING)
                << "mipiSpeed is not being set by SDK.Setting default 2.5Gbps";
        }

        if (!m_isdeskewEnabled) {
            status = adsd3500SetEnableDeskewAtStreamOn(m_isdeskewEnabled);
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to set Enable Deskew at stream on.";
                return status;
            }
        } else {
            status = adsd3500SetEnableDeskewAtStreamOn(m_isdeskewEnabled);
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to set Enable Deskew at stream on.";
                return status;
            }
            LOG(WARNING)
                << "deskew is not being set by SDK, Setting it by default.";
        }

        if (m_enableTempCompenstation >= 0) {
            status = adsd3500SetEnableTemperatureCompensation(
                m_enableTempCompenstation);
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to set enableTempCompenstation.";
                return status;
            }
        } else {
            LOG(WARNING) << "enableTempCompenstation is not being set by SDK.";
        }

        if (m_enableEdgeConfidence >= 0) {
            status = adsd3500SetEnableEdgeConfidence(m_enableEdgeConfidence);
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to set enableEdgeConfidence.";
                return status;
            }
        } else {
            LOG(WARNING) << "enableEdgeConfidence is not being set by SDK.";
        }

        std::string serialNumber;
        status = readSerialNumber(serialNumber);
        if (status == Status::OK) {
            LOG(INFO) << "Module serial number: " << serialNumber;
        } else if (status == Status::UNAVAILABLE) {
            LOG(INFO) << "Serial read is not supported in this firmware!";
        } else {
            LOG(ERROR) << "Failed to read serial number!";
            return status;
        }

        LOG(INFO) << "Camera initialized";

        return status;
    }
}

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

aditof::Status CameraItof::stop() {
    aditof::Status status = aditof::Status::OK;

    if (m_isOffline) {
        status = m_depthSensor->stopPlayback();
    }

    status = m_depthSensor->stop();
    if (status != aditof::Status::OK) {
        LOG(INFO) << "Failed to stop camera!";
    }

    m_devStreaming = false;

    return status;
}

aditof::Status
CameraItof::getDepthParamtersMap(uint16_t mode,
                                 std::map<std::string, std::string> &params) {
    using namespace aditof;

    if (m_depth_params_map.find(mode) != m_depth_params_map.end()) {
        params = m_depth_params_map[mode];
        return Status::OK;
    }

    return Status::INVALID_ARGUMENT;
}

aditof::Status CameraItof::setMode(const uint8_t &mode) {
    using namespace aditof;
    Status status = Status::OK;

    if (m_isOffline) {

        /*
        static struct {
            uint32_t numberOfFrames;
            const uint32_t formatVersion = 0x00000001;
            uint16_t frameRate;
            uint16_t enableMetaDatainAB;
            TofiXYZDealiasData dealias;
            aditof::CameraDetails details;
            aditof::DepthSensorModeDetails modeDetailsCache;
            uint32_t fdatadetailsCount;
            aditof::FrameDataDetails fDataDetails[MAX_FRAME_DATA_DETAILS_SAVE];
        } m_offline_parameters;
        */

        // TODO:
        // 0. Get the parameters
        // 1. Frame details.
        // 2. Frame dealias parameters

        // 0. Get the parameters
        memset((void *)&m_offline_parameters, 0, sizeof(m_offline_parameters));

        status = m_depthSensor->getHeader((uint8_t *)&m_offline_parameters,
                                          sizeof(m_offline_parameters));
        if (status != Status::OK) {
            LOG(WARNING) << "Failed to get frame from device";
            return status;
        }

        LOG(INFO) << "Number of Frames: "
                  << m_offline_parameters.numberOfFrames;
        LOG(INFO) << "Frame Rate: " << m_offline_parameters.frameRate;

        m_modeDetailsCache.modeNumber =
            m_offline_parameters.modeDetailsCache.modeNumber;
        m_modeDetailsCache.numberOfPhases =
            m_offline_parameters.modeDetailsCache.numberOfPhases;
        m_modeDetailsCache.pixelFormatIndex =
            m_offline_parameters.modeDetailsCache.pixelFormatIndex;
        m_modeDetailsCache.frameHeightInBytes =
            m_offline_parameters.modeDetailsCache.frameHeightInBytes;
        m_modeDetailsCache.frameWidthInBytes =
            m_offline_parameters.modeDetailsCache.frameWidthInBytes;
        m_modeDetailsCache.baseResolutionWidth =
            m_offline_parameters.modeDetailsCache.baseResolutionWidth;
        m_modeDetailsCache.baseResolutionHeight =
            m_offline_parameters.modeDetailsCache.baseResolutionHeight;
        m_modeDetailsCache.metadataSize =
            m_offline_parameters.modeDetailsCache.metadataSize;
        m_modeDetailsCache.isPCM = m_offline_parameters.modeDetailsCache.isPCM;
        m_modeDetailsCache.frameContent.clear();

        LOG(INFO) << "[PLAYBACK] Reading frameContent from file header...";
        for (uint32_t idx = 0; idx < m_offline_parameters.MAX_FRAME_CONTENT;
             idx++) {
            std::string frameContent =
                m_offline_parameters.modeDetailsCache.frameContent[idx];
            if (!frameContent.empty()) {
                LOG(INFO) << "[PLAYBACK] Found in header: " << frameContent;
                m_modeDetailsCache.frameContent.emplace_back(frameContent);
            }
        }
        LOG(INFO) << "[PLAYBACK] Total frame types loaded: "
                  << m_modeDetailsCache.frameContent.size();

        m_pcmFrame = m_modeDetailsCache.isPCM;

        // 1. Frame details
        m_details.mode = m_offline_parameters.details.mode;
        m_details.frameType.width = m_offline_parameters.details.width;
        m_details.frameType.height = m_offline_parameters.details.height;
        m_details.frameType.totalCaptures =
            m_offline_parameters.details.totalCaptures;
        m_details.frameType.dataDetails.clear();
        LOG(INFO) << "[PLAYBACK] Loading "
                  << m_offline_parameters.fdatadetailsCount
                  << " fDataDetails entries...";
        for (uint32_t idx = 0; idx < m_offline_parameters.fdatadetailsCount;
             idx++) {
            FrameDataDetails fDataDetails;
            fDataDetails.type = m_offline_parameters.fDataDetails[idx].type;
            LOG(INFO) << "[PLAYBACK] fDataDetails[" << idx
                      << "]: type=" << fDataDetails.type;
            fDataDetails.width = m_offline_parameters.fDataDetails[idx].width;
            fDataDetails.height = m_offline_parameters.fDataDetails[idx].height;
            fDataDetails.subelementSize =
                m_offline_parameters.fDataDetails[idx].subelementSize;
            fDataDetails.subelementsPerElement =
                m_offline_parameters.fDataDetails[idx].subelementsPerElement;
            fDataDetails.bytesCount =
                m_offline_parameters.fDataDetails[idx].bytesCount;
            m_details.frameType.dataDetails.emplace_back(fDataDetails);
        }

        // 2. Frame dealias parameters
        const int GEN_XYZ_ITERATIONS = 20;
        TofiXYZDealiasData *pDealias = &m_offline_parameters.dealias;
        cleanupXYZtables();
        int ret = Algorithms::GenerateXYZTables(
            &m_xyzTable.p_x_table, &m_xyzTable.p_y_table, &m_xyzTable.p_z_table,
            &(pDealias->camera_intrinsics), pDealias->n_sensor_rows,
            pDealias->n_sensor_cols, m_modeDetailsCache.baseResolutionHeight,
            m_modeDetailsCache.baseResolutionWidth, pDealias->n_offset_rows,
            pDealias->n_offset_cols, pDealias->row_bin_factor,
            pDealias->col_bin_factor, GEN_XYZ_ITERATIONS);
        if (ret != 0 || !m_xyzTable.p_x_table || !m_xyzTable.p_y_table ||
            !m_xyzTable.p_z_table) {
            LOG(ERROR) << "Failed to generate the XYZ tables";
            return Status::GENERIC_ERROR;
        }

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

        m_iniKeyValPairs = m_depth_params_map[mode];
        configureSensorModeDetails();
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

        m_pcmFrame = m_modeDetailsCache.isPCM;

        uint16_t chipCmd = 0xDA00;
        chipCmd += mode;
        status = m_depthSensor->adsd3500_write_cmd(chipCmd, 0x280F, 200000);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to switch mode in chip using host commands!";
            return status;
        }

        setDepthIniParams(m_iniKeyValPairs, false);
        configureSensorModeDetails();
        m_details.mode = mode;

        LOG(INFO) << "Using the following configuration parameters for mode "
                  << int(mode);
        for (auto param : m_iniKeyValPairs) {
            LOG(INFO) << param.first << " : " << param.second;
        }

        if (m_enableMetaDatainAB > 0) {
            if (!m_pcmFrame) {
                status = adsd3500SetEnableMetadatainAB(m_enableMetaDatainAB);
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
            status = adsd3500SetEnableMetadatainAB(m_enableMetaDatainAB);
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to set enableMetaDatainAB.";
                return status;
            }

            LOG(WARNING) << "Metadata in AB is disabled.";
        }

        // Store the frame details in camera details
        m_details.mode = mode;
        m_details.frameType.width = (*modeIt).baseResolutionWidth;
        m_details.frameType.height = (*modeIt).baseResolutionHeight;
        m_details.frameType.totalCaptures = 1;
        m_details.frameType.dataDetails.clear();
        for (const auto &item : (*modeIt).frameContent) {
            if (item == "xyz" && !m_xyzEnabled) {
                continue;
            }
            if (item == "raw") { // "raw" is not supported right now
                continue;
            }

            FrameDataDetails fDataDetails;
            fDataDetails.type = item;
            fDataDetails.width = m_modeDetailsCache.baseResolutionWidth;
            fDataDetails.height = m_modeDetailsCache.baseResolutionHeight;
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
        if (!m_pcmFrame) {
            size_t dataSize = 0;
            std::string s;

            auto iniParamsMode = m_depth_params_map.find(mode);
            // Create a string from the ini parameters
            for (auto &param : iniParamsMode->second) {
                s += param.first + "=" + param.second + "\n";
            }
            dataSize = s.size();
            //LOG(INFO) << s;

            aditof::Status localStatus;
            localStatus = m_depthSensor->initTargetDepthCompute(
                (uint8_t *)s.c_str(), dataSize, (uint8_t *)m_xyz_dealias_data,
                sizeof(TofiXYZDealiasData) * 10);
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
            uint8_t mode = m_modeDetailsCache.modeNumber;

            const int GEN_XYZ_ITERATIONS = 20;
            TofiXYZDealiasData *pDealias = &m_xyz_dealias_data[mode];

            cleanupXYZtables();
            int ret = Algorithms::GenerateXYZTables(
                &m_xyzTable.p_x_table, &m_xyzTable.p_y_table,
                &m_xyzTable.p_z_table, &(pDealias->camera_intrinsics),
                pDealias->n_sensor_rows, pDealias->n_sensor_cols,
                m_modeDetailsCache.baseResolutionHeight,
                m_modeDetailsCache.baseResolutionWidth, pDealias->n_offset_rows,
                pDealias->n_offset_cols, pDealias->row_bin_factor,
                pDealias->col_bin_factor, GEN_XYZ_ITERATIONS);
            if (ret != 0 || !m_xyzTable.p_x_table || !m_xyzTable.p_y_table ||
                !m_xyzTable.p_z_table) {
                LOG(ERROR) << "Failed to generate the XYZ tables";
                return Status::GENERIC_ERROR;
            }
        }

        // If a Dynamic Mode Switching sequences has been loaded from config file then configure ADSD3500
        if (m_configDmsSequence.size() > 0) {
            status = this->adsd3500setEnableDynamicModeSwitching(true);
            if (status != Status::OK) {
                LOG(WARNING) << "Could not enable 'Dynamic Mode Switching.";
                return status;
            }

            status = this->adsds3500setDynamicModeSwitchingSequence(
                m_configDmsSequence);
            m_configDmsSequence.clear();
            if (status != Status::OK) {
                LOG(WARNING) << "Could not set a sequence for the 'Dynamic "
                                "Mode Switching'.";
                return status;
            }
        }
    }

    return status;
}

aditof::Status
CameraItof::getFrameProcessParams(std::map<std::string, std::string> &params) {

    aditof::Status status = aditof::Status::OK;

    assert(!m_isOffline);

    //params = m_depth_params_map[m_details.mode]; Should really be this

    status = m_depthSensor->getDepthComputeParams(params);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "get ini parameters failed.";
    }

    return status;
}

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

aditof::Status CameraItof::setPlaybackFile(std::string &filePath) {
    aditof::Status status = aditof::Status::OK;

    if (!m_isOffline) {
        status = aditof::Status::GENERIC_ERROR; // Invalid call
    } else {

        status = m_depthSensor->setPlaybackFile(filePath);
    }

    return status;
}

aditof::Status CameraItof::startRecording(std::string &filePath) {

    using namespace aditof;

    aditof::Status status = aditof::Status::OK;

    if (m_isOffline) {
        status = aditof::Status::GENERIC_ERROR; // Invalid call
    } else {

        /*
        static struct {
            uint32_t numberOfFrames;
            const uint32_t formatVersion = 0x00000001;
            uint16_t frameRate;
            uint16_t enableMetaDatainAB;
            TofiXYZDealiasData dealias;
            aditof::CameraDetails details;
            aditof::DepthSensorModeDetails modeDetailsCache;
            uint32_t fdatadetailsCount;
            aditof::FrameDataDetails fDataDetails[MAX_FRAME_DATA_DETAILS_SAVE];
        } m_offline_parameters;
        */

        // m_offline_parameters.numberOfFrames
        m_offline_parameters.numberOfFrames = 0;

        // m_offline_parameters.dealias
        memcpy(&m_offline_parameters.dealias,
               &m_xyz_dealias_data[m_details.mode],
               sizeof(m_xyz_dealias_data[m_details.mode]));

        // m_offline_parameters.frameRate
        m_offline_parameters.frameRate = m_cameraFps;
        if (status != Status::OK) {
            m_offline_parameters.frameRate = 0; // Unknown framerate
        }

        // m_offline_parameters.enableMetaDatainAB
        m_offline_parameters.enableMetaDatainAB = m_enableMetaDatainAB;

        // modeDetailsCache
        m_offline_parameters.modeDetailsCache.modeNumber =
            m_modeDetailsCache.modeNumber;
        m_offline_parameters.modeDetailsCache.numberOfPhases =
            m_modeDetailsCache.numberOfPhases;
        m_offline_parameters.modeDetailsCache.pixelFormatIndex =
            m_modeDetailsCache.pixelFormatIndex;
        m_offline_parameters.modeDetailsCache.frameHeightInBytes =
            m_modeDetailsCache.frameHeightInBytes;
        m_offline_parameters.modeDetailsCache.frameWidthInBytes =
            m_modeDetailsCache.frameWidthInBytes;
        m_offline_parameters.modeDetailsCache.baseResolutionWidth =
            m_modeDetailsCache.baseResolutionWidth;
        m_offline_parameters.modeDetailsCache.baseResolutionHeight =
            m_modeDetailsCache.baseResolutionHeight;
        m_offline_parameters.modeDetailsCache.metadataSize =
            m_modeDetailsCache.metadataSize;
        m_offline_parameters.modeDetailsCache.isPCM = m_modeDetailsCache.isPCM;

        // Zero out frameContent array to avoid garbage data
        memset(m_offline_parameters.modeDetailsCache.frameContent, 0,
               sizeof(m_offline_parameters.modeDetailsCache.frameContent));

        uint32_t idx = 0;
        LOG(INFO) << "[RECORDING] Enable flags: depth=" << m_depthEnabled
                  << " ab=" << m_abEnabled << " conf=" << m_confEnabled
                  << " xyz=" << m_xyzEnabled;
        LOG(INFO) << "[RECORDING] Original frameContent has "
                  << m_modeDetailsCache.frameContent.size() << " items";
        for (std::string val : m_modeDetailsCache.frameContent) {
            if (val == "raw") {
                LOG(INFO) << "[RECORDING] Skipping raw (not supported)";
                continue; // raw not supported
            }

            LOG(INFO) << "[RECORDING] Adding to header: " << val;
            memcpy((char *)m_offline_parameters.modeDetailsCache
                       .frameContent[idx++],
                   val.c_str(), val.length());
        }
        LOG(INFO) << "[RECORDING] Total frame types in header: " << idx;

        // m_offline_parameters.details, m_offline_parameters.fdatadetailsCount & m_offline_parameters.fDataDetails
        m_offline_parameters.fdatadetailsCount = 0;

        uint32_t mode = m_details.mode;
        auto modeIt = std::find_if(m_availableSensorModeDetails.begin(),
                                   m_availableSensorModeDetails.end(),
                                   [&mode](const DepthSensorModeDetails &d) {
                                       return (d.modeNumber == mode);
                                   });

        if (modeIt == m_availableSensorModeDetails.end()) {
            LOG(WARNING) << "Mode: " << (int)mode << " not supported by camera";
            return Status::INVALID_ARGUMENT;
        }

        m_offline_parameters.details.mode = mode;
        m_offline_parameters.details.width = (*modeIt).baseResolutionWidth;
        m_offline_parameters.details.height = (*modeIt).baseResolutionHeight;
        m_offline_parameters.details.totalCaptures = 1;
        LOG(INFO) << "[RECORDING] Building fDataDetails from frameContent...";
        for (const auto &item : (*modeIt).frameContent) {

            if (m_offline_parameters.fdatadetailsCount >
                m_offline_parameters.MAX_FRAME_DATA_DETAILS_SAVE) {

                LOG(ERROR) << "Frame data details count exceeded the limit";
                break;
            }

            if (item == "raw") { // "raw" is not supported right now
                LOG(INFO) << "[RECORDING] Skipping raw from fDataDetails (not "
                             "supported)";
                continue;
            }

            LOG(INFO) << "[RECORDING] Adding to fDataDetails: " << item;
            uint32_t idx = m_offline_parameters.fdatadetailsCount;

            // Copy frame type name safely
            strncpy(m_offline_parameters.fDataDetails[idx].type, item.c_str(),
                    sizeof(m_offline_parameters.fDataDetails[idx].type) - 1);
            m_offline_parameters.fDataDetails[idx]
                .type[sizeof(m_offline_parameters.fDataDetails[idx].type) - 1] =
                '\0';

            m_offline_parameters.fDataDetails[idx].width =
                m_modeDetailsCache.baseResolutionWidth;
            m_offline_parameters.fDataDetails[idx].height =
                m_modeDetailsCache.baseResolutionHeight;
            m_offline_parameters.fDataDetails[idx].subelementSize =
                sizeof(uint16_t);
            m_offline_parameters.fDataDetails[idx].subelementsPerElement = 1;

            if (item == "xyz") {
                m_offline_parameters.fDataDetails[idx].subelementsPerElement =
                    3;
            } else if (item == "metadata") {
                m_offline_parameters.fDataDetails[idx].subelementSize = 1;
                m_offline_parameters.fDataDetails[idx].width = 128;
                m_offline_parameters.fDataDetails[idx].height = 1;
            } else if (item == "conf") {
                m_offline_parameters.fDataDetails[idx].subelementSize =
                    sizeof(float);
            }
            m_offline_parameters.fDataDetails[idx].bytesCount =
                m_offline_parameters.fDataDetails[idx].width *
                m_offline_parameters.fDataDetails[idx].height *
                m_offline_parameters.fDataDetails[idx].subelementSize *
                m_offline_parameters.fDataDetails[idx].subelementsPerElement;

            //m_details.frameType.dataDetails.emplace_back(fDataDetails); // FIXME: Why is this being done? (2025-05-02)

            m_offline_parameters.fdatadetailsCount++;
        }

        LOG(INFO) << "[RECORDING] Final fDataDetails count: "
                  << m_offline_parameters.fdatadetailsCount;
        LOG(INFO) << "[RECORDING] Writing header to file: " << filePath;

        // Write m_offline_parameters to the recording file.
        status = m_depthSensor->startRecording(filePath,
                                               (uint8_t *)&m_offline_parameters,
                                               sizeof(m_offline_parameters));
    }

    return status;
}

aditof::Status CameraItof::stopRecording() {
    aditof::Status status = aditof::Status::OK;

    if (m_isOffline) {
        status = aditof::Status::GENERIC_ERROR; // Invalid call
    } else {
        status = m_depthSensor->stopRecording();
    }

    return status;
}

aditof::Status CameraItof::adsd3500ResetIniParamsForMode(const uint16_t mode) {
    aditof::Status status = aditof::Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x40, mode);

    return status;
}

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
    Status status = Status::OK;

    if (frame == nullptr) {
        return Status::INVALID_ARGUMENT;
    }

    FrameDetails frameDetails;
    frame->getDetails(frameDetails);

    if (m_details.frameType != frameDetails) {
        frame->setDetails(m_details.frameType, m_confBitsPerPixel,
                          m_abBitsPerPixel);
    }

    uint16_t *frameDataLocation = nullptr;
    if (!m_pcmFrame) {
        status = frame->getData("frameData", &frameDataLocation);
    } else {
        status = frame->getData("ab", &frameDataLocation);
    }

    if (status != Status::OK) {
        LOG(ERROR) << "Failed to get frame data location, status: "
                   << (int)status;
        return status;
    }
    if (!frameDataLocation) {
        LOG(ERROR) << "Frame data location is null despite success status";
        return Status::GENERIC_ERROR;
    }

    if (m_dropFirstFrame && m_dropFrameOnce && !m_isOffline) {
        status = m_depthSensor->getFrame(frameDataLocation, index);
        if (status != Status::OK) {
            m_dropFrameOnce = true;
            LOG(INFO) << "Failed to drop first frame!";
            return status;
        }
        m_dropFrameOnce = false;
        LOG(INFO) << "Dropped first frame";
    }

    status = m_depthSensor->getFrame(frameDataLocation, index);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to get frame from device";
        return status;
    }

    // The incoming sensor frames are already processed. Need to just create XYZ data
    if (m_xyzEnabled && m_depthEnabled && frame->haveDataType("xyz")) {
        uint16_t *depthFrame = nullptr;
        uint16_t *xyzFrame = nullptr;

        Status getDepthStatus = frame->getData("depth", &depthFrame);
        Status getXYZStatus = frame->getData("xyz", &xyzFrame);

        if (getDepthStatus == Status::OK && getXYZStatus == Status::OK &&
            depthFrame != nullptr && xyzFrame != nullptr) {
            Algorithms::ComputeXYZ((const uint16_t *)depthFrame, &m_xyzTable,
                                   (int16_t *)xyzFrame,
                                   m_modeDetailsCache.baseResolutionHeight,
                                   m_modeDetailsCache.baseResolutionWidth);
        } else {
            LOG(WARNING) << "XYZ enabled but frame buffers not allocated. "
                         << "Depth status: " << (int)getDepthStatus
                         << ", XYZ status: " << (int)getXYZStatus;
        }
    }

    if (!m_depthEnabled && frame->haveDataType("depth")) {
        uint16_t *depthFrame;

        status = frame->getData("depth", &depthFrame);
        if (status != Status::OK || depthFrame == nullptr) {
            LOG(ERROR) << "Failed to get depth frame location";
            return status;
        }
        memset(depthFrame, 0,
               m_modeDetailsCache.baseResolutionHeight *
                   m_modeDetailsCache.baseResolutionWidth * sizeof(uint16_t));
    }

    if (!m_abEnabled && frame->haveDataType("ab")) {
        uint16_t *abFrame;

        status = frame->getData("ab", &abFrame);
        if (status != Status::OK || abFrame == nullptr) {
            LOG(ERROR) << "Failed to get ab frame location";
            return status;
        }
        memset(abFrame, 0,
               m_modeDetailsCache.baseResolutionHeight *
                   m_modeDetailsCache.baseResolutionWidth * sizeof(uint16_t));
    }

    Metadata metadata;

    if (m_enableMetaDatainAB && m_abEnabled) {
        uint16_t *abFrame;
        status = frame->getData("ab", &abFrame);
        if (status != Status::OK || abFrame == nullptr) {
            LOG(ERROR) << "Failed to get ab frame location";
            return status;
        }
        static_assert(sizeof(Metadata) <= skMetaDataBytesCount,
                      "Metadata struct exceeds AB frame header size");
        memcpy(reinterpret_cast<uint8_t *>(&metadata), abFrame,
               sizeof(metadata));
        memset(abFrame, 0, sizeof(metadata));
    } else {
        // If metadata from ADSD3500 is not available/disabled, generate one here
        memset(static_cast<void *>(&metadata), 0, sizeof(metadata));
        metadata.width = m_modeDetailsCache.baseResolutionWidth;
        metadata.height = m_modeDetailsCache.baseResolutionHeight;
        metadata.imagerMode = m_modeDetailsCache.modeNumber;
        metadata.bitsInDepth = m_depthBitsPerPixel;
        metadata.bitsInAb = m_abBitsPerPixel;
        metadata.bitsInConfidence = m_confBitsPerPixel;

        // For frame with PCM content we need to store ab bits
        if (m_pcmFrame) {
            metadata.bitsInAb = 16;
        }
    }

    metadata.xyzEnabled = m_xyzEnabled;

    uint16_t *metadataLocation;
    status = frame->getData("metadata", &metadataLocation);
    if (status != Status::OK || metadataLocation == nullptr) {
        LOG(ERROR) << "Failed to get metadata location";
        return status;
    }

    memcpy(reinterpret_cast<uint8_t *>(metadataLocation),
           reinterpret_cast<uint8_t *>(&metadata), sizeof(metadata));

    return Status::OK;
}

aditof::Status CameraItof::getDetails(aditof::CameraDetails &details) const {
    using namespace aditof;
    Status status = Status::OK;

    details = m_details;

    return status;
}

std::shared_ptr<aditof::DepthSensorInterface> CameraItof::getSensor() {
    return m_depthSensor;
}

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

aditof::Status CameraItof::setControl(const std::string &control,
                                      const std::string &value) {
    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    if (m_controls.count(control) > 0) {
        if (value == "call") {
            return m_noArgCallables.at(control)();
        } else {
            m_controls[control] = value;
        }
    } else {
        LOG(WARNING) << "Unsupported control";
        return Status::INVALID_ARGUMENT;
    }

    return status;
}

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
        if (!m_details.serialNumber.empty()) {
            serialNumber = m_details.serialNumber;
            return status;
        } else {
            LOG(INFO)
                << "No serial number stored in cache. Reading from memory.";
        }
    }

    uint8_t serial[32] = {0};

    status = m_depthSensor->adsd3500_read_payload_cmd(0x19, serial, 32);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to read serial number!";
        return status;
    }

    m_details.serialNumber = std::string(reinterpret_cast<char *>(serial), 32);
    serialNumber = m_details.serialNumber;

    return status;
}

aditof::Status CameraItof::saveModuleCCB(const std::string &filepath) {
    aditof::Status status =
        aditof::Status::GENERIC_ERROR; //Defining with error for ccb read

    assert(!m_isOffline);

    if (filepath.empty()) {
        LOG(ERROR) << "File path where CCB should be written is empty.";
        return aditof::Status::INVALID_ARGUMENT;
    }

    std::string ccbContent;
    for (int i = 0; (i < NR_READADSD3500CCB && status != aditof::Status::OK);
         i++) {
        LOG(INFO) << "readAdsd3500CCB read attempt nr :" << i;
        status = readAdsd3500CCB(ccbContent);
    }

    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to read CCB from adsd3500 module after "
                   << NR_READADSD3500CCB << " reads!";
        return aditof::Status::GENERIC_ERROR;
    }

    std::ofstream destination(filepath, std::ios::binary);
    destination << ccbContent;

    return aditof::Status::OK;
}

aditof::Status CameraItof::saveModuleCFG(const std::string &filepath) const {

    assert(!m_isOffline);

    if (filepath.empty()) {
        LOG(ERROR) << "File path where CFG should be written is empty.";
        return aditof::Status::INVALID_ARGUMENT;
    }

    LOG(ERROR) << "CFG files is unavailable";

    return aditof::Status::UNAVAILABLE;
}

aditof::Status CameraItof::enableXYZframe(bool enable) {
    m_xyzEnabled = enable;
    m_xyzSetViaApi = true;

    return aditof::Status::OK;
}

aditof::Status CameraItof::enableDepthCompute(bool enable) {
    return aditof::Status::UNAVAILABLE;
}

#pragma pack(push, 1)
typedef union {
    uint8_t cmd_header_byte[16];
    struct {
        uint8_t id8;                // 0xAD
        uint16_t chunk_size16;      // 256 is flash page size
        uint8_t cmd8;               // 0x04 is the CMD for fw upgrade
        uint32_t total_size_fw32;   // 4 bytes (total size of firmware)
        uint32_t header_checksum32; // 4 bytes header checksum
        uint32_t crc_of_fw32;       // 4 bytes CRC of the Firmware Binary
    };
} cmd_header_t;
#pragma pack(pop)

aditof::Status
CameraItof::adsd3500UpdateFirmware(const std::string &fwFilePath) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    m_fwUpdated = false;
    m_adsd3500Status = Adsd3500Status::OK;
    aditof::SensorInterruptCallback cb = [this](Adsd3500Status status) {
        m_adsd3500Status = status;
        m_fwUpdated = true;
    };
    status = m_depthSensor->adsd3500_register_interrupt_callback(cb);
    bool interruptsAvailable = (status == Status::OK);

    // Read Chip ID in STANDARD mode
    uint16_t chip_id;
    status = m_depthSensor->adsd3500_read_cmd(0x0112, &chip_id);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to read adsd3500 chip id!";
        return status;
    }

    LOG(INFO) << "The readback chip ID is: " << chip_id;

    // Switch to BURST mode.
    status = m_depthSensor->adsd3500_write_cmd(0x0019, 0x0000);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to switch to burst mode!";
        return status;
    }

    // Send FW content, each chunk is 256 bytes
    const int flashPageSize = 256;

    // Read the firmware binary file
    std::ifstream fw_file(fwFilePath, std::ios::binary);
    // copy all data into buffer
    std::vector<uint8_t> buffer(std::istreambuf_iterator<char>(fw_file), {});

    uint32_t fw_len = buffer.size();
    uint8_t *fw_content = buffer.data();
    cmd_header_t fw_upgrade_header;
    fw_upgrade_header.id8 = 0xAD;
    fw_upgrade_header.chunk_size16 = 0x0100; // 256=0x100
    fw_upgrade_header.cmd8 = 0x04;           // FW Upgrade CMD = 0x04
    fw_upgrade_header.total_size_fw32 = fw_len;
    fw_upgrade_header.header_checksum32 = 0;

    for (int i = 1; i < 8; i++) {
        fw_upgrade_header.header_checksum32 +=
            fw_upgrade_header.cmd_header_byte[i];
    }

    uint32_t res = crcFast(fw_content, fw_len, true) ^ 0xFFFFFFFF;
    fw_upgrade_header.crc_of_fw32 = ~res;

    status = m_depthSensor->adsd3500_write_payload(
        fw_upgrade_header.cmd_header_byte, 16);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to send fw upgrade header";
        return status;
    }

    int packetsToSend;
    if ((fw_len % flashPageSize) != 0) {
        packetsToSend = (fw_len / flashPageSize + 1);
    } else {
        packetsToSend = (fw_len / flashPageSize);
    }

    uint8_t data_out[flashPageSize];

    for (int i = 0; i < packetsToSend; i++) {
        int start = flashPageSize * i;
        int end = flashPageSize * (i + 1);

        for (int j = start; j < end; j++) {
            if (j < static_cast<int>(fw_len)) {
                data_out[j - start] = fw_content[j];
            } else {
                // padding with 0x00
                data_out[j - start] = 0x00;
            }
        }
        status = m_depthSensor->adsd3500_write_payload(data_out, flashPageSize);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to send packet number " << i << " out of "
                       << packetsToSend << " packets!";
            return status;
        }

        if (i % 25 == 0) {
            LOG(INFO) << "Succesfully sent " << i << " out of " << packetsToSend
                      << " packets";
        }
    }

    //Commands to switch back to standard mode
    uint8_t switchBuf[] = {0xAD, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,
                           0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    status = m_depthSensor->adsd3500_write_payload(
        switchBuf, sizeof(switchBuf) / sizeof(switchBuf[0]));
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to switch adsd3500 to standard mode!";
        return status;
    }

    if (interruptsAvailable) {
        LOG(INFO) << "Waiting for ADSD3500 to update itself";
        int secondsTimeout = 60;
        int secondsWaited = 0;
        int secondsWaitingStep = 1;
        while (!m_fwUpdated && secondsWaited < secondsTimeout) {
            LOG(INFO) << ".";
            std::this_thread::sleep_for(
                std::chrono::seconds(secondsWaitingStep));
            secondsWaited += secondsWaitingStep;
        }
        LOG(INFO) << "Waited: " << secondsWaited << " seconds";
        m_depthSensor->adsd3500_unregister_interrupt_callback(cb);
        if (!m_fwUpdated && secondsWaited >= secondsTimeout) {
            LOG(WARNING) << "Adsd3500 firmware updated has timeout after: "
                         << secondsWaited << "seconds";
            return aditof::Status::GENERIC_ERROR;
        }

        if (m_adsd3500Status == Adsd3500Status::OK ||
            m_adsd3500Status == Adsd3500Status::FIRMWARE_UPDATE_COMPLETE) {
            LOG(INFO) << "Adsd3500 firmware updated succesfully!";
        } else {
            LOG(ERROR) << "Adsd3500 firmware updated but with error: "
                       << (int)m_adsd3500Status;
        }
    } else {
        LOG(INFO) << "Adsd3500 firmware updated succesfully! Waiting 60 "
                     "seconds since interrupts support was not detected.";
        std::this_thread::sleep_for(std::chrono::seconds(60));
    }

    return aditof::Status::OK;
}

aditof::Status CameraItof::readAdsd3500CCB(std::string &ccb) {
    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    uint8_t ccbHeader[16] = {0};
    ccbHeader[0] = 1;

    //For this case adsd3500 will remain in burst mode
    //A manuall switch to standard mode will be required at the end of the function
    status = m_depthSensor->adsd3500_read_payload_cmd(0x13, ccbHeader, 16);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to get ccb command header";
        return status;
    }

    uint16_t chunkSize;
    uint32_t ccbFileSize;
    uint32_t crcOfCCB;

    memcpy(&chunkSize, ccbHeader + 1, 2);
    memcpy(&ccbFileSize, ccbHeader + 4, 4);
    memcpy(&crcOfCCB, ccbHeader + 12, 4);

    // Validate header values to prevent division by zero and unbounded allocation
    const uint32_t MAX_CCB_FILE_SIZE = 10 * 1024 * 1024; // 10 MB safety limit
    if (chunkSize == 0 || ccbFileSize == 0 || ccbFileSize > MAX_CCB_FILE_SIZE) {
        LOG(ERROR) << "Invalid CCB header: chunkSize=" << chunkSize
                   << " fileSize=" << ccbFileSize;
        return Status::GENERIC_ERROR;
    }

    uint16_t numOfChunks = ccbFileSize / chunkSize;
    std::vector<uint8_t> ccbContent(ccbFileSize); // RAII: automatic cleanup

    for (int i = 0; i < numOfChunks; i++) {
        status = m_depthSensor->adsd3500_read_payload(
            ccbContent.data() + i * chunkSize, chunkSize);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to read chunk number " << i << " out of "
                       << numOfChunks + 1 << " chunks for adsd3500!";
            return status;
        }

        if (i % 20 == 0) {
            LOG(INFO) << "Succesfully read chunk number " << i << " out of "
                      << numOfChunks + 1 << " chunks for adsd3500!";
        }
    }

    //read last chunk. smaller size than the rest
    if (ccbFileSize % chunkSize != 0) {
        status = m_depthSensor->adsd3500_read_payload(
            ccbContent.data() + numOfChunks * chunkSize,
            ccbFileSize % chunkSize);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to read chunk number " << numOfChunks + 1
                       << " out of " << numOfChunks + 1
                       << " chunks for adsd3500!";
            return status;
        }
    }

    //Commands to switch back to standard mode
    uint8_t switchBuf[] = {0xAD, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,
                           0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    status = m_depthSensor->adsd3500_write_payload(
        switchBuf, sizeof(switchBuf) / sizeof(switchBuf[0]));
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to switch adsd3500 to standard mode!";
        return status;
    }

    LOG(INFO) << "Succesfully read ccb from adsd3500. Checking crc...";

    uint32_t computedCrc =
        crcFast(ccbContent.data(), ccbFileSize - 4, true) ^ 0xFFFFFFFF;

    if (crcOfCCB != ~computedCrc) {
        LOG(ERROR) << "Invalid crc for ccb read from memory!";
        return Status::GENERIC_ERROR;
    } else {
        LOG(INFO) << "Crc of ccb is valid.";
    }

    //remove the trailling 4 bytes containing the crc
    ccb = std::string(reinterpret_cast<char *>(ccbContent.data()),
                      ccbFileSize - 4);

    return status;
}

void CameraItof::configureSensorModeDetails() {

    if (m_isOffline) {
        // Set enable flags based on what was actually recorded
        // Check m_modeDetailsCache.frameContent to see what's available
        LOG(INFO) << "[PLAYBACK] Setting enable flags based on frameContent...";
        m_depthEnabled = false;
        m_abEnabled = false;
        m_confEnabled = false;
        m_xyzEnabled = false;

        for (const auto &content : m_modeDetailsCache.frameContent) {
            if (content == "depth")
                m_depthEnabled = true;
            else if (content == "ab")
                m_abEnabled = true;
            else if (content == "conf")
                m_confEnabled = true;
            else if (content == "xyz")
                m_xyzEnabled = true;
        }

        LOG(INFO) << "[PLAYBACK] Enable flags set: depth=" << m_depthEnabled
                  << " ab=" << m_abEnabled << " conf=" << m_confEnabled
                  << " xyz=" << m_xyzEnabled;

    } else {
        std::string value;

        m_depthEnabled = true;
        m_abEnabled = true;
        m_confEnabled = true;
        //m_xyzEnabled = false;
        //m_xyzSetViaApi = true;

        auto it = m_iniKeyValPairs.find("bitsInPhaseOrDepth");
        if (it != m_iniKeyValPairs.end()) {
            value = it->second;
            m_depthBitsPerPixel = std::stoi(value);
            if (value == "16")
                value = "6";
            else if (value == "14")
                value = "5";
            else if (value == "12")
                value = "4";
            else if (value == "10")
                value = "3";
            else if (value == "8")
                value = "2";
            else {
                value = "0";
                m_depthEnabled = false;
            }
            m_depthSensor->setControl("phaseDepthBits", value);
        } else {
            LOG(WARNING)
                << "bitsInPhaseOrDepth was not found in parameter list";
        }

        it = m_iniKeyValPairs.find("bitsInConf");
        if (it != m_iniKeyValPairs.end()) {
            value = it->second;
            m_confBitsPerPixel = std::stoi(value);
            if (value == "8")
                value = "2";
            else if (value == "4")
                value = "1";
            else {
                value = "0";
                m_confEnabled = false;
            }
            m_depthSensor->setControl("confidenceBits", value);
        } else {
            LOG(WARNING) << "bitsInConf was not found in parameter list";
        }

        it = m_iniKeyValPairs.find("bitsInAB");
        if (it != m_iniKeyValPairs.end()) {
            value = it->second;
            m_abEnabled = true;
            m_abBitsPerPixel = std::stoi(value);
            if (value == "16")
                value = "6";
            else if (value == "14")
                value = "5";
            else if (value == "12")
                value = "4";
            else if (value == "10")
                value = "3";
            else if (value == "8")
                value = "2";
            else {
                value = "0";
                m_abEnabled = false;
            }
            m_depthSensor->setControl("abBits", value);
        } else {
            LOG(WARNING) << "bitsInAB was not found in parameter list";
        }

        it = m_iniKeyValPairs.find("partialDepthEnable");
        if (it != m_iniKeyValPairs.end()) {
            std::string en = (it->second == "0") ? "1" : "0";
            m_depthSensor->setControl("depthEnable", en);
            m_depthSensor->setControl("abAveraging", en);
        } else {
            LOG(WARNING)
                << "partialDepthEnable was not found in parameter list";
        }

        it = m_iniKeyValPairs.find("inputFormat");
        if (it != m_iniKeyValPairs.end()) {
            value = it->second;
            m_depthSensor->setControl("inputFormat", value);
        } else {
            LOG(WARNING) << "inputFormat was not found in parameter list";
        }

        // XYZ set through camera control takes precedence over the setting from parameter list
        if (!m_xyzSetViaApi) {
            it = m_iniKeyValPairs.find("xyzEnable");
            if (it != m_iniKeyValPairs.end()) {
                m_xyzEnabled = !(it->second == "0");
            } else {
                LOG(WARNING) << "xyzEnable was not found in parameter list";
            }
        }

        //Embedded header is being set from theparameter list
        it = m_iniKeyValPairs.find("headerSize");
        if (it != m_iniKeyValPairs.end()) {
            value = it->second;
            if (std::stoi(value) == 128 && m_abEnabled) {
                m_enableMetaDatainAB = 1;
            } else {
                m_enableMetaDatainAB = 0;
            }
        } else {
            LOG(WARNING) << "headerSize was not found in parameter list";
        }

        // Rebuild frameContent based on enabled frame types
        // This ensures recording/playback respects config overrides
        // Only rebuild if we have INI params (not during early initialization)
        if (!m_iniKeyValPairs.empty()) {
            m_modeDetailsCache.frameContent.clear();
            if (m_depthEnabled) {
                m_modeDetailsCache.frameContent.emplace_back("depth");
            }
            if (m_abEnabled) {
                m_modeDetailsCache.frameContent.emplace_back("ab");
            }
            if (m_confEnabled) {
                m_modeDetailsCache.frameContent.emplace_back("conf");
            }
            if (m_xyzEnabled) {
                m_modeDetailsCache.frameContent.emplace_back("xyz");
            }
            // Always include metadata last (matches sensor frameContent order)
            m_modeDetailsCache.frameContent.emplace_back("metadata");
        }
    }
}

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
            m_depth_params_map.emplace(mode, paramsMap);
        }
        if (m_depth_params_map_reset.empty()) {
            m_depth_params_map_reset = m_depth_params_map;
        }
    } else {
        loadDepthParamsFromJsonFile(m_initConfigFilePath);
    }
    return status;
}

aditof::Status CameraItof::resetDepthProcessParams() {
    if (m_depth_params_map_reset.empty()) {
        LOG(WARNING) << "No reset parameters available. "
                        "Please load depth parameters from file first.";
        return aditof::Status::GENERIC_ERROR;
    }

    m_depth_params_map = m_depth_params_map_reset;

    return aditof::Status::OK;
}

aditof::Status
CameraItof::saveDepthParamsToJsonFile(const std::string &savePathFile) {
    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    json_object *rootjson = json_object_new_object();

    json_object_object_add(rootjson, "errata1",
                           json_object_new_int(m_dropFirstFrame ? 1 : 0));

    json_object_object_add(rootjson, "fsyncMode",
                           json_object_new_int(m_fsyncMode));
    json_object_object_add(rootjson, "mipiOutputSpeed",
                           json_object_new_int(m_mipiOutputSpeed));
    json_object_object_add(rootjson, "isdeskewEnabled",
                           json_object_new_int(m_isdeskewEnabled));
    json_object_object_add(rootjson, "enableTempCompensation",
                           json_object_new_int(m_enableTempCompenstation));
    json_object_object_add(rootjson, "enableEdgeConfidence",
                           json_object_new_int(m_enableEdgeConfidence));

    std::list<std::string> depth_compute_keys_list = {
        "abThreshMin",         "radialThreshMax",
        "radialThreshMin",     "depthComputeIspEnable",
        "partialDepthEnable",  "interleavingEnable",
        "bitsInPhaseOrDepth",  "bitsInAB",
        "bitsInConf",          "confThresh",
        "phaseInvalid",        "inputFormat",
        "jblfABThreshold",     "jblfApplyFlag",
        "jblfExponentialTerm", "jblfGaussianSigma",
        "jblfMaxEdge",         "jblfWindowSize"};

    for (auto pfile = m_depth_params_map.begin();
         pfile != m_depth_params_map.end(); pfile++) {

        std::map<std::string, std::string> iniKeyValPairs = pfile->second;

        if (status == Status::OK) {
            json_object *json = json_object_new_object();
            json_object *dept_compute_group_keys = json_object_new_object();
            json_object *configuration_param_keys = json_object_new_object();

            for (auto item = iniKeyValPairs.begin();
                 item != iniKeyValPairs.end(); item++) {
                double valued = strtod(item->second.c_str(), NULL);

                auto it = std::find_if(
                    std::begin(depth_compute_keys_list),
                    std::end(depth_compute_keys_list),
                    [&](const std::string key) { return item->first == key; });
                if (depth_compute_keys_list.end() != it) {
                    if (isConvertibleToDouble(item->second)) {
                        json_object_object_add(dept_compute_group_keys,
                                               item->first.c_str(),
                                               json_object_new_double(valued));
                    } else {
                        json_object_object_add(
                            dept_compute_group_keys, item->first.c_str(),
                            json_object_new_string(item->second.c_str()));
                    }
                } else {
                    if (isConvertibleToDouble(item->second)) {
                        json_object_object_add(configuration_param_keys,
                                               item->first.c_str(),
                                               json_object_new_double(valued));
                    } else {
                        json_object_object_add(
                            configuration_param_keys, item->first.c_str(),
                            json_object_new_string(item->second.c_str()));
                    }
                }
            }
            json_object_object_add(json, "depth-compute",
                                   dept_compute_group_keys);
            json_object_object_add(json, "configuration-parameters",
                                   configuration_param_keys);
            json_object_object_add(rootjson,
                                   std::to_string(pfile->first).c_str(), json);
        }
    }

    const char *json_str =
        json_object_to_json_string_ext(rootjson, JSON_C_TO_STRING_PRETTY);

    FILE *fp = fopen(savePathFile.c_str(), "w");
    if (fp == NULL) {
        LOG(WARNING) << " Unable to open the file. " << savePathFile.c_str();
        json_object_put(rootjson);
        return Status::GENERIC_ERROR;
    }
    fputs(json_str, fp);
    fclose(fp);

    json_object_put(rootjson);

    return status;
}

aditof::Status
CameraItof::loadDepthParamsFromJsonFile(const std::string &pathFile,
                                        const int16_t mode_in_use) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    m_depth_params_map.clear();

    // Parse json
    std::ifstream ifs(pathFile.c_str());
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));

    json_object *config_json = json_tokener_parse(content.c_str());
    if (config_json != NULL) {

        json_object *errata1 = NULL;
        double errata1val = 1;
        if (json_object_object_get_ex(config_json, "errata1", &errata1)) {
            if (json_object_is_type(errata1, json_type_int) ||
                json_object_is_type(errata1, json_type_double)) {
                errata1val = json_object_get_double(errata1);
            }
        }
        if (errata1val == 1) {
            m_dropFirstFrame = true;
        } else {
            m_dropFirstFrame = false;
        }

        json_object *fsyncMode = NULL;
        if (json_object_object_get_ex(config_json, "fsyncMode", &fsyncMode)) {
            if (json_object_is_type(fsyncMode, json_type_int) ||
                json_object_is_type(fsyncMode, json_type_double)) {
                m_fsyncMode = json_object_get_int(fsyncMode);
            }
        }

        json_object *mipiOutputSpeed = NULL;
        if (json_object_object_get_ex(config_json, "mipiOutputSpeed",
                                      &mipiOutputSpeed)) {
            if (json_object_is_type(mipiOutputSpeed, json_type_int) ||
                json_object_is_type(mipiOutputSpeed, json_type_double)) {
                m_mipiOutputSpeed = json_object_get_int(mipiOutputSpeed);
            }
        }

        json_object *isdeskewEnabled = NULL;
        if (json_object_object_get_ex(config_json, "isdeskewEnabled",
                                      &isdeskewEnabled)) {
            if (json_object_is_type(isdeskewEnabled, json_type_int) ||
                json_object_is_type(isdeskewEnabled, json_type_double)) {
                m_isdeskewEnabled = json_object_get_int(isdeskewEnabled);
            }
        }

        json_object *enableTempCompensation = NULL;
        if (json_object_object_get_ex(config_json, "enableTempCompensation",
                                      &enableTempCompensation)) {
            if (json_object_is_type(enableTempCompensation, json_type_int) ||
                json_object_is_type(enableTempCompensation, json_type_double)) {
                m_enableTempCompenstation =
                    json_object_get_int(enableTempCompensation);
            }
        }

        json_object *enableEdgeConfidence = NULL;
        if (json_object_object_get_ex(config_json, "enableEdgeConfidence",
                                      &enableEdgeConfidence)) {
            if (json_object_is_type(enableEdgeConfidence, json_type_int) ||
                json_object_is_type(enableEdgeConfidence, json_type_double)) {
                m_enableEdgeConfidence =
                    json_object_get_int(enableEdgeConfidence);
            }
        }

        json_object *dmsSequence = NULL;
        if (json_object_object_get_ex(config_json, "dynamicModeSwitching",
                                      &dmsSequence)) {
            if (json_object_is_type(dmsSequence, json_type_array)) {

                m_configDmsSequence.clear();

                int arraylen = json_object_array_length(dmsSequence);
                for (int i = 0; i < arraylen; i++) {
                    json_object *dmsPair =
                        json_object_array_get_idx(dmsSequence, i);
                    json_object *dmsMode = NULL;
                    json_object *dmsRepeat = NULL;

                    if (json_object_object_get_ex(dmsPair, "mode", &dmsMode) &&
                        json_object_object_get_ex(dmsPair, "repeat",
                                                  &dmsRepeat)) {
                        if ((json_object_is_type(dmsMode, json_type_int) ||
                             json_object_is_type(dmsMode, json_type_double)) &&
                            (json_object_is_type(dmsRepeat, json_type_int) ||
                             json_object_is_type(dmsRepeat,
                                                 json_type_double))) {
                            m_configDmsSequence.emplace_back(
                                std::make_pair(json_object_get_int(dmsMode),
                                               json_object_get_int(dmsRepeat)));
                        }
                    }
                }
            }
        }

        for (const auto &mode : m_availableModes) {

            if (mode_in_use >= 0 && mode != mode_in_use) {
                continue;
            }

            std::string modeStr = std::to_string(mode);

            json_object *depthframeType = NULL;
            if (!json_object_object_get_ex(config_json, modeStr.c_str(),
                                           &depthframeType)) {
                continue;
            }

            json_object *dept_compute_group_keys = NULL;
            json_object_object_get_ex(depthframeType, "depth-compute",
                                      &dept_compute_group_keys);

            std::map<std::string, std::string> iniKeyValPairs;

            if (dept_compute_group_keys) {

                json_object_object_foreach(dept_compute_group_keys, key, val) {

                    std::string value = "";

                    if (json_object_is_type(val, json_type_string)) {
                        value = std::string(json_object_get_string(val));
                    } else {
                        std::ostringstream stream;
                        stream << std::fixed << std::setprecision(1)
                               << json_object_get_double(val);
                        value = stream.str();
                        std::size_t found = value.find(".0");
                        if (found != std::string::npos) {
                            value = std::to_string(json_object_get_int(val));
                        }
                    }
                    iniKeyValPairs.emplace(std::string(key), value);
                }
            }

            json_object *configuration_param_keys = NULL;
            json_object_object_get_ex(depthframeType,
                                      "configuration-parameters",
                                      &configuration_param_keys);

            if (configuration_param_keys) {
                json_object_object_foreach(configuration_param_keys, key, val) {

                    std::string value = "";

                    if (json_object_is_type(val, json_type_string)) {
                        value = std::string(json_object_get_string(val));
                    } else {
                        std::ostringstream stream;
                        stream << std::fixed << std::setprecision(1)
                               << json_object_get_double(val);
                        value = stream.str();
                        std::size_t found = value.find(".0");
                        if (found != std::string::npos) {
                            value = std::to_string(json_object_get_int(val));
                        }
                    }
                    iniKeyValPairs.emplace(std::string(key), value);
                }
            }
            m_depth_params_map.emplace(mode, iniKeyValPairs);
        }
        json_object_put(config_json);
    }

    return status;
}

bool CameraItof::isConvertibleToDouble(const std::string &str) {
    bool result = false;
    try {
        std::stod(str);
        result = true;
    } catch (...) {
    }
    return result;
}

void CameraItof::dropFirstFrame(bool dropFrame) {
    assert(!m_isOffline);
    m_dropFirstFrame = dropFrame;
}

aditof::Status
CameraItof::setSensorConfiguration(const std::string &sensorConf) {
    aditof::Status status = aditof::Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->setSensorConfiguration(sensorConf);

    return status;
}

aditof::Status CameraItof::adsd3500SetToggleMode(int mode) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    /*mode = 2, adsd3500 fsync does not automatically toggle - Pin set as input (Slave)*/
    /*mode = 1, adsd3500 fsync automatically toggles at user specified framerate*/
    /*mode = 0, adsd3500 fsync does not automatically toggle*/

    status = m_depthSensor->adsd3500_write_cmd(0x0025, mode);
    if (status != Status::OK) {
        LOG(ERROR) << "Unable to set FSYNC Toggle mode!";
        return status;
    }

    if (mode == 2) {
        m_adsd3500_master = false;
    }

    return status;
}

aditof::Status CameraItof::adsd3500ToggleFsync() {
    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    if (!m_adsd3500_master) {
        LOG(ERROR) << "ADSD3500 not set as master - cannot toggle FSYNC";
    } else {
        // Toggle Fsync
        status = m_depthSensor->adsd3500_write_cmd(0x0026, 0x0000);
        if (status != Status::OK) {
            LOG(ERROR) << "Unable to Toggle FSYNC!";
            return status;
        }
    }

    return status;
}

aditof::Status CameraItof::adsd3500GetFirmwareVersion(std::string &fwVersion,
                                                      std::string &fwHash) {
    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    uint8_t fwData[44] = {0};
    fwData[0] = uint8_t(1);

    status = m_depthSensor->adsd3500_read_payload_cmd(0x05, fwData, 44);
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

aditof::Status CameraItof::adsd3500SetABinvalidationThreshold(int threshold) {
    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x0010, threshold);

    return status;
}

aditof::Status CameraItof::adsd3500GetABinvalidationThreshold(int &threshold) {
    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    threshold = 0;
    status = m_depthSensor->adsd3500_read_cmd(
        0x0015, reinterpret_cast<uint16_t *>(&threshold));

    return status;
}

aditof::Status CameraItof::adsd3500SetConfidenceThreshold(int threshold) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x0011, threshold);

    return status;
}
aditof::Status CameraItof::adsd3500GetConfidenceThreshold(int &threshold) {

    using namespace aditof;
    Status status = Status::OK;

    if (m_isOffline) {

    } else {
        threshold = 0;
        status = m_depthSensor->adsd3500_read_cmd(
            0x0016, reinterpret_cast<uint16_t *>(&threshold));
    }
    return status;
}

aditof::Status CameraItof::adsd3500SetJBLFfilterEnableState(bool enable) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x0013, enable ? 1 : 0);

    return status;
}
aditof::Status CameraItof::adsd3500GetJBLFfilterEnableState(bool &enabled) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    int intEnabled = 0;
    status = m_depthSensor->adsd3500_read_cmd(
        0x0017, reinterpret_cast<uint16_t *>(&intEnabled));
    enabled = !!intEnabled;

    return status;
}

aditof::Status CameraItof::adsd3500SetJBLFfilterSize(int size) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x0014, size);

    return status;
}
aditof::Status CameraItof::adsd3500GetJBLFfilterSize(int &size) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    size = 0;
    status = m_depthSensor->adsd3500_read_cmd(
        0x0018, reinterpret_cast<uint16_t *>(&size));

    return status;
}

aditof::Status CameraItof::adsd3500SetRadialThresholdMin(int threshold) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x0027, threshold);

    return status;
}
aditof::Status CameraItof::adsd3500GetRadialThresholdMin(int &threshold) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    threshold = 0;
    status = m_depthSensor->adsd3500_read_cmd(
        0x0028, reinterpret_cast<uint16_t *>(&threshold));

    return status;
}

aditof::Status CameraItof::adsd3500SetRadialThresholdMax(int threshold) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x0029, threshold);

    return status;
}

aditof::Status CameraItof::adsd3500GetRadialThresholdMax(int &threshold) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    threshold = 0;
    status = m_depthSensor->adsd3500_read_cmd(
        0x0030, reinterpret_cast<uint16_t *>(&threshold));

    return status;
}

aditof::Status CameraItof::adsd3500SetMIPIOutputSpeed(uint16_t speed) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x0031, speed);

    return status;
}

aditof::Status CameraItof::adsd3500SetEnableDeskewAtStreamOn(uint16_t value) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x00AB, value);

    return status;
}

aditof::Status CameraItof::adsd3500GetMIPIOutputSpeed(uint16_t &speed) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_read_cmd(
        0x0034, reinterpret_cast<uint16_t *>(&speed));

    return status;
}

aditof::Status CameraItof::adsd3500GetImagerErrorCode(uint16_t &errcode) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_read_cmd(
        0x0038, reinterpret_cast<uint16_t *>(&errcode));

    return status;
}

aditof::Status CameraItof::adsd3500SetVCSELDelay(uint16_t delay) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x0066, delay);

    return status;
}

aditof::Status CameraItof::adsd3500GetVCSELDelay(uint16_t &delay) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_read_cmd(
        0x0068, reinterpret_cast<uint16_t *>(&delay));

    return status;
}

aditof::Status CameraItof::adsd3500SetJBLFMaxEdgeThreshold(uint16_t threshold) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x0074, threshold);

    return status;
}

aditof::Status CameraItof::adsd3500SetJBLFABThreshold(uint16_t threshold) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x0075, threshold);

    return status;
}

aditof::Status CameraItof::adsd3500SetJBLFGaussianSigma(uint16_t value) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x006B, value);

    return status;
}

aditof::Status CameraItof::adsd3500GetJBLFGaussianSigma(uint16_t &value) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    value = 0;
    status = m_depthSensor->adsd3500_read_cmd(
        0x0069, reinterpret_cast<uint16_t *>(&value));

    return status;
}

aditof::Status CameraItof::adsd3500SetJBLFExponentialTerm(uint16_t value) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x006C, value);

    return status;
}

aditof::Status CameraItof::adsd3500GetJBLFExponentialTerm(uint16_t &value) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_read_cmd(
        0x006A, reinterpret_cast<uint16_t *>(&value));

    return status;
}

aditof::Status CameraItof::adsd3500GetFrameRate(uint16_t &fps) {

    using namespace aditof;
    Status status = Status::OK;

    if (m_isOffline) {

        fps = m_offline_parameters.frameRate;

    } else {
        status = m_depthSensor->adsd3500_read_cmd(
            0x0023, reinterpret_cast<uint16_t *>(&fps));
    }
    return status;
}

aditof::Status CameraItof::adsd3500SetFrameRate(uint16_t fps) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    if (fps == 0) {
        fps = 10;
        LOG(WARNING) << "Using a default frame rate of " << fps;
    }

    status = m_depthSensor->setControl("fps", std::to_string(fps));
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to set fps at: " << fps << "!";
    } else {
        m_cameraFps = fps;
        LOG(INFO) << "Camera FPS set from parameter list at: " << m_cameraFps;
    }

    return status;
}

aditof::Status CameraItof::adsd3500SetEnableEdgeConfidence(uint16_t value) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x0062, value);

    return status;
}

aditof::Status
CameraItof::adsd3500GetTemperatureCompensationStatus(uint16_t &value) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_read_cmd(
        0x0076, reinterpret_cast<uint16_t *>(&value));

    return status;
}

aditof::Status CameraItof::adsd3500SetEnablePhaseInvalidation(uint16_t value) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x0072, value);

    return status;
}

aditof::Status
CameraItof::adsd3500SetEnableTemperatureCompensation(uint16_t value) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x0021, value);

    return status;
}

aditof::Status CameraItof::adsd3500SetEnableMetadatainAB(uint16_t value) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x0036, value);

    return status;
}

aditof::Status CameraItof::adsd3500GetEnableMetadatainAB(uint16_t &value) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_read_cmd(
        0x0037, reinterpret_cast<uint16_t *>(&value));

    return status;
}

aditof::Status CameraItof::adsd3500SetGenericTemplate(uint16_t reg,
                                                      uint16_t value) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(reg, value);

    return status;
}

aditof::Status CameraItof::adsd3500GetGenericTemplate(uint16_t reg,
                                                      uint16_t &value) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_read_cmd(
        reg, reinterpret_cast<uint16_t *>(&value));

    return status;
}

aditof::Status CameraItof::adsd3500DisableCCBM(bool disable) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->setControl("disableCCBM", std::to_string(disable));

    return status;
}

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

aditof::Status CameraItof::adsd3500GetStatus(int &chipStatus,
                                             int &imagerStatus) {
    aditof::Status status = aditof::Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_get_status(chipStatus, imagerStatus);
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
                LOG(ERROR) << "ADSD3030 imager error detected: "
                           << m_adsdErrors.GetStringADSD3030(imagerStatus);
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

aditof::Status CameraItof::adsd3500GetSensorTemperature(uint16_t &tmpValue) {
    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    unsigned int usDelay = 0;
    if (m_cameraFps > 0) {
        usDelay =
            static_cast<unsigned int>((1 / (double)m_cameraFps) * 1000000);
    }
    status = m_depthSensor->adsd3500_read_cmd(0x0054, &tmpValue, usDelay);
    if (status != Status::OK) {
        LOG(ERROR) << "Can not read sensor temperature";
        return Status::GENERIC_ERROR;
    }

    return status;
}
aditof::Status CameraItof::adsd3500GetLaserTemperature(uint16_t &tmpValue) {
    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    unsigned int usDelay = 0;
    if (m_cameraFps > 0) {
        usDelay =
            static_cast<unsigned int>((1 / (double)m_cameraFps) * 1000000);
    }
    status = m_depthSensor->adsd3500_read_cmd(0x0055, &tmpValue, usDelay);
    if (status != Status::OK) {
        LOG(ERROR) << "Can not read laser temperature";
        return Status::GENERIC_ERROR;
    }

    return status;
}

void CameraItof::UpdateDepthParamsMap(bool update, const char *index,
                                      std::string value) {

    if (update) {
        auto it = m_depth_params_map.find(m_details.mode);

        if (it != m_depth_params_map.end()) {
            auto indexIt = it->second.find(index);
            if (indexIt != it->second.end()) {
                indexIt->second = value;
            }
        }
    }
}

aditof::Status CameraItof::setDepthIniParams(
    const std::map<std::string, std::string> &iniKeyValPairs,
    bool updateDepthMap) {

    aditof::Status status = aditof::Status::OK;

    assert(!m_isOffline);

    auto it = iniKeyValPairs.find("abThreshMin");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetABinvalidationThreshold(std::stoi(it->second));
        if (status != aditof::Status::OK)
            LOG(WARNING) << "Could not set abThreshMin";

        UpdateDepthParamsMap(updateDepthMap, "abThreshMin", it->second);

    } else {
        LOG(WARNING)
            << "abThreshMin was not found in parameter list, not setting.";
    }

    it = iniKeyValPairs.find("confThresh");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetConfidenceThreshold(std::stoi(it->second));
        if (status != aditof::Status::OK)
            LOG(WARNING) << "Could not set confThresh";

        UpdateDepthParamsMap(updateDepthMap, "confThresh", it->second);
    } else {
        LOG(WARNING)
            << "confThresh was not found in parameter list, not setting.";
    }

    it = iniKeyValPairs.find("radialThreshMin");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetRadialThresholdMin(std::stoi(it->second));
        if (status != aditof::Status::OK)
            LOG(WARNING) << "Could not set radialThreshMin";

        UpdateDepthParamsMap(updateDepthMap, "radialThreshMin", it->second);
    } else {
        LOG(WARNING) << "radialThreshMin was not found in parameter list, "
                        "not setting.";
    }

    it = iniKeyValPairs.find("radialThreshMax");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetRadialThresholdMax(std::stoi(it->second));
        if (status != aditof::Status::OK)
            LOG(WARNING) << "Could not set radialThreshMax";

        UpdateDepthParamsMap(updateDepthMap, "radialThreshMax", it->second);
    } else {
        LOG(WARNING) << "radialThreshMax was not found in parameter list, "
                        "not setting.";
    }

    it = iniKeyValPairs.find("jblfWindowSize");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetJBLFfilterSize(std::stoi(it->second));
        if (status != aditof::Status::OK)
            LOG(WARNING) << "Could not set jblfWindowSize";

        UpdateDepthParamsMap(updateDepthMap, "jblfWindowSize", it->second);
    } else {
        LOG(WARNING) << "jblfWindowSize was not found in parameter list, "
                        "not setting.";
    }

    it = iniKeyValPairs.find("jblfApplyFlag");
    if (it != iniKeyValPairs.end()) {
        bool en = !(it->second == "0");
        status = adsd3500SetJBLFfilterEnableState(en);
        if (status != aditof::Status::OK)
            LOG(WARNING) << "Could not set jblfApplyFlag";

        UpdateDepthParamsMap(updateDepthMap, "jblfApplyFlag", it->second);
    } else {
        LOG(WARNING) << "jblfApplyFlag was not found in parameter list, "
                        "not setting.";
    }

    it = iniKeyValPairs.find("fps");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetFrameRate(std::stoi(it->second));
        if (status != aditof::Status::OK)
            LOG(WARNING) << "Could not set fps";

        UpdateDepthParamsMap(updateDepthMap, "fps", it->second);
    } else {
        LOG(WARNING) << "fps was not found in parameter list, not setting.";
    }

    it = iniKeyValPairs.find("vcselDelay");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetVCSELDelay((uint16_t)(std::stoi(it->second)));
        if (status != aditof::Status::OK)
            LOG(WARNING) << "Could not set vcselDelay";

        UpdateDepthParamsMap(updateDepthMap, "vcselDelay", it->second);
    } else {
        LOG(WARNING)
            << "vcselDelay was not found in parameter list, not setting.";
    }

    it = iniKeyValPairs.find("jblfMaxEdge");
    if (it != iniKeyValPairs.end()) {
        status =
            adsd3500SetJBLFMaxEdgeThreshold((uint16_t)(std::stoi(it->second)));
        if (status != aditof::Status::OK)
            LOG(WARNING) << "Could not set jblfMaxEdge";

        UpdateDepthParamsMap(updateDepthMap, "jblfMaxEdge", it->second);
    } else {
        LOG(WARNING) << "jblfMaxEdge was not found in parameter list, "
                        "not setting.";
    }

    it = iniKeyValPairs.find("jblfABThreshold");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetJBLFABThreshold((uint16_t)(std::stoi(it->second)));
        if (status != aditof::Status::OK)
            LOG(WARNING) << "Could not set jblfABThreshold";

        UpdateDepthParamsMap(updateDepthMap, "jblfABThreshold", it->second);
    } else {
        LOG(WARNING) << "jblfABThreshold was not found in parameter list";
    }

    it = iniKeyValPairs.find("jblfGaussianSigma");
    if (it != iniKeyValPairs.end()) {
        status =
            adsd3500SetJBLFGaussianSigma((uint16_t)(std::stoi(it->second)));
        if (status != aditof::Status::OK)
            LOG(WARNING) << "Could not set jblfGaussianSigma";

        UpdateDepthParamsMap(updateDepthMap, "jblfGaussianSigma", it->second);
    } else {
        LOG(WARNING) << "jblfGaussianSigma was not found in parameter list, "
                        "not setting.";
    }

    it = iniKeyValPairs.find("jblfExponentialTerm");
    if (it != iniKeyValPairs.end()) {
        status =
            adsd3500SetJBLFExponentialTerm((uint16_t)(std::stoi(it->second)));
        if (status != aditof::Status::OK)
            LOG(WARNING) << "Could not set jblfExponentialTerm";

        UpdateDepthParamsMap(updateDepthMap, "jblfExponentialTerm", it->second);
    } else {
        LOG(WARNING) << "jblfExponentialTerm was not found in parameter list, "
                        "not setting.";
    }

    it = iniKeyValPairs.find("enablePhaseInvalidation");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetEnablePhaseInvalidation((uint16_t)(std::stoi(it->second)));
        if (status != aditof::Status::OK)
            LOG(WARNING) << "Could not set enablePhaseInvalidation";

        UpdateDepthParamsMap(updateDepthMap, "enablePhaseInvalidation",
                             it->second);
    } else {
        LOG(WARNING)
            << "enablePhaseInvalidation was not found in parameter list, "
               "not setting.";
    }

#if 0 // This is updated from setMode, and this may not be the best place for this
    status = m_depthSensor->setDepthComputeParams(iniKeyValPairs); // Update Depth Compute Library
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "set ini parameters failed in depth-compute.";
    }
#endif //0

    return aditof::Status::OK;
}

void CameraItof::cleanupXYZtables() {
    if (m_xyzTable.p_x_table) {
        free((void *)m_xyzTable.p_x_table);
        m_xyzTable.p_x_table = nullptr;
    }
    if (m_xyzTable.p_y_table) {
        free((void *)m_xyzTable.p_y_table);
        m_xyzTable.p_y_table = nullptr;
    }
    if (m_xyzTable.p_z_table) {
        free((void *)m_xyzTable.p_z_table);
        m_xyzTable.p_z_table = nullptr;
    }
}

aditof::Status CameraItof::getImagerType(aditof::ImagerType &imagerType) const {
    imagerType = m_imagerType;

    return aditof::Status::OK;
}

aditof::Status CameraItof::adsd3500setEnableDynamicModeSwitching(bool en) {

    using namespace aditof;
    Status status = Status::OK;

    assert(!m_isOffline);

    status = m_depthSensor->adsd3500_write_cmd(0x0080, en ? 0x0001 : 0x0000);

    return status;
}

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
    status = m_depthSensor->adsd3500_write_cmd(0x0081, *sequence0);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to set sequence 0 for the Dynamic Mode Switching";
        return status;
    }
    status = m_depthSensor->adsd3500_write_cmd(0x0082, *sequence1);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to set sequence 1 for the Dynamic Mode Switching";
        return status;
    }

    uint16_t *repCount0 = reinterpret_cast<uint16_t *>(&entireRepCount);
    uint16_t *repCount1 = reinterpret_cast<uint16_t *>(&entireRepCount) + 1;
    status = m_depthSensor->adsd3500_write_cmd(0x0083, *repCount0);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to set mode repeat count 0 for the Dynamic Mode "
                      "Switching";
        return status;
    }
    status = m_depthSensor->adsd3500_write_cmd(0x0084, *repCount1);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to set mode repeat count 0 for the Dynamic Mode "
                      "Switching";
        return status;
    }

    return Status::OK;
}
