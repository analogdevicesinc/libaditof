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

#include "camera_initialization_manager.h"
#include "../hardware/adsd3500_registers.h"
#include "adsd3500_controller.h"
#include "calibration_manager.h"
#include "camera_configuration.h"
#include "platform/platform_impl.h"
#include "tofi/tofi_camera_intrinsics.h"
#include "tofi/tofi_util.h"
#include <aditof/adsd3500_hardware_interface.h>
#include <aditof/camera_definitions.h>
#include <aditof/log.h>
#include <chrono>
#include <cstring>
#include <fstream>
#include <thread>
#include <unordered_map>
#ifdef __linux__
#include <limits.h>
#include <unistd.h>
#endif

namespace aditof {

CameraInitializationManager::CameraInitializationManager(
    std::shared_ptr<DepthSensorInterface> depthSensor,
    CalibrationManager *calibrationMgr, Adsd3500Controller *controller,
    CameraConfiguration *config)
    : m_depthSensor(depthSensor), m_adsd3500Hardware(nullptr),
      m_calibrationMgr(calibrationMgr), m_controller(controller),
      m_config(config) {

    // Cast to Adsd3500HardwareInterface if sensor supports it
    m_adsd3500Hardware =
        std::dynamic_pointer_cast<Adsd3500HardwareInterface>(depthSensor);

    if (!m_adsd3500Hardware) {
        LOG(WARNING) << "ADSD3500 hardware interface not available (playback "
                        "mode or unsupported sensor)";
    }
}

Status CameraInitializationManager::initializeOfflineMode() {
    LOG(INFO) << "Initializing camera: Offline";
    return Status::OK;
}

Status CameraInitializationManager::initializeOnlineMode(
    CameraDetails &cameraDetails, std::vector<uint8_t> &availableModes,
    std::vector<DepthSensorModeDetails> &availableSensorModeDetails,
    ImagerType &imagerType,
    std::pair<std::string, std::string> &adsd3500FwVersion,
    const std::string &configFilepath, std::string &effectiveConfigPath) {

    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << "Initializing camera: Online";

    // Auto-discover config file path
    effectiveConfigPath = configFilepath;
    if (effectiveConfigPath.empty()) {
        effectiveConfigPath = m_config->autoDiscoverConfigFile();
        if (!effectiveConfigPath.empty()) {
            LOG(INFO) << "Auto-discovered configuration file: "
                      << effectiveConfigPath;
        }
    }

    // Load firmware version configuration from the SDK config directory.
    // Priority: 1) SDK source/build config dir (ADITOF_SDK_CONFIG_DIR, set by CMake)
    //           2) System data installation path (CMAKE_INSTALL_DATADIR/aditof)
    const std::string fwConfigFilename = "firmware_version_config.json";
    std::string fwConfigPath;

#ifdef ADITOF_SDK_CONFIG_DIR
    {
        std::string sdkConfigPath =
            std::string(ADITOF_SDK_CONFIG_DIR) + "/" + fwConfigFilename;
        std::ifstream testFile(sdkConfigPath);
        if (testFile.good()) {
            fwConfigPath = sdkConfigPath;
        }
    }
#endif

    // Fallback to installed data path
    if (fwConfigPath.empty()) {
        fwConfigPath =
            std::string(ADITOF_SDK_DATA_INSTALL_DIR) + "/" + fwConfigFilename;
    }

    status = m_config->loadFirmwareVersionConfig(fwConfigPath);
    if (status != Status::OK && status != Status::INVALID_ARGUMENT) {
        LOG(WARNING) << "Could not load firmware version config, continuing...";
    }

    // Open sensor hardware interface
    bool devStarted = false;
    status = openSensor(devStarted);
    if (status != Status::OK) {
        return status;
    }

    // Detect imager type (ADSD3100, ADTF3080, etc.)
    status = detectImagerType(imagerType);
    if (status != Status::OK) {
        return status;
    }

    // Discover available modes and get their details
    status = discoverAvailableModes(availableModes, availableSensorModeDetails);
    if (status != Status::OK) {
        return status;
    }

    // Read intrinsics and dealias params for each mode
    for (const auto &modeDetails : availableSensorModeDetails) {
        TofiXYZDealiasData dealiasData;
        status = readModeCalibrationData(modeDetails.modeNumber, dealiasData,
                                         cameraDetails);
        if (status != Status::OK) {
            return status;
        }

        // Store in calibration manager
        m_calibrationMgr->setXYZDealiasData(modeDetails.modeNumber,
                                            dealiasData);
    }

    // Read firmware version
    std::string fwVersion, fwHash;
    status = readFirmwareVersion(fwVersion, fwHash);
    if (status != Status::OK) {
        LOG(WARNING) << "Could not read firmware version - device may not "
                        "support this command yet";
        LOG(WARNING)
            << "Continuing initialization without firmware version info";
        fwVersion = "unknown";
        fwHash = "unknown";
    } else {
        LOG(INFO) << "Current adsd3500 firmware version is: " << fwVersion;
        LOG(INFO) << "Current adsd3500 firmware git hash is: " << fwHash;
    }
    adsd3500FwVersion = {fwVersion, fwHash};

    // Check firmware version against expected version from JSON config
    std::string expectedFwVersion = m_config->getExpectedFirmwareVersion();
    bool skipFwCheck = m_config->getSkipFirmwareVersionCheck();

    // Parse the major version from the leading numeric component of a
    // version string (e.g. "8.1.0.0" -> 8). std::stoi stops at '.'.
    auto parseMajorVersion = [](const std::string &version) -> int {
        try {
            return std::stoi(version);
        } catch (...) {
            return -1;
        }
    };

    // Parse full version into a comparable tuple (major, minor, patch, build).
    auto parseFullVersion =
        [](const std::string &v) -> std::tuple<int, int, int, int> {
        int major = 0, minor = 0, patch = 0, build = 0;
        sscanf(v.c_str(), "%d.%d.%d.%d", &major, &minor, &patch, &build);
        return {major, minor, patch, build};
    };

    // Hard minimum major version — enforced unconditionally, but can be
    // bypassed by setting skipFirmwareVersionCheck: true in the config file
    // (intended for development/testing with older firmware only).
    constexpr int MINIMUM_REQUIRED_MAJOR = 8;

    if (fwVersion != "unknown" && !skipFwCheck) {
        const int actualMajor = parseMajorVersion(fwVersion);
        if (actualMajor >= 0 && actualMajor < MINIMUM_REQUIRED_MAJOR) {
            LOG(ERROR) << "Firmware version is too old to proceed!";
            LOG(ERROR) << "  Actual: " << fwVersion;
            LOG(ERROR) << "  Actual major (" << actualMajor
                       << ") < required minimum major ("
                       << MINIMUM_REQUIRED_MAJOR << ")";
            LOG(ERROR) << "Please update the ADSD3500 firmware before using "
                          "this SDK version";
            return Status::GENERIC_ERROR;
        }
    } else if (fwVersion != "unknown" && skipFwCheck) {
        const int actualMajor = parseMajorVersion(fwVersion);
        if (actualMajor >= 0 && actualMajor < MINIMUM_REQUIRED_MAJOR) {
            LOG(WARNING)
                << "Firmware version is below minimum required major ("
                << MINIMUM_REQUIRED_MAJOR
                << ") but check is bypassed via skipFirmwareVersionCheck";
        }
    }

    // Config-based version check (runs only when a config file was found)
    if (!expectedFwVersion.empty() && !skipFwCheck && fwVersion != "unknown") {
        const int actualMajor = parseMajorVersion(fwVersion);
        const int expectedMajor = parseMajorVersion(expectedFwVersion);

        if (fwVersion == expectedFwVersion) {
            LOG(INFO) << "Firmware version matches expected version";
        } else if (actualMajor >= 0 && expectedMajor >= 0 &&
                   actualMajor < expectedMajor) {
            // Major below what the config requires — abort
            LOG(ERROR) << "Firmware version is too old to proceed!";
            LOG(ERROR) << "  Expected: " << expectedFwVersion;
            LOG(ERROR) << "  Actual:   " << fwVersion;
            LOG(ERROR) << "  Actual major (" << actualMajor
                       << ") < required major (" << expectedMajor << ")";
            LOG(ERROR) << "Please update the ADSD3500 firmware before using "
                          "this SDK version";
            return Status::GENERIC_ERROR;
        } else {
            // Compare full versions: only warn if actual is older than expected.
            if (parseFullVersion(fwVersion) >=
                parseFullVersion(expectedFwVersion)) {
                LOG(INFO) << "Firmware version is newer than expected: "
                          << fwVersion << " (expected: " << expectedFwVersion
                          << "), continuing without warning";
            } else {
                LOG(WARNING) << "Firmware version mismatch!";
                LOG(WARNING) << "  Expected: " << expectedFwVersion;
                LOG(WARNING) << "  Actual:   " << fwVersion;
                LOG(WARNING) << "Delaying 10 seconds before proceeding...";
                LOG(WARNING) << "This may cause issues with the SDK";

                for (int i = 10; i > 0; i--) {
                    LOG(INFO) << "Waiting... " << i << " seconds remaining";
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }

                LOG(INFO) << "Continuing initialization with mismatched "
                             "firmware version";
            }
        }
    } else if (!expectedFwVersion.empty() && skipFwCheck) {
        LOG(INFO) << "Firmware version check skipped per JSON config override";
    }

    // Load CCB data if any mode needs non-ISP processing
    status = loadCCBDataIfNeeded(availableModes);
    if (status != Status::OK) {
        // Warning only - ISP modes will still work
        LOG(WARNING) << "CCB loading had issues, continuing initialization";
    }

    // Apply hardware configurations
    status = applyHardwareConfiguration();
    if (status != Status::OK) {
        return status;
    }

    // Read serial number (optional - some firmware versions don't support it)
    std::string serialNumber;
    status = readSerialNumber(serialNumber);
    if (status == Status::OK) {
        LOG(INFO) << "Module serial number: " << serialNumber;
        cameraDetails.serialNumber = serialNumber;
    } else if (status == Status::UNAVAILABLE) {
        LOG(INFO) << "Serial read is not supported in this firmware!";
    } else {
        LOG(ERROR) << "Failed to read serial number!";
        return status;
    }

    LOG(INFO) << "Camera initialized";
    return Status::OK;
}

Status CameraInitializationManager::openSensor(bool &devStarted) {
    if (devStarted) {
        return Status::OK;
    }

    Status status = m_depthSensor->open();
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to open device";
        return status;
    }

    devStarted = true;
    return Status::OK;
}

Status CameraInitializationManager::detectImagerType(ImagerType &detectedType) {
    // Map of imager type enum values to string representations (numeric values from sensor)
    // Sensor getControl("imagerType") returns std::to_string((int)SensorImagerType)
    static const std::unordered_map<ImagerType, std::string> ControlValue = {
        {ImagerType::ADSD3100,
         "1"}, // Maps to SensorImagerType::IMAGER_ADSD3100
        {ImagerType::ADSD3030,
         "2"}, // Maps to SensorImagerType::IMAGER_ADSD3030
        {ImagerType::ADTF3080,
         "3"}, // Maps to SensorImagerType::IMAGER_ADTF3080
        {ImagerType::ADTF3066,
         "4"}}; // Maps to SensorImagerType::IMAGER_ADTF3066

    std::string controlValue;
    Status status = m_depthSensor->getControl("imagerType", controlValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to get imager type from sensor";
        return status;
    }

    // Find matching imager type
    for (const auto &entry : ControlValue) {
        if (controlValue == entry.second) {
            detectedType = entry.first;
            // Look up human-readable name from imagerType map
            auto nameIt = imagerType.find(entry.first);
            std::string imagerName =
                (nameIt != imagerType.end()) ? nameIt->second : "unknown";
            LOG(INFO) << "Detected imager type: " << imagerName;
            return Status::OK;
        }
    }

    // Unknown imager type
    LOG(ERROR) << "Unknown imager type: " << controlValue;
    return Status::UNAVAILABLE;
}

Status CameraInitializationManager::discoverAvailableModes(
    std::vector<uint8_t> &modes,
    std::vector<DepthSensorModeDetails> &modeDetails) {

    Status status = m_depthSensor->getAvailableModes(modes);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to get available frame types name!";
        return status;
    }

    modeDetails.clear();
    for (const auto &mode : modes) {
        DepthSensorModeDetails details;
        status = m_depthSensor->getModeDetails(mode, details);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to get details for mode " << (int)mode;
            return status;
        }
        modeDetails.push_back(details);
    }

    return Status::OK;
}

Status CameraInitializationManager::readModeCalibrationData(
    uint8_t modeNumber, TofiXYZDealiasData &dealiasData,
    CameraDetails &cameraDetails) {

    uint8_t intrinsics[56] = {0};
    uint8_t dealiasParams[32] = {0};

    // First byte of readback_data is used for the mode number (custom command param)
    intrinsics[0] = modeNumber;
    dealiasParams[0] = modeNumber;

    // Command 0x01: Read intrinsics (56 bytes)
    Status status =
        m_adsd3500Hardware->adsd3500_read_payload_cmd(0x01, intrinsics, 56);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to read intrinsics for mode " << (int)modeNumber;
        return status;
    }

    // Command 0x02: Read dealias parameters (32 bytes)
    status =
        m_adsd3500Hardware->adsd3500_read_payload_cmd(0x02, dealiasParams, 32);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to read dealias parameters for mode "
                   << (int)modeNumber;
        return status;
    }

    // Combine into TofiXYZDealiasData structure
    memcpy(&dealiasData, dealiasParams,
           sizeof(TofiXYZDealiasData) - sizeof(CameraIntrinsics));
    memcpy(&dealiasData.camera_intrinsics, intrinsics,
           sizeof(CameraIntrinsics));

    // Update camera details with intrinsics from this mode
    memcpy(&cameraDetails.intrinsics, &dealiasData.camera_intrinsics,
           sizeof(CameraIntrinsics));

    return Status::OK;
}

Status CameraInitializationManager::loadCCBDataIfNeeded(
    const std::vector<uint8_t> &modes) {

    // Check if any mode requires non-ISP depth computation
    bool needsNonIspSupport = false;
    for (uint8_t mode : modes) {
        std::map<std::string, std::string> params;
        if (m_config->getDepthParamsForMode(mode, params) == Status::OK) {
            auto it = params.find("depthComputeIspEnable");
            // non-ISP support needed if key is missing or set to "0"
            if (it == params.end() || it->second != "1") {
                needsNonIspSupport = true;
                break;
            }
        }
    }

    if (!needsNonIspSupport) {
        LOG(INFO) << "All modes configured for ISP depth compute - "
                  << "CCB data not required";
        return Status::OK;
    }

    LOG(INFO) << "Reading raw CCB data for non-ISP mode support...";
    std::string rawCCB;
    Status status = m_calibrationMgr->readCCB(rawCCB);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to read raw CCB data - non-ISP modes "
                        "may not work correctly";
        return status;
    }

    m_calibrationMgr->setRawCCBData(rawCCB);
    LOG(INFO) << "Raw CCB data stored successfully (" << rawCCB.size()
              << " bytes)";

    return Status::OK;
}

Status CameraInitializationManager::applyHardwareConfiguration() {
    Status status = Status::OK;

    // 1. FSYNC toggle mode configuration
    if (m_config->getFsyncMode() >= 0) {
        status = m_controller->setToggleMode(m_config->getFsyncMode());
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to set fsyncMode.";
            return status;
        }
    } else {
        LOG(WARNING) << "fsyncMode is not being set by SDK.";
    }

    // 2. Platform-based configuration for MIPI and deskew
    int platformMipiSpeed =
        aditof::platform::Platform::getInstance().getMipiOutputSpeed();
    int platformDeskewEnabled =
        aditof::platform::Platform::getInstance().getDeskewEnabled();

    // Apply platform defaults if not already configured
    if (platformMipiSpeed >= 0 && m_config->getMipiOutputSpeed() < 0) {
        m_config->setMipiOutputSpeed(platformMipiSpeed);
        LOG(INFO) << "Using platform MIPI output speed: " << platformMipiSpeed;
    }
    if (platformDeskewEnabled >= 0 && m_config->getDeskewEnabled() < 0) {
        m_config->setDeskewEnabled(platformDeskewEnabled);
        LOG(INFO) << "Using platform deskew setting: " << platformDeskewEnabled;
    }

    // Use hardware defaults if platform didn't specify
    if (m_config->getMipiOutputSpeed() < 0) {
        m_config->setMipiOutputSpeed(0);
        LOG(INFO) << "Using hardware default MIPI output speed";
    }
    if (m_config->getDeskewEnabled() < 0) {
        m_config->setDeskewEnabled(0);
        LOG(INFO) << "Using hardware default deskew setting";
    }

    // 3. Apply MIPI output speed configuration
    if (m_config->getMipiOutputSpeed() > 0) {
        status =
            m_controller->setMIPIOutputSpeed(m_config->getMipiOutputSpeed());
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to set MIPI output speed to "
                       << m_config->getMipiOutputSpeed();
            return status;
        }
        LOG(INFO) << "MIPI output speed set to "
                  << m_config->getMipiOutputSpeed();
    }

    // 4. Apply deskew configuration
    if (m_config->getDeskewEnabled() > 0) {
        status = m_controller->setEnableDeskewAtStreamOn(
            m_config->getDeskewEnabled());
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to enable deskew at stream on";
            return status;
        }
        LOG(INFO) << "Deskew enabled at stream on";
    }

    // 5. Temperature compensation configuration
    if (m_config->getTempCompensation() >= 0) {
        status = m_controller->setEnableTemperatureCompensation(
            m_config->getTempCompensation());
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to set enableTempCompenstation.";
            return status;
        }
    } else {
        LOG(WARNING) << "enableTempCompenstation is not being set by SDK.";
    }

    // 6. Edge confidence configuration
    if (m_config->getEdgeConfidence() >= 0) {
        status = m_controller->setEnableEdgeConfidence(
            m_config->getEdgeConfidence());
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to set enableEdgeConfidence.";
            return status;
        }
    } else {
        LOG(WARNING) << "enableEdgeConfidence is not being set by SDK.";
    }

    return Status::OK;
}

Status CameraInitializationManager::readFirmwareVersion(std::string &fwVersion,
                                                        std::string &fwHash) {
    return m_controller->getFirmwareVersion(fwVersion, fwHash);
}

Status
CameraInitializationManager::readSerialNumber(std::string &serialNumber) {
    // Read serial number using payload command (same as camera_itof.cpp)
    // Serial number reading requires firmware version >= 4.7.1.0 (4710)
    // Since we don't have access to firmware version here, we'll attempt
    // the read and let the caller handle UNAVAILABLE status

    uint8_t serial[32] = {0};
    Status status =
        m_adsd3500Hardware->adsd3500_read_payload_cmd(0x19, serial, 32);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to read serial number from sensor";
        return status;
    }

    // Convert byte array to string
    serialNumber = std::string(reinterpret_cast<char *>(serial));
    return Status::OK;
}

} // namespace aditof
