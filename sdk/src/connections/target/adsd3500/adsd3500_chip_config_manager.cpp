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
#include "adsd3500_chip_config_manager.h"
#include "adsd3500_mode_selector.h"
#include "adsd3500_protocol_manager.h"
#include "adsd3500_sensor.h"
#include "sensor-tables/device_parameters.h"
#include "tofi/tofi_compute.h"
#include "tofi/tofi_config.h"
#include <aditof/log.h>

#include <cstring>

// ADSD3500 command codes
#define ADSD3500_CMD_GET_FW_VERSION 0x05
#define ADSD3500_CMD_GET_CHIP_INFO 0x0032
#define ADSD3500_CMD_CHECK_CCBM_SUPPORT 0x39
#define ADSD3500_CMD_READ_MODE_MAP 0x24
#define ADSD3500_CMD_READ_INI_CONTENT 0x25
#define ADSD3500_CMD_READ_DEALIAS_PARAMS 0x02

// Buffer and payload sizes
#define ADSD3500_FW_VERSION_BUFFER_SIZE 44
#define ADSD3500_FW_VERSION_STRING_LENGTH 4
#define ADSD3500_INI_CONTENT_SIZE 0x26
#define ADSD3500_DEALIAS_PARAMS_SIZE 32

// Chip info register masks
#define ADSD3500_CHIP_INFO_CCB_MASK 0x00FF
#define ADSD3500_CHIP_INFO_IMAGER_MASK 0xFF00
#define ADSD3500_CHIP_INFO_IMAGER_SHIFT 8

// Invalid markers
#define ADSD3500_INVALID_MODE_NUMBER 0xFF

// Firmware version requirements
#define ADSD3500_MIN_FW_VERSION_FOR_CHIP_INFO 3

// CCBM mode reading constants
#define NR_OF_MODES_FROM_CCB 10
#define SIZE_OF_MODES_FROM_CCB 256

Adsd3500ChipConfigManager::Adsd3500ChipConfigManager(
    Adsd3500ProtocolManager &protocolManager,
    aditof::Adsd3500ModeSelector &modeSelector, SensorImagerType &imagerType,
    CCBVersion &ccbVersion, std::string &fwVersion,
    std::unordered_map<std::string, std::string> &controls, uint16_t &chipId)
    : m_protocolManager(protocolManager), m_modeSelector(modeSelector),
      m_imagerType(imagerType), m_ccbVersion(ccbVersion),
      m_fwVersion(fwVersion), m_controls(controls), m_chipId(chipId) {}

aditof::Status Adsd3500ChipConfigManager::queryChipConfiguration(
    std::vector<aditof::DepthSensorModeDetails> &availableModes,
    std::vector<IniTableEntry> &ccbmINIContent,
    std::vector<iniFileStruct> &iniFileStructList,
    std::vector<uint8_t> &bitsInAB, std::vector<uint8_t> &bitsInConf,
    bool &ccbmEnabled) {

    using namespace aditof;
    Status status = Status::OK;

    // Discover chip capabilities (FW version, CCB version, imager type)
    status = discoverChipCapabilities();
    if (status != Status::OK) {
        return status;
    }

    if (m_ccbVersion != CCBVersion::CCB_UNKNOWN) {
        if (m_ccbVersion == CCBVersion::CCB_VERSION0) {
            LOG(ERROR) << "Old modes are no longer supported!";
            return Status::GENERIC_ERROR;
        }

        // Check if CCBM (CCB Master) is supported and enabled
        if (m_ccbVersion == CCBVersion::CCB_VERSION3 ||
            (m_ccbVersion == CCBVersion::CCB_VERSION2 &&
             m_controls["disableCCBM"] == "0")) {

            // Read modes from chip NVM
            status = readModesFromCCBM(availableModes, ccbmINIContent);
            if (status != Status::OK) {
                return status;
            }
            ccbmEnabled = true;

        } else {
            // Use SDK-defined modes
            status = readModesFromSDK(availableModes);
            if (status != Status::OK) {
                return status;
            }
            ccbmEnabled = false;
        }
    }

    // Initialize the bits table
    status = m_modeSelector.init_bitsPerPixelTable();
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to initialize bits per pixel table!";
        return status;
    }

    // Create INI parameter structures for all modes
    status = createIniParamsForModes(availableModes, iniFileStructList);
    if (status != Status::OK) {
        return status;
    }

    // Configure frame content for each mode
    status = configureFrameContent(availableModes, iniFileStructList, bitsInAB,
                                   bitsInConf);
    if (status != Status::OK) {
        return status;
    }

    // Merge INI parameters across modes
    status = mergeIniParams(iniFileStructList, ccbmEnabled, ccbmINIContent);
    if (status != Status::OK) {
        return status;
    }

    return status;
}

aditof::Status Adsd3500ChipConfigManager::discoverChipCapabilities() {
    using namespace aditof;
    Status status = Status::OK;

    // Skip if already queried
    if (m_imagerType != SensorImagerType::IMAGER_UNKNOWN &&
        m_ccbVersion != CCBVersion::CCB_UNKNOWN) {
        return Status::OK;
    }

    // Read firmware version
    uint8_t fwData[ADSD3500_FW_VERSION_BUFFER_SIZE] = {0};
    fwData[0] = uint8_t(1);
    status = m_protocolManager.adsd3500_read_payload_cmd(
        ADSD3500_CMD_GET_FW_VERSION, fwData, ADSD3500_FW_VERSION_BUFFER_SIZE);
    if (status != Status::OK) {
        LOG(ERROR)
            << "Failed to retrieve fw version and git hash for adsd3500!";
        return status;
    }
    m_fwVersion =
        std::string((char *)(fwData), ADSD3500_FW_VERSION_STRING_LENGTH);

    // Determine major version
    int majorVersion = m_fwVersion.at(0);
    if (majorVersion ==
        0) { // 0 means beta version, version starts at position 1
        majorVersion = m_fwVersion.at(1);
    }

    // Read CCB and imager version (only supported in FW major version > 3)
    uint16_t readValue = 0;
    if (majorVersion > ADSD3500_MIN_FW_VERSION_FOR_CHIP_INFO) {
        status = m_protocolManager.adsd3500_read_cmd(ADSD3500_CMD_GET_CHIP_INFO,
                                                     &readValue);
    } else {
        status = Status::GENERIC_ERROR;
    }

    if (status == Status::OK) {
        // Parse CCB version
        uint8_t ccb_version = readValue & ADSD3500_CHIP_INFO_CCB_MASK;
        switch (ccb_version) {
        case 1:
            m_ccbVersion = CCBVersion::CCB_VERSION0;
            break;
        case 2:
            m_ccbVersion = CCBVersion::CCB_VERSION1;
            break;
        case 3:
            m_ccbVersion = CCBVersion::CCB_VERSION2;
            break;
        case 4:
            m_ccbVersion = CCBVersion::CCB_VERSION3;
            break;
        default:
            LOG(WARNING) << "Unknown CCB version read from ADSD3500: "
                         << static_cast<int>(ccb_version);
            break;
        }

        // Parse imager type
        uint8_t imager_version = (readValue & ADSD3500_CHIP_INFO_IMAGER_MASK) >>
                                 ADSD3500_CHIP_INFO_IMAGER_SHIFT;
        switch (imager_version) {
        case 1:
            m_imagerType = SensorImagerType::IMAGER_ADSD3100;
            m_modeSelector.setControl("imagerType", "adsd3100");
            break;
        case 2:
            m_imagerType = SensorImagerType::IMAGER_ADSD3030;
            m_modeSelector.setControl("imagerType", "adsd3030");
            break;
        case 3:
            m_imagerType = SensorImagerType::IMAGER_ADTF3080;
            m_modeSelector.setControl("imagerType", "adtf3080");
            break;
        case 4:
            m_imagerType = SensorImagerType::IMAGER_ADTF3066;
            m_modeSelector.setControl("imagerType", "adtf3066");
            break;
        default:
            LOG(WARNING) << "Unknown imager type read from ADSD3500: "
                         << static_cast<int>(imager_version);
            break;
        }
    } else {
        LOG(ERROR) << "Failed to read imager type and CCB version (command "
                      "0x0032). Possibly command is not implemented on the "
                      "current adsd3500 firmware.";
        return Status::UNAVAILABLE;
    }

    return Status::OK;
}

aditof::Status Adsd3500ChipConfigManager::readModesFromCCBM(
    std::vector<aditof::DepthSensorModeDetails> &availableModes,
    std::vector<IniTableEntry> &ccbmINIContent) {

    using namespace aditof;
    Status status = Status::OK;

    // Check if CCB supports mode map table
    uint16_t data;
    status = m_protocolManager.adsd3500_read_cmd(
        ADSD3500_CMD_CHECK_CCBM_SUPPORT, &data);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to check if ccb has mode map table support!";
        return status;
    }

    LOG(INFO) << "CCB master is supported. Reading mode details from nvm.";

    availableModes.clear();
    ccbmINIContent.clear();

    // Read mode map table from chip
    CcbMode modeStruct[NR_OF_MODES_FROM_CCB];
    status = m_protocolManager.adsd3500_read_payload_cmd(
        ADSD3500_CMD_READ_MODE_MAP, (uint8_t *)&modeStruct[0],
        SIZE_OF_MODES_FROM_CCB);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to read mode map table from ccb!";
        return status;
    }

    // Parse each mode entry
    for (int i = 0; i < NR_OF_MODES_FROM_CCB; i++) {
        DepthSensorModeDetails modeDetails;
        memset((void *)&modeDetails, 0, sizeof(DepthSensorModeDetails));

        modeDetails.modeNumber = modeStruct[i].CFG_mode;
        if (modeDetails.modeNumber == ADSD3500_INVALID_MODE_NUMBER) {
            continue; // Invalid mode
        }

        modeDetails.baseResolutionHeight = modeStruct[i].heigth;
        modeDetails.baseResolutionWidth = modeStruct[i].width;
        modeDetails.numberOfPhases = modeStruct[i].noOfPhases;
        modeDetails.numberOfFrequencies = modeStruct[i].nFreq;
        modeDetails.isPCM = modeStruct[i].isPCM;

        if (modeDetails.baseResolutionWidth == 0 ||
            modeDetails.baseResolutionHeight == 0) {
            continue; // Invalid dimensions
        }

        // Read INI file content for non-PCM modes
        IniTableEntry iniTableContent;
        memset(&iniTableContent, 0, sizeof(IniTableEntry));
        iniTableContent.INIIndex = modeDetails.modeNumber;

        if (!modeDetails.isPCM) {
            status = m_protocolManager.adsd3500_read_payload_cmd(
                ADSD3500_CMD_READ_INI_CONTENT, (uint8_t *)(&iniTableContent),
                ADSD3500_INI_CONTENT_SIZE);
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to read ini content from nvm";
                return status;
            }

            if (iniTableContent.INIIndex == ADSD3500_INVALID_MODE_NUMBER) {
                LOG(INFO) << "No ini content for mode "
                          << static_cast<int>(modeDetails.modeNumber)
                          << " in nvm!";
                continue;
            }
        }

        iniTableContent.modeNumber = modeDetails.modeNumber;

        availableModes.emplace_back(modeDetails);
        ccbmINIContent.emplace_back(iniTableContent);
    }

    return Status::OK;
}

aditof::Status Adsd3500ChipConfigManager::readModesFromSDK(
    std::vector<aditof::DepthSensorModeDetails> &availableModes) {

    using namespace aditof;
    Status status = Status::OK;

    if (m_controls["disableCCBM"] == "1") {
        LOG(INFO)
            << "CCB master is disabled via control. Using sdk defined modes.";
    } else {
        LOG(INFO) << "CCB master not supported. Using sdk defined modes.";
    }

    // Test mode 5 dimensions to determine if mixed modes are supported
    int modeToTest = 5;
    uint8_t tempDealiasParams[32] = {0};
    tempDealiasParams[0] = modeToTest;

    TofiXYZDealiasData tempDealiasStruct;
    uint16_t width1 = 512;
    uint16_t height1 = 512;
    uint16_t width2 = 320;
    uint16_t height2 = 256;

    // Read dealias parameters to determine mode 5 dimensions
    status = m_protocolManager.adsd3500_read_payload_cmd(
        ADSD3500_CMD_READ_DEALIAS_PARAMS, tempDealiasParams,
        ADSD3500_DEALIAS_PARAMS_SIZE);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to read dealias parameters for adsd3500!";
        return status;
    }

    memcpy(&tempDealiasStruct, tempDealiasParams,
           sizeof(TofiXYZDealiasData) - sizeof(CameraIntrinsics));

    // Determine if mixed modes have accurate dimensions
    if ((tempDealiasStruct.n_rows == width1 &&
         tempDealiasStruct.n_cols == height1) ||
        (tempDealiasStruct.n_rows == width2 &&
         tempDealiasStruct.n_cols == height2)) {
        m_modeSelector.setControl("mixedModes", "1");
    } else {
        m_modeSelector.setControl("mixedModes", "0");
    }

    // Get available mode details from SDK mode selector
    status = m_modeSelector.getAvailableModeDetails(availableModes);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to get available frame types for the "
                      "current configuration.";
        return status;
    }

    return Status::OK;
}

aditof::Status Adsd3500ChipConfigManager::createIniParamsForModes(
    const std::vector<aditof::DepthSensorModeDetails> &availableModes,
    std::vector<iniFileStruct> &iniFileStructList) {

    using namespace aditof;
    Status status = Status::OK;

    // Determine imager name string
    std::string imagerName;
    if (m_imagerType == SensorImagerType::IMAGER_ADSD3100) {
        imagerName = "adsd3100";
    } else if (m_imagerType == SensorImagerType::IMAGER_ADSD3030) {
        imagerName = "adsd3030";
    } else if (m_imagerType == SensorImagerType::IMAGER_ADTF3080) {
        imagerName = "adtf3080";
    } else if (m_imagerType == SensorImagerType::IMAGER_ADTF3066) {
        imagerName = "adtf3066";
    } else {
        LOG(ERROR) << "Unknown imager type, cannot create INI params!";
        return Status::GENERIC_ERROR;
    }

    // Create INI parameters for each mode - cast away const since createIniParams doesn't modify
    status = DeviceParameters::createIniParams(
        iniFileStructList,
        const_cast<std::vector<aditof::DepthSensorModeDetails> &>(
            availableModes),
        imagerName, m_chipId);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to populate ini params struct!";
        return status;
    }

    return Status::OK;
}

aditof::Status Adsd3500ChipConfigManager::configureFrameContent(
    std::vector<aditof::DepthSensorModeDetails> &availableModes,
    const std::vector<iniFileStruct> &iniFileStructList,
    std::vector<uint8_t> &bitsInAB, std::vector<uint8_t> &bitsInConf) {

    using namespace aditof;

    // Allocate bit depth vectors based on number of modes
    bitsInAB.resize(availableModes.size());
    bitsInConf.resize(availableModes.size());

    // Configure frame content for each mode
    for (size_t i = 0; i < availableModes.size(); ++i) {
        const iniFileStruct &iniFile = iniFileStructList[i];
        auto &modeDetails = availableModes[i];
        std::string value;

        if (!modeDetails.isPCM) {
            // Non-PCM mode: depth, raw, optional AB/conf/xyz
            modeDetails.frameContent.clear();
            modeDetails.frameContent = {"raw", "depth"};

            // Check for AB frame
            auto it = iniFile.iniKeyValPairs.find("bitsInAB");
            if (it != iniFile.iniKeyValPairs.end()) {
                value = it->second;
                bitsInAB[modeDetails.modeNumber] = (uint8_t)std::stoi(value);
                if (bitsInAB[modeDetails.modeNumber] != 0) {
                    modeDetails.frameContent.push_back("ab");
                }
            } else {
                LOG(WARNING) << "bits In AB was not found in parameter list, "
                                "discarding it";
            }

            // Check for Confidence frame
            it = iniFile.iniKeyValPairs.find("bitsInConf");
            if (it != iniFile.iniKeyValPairs.end()) {
                value = it->second;
                bitsInConf[modeDetails.modeNumber] = (uint8_t)std::stoi(value);
                if (bitsInConf[modeDetails.modeNumber] != 0) {
                    modeDetails.frameContent.push_back("conf");
                }
            } else {
                LOG(WARNING)
                    << "bits In Confidence was not found in parameter list, "
                       "discarding it";
            }

            // Check for XYZ frame
            it = iniFile.iniKeyValPairs.find("xyzEnable");
            if (it != iniFile.iniKeyValPairs.end()) {
                value = it->second;
                if (value != "0") {
                    modeDetails.frameContent.push_back("xyz");
                }
            } else {
                LOG(WARNING) << "XYZ frame is disabled therefore discarding it";
            }

            // Add metadata frame
            modeDetails.frameContent.push_back("metadata");

        } else {
            // PCM mode: optional AB, no confidence
            modeDetails.frameContent.clear();

            // Check for AB frame
            auto it = iniFile.iniKeyValPairs.find("bitsInAB");
            if (it != iniFile.iniKeyValPairs.end()) {
                value = it->second;
                bitsInAB[modeDetails.modeNumber] = (uint8_t)std::stoi(value);
                if (bitsInAB[modeDetails.modeNumber] != 0) {
                    modeDetails.frameContent.push_back("ab");
                }
            } else {
                LOG(WARNING) << "bits In AB was not found in parameter list, "
                                "discarding it";
            }

            bitsInConf[modeDetails.modeNumber] = 0;
            modeDetails.frameContent.push_back("metadata");
        }
    }

    return Status::OK;
}

aditof::Status Adsd3500ChipConfigManager::mergeIniParams(
    std::vector<iniFileStruct> &iniFileStructList, bool ccbmEnabled,
    const std::vector<IniTableEntry> &ccbmINIContent) {

    using namespace aditof;

    // Merge CCBM INI content with SDK INI parameters
    // Only merge if CCBM is enabled and we have CCBM content
    if (!ccbmEnabled) {
        return Status::OK; // Nothing to merge
    }

    // Iterate through CCBM content and merge with corresponding INI file structures
    for (const auto &ccbmParams : ccbmINIContent) {
        for (auto &iniList : iniFileStructList) {
            if (iniList.modeName.empty()) {
                continue;
            }

            // Match by mode number
            if (ccbmParams.modeNumber == std::stoi(iniList.modeName)) {
                // Merge CCBM parameters into INI key-value pairs
                iniList.iniKeyValPairs["abThreshMin"] =
                    std::to_string(ccbmParams.abThreshMin);
                iniList.iniKeyValPairs["confThresh"] =
                    std::to_string(ccbmParams.confThresh);
                iniList.iniKeyValPairs["radialThreshMin"] =
                    std::to_string(ccbmParams.radialThreshMin);
                iniList.iniKeyValPairs["radialThreshMax"] =
                    std::to_string(ccbmParams.radialThreshMax);
                iniList.iniKeyValPairs["jblfApplyFlag"] =
                    std::to_string(ccbmParams.jblfApplyFlag);
                iniList.iniKeyValPairs["jblfWindowSize"] =
                    std::to_string(ccbmParams.jblfWindowSize);
                iniList.iniKeyValPairs["jblfGaussianSigma"] =
                    std::to_string(ccbmParams.jblfGaussianSigma);
                iniList.iniKeyValPairs["jblfExponentialTerm"] =
                    std::to_string(ccbmParams.jblfExponentialTerm);
                iniList.iniKeyValPairs["jblfMaxEdge"] =
                    std::to_string(ccbmParams.jblfMaxEdge);
                iniList.iniKeyValPairs["jblfABThreshold"] =
                    std::to_string(ccbmParams.jblfABThreshold);

                break; // Found matching mode, move to next CCBM params
            }
        }
    }

    return Status::OK;
}
