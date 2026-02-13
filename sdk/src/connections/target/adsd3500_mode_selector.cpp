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
#include "adsd3500_mode_selector.h"
#include <algorithm>

/**
 * @brief Constructs an Adsd3500ModeSelector object.
 *
 * Initializes the mode selector with default "standard" configuration and populates
 * available configurations (standard, standardRaw, custom, customRaw). Also initializes
 * control parameters map for mode selection including imagerType, mode, mixedModes,
 * depthBits, abBits, confBits, and inputFormat.
 */
Adsd3500ModeSelector::Adsd3500ModeSelector() : m_configuration("standard") {

    m_availableConfigurations.emplace_back("standard");
    m_availableConfigurations.emplace_back("standardRaw");
    m_availableConfigurations.emplace_back("custom");
    m_availableConfigurations.emplace_back("customRaw");

    m_controls.emplace("imagerType", "");
    m_controls.emplace("mode", "");
    m_controls.emplace("mixedModes", "");

    m_controls.emplace("depthBits", "");
    m_controls.emplace("abBits", "");
    m_controls.emplace("confBits", "");

    m_controls.emplace("inputFormat", "");
}

/**
 * @brief Sets the sensor mode configuration.
 *
 * Selects which configuration table to use for mode lookups. Valid configurations
 * include "standard", "standardRaw", "custom", and "customRaw".
 *
 * @param[in] configuration Configuration name to set
 *
 * @return Status::OK if configuration is valid and set successfully,
 *         Status::INVALID_ARGUMENT if configuration is not in available list
 */
aditof::Status
Adsd3500ModeSelector::setConfiguration(const std::string &configuration) {
    aditof::Status status = aditof::Status::OK;
    if (std::find(m_availableConfigurations.begin(),
                  m_availableConfigurations.end(),
                  configuration) != m_availableConfigurations.end()) {
        m_configuration = configuration;
        status = aditof::Status::OK;
    } else {
        status = aditof::Status::INVALID_ARGUMENT;
    }
    return status;
}

/**
 * @brief Retrieves available mode details based on current configuration and imager type.
 *
 * Returns a list of available DepthSensorModeDetails for the currently selected configuration
 * (e.g., "standard") and imager type (ADSD3100, ADSD3030, ADTF3080, or ADTF3066). The returned modes
 * include resolution, phase count, and other sensor-specific parameters.
 *
 * @param[out] m_depthSensorModeDetails Vector to be populated with available mode details
 *
 * @return Status::OK on success
 */
aditof::Status Adsd3500ModeSelector::getAvailableModeDetails(
    std::vector<DepthSensorModeDetails> &m_depthSensorModeDetails) {

    m_depthSensorModeDetails.clear();

    if (m_configuration == "standard") {
        if (m_controls["imagerType"] == imagerType.at(ImagerType::ADSD3100)) {
            m_depthSensorModeDetails = adsd3100_standardModes;
        } else if (m_controls["imagerType"] ==
                   imagerType.at(ImagerType::ADSD3030)) {
            m_depthSensorModeDetails = adsd3030_standardModes;
        } else if (m_controls["imagerType"] ==
                   imagerType.at(ImagerType::ADTF3080)) {
            m_depthSensorModeDetails = adtf3080_standardModes;
        } else if (m_controls["imagerType"] ==
                   imagerType.at(ImagerType::ADTF3066)) {
            m_depthSensorModeDetails = adtf3066_standardModes;
        }
    }

    return aditof::Status::OK;
}

/**
 * @brief Retrieves the configuration table for the currently selected mode.
 *
 * Searches the appropriate mode table (based on configuration and imager type) for the
 * mode matching the current "mode" control value, then returns its full configuration details.
 *
 * @param[out] configurationTable DepthSensorModeDetails structure to be populated with mode config
 *
 * @return Status::OK if mode is found and configuration returned,
 *         Status::INVALID_ARGUMENT if mode is not found or imager type is invalid
 */
aditof::Status Adsd3500ModeSelector::getConfigurationTable(
    DepthSensorModeDetails &configurationTable) {

    if (m_configuration == "standard") {
        if (m_controls["imagerType"] == imagerType.at(ImagerType::ADSD3100)) {
            m_tableInUse = adsd3100_standardModes;
            for (auto &modes : adsd3100_standardModes) {
                if (m_controls["mode"] == std::to_string(modes.modeNumber)) {
                    configurationTable = modes;
                    return aditof::Status::OK;
                }
            }
        } else if (m_controls["imagerType"] ==
                   imagerType.at(ImagerType::ADSD3030)) {
            m_tableInUse = adsd3030_standardModes;
            for (auto &modes : adsd3030_standardModes) {
                if (m_controls["mode"] == std::to_string(modes.modeNumber)) {
                    configurationTable = modes;
                    return aditof::Status::OK;
                }
            }
        } else if (m_controls["imagerType"] ==
                   imagerType.at(ImagerType::ADTF3080)) {
            m_tableInUse = adtf3080_standardModes;
            for (auto &modes : adtf3080_standardModes) {
                if (m_controls["mode"] == std::to_string(modes.modeNumber)) {
                    configurationTable = modes;
                    return aditof::Status::OK;
                }
            }
        } else if (m_controls["imagerType"] ==
                   imagerType.at(ImagerType::ADTF3066)) {
            m_tableInUse = adtf3066_standardModes;
            for (auto &modes : adtf3066_standardModes) {
                if (m_controls["mode"] == std::to_string(modes.modeNumber)) {
                    configurationTable = modes;
                    return aditof::Status::OK;
                }
            }
        }
    }

    return aditof::Status::INVALID_ARGUMENT;
}

/**
 * @brief Updates configuration table with driver-specific parameters.
 *
 * Matches the base configuration with ADSD3500 driver requirements to populate frame
 * width/height in bytes and pixel format index. For standard modes, searches m_adsd3500standard
 * for matching resolution, phases, and bit configuration. For custom modes, calculates dimensions
 * based on bit configuration. Special handling for PCM modes.
 *
 * @param[in,out] configurationTable Configuration to update with driver-specific parameters
 *
 * @return Status::OK if configuration is successfully updated,
 *         Status::INVALID_ARGUMENT if bits combination is invalid
 */
aditof::Status Adsd3500ModeSelector::updateConfigurationTable(
    DepthSensorModeDetails &configurationTable) {

    for (auto driverConf : m_adsd3500standard) {
        if (driverConf.baseWidth ==
                std::to_string(configurationTable.baseResolutionWidth) &&
            driverConf.baseHeigth ==
                std::to_string(configurationTable.baseResolutionHeight) &&
            std::stoi(driverConf.noOfPhases) ==
                configurationTable.numberOfPhases &&
            driverConf.depthBits == m_controls["depthBits"] &&
            driverConf.abBits == m_controls["abBits"] &&
            driverConf.confBits == m_controls["confBits"] &&
            driverConf.pixelFormat == m_controls["inputFormat"]) {
            configurationTable.frameWidthInBytes = driverConf.driverWidth;
            configurationTable.frameHeightInBytes = driverConf.driverHeigth;
            configurationTable.pixelFormatIndex = driverConf.pixelFormatIndex;

            return aditof::Status::OK;
        }
    }

    int depth_i = std::stoi(m_controls["depthBits"]);
    int ab_i = std::stoi(m_controls["abBits"]);
    int conf_i = std::stoi(m_controls["confBits"]);

    std::string key = make_key(depth_i, conf_i, ab_i);
    if (m_bitsPerPixelTable.find(key) == m_bitsPerPixelTable.end()) {
        LOG(ERROR) << "Invalid bits combination";
        return aditof::Status::INVALID_ARGUMENT;
    }

    int totalBits = depth_i + ab_i + conf_i;
    int width = configurationTable.baseResolutionWidth * totalBits / 8;

    int height = configurationTable.baseResolutionHeight;

    if (configurationTable.isPCM) {
        configurationTable.frameWidthInBytes =
            configurationTable.baseResolutionWidth;
        configurationTable.frameHeightInBytes =
            configurationTable.baseResolutionHeight;
        configurationTable.pixelFormatIndex = 1;

        if (configurationTable.numberOfPhases == 9) {
            configurationTable.frameWidthInBytes = 1024;
            configurationTable.frameHeightInBytes = 2880;
        }
        return aditof::Status::OK;
    }

    configurationTable.frameWidthInBytes = width;
    configurationTable.frameHeightInBytes = height;
    configurationTable.pixelFormatIndex = 0;

    return aditof::Status::OK;
}

/**
 * @brief Sets a control parameter value.
 *
 * Updates the specified control parameter with the given value. Valid controls include:
 * imagerType, mode, mixedModes, depthBits, abBits, confBits, and inputFormat.
 * When imagerType is set, automatically updates the mode table in use.
 *
 * @param[in] control Name of the control parameter to set
 * @param[in] value Value to assign to the control
 *
 * @return Status::OK if control is valid and set successfully,
 *         Status::INVALID_ARGUMENT if control name is not recognized
 */
aditof::Status Adsd3500ModeSelector::setControl(const std::string &control,
                                                const std::string &value) {
    using namespace aditof;
    Status status = Status::OK;

    if (m_controls.count(control) == 0) {
        return Status::INVALID_ARGUMENT;
    }

    m_controls[control] = value;

    //ccbm will work only with standard modes
    if (control == "imagerType") {
        if (value == imagerType.at(ImagerType::ADSD3100)) {
            m_tableInUse = adsd3100_standardModes;
        } else if (value == imagerType.at(ImagerType::ADSD3030)) {
            m_tableInUse = adsd3030_standardModes;
        } else if (value == imagerType.at(ImagerType::ADTF3080)) {
            m_tableInUse = adtf3080_standardModes;
        } else if (value == imagerType.at(ImagerType::ADTF3066)) {
            m_tableInUse = adtf3066_standardModes;
        }
    }

    return status;
}

/**
 * @brief Retrieves a control parameter value.
 *
 * Returns the current value of the specified control parameter.
 *
 * @param[in] control Name of the control parameter to retrieve
 * @param[out] value String to receive the control value
 *
 * @return Status::OK if control is valid and value returned,
 *         Status::INVALID_ARGUMENT if control name is not recognized
 */
aditof::Status Adsd3500ModeSelector::getControl(const std::string &control,
                                                std::string &value) {
    using namespace aditof;
    Status status = Status::OK;

    if (m_controls.count(control) == 0) {
        return Status::INVALID_ARGUMENT;
    }

    value = m_controls[control];

    return status;
}

/**
 * @brief Creates a lookup key string from bit configuration.
 *
 * Generates a string key in format "depth_conf_ab" for use in the bits-per-pixel
 * validation table. Used to validate whether a specific combination of depth, confidence,
 * and AB bits is supported.
 *
 * @param[in] depthbits Number of bits for depth channel
 * @param[in] confbits Number of bits for confidence channel
 * @param[in] ABbits Number of bits for AB (amplitude/brightness) channel
 *
 * @return String key in format "depth_conf_ab"
 */
std::string Adsd3500ModeSelector::make_key(int &depthbits, int &confbits,
                                           int &ABbits) {
    return std::to_string(depthbits) + "_" + std::to_string(confbits) + "_" +
           std::to_string(ABbits);
}

/**
 * @brief Initializes the bits-per-pixel validation lookup table.
 *
 * Populates m_bitsPerPixelTable with valid combinations of depth, confidence, and AB
 * bit configurations from m_validbitsperpixel. This table is used to validate custom
 * mode bit configurations.
 *
 * @return Status::OK on success
 */
aditof::Status Adsd3500ModeSelector::init_bitsPerPixelTable() {
    for (auto row : m_validbitsperpixel) {
        m_bitsPerPixelTable[make_key(row.depth_bits, row.conf_bits,
                                     row.ab_bits)] = true;
    }
    return aditof::Status::OK;
}
