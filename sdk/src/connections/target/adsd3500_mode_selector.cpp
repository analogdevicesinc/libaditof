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

//functions to set which table configuration to use
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
        }
    }

    return aditof::Status::OK;
}

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
        }
    }

    return aditof::Status::INVALID_ARGUMENT;
}

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

//Functions used to set mode, number of bits, pixel format, etc
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
        }
    }

    return status;
}

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

std::string Adsd3500ModeSelector::make_key(int &depthbits, int &confbits,
                                           int &ABbits) {
    return std::to_string(depthbits) + "_" + std::to_string(confbits) + "_" +
           std::to_string(ABbits);
}

aditof::Status Adsd3500ModeSelector::init_bitsPerPixelTable() {
    for (auto row : m_validbitsperpixel) {
        m_bitsPerPixelTable[make_key(row.depth_bits, row.conf_bits,
                                     row.ab_bits)] = true;
    }
    return aditof::Status::OK;
}
