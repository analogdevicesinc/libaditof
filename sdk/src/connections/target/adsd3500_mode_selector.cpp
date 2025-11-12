/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
    if (std::find(m_availableConfigurations.begin(),
                  m_availableConfigurations.end(),
                  configuration) != m_availableConfigurations.end()) {
        m_configuration = configuration;
        return aditof::Status::OK;
    } else {
        return aditof::Status::INVALID_ARGUMENT;
    }
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

        //TO DO: hardcoded for now for the 9pcm usecase
        if (configurationTable.numberOfPhases == 9) {
            configurationTable.frameWidthInBytes = 1024;
            configurationTable.frameHeightInBytes = 2880;
        }
        return aditof::Status::OK;
    }
    if (m_controls["imagerType"] == imagerType.at(ImagerType::ADTF3080)) {

        configurationTable.frameWidthInBytes =
            configurationTable.baseResolutionWidth * totalBits / 8;
        configurationTable.frameHeightInBytes =
            configurationTable.baseResolutionHeight;
        configurationTable.pixelFormatIndex = 0;
        return aditof::Status::OK;

    } else {

        if ((configurationTable.modeNumber == 2 ||
             configurationTable.modeNumber == 3 ||
             configurationTable.modeNumber == 5 ||
             configurationTable.modeNumber == 6) &&
            m_controls["imagerType"] == imagerType.at(ImagerType::ADSD3100)) {
            height = 512;
        } else if ((configurationTable.modeNumber == 0 ||
                    configurationTable.modeNumber == 1) &&
                   m_controls["imagerType"] ==
                       imagerType.at(ImagerType::ADSD3030)) {
            height = 640;
        } else if (configurationTable.modeNumber == 7 &&
                   m_controls["imagerType"] ==
                       imagerType.at(ImagerType::ADSD3030)) {
            width = 2560;
            height = 640;
        } else if (configurationTable.modeNumber >= 2 &&
                   m_controls["imagerType"] ==
                       imagerType.at(ImagerType::ADSD3030)) {
            configurationTable.frameWidthInBytes = 1280;
            configurationTable.frameHeightInBytes = 320;
            configurationTable.pixelFormatIndex = 0;
            return aditof::Status::OK;
        } else if ((configurationTable.modeNumber < 0 ||
                    configurationTable.modeNumber > 6) &&
                   m_controls["imagerType"] !=
                       imagerType.at(ImagerType::ADSD3100) &&
                   m_controls["imagerType"] !=
                       imagerType.at(ImagerType::ADSD3030)) {
            return aditof::Status::INVALID_ARGUMENT;
        }
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
