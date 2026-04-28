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
#include "ini_config_manager.h"
#include "tofi/tofi_config.h"
#include <aditof/log.h>
#include <algorithm>

// IniTableEntry structure from adsd3500_sensor.h
struct IniTableEntry {
    uint8_t INIIndex;
    uint8_t rsvd;
    uint16_t abThreshMin;
    uint16_t confThresh;
    uint16_t radialThreshMin;
    uint16_t radialThreshMax;
    uint16_t jblfApplyFlag;
    uint16_t jblfWindowSize;
    uint16_t jblfGaussianSigma;
    uint16_t jblfExponentialTerm;
    uint16_t jblfMaxEdge;
    uint16_t jblfABThreshold;
    uint16_t spare0;
    uint16_t spare1;
    uint16_t spare2;
    uint16_t spare3;
    uint16_t spare4;
    uint16_t spare5;
    uint16_t spare6;
    uint16_t spare7;
    uint16_t spare8;
    uint16_t modeNumber;
};

// SensorImagerType enum from adsd3500_sensor.cpp
enum class SensorImagerType {
    IMAGER_UNKNOWN,
    IMAGER_ADSD3100,
    IMAGER_ADSD3030,
    IMAGER_ADTF3080,
    IMAGER_ADTF3066
};

IniConfigManager::IniConfigManager(
    std::vector<iniFileStruct> &iniFileStructList,
    std::vector<IniTableEntry> &ccbmINIContent, bool &ccbmEnabled,
    SensorImagerType &imagerType)
    : m_iniFileStructList(iniFileStructList), m_ccbmINIContent(ccbmINIContent),
      m_ccbmEnabled(ccbmEnabled), m_imagerType(imagerType) {}

aditof::Status
IniConfigManager::getIniParamsImpl(void *p_config_params, int params_group,
                                   const void *p_tofi_cal_config) {
    using namespace aditof;
    Status status = Status::OK;
    uint32_t ret;
    ret = TofiGetINIParams(p_config_params, params_group, p_tofi_cal_config);
    status = static_cast<Status>(ret);

    if (status != Status::OK) {
        LOG(ERROR) << "Failed getting ini parameters";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status
IniConfigManager::setIniParamsImpl(void *p_config_params, int params_group,
                                   const void *p_tofi_cal_config) {
    using namespace aditof;
    Status status = Status::OK;
    uint32_t ret;
    ret = TofiSetINIParams(p_config_params, params_group, p_tofi_cal_config);
    status = static_cast<Status>(ret);

    if (status != Status::OK) {
        LOG(ERROR) << "Failed setting ini parameters";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status IniConfigManager::getDefaultIniParamsForMode(
    const std::string &imager, const std::string &mode,
    std::map<std::string, std::string> &params) {

    auto it = std::find_if(
        m_iniFileStructList.begin(), m_iniFileStructList.end(),
        [&imager, &mode](const iniFileStruct &iniF) {
            return (iniF.imagerName == imager && iniF.modeName == mode);
        });

    if (it == m_iniFileStructList.end()) {
        LOG(WARNING) << "Cannot find default parameters for imager: " << imager
                     << " and mode: " << mode;
        return aditof::Status::INVALID_ARGUMENT;
    }

    params = it->iniKeyValPairs;

    return aditof::Status::OK;
}

aditof::Status IniConfigManager::mergeIniParams(
    std::vector<iniFileStruct> &iniFileStructList) {

    using namespace std;
    using namespace aditof;

    if (m_ccbmEnabled) {

        for (auto &ccbmParams : m_ccbmINIContent) {

            for (auto &iniList : iniFileStructList) {

                if (iniList.modeName != "") {
                    if (ccbmParams.modeNumber ==
                        std::stoi(iniList.modeName.c_str())) {

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

                        break;
                    }
                }
            }
        }
    }

    return Status::OK;
}

aditof::Status IniConfigManager::convertIniParams(iniFileStruct &iniStruct,
                                                  std::string &inistr) {

    inistr = "";
    for (auto iniPairs : iniStruct.iniKeyValPairs) {
        inistr += iniPairs.first + "=" + iniPairs.second + "\n";
    }

    return aditof::Status::OK;
}

aditof::Status IniConfigManager::getIniParamsArrayForMode(int mode,
                                                          std::string &iniStr) {
    std::string modestr = std::to_string(mode);
    std::string imager = "adsd3030";
    if (m_imagerType == SensorImagerType::IMAGER_ADSD3100) {
        imager = "adsd3100";
    } else if (m_imagerType == SensorImagerType::IMAGER_ADTF3080) {
        imager = "adtf3080";
    } else if (m_imagerType == SensorImagerType::IMAGER_ADTF3066) {
        imager = "adtf3066";
    }

    auto it = std::find_if(
        m_iniFileStructList.begin(), m_iniFileStructList.end(),
        [&imager, &modestr](const iniFileStruct &iniF) {
            return (iniF.imagerName == imager && iniF.modeName == modestr);
        });

    if (it == m_iniFileStructList.end()) {
        LOG(WARNING) << "Cannot find default parameters for imager: " << imager
                     << " and mode: " << mode;
        return aditof::Status::INVALID_ARGUMENT;
    }

    convertIniParams(*it, iniStr);

    return aditof::Status::OK;
}
