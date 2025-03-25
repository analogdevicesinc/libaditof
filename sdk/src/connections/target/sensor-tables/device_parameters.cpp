/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/

#include "device_parameters.h"

aditof::Status DeviceParameters::createIniParams(
    std::vector<iniFileStruct> &iniFileStructList,
    std::vector<aditof::DepthSensorModeDetails> &modeDetailsList,
    std::string imagerType) {

    using namespace std;

    for (auto mode : modeDetailsList) {
        iniFileStruct iniF;
        iniF.imagerName = imagerType;
        iniF.modeName = std::to_string(mode.modeNumber);

        if (mode.isPCM) {
            iniF.iniKeyValPairs = adsd_PCM;
        } else if (mode.baseResolutionWidth == 1024 &&
                   mode.baseResolutionHeight == 1024) {
            iniF.iniKeyValPairs = adsd3100_partialDepth;
        } else if (imagerType == "adsd3100") {
            iniF.iniKeyValPairs = adsd3100_fullDepth;
        } else if (imagerType == "adsd3030") {
            iniF.iniKeyValPairs = adsd3030_fullDepth;
        } else if (imagerType == "adtf3080") {
            iniF.iniKeyValPairs = adtf3080_fullDepth;
        }

        iniFileStructList.emplace_back(iniF);
    }

    return aditof::Status::OK;
}