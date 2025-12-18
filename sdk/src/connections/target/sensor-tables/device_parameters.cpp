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
#include "device_parameters.h"

aditof::Status DeviceParameters::createIniParams(
    std::vector<iniFileStruct> &iniFileStructList,
    std::vector<aditof::DepthSensorModeDetails> &modeDetailsList,
    std::string imagerType, const uint16_t &chipID) {

    using namespace std;

    for (auto mode : modeDetailsList) {
        iniFileStruct iniF;
        iniF.imagerName = imagerType;
        iniF.modeName = std::to_string(mode.modeNumber);

        if (mode.isPCM) {
            iniF.iniKeyValPairs = adsd_PCM;
        } else if (mode.baseResolutionWidth == 1024 &&
                   mode.baseResolutionHeight == 1024 &&
                   chipID == CHIP_ID_SINGLE) {
            iniF.iniKeyValPairs = adsd3100_partialDepth;
        } else if (mode.baseResolutionWidth == 1024 &&
                   mode.baseResolutionHeight == 1024 &&
                   chipID != CHIP_ID_SINGLE) {
            iniF.iniKeyValPairs = adsd3100_dual_fullDepth;
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