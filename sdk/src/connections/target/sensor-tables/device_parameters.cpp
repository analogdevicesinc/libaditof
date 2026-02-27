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

/**
 * @brief Creates INI parameter structures for all available sensor modes.
 *
 * Populates a list of INI file structures with default parameters for each mode based on
 * the imager type, resolution, chip ID, and whether it's a PCM mode. Selects appropriate
 * parameter sets for different hardware configurations (single vs dual ISP, full vs partial depth).
 *
 * @param[out] iniFileStructList Vector to be populated with INI file structures for each mode
 * @param[in] modeDetailsList Vector of mode details describing available sensor modes
 * @param[in] imagerType Imager type identifier ("adsd3100", "adsd3030", "adtf3080", or "adtf3066")
 * @param[in] chipID Chip identifier to differentiate single vs dual ISP configurations
 *
 * @return Status::OK on success
 */
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
        } else if (imagerType == "adtf3066") {
            iniF.iniKeyValPairs = adtf3066_fullDepth;
        }

        iniFileStructList.emplace_back(iniF);
    }

    return aditof::Status::OK;
}