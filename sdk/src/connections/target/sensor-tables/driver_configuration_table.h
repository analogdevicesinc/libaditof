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
#ifndef DRIVER_CONFIGURATION_TABLE
#define DRIVER_CONFIGURATION_TABLE

#include <aditof/depth_sensor_interface.h>
#include <aditof/sensor_definitions.h>
#include <vector>

#define MP_BASE_HEIGHT 1024
#define MP_BASE_WIDTH 1024
#define QMP_BASE_HEIGHT 512
#define QMP_BASE_WIDTH 512
#define VGA_BASE_HEIGHT 640
#define VGA_BASE_WIDTH 512
#define QVGA_BASE_HEIGHT 320
#define QVGA_BASE_WIDTH 256

using namespace aditof;

const std::vector<DriverConfiguration> m_adsd3500standard = {
    /* imagerType  mode depth  ab   conf  pixelF dWidth dHeight pixFIndex
   sr-native */
    {"1024", "1024", "2", "16", "16", "0", "raw16", 2048, 3072, 0},
    {"1024", "1024", "2", "12", "12", "0", "raw16_bits12_shift4", 1024, 3072,
     1},
    {"1024", "1024", "2", "12", "0", "0", "raw16_bits12_shift4", 1024, 1024, 1},
    {"1024", "1024", "2", "12", "16", "0", "mipiRaw12_8", 2048, 2560, 0},
    {"1024", "1024", "3", "16", "16", "0", "raw16", 2048, 3072, 0},
    {"1024", "1024", "3", "12", "12", "0", "raw16_bits12_shift4", 1024, 4096,
     1},
    {"1024", "1024", "3", "12", "0", "0", "raw16_bits12_shift4", 1024, 1024, 1},
    {"1024", "1024", "3", "12", "16", "0", "mipiRaw12_8", 2048, 3328, 0},
    {"1024", "1024", "3", "16", "16", "0", "mipiRaw12_8", 1024, 4096, 0},
    {"1024", "1024", "2", "16", "16", "0", "mipiRaw12_8", 1024, 4096, 0},
    {"1024", "1024", "2", "16", "0", "0", "mipiRaw12_8", 1024, 4096, 0},
};

// All supported valid bitsPerPixel combination : {bitsInDepth, bitsInConf, bitsInAB}
const std::vector<BitsConfiguration> m_validbitsperpixel = {
    /*depth confidence AB*/
    {16, 8, 16}, {16, 8, 8},  {16, 8, 0}, {16, 0, 16}, {16, 0, 8},
    {16, 0, 0},  {12, 4, 16}, {12, 4, 8}, {12, 4, 0}};

const std::vector<DepthSensorModeDetails> adsd3100_standardModes = {
    {0, {}, 2, 0, 0, 0, 1024, 1024, 128, 0, 0, DriverConfiguration()},
    {1, {}, 3, 0, 0, 0, 1024, 1024, 128, 0, 0, DriverConfiguration()},
    {4, {}, 1, 0, 0, 0, 1024, 1024, 128, 1, 0, DriverConfiguration()},
    {2, {}, 2, 0, 0, 0, 512, 512, 128, 0, 0, DriverConfiguration()},
    {3, {}, 3, 0, 0, 0, 512, 512, 128, 0, 0, DriverConfiguration()},
    {6, {}, 2, 0, 0, 0, 512, 512, 128, 0, 0, DriverConfiguration()},
    {5, {}, 3, 0, 0, 0, 512, 512, 128, 0, 0, DriverConfiguration()}};

const std::vector<DepthSensorModeDetails> adsd3030_standardModes = {
    {0, {}, 2, 0, 0, 0, 512, 640, 128, 0, 0, DriverConfiguration()},
    {1, {}, 3, 0, 0, 0, 512, 640, 128, 0, 0, DriverConfiguration()},
    {4, {}, 1, 0, 0, 0, 512, 640, 128, 1, 0, DriverConfiguration()},
    {2, {}, 2, 0, 0, 0, 256, 320, 128, 0, 0, DriverConfiguration()},
    {3, {}, 3, 0, 0, 0, 256, 320, 128, 0, 0, DriverConfiguration()},
    {6, {}, 2, 0, 0, 0, 256, 320, 128, 0, 0, DriverConfiguration()},
    {5, {}, 3, 0, 0, 0, 256, 320, 128, 0, 0, DriverConfiguration()}};

const std::vector<DepthSensorModeDetails> adtf3080_standardModes = {
    {0, {}, 2, 0, 0, 0, 512, 640, 128, 0, 0, DriverConfiguration()},
    {1, {}, 3, 0, 0, 0, 512, 640, 128, 0, 0, DriverConfiguration()},
    {4, {}, 1, 0, 0, 0, 512, 640, 128, 1, 0, DriverConfiguration()},
    {7, {}, 2, 0, 0, 0, 512, 640, 128, 0, 0, DriverConfiguration()},
    {3, {}, 3, 0, 0, 0, 256, 320, 128, 0, 0, DriverConfiguration()},
    {6, {}, 2, 0, 0, 0, 256, 320, 128, 0, 0, DriverConfiguration()},
    {5, {}, 3, 0, 0, 0, 256, 320, 128, 0, 0, DriverConfiguration()},
    {8, {}, 2, 0, 0, 0, 256, 320, 128, 0, 0, DriverConfiguration()},
    {9, {}, 2, 0, 0, 0, 256, 320, 128, 0, 0, DriverConfiguration()}};

/**
 * @brief Standard mode configurations for ADTF3066 imager.
 * 
 * Defines 9 operating modes (0-9) cloned from ADTF3080 configuration.
 * Each mode specifies resolution, phase count, and driver parameters.
 */
const std::vector<DepthSensorModeDetails> adtf3066_standardModes = {
    {0, {}, 2, 0, 0, 0, 512, 640, 128, 0, 0, DriverConfiguration()},
    {1, {}, 3, 0, 0, 0, 512, 640, 128, 0, 0, DriverConfiguration()},
    {4, {}, 1, 0, 0, 0, 512, 640, 128, 1, 0, DriverConfiguration()},
    {7, {}, 2, 0, 0, 0, 512, 640, 128, 0, 0, DriverConfiguration()},
    {3, {}, 3, 0, 0, 0, 256, 320, 128, 0, 0, DriverConfiguration()},
    {6, {}, 2, 0, 0, 0, 256, 320, 128, 0, 0, DriverConfiguration()},
    {5, {}, 3, 0, 0, 0, 256, 320, 128, 0, 0, DriverConfiguration()},
    {8, {}, 2, 0, 0, 0, 256, 320, 128, 0, 0, DriverConfiguration()},
    {9, {}, 2, 0, 0, 0, 256, 320, 128, 0, 0, DriverConfiguration()}};

/**
 * @brief Raw bypass driver configuration table for ADSD3500 with ADTF3175D.
 * 
 * Used when rawBypassMode=1 to capture unprocessed raw Bayer sensor data.
 * All depth/AB/conf bits set to "0", pixel format is "RG12".
 * ISP outputs raw frames with full buffer dimensions as single frame.
 * No phase-based ToF computation - phases set to "1" (single frame capture).
 * 
 * Format: {baseWidth, baseHeight, phases, depthBits, abBits, confBits, pixelFormat, driverWidth, driverHeight, pixelFormatIndex}
 */
const std::vector<DriverConfiguration> m_adsd3500rawBypass = {
    /* Mode 0: MP raw frame 2048×3072 */
    {"1024", "1024", "1", "0", "0", "0", "RG12", 2048, 3072, 2},

    /* Mode 1: MP raw frame 2048×3072 */
    {"1024", "1024", "1", "0", "0", "0", "RG12", 2048, 3072, 2},

    /* Mode 2: QMP raw frame 1024×4608 */
    {"512", "512", "1", "0", "0", "0", "RG12", 1024, 4608, 2},

    /* Mode 3: QMP raw frame 1024×4608 */
    {"512", "512", "1", "0", "0", "0", "RG12", 1024, 4608, 2},

    /* Mode 4: QMP raw frame 1024×4608 */
    {"512", "512", "1", "0", "0", "0", "RG12", 1024, 4608, 2},

    /* Mode 5: QMP raw frame 1024×4608 */
    {"512", "512", "1", "0", "0", "0", "RG12", 1024, 4608, 2},

    /* Mode 6: QMP raw frame 1024×4608 */
    {"512", "512", "1", "0", "0", "0", "RG12", 1024, 4608, 2},
};

#endif
