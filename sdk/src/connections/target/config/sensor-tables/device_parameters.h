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
#ifndef DEVICE_PARAMETERS_H
#define DEVICE_PARAMETERS_H

#include "ini_file_definitions.h"
#include <aditof/sensor_definitions.h>
#include <aditof/status_definitions.h>
#include <vector>

#define CHIP_ID_SINGLE 0x5931

class DeviceParameters {
  public:
    static aditof::Status createIniParams(
        std::vector<iniFileStruct> &iniFileStructList,
        std::vector<aditof::DepthSensorModeDetails> &modeDetailsList,
        std::string imagerType, const uint16_t &chipID);
};

static std::map<std::string, std::string> adsd3100_partialDepth = {
    {"abThreshMin", "3.0"},
    {"confThresh", "25.0"},
    {"radialThreshMin", "30.0"},
    {"radialThreshMax", "4200.0"},
    {"jblfApplyFlag", "1"},
    {"jblfWindowSize", "7"},
    {"jblfGaussianSigma", "10.0"},
    {"jblfExponentialTerm", "5.0"},
    {"jblfMaxEdge", "12.0"},
    {"jblfABThreshold", "10.0"},
    {"lensScatterCompensationEnabled", "0"},
    {"headerSize", "128"},
    {"inputFormat", "mipiRaw12_8"},
    {"depthComputeIspEnable", "1"},
    {"partialDepthEnable", "1"},
    {"interleavingEnable", "0"},
    {"bitsInPhaseOrDepth", "12"},
    {"dualPulsatrixSystemEnabled", "0"},
    {"bitsInConf", "0"},
    {"bitsInAB", "16"},
    {"abAveraging", "0"},
    {"phaseInvalid", "0"},
    {"enableRotation", "0"},
    {"xyzEnable", "1"},
    {"fps", "10"},
    {"rgbCameraEnable", "1"}};

static std::map<std::string, std::string> adsd3100_dual_fullDepth = {
    {"abThreshMin", "3.0"},
    {"confThresh", "25.0"},
    {"radialThreshMin", "100.0"},
    {"radialThreshMax", "10000.0"},
    {"jblfApplyFlag", "1"},
    {"jblfWindowSize", "7"},
    {"jblfGaussianSigma", "10.0"},
    {"jblfExponentialTerm", "5.0"},
    {"jblfMaxEdge", "12.0"},
    {"jblfABThreshold", "10.0"},
    {"headerSize", "128"},
    {"deltaCompEnable", "0"},
    {"inputFormat", "raw16_bits11_shift4"},
    {"depthComputeIspEnable", "1"},
    {"partialDepthEnable", "0"},
    {"interleavingEnable", "1"},
    {"bitsInPhaseOrDepth", "16"},
    {"bitsInConf", "8"},
    {"bitsInAB", "16"},
    {"phaseInvalid", "0"},
    {"abAveraging", "0"},
    {"dualPulsatrixSystemEnabled", "1"},
    {"lensScatterCompensationEnabled", "0"},
    {"enableRotation", "0"},
    {"xyzEnable", "1"},
    {"withABFrame", "1"},
    {"fps", "10"},
    {"rgbCameraEnable", "1"}};

static std::map<std::string, std::string> adsd3100_fullDepth = {
    {"abThreshMin", "3.0"},
    {"confThresh", "25.0"},
    {"radialThreshMin", "100.0"},
    {"radialThreshMax", "10000.0"},
    {"jblfApplyFlag", "1"},
    {"jblfWindowSize", "7"},
    {"jblfGaussianSigma", "10.0"},
    {"jblfExponentialTerm", "5.0"},
    {"jblfMaxEdge", "12.0"},
    {"jblfABThreshold", "10.0"},
    {"lensScatterCompensationEnabled", "0"},
    {"headerSize", "128"},
    {"inputFormat", "raw8"},
    {"depthComputeIspEnable", "1"},
    {"partialDepthEnable", "0"},
    {"interleavingEnable", "1"},
    {"bitsInPhaseOrDepth", "16"},
    {"dualPulsatrixSystemEnabled", "1"},
    {"bitsInConf", "8"},
    {"bitsInAB", "16"},
    {"abAveraging", "1"},
    {"phaseInvalid", "0"},
    {"enableRotation", "0"},
    {"xyzEnable", "1"},
    {"fps", "40"},
    {"rgbCameraEnable", "1"}};

static std::map<std::string, std::string> adsd_PCM = {
    {"abThreshMin", "3.0"},
    {"confThresh", "25.0"},
    {"radialThreshMin", "100.0"},
    {"radialThreshMax", "10000.0"},
    {"jblfApplyFlag", "1"},
    {"jblfWindowSize", "7"},
    {"jblfGaussianSigma", "10.0"},
    {"jblfExponentialTerm", "5.0"},
    {"jblfMaxEdge", "12.0"},
    {"jblfABThreshold", "10.0"},
    {"lensScatterCompensationEnabled", "0"},
    {"headerSize", "0"},
    {"inputFormat", "mipiRaw12_8"},
    {"depthComputeIspEnable", "0"},
    {"partialDepthEnable", "0"},
    {"interleavingEnable", "0"},
    {"bitsInPhaseOrDepth", "0"},
    {"dualPulsatrixSystemEnabled", "1"},
    {"bitsInConf", "0"},
    {"bitsInAB", "0"},
    {"abAveraging", "0"},
    {"phaseInvalid", "0"},
    {"enableRotation", "0"},
    {"xyzEnable", "0"},
    {"fps", "15"},
    {"rgbCameraEnable", "1"}};

static std::map<std::string, std::string> adsd3030_fullDepth = {
    {"abThreshMin", "3.0"},
    {"confThresh", "25.0"},
    {"radialThreshMin", "100.0"},
    {"radialThreshMax", "10000.0"},
    {"jblfApplyFlag", "1"},
    {"jblfWindowSize", "7"},
    {"jblfGaussianSigma", "10.0"},
    {"jblfExponentialTerm", "5.0"},
    {"jblfMaxEdge", "12.0"},
    {"jblfABThreshold", "10.0"},
    {"lensScatterCompensationEnabled", "0"},
    {"headerSize", "128"},
    {"inputFormat", "raw8"},
    {"depthComputeIspEnable", "1"},
    {"partialDepthEnable", "0"},
    {"interleavingEnable", "1"},
    {"bitsInPhaseOrDepth", "16"},
    {"dualPulsatrixSystemEnabled", "1"},
    {"bitsInConf", "8"},
    {"bitsInAB", "16"},
    {"abAveraging", "1"},
    {"phaseInvalid", "0"},
    {"enableRotation", "0"},
    {"xyzEnable", "1"},
    {"fps", "40"},
    {"rgbCameraEnable", "1"}};

static std::map<std::string, std::string> adtf3080_fullDepth = {
    {"abThreshMin", "3.0"},
    {"confThresh", "25.0"},
    {"radialThreshMin", "100.0"},
    {"radialThreshMax", "10000.0"},
    {"jblfApplyFlag", "1"},
    {"jblfWindowSize", "7"},
    {"jblfGaussianSigma", "10.0"},
    {"jblfExponentialTerm", "5.0"},
    {"jblfMaxEdge", "12.0"},
    {"jblfABThreshold", "10.0"},
    {"lensScatterCompensationEnabled", "0"},
    {"headerSize", "128"},
    {"inputFormat", "raw8"},
    {"depthComputeIspEnable", "1"},
    {"partialDepthEnable", "0"},
    {"interleavingEnable", "1"},
    {"bitsInPhaseOrDepth", "16"},
    {"dualPulsatrixSystemEnabled", "1"},
    {"bitsInConf", "8"},
    {"bitsInAB", "16"},
    {"abAveraging", "1"},
    {"phaseInvalid", "0"},
    {"enableRotation", "0"},
    {"xyzEnable", "1"},
    {"fps", "40"},
    {"rgbCameraEnable", "1"}};

/**
 * @brief Default depth computation parameters for ADTF3066 imager.
 */
static std::map<std::string, std::string> adtf3066_fullDepth = {
    {"abThreshMin", "10.0"},
    {"abSumThresh", "25.0"},
    {"confThresh", "25.0"},
    {"radialThreshMin", "100.0"},
    {"radialThreshMax", "10000.0"},
    {"jblfApplyFlag", "1"},
    {"jblfWindowSize", "7"},
    {"jblfGaussianSigma", "10.0"},
    {"jblfExponentialTerm", "5.0"},
    {"jblfMaxEdge", "12.0"},
    {"jblfABThreshold", "10.0"},
    {"lensScatterCompensationEnabled", "0"},
    {"headerSize", "128"},
    {"numMetaDataBytes", "1280"},
    {"inputFormat", "raw8"},
    {"depthComputeIspEnable", "1"},
    {"partialDepthEnable", "0"},
    {"interleavingEnable", "1"},
    {"residualUnwrappingEnabled", "0"},
    {"defaultdepthOutput", "0"},
    {"sobelEdgeDetectionEnabled", "0"},
    {"modeFusionEnabled", "0"},
    {"IsSRFrameFirst", "0"},
    {"srlrFusionEnabled", "1"},
    {"ABFusionEnabled", "0"},
    {"lrInvalEnabled", "0"},
    {"lrInvalType", "1"},
    {"lrInvalThreshold", "150"},
    {"lrInvalRadialMin", "100"},
    {"lrInvalRadialMax", "7500"},
    {"twoLRFramesFusionEnabled", "0"},
    {"twoLRFramesThreshold", "100"},
    {"frameAOutputEnabled", "0"},
    {"edgeConfidenceEnabled", "0"},
    {"bitsInPhaseOrDepth", "16"},
    {"dualPulsatrixSystemEnabled", "1"},
    {"bitsInConf", "8"},
    {"bitsInAB", "16"},
    {"abAveraging", "1"},
    {"phaseInvalid", "0"},
    {"enableRotation", "0"},
    {"xyzEnable", "1"},
    {"fps", "30"},
    {"rgbCameraEnable", "1"}};

/**
 * @brief ADTF3066 depth parameters overridden by lens scatter compensation.
 *
 * Applied over adtf3066_fullDepth by Camera::setLensScatterCompensation():
 * raw bypass with host CUDA depth compute, parameters from
 * TOF_DepthComputeEngine_CUDA-Rel5.0.8-LS RawToDepth_LS.ini.
 */
static const std::map<std::string, std::string> adtf3066_lensScatter = {
    {"lensScatterCompensationEnabled", "1"},
    {"headerSize", "0"},
    {"inputFormat", "raw16"},
    {"depthComputeIspEnable", "0"},
    {"interleavingEnable", "0"},
    {"bitsInConf", "0"},
    {"withABFrame", "1"},
    {"abFilterEnable", "0"},
    {"deltaCompEnable", "0"},
    {"sigma_x0", "10.0"},
    {"sigma_x1", "30.0"},
    {"sigma_x2", "162.0"},
    {"k0", "1.1327"},
    {"k1", "1.0"},
    {"k2", "1.0"},
    {"weight0", "0.1970"},
    {"weight1", "0.0186"},
    {"weight2", "0.0122"},
    {"lp0", "1.0"},
    {"lp1", "1.0"},
    {"lp2", "2.0"},
    {"jblfv2_window_size", "4"},
    {"sga", "30"}};

#endif
