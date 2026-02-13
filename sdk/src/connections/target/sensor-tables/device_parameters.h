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

using namespace std;

static map<string, string> adsd3100_partialDepth = {
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
    {"headerSize", "128"},
    {"inputFormat", "mipiRaw12_8"},
    {"depthComputeIspEnable", "1"},
    {"partialDepthEnable", "1"},
    {"interleavingEnable", "0"},
    {"bitsInPhaseOrDepth", "12"},
    {"bitsInConf", "0"},
    {"bitsInAB", "16"},
    {"phaseInvalid", "0"},
    {"xyzEnable", "1"},
    {"fps", "10"}};

static map<string, string> adsd3100_dual_fullDepth = {
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
    {"inputFormat", "mipiRaw12_8"},
    {"depthComputeIspEnable", "1"},
    {"partialDepthEnable", "1"},
    {"interleavingEnable", "0"},
    {"bitsInPhaseOrDepth", "16"},
    {"bitsInConf", "0"},
    {"bitsInAB", "16"},
    {"phaseInvalid", "0"},
    {"xyzEnable", "1"},
    {"fps", "10"}};

static map<string, string> adsd3100_fullDepth = {{"abThreshMin", "3.0"},
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
                                                 {"inputFormat", "raw8"},
                                                 {"depthComputeIspEnable", "1"},
                                                 {"partialDepthEnable", "0"},
                                                 {"interleavingEnable", "1"},
                                                 {"bitsInPhaseOrDepth", "16"},
                                                 {"bitsInConf", "8"},
                                                 {"bitsInAB", "16"},
                                                 {"phaseInvalid", "0"},
                                                 {"xyzEnable", "1"},
                                                 {"fps", "40"}};

static map<string, string> adsd_PCM = {{"abThreshMin", "3.0"},
                                       {"confThresh", "25.0"},
                                       {"radialThreshMin", "100.0"},
                                       {"radialThreshMax", "10000.0"},
                                       {"jblfApplyFlag", "1"},
                                       {"jblfWindowSize", "7"},
                                       {"jblfGaussianSigma", "10.0"},
                                       {"jblfExponentialTerm", "5.0"},
                                       {"jblfMaxEdge", "12.0"},
                                       {"jblfABThreshold", "10.0"},
                                       {"headerSize", "0"},
                                       {"inputFormat", "mipiRaw12_8"},
                                       {"depthComputeIspEnable", "0"},
                                       {"partialDepthEnable", "0"},
                                       {"interleavingEnable", "0"},
                                       {"bitsInPhaseOrDepth", "0"},
                                       {"bitsInConf", "0"},
                                       {"bitsInAB", "0"},
                                       {"phaseInvalid", "0"},
                                       {"xyzEnable", "0"},
                                       {"fps", "15"}};

static map<string, string> adsd3030_fullDepth = {{"abThreshMin", "3.0"},
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
                                                 {"inputFormat", "raw8"},
                                                 {"depthComputeIspEnable", "1"},
                                                 {"partialDepthEnable", "0"},
                                                 {"interleavingEnable", "1"},
                                                 {"bitsInPhaseOrDepth", "16"},
                                                 {"bitsInConf", "8"},
                                                 {"bitsInAB", "16"},
                                                 {"phaseInvalid", "0"},
                                                 {"xyzEnable", "1"},
                                                 {"fps", "40"}};

static map<string, string> adtf3080_fullDepth = {{"abThreshMin", "3.0"},
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
                                                 {"inputFormat", "raw8"},
                                                 {"depthComputeIspEnable", "1"},
                                                 {"partialDepthEnable", "0"},
                                                 {"interleavingEnable", "1"},
                                                 {"bitsInPhaseOrDepth", "16"},
                                                 {"bitsInConf", "8"},
                                                 {"bitsInAB", "16"},
                                                 {"phaseInvalid", "0"},
                                                 {"xyzEnable", "1"},
                                                 {"fps", "40"}};

/**
 * @brief Default depth computation parameters for ADTF3066 imager.
 * 
 * Contains 21 key-value pairs for ISP configuration including JBLF filtering,
 * thresholds, and output format settings. Cloned from ADTF3080 configuration.
 */
static map<string, string> adtf3066_fullDepth = {{"abThreshMin", "3.0"},
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
                                                 {"inputFormat", "raw8"},
                                                 {"depthComputeIspEnable", "1"},
                                                 {"partialDepthEnable", "0"},
                                                 {"interleavingEnable", "1"},
                                                 {"bitsInPhaseOrDepth", "16"},
                                                 {"bitsInConf", "8"},
                                                 {"bitsInAB", "16"},
                                                 {"phaseInvalid", "0"},
                                                 {"xyzEnable", "1"},
                                                 {"fps", "40"}};

#endif
