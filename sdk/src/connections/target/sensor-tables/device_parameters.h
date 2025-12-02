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
    {"xyzEnable", "0"},
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
                                                 {"bitsInConf", "0"},
                                                 {"bitsInAB", "8"},
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

#endif
