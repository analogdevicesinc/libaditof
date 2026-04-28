/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 */
#include "depth_parameter_mapper.h"
#include "camera_itof.h"
#include <aditof/log.h>

using namespace aditof;

DepthParameterMapper::DepthParameterMapper(CameraItof *camera)
    : m_camera(camera) {
    buildParameterTable();
}

void DepthParameterMapper::buildParameterTable() {
    // Build parameter mapping table
    m_paramTable = {
        {"abThreshMin",
         [this](const std::string &val) {
             return m_camera->adsd3500SetABinvalidationThreshold(
                 std::stoi(val));
         },
         false},

        {"confThresh",
         [this](const std::string &val) {
             return m_camera->adsd3500SetConfidenceThreshold(std::stoi(val));
         },
         false},

        {"radialThreshMin",
         [this](const std::string &val) {
             return m_camera->adsd3500SetRadialThresholdMin(std::stoi(val));
         },
         false},

        {"radialThreshMax",
         [this](const std::string &val) {
             return m_camera->adsd3500SetRadialThresholdMax(std::stoi(val));
         },
         false},

        {"jblfWindowSize",
         [this](const std::string &val) {
             return m_camera->adsd3500SetJBLFfilterSize(std::stoi(val));
         },
         false},

        {"jblfApplyFlag",
         [this](const std::string &val) {
             bool en = !(val == "0");
             return m_camera->adsd3500SetJBLFfilterEnableState(en);
         },
         false},

        {"fps",
         [this](const std::string &val) {
             return m_camera->adsd3500SetFrameRate(std::stoi(val));
         },
         false},

        {"vcselDelay",
         [this](const std::string &val) {
             return m_camera->adsd3500SetVCSELDelay(
                 static_cast<uint16_t>(std::stoi(val)));
         },
         false},

        {"jblfMaxEdge",
         [this](const std::string &val) {
             return m_camera->adsd3500SetJBLFMaxEdgeThreshold(
                 static_cast<uint16_t>(std::stoi(val)));
         },
         false},

        {"jblfABThreshold",
         [this](const std::string &val) {
             return m_camera->adsd3500SetJBLFABThreshold(
                 static_cast<uint16_t>(std::stoi(val)));
         },
         false},

        {"jblfGaussianSigma",
         [this](const std::string &val) {
             return m_camera->adsd3500SetJBLFGaussianSigma(
                 static_cast<uint16_t>(std::stoi(val)));
         },
         false},

        {"jblfExponentialTerm",
         [this](const std::string &val) {
             return m_camera->adsd3500SetJBLFExponentialTerm(
                 static_cast<uint16_t>(std::stoi(val)));
         },
         false},

        {"enablePhaseInvalidation",
         [this](const std::string &val) {
             return m_camera->adsd3500SetEnablePhaseInvalidation(
                 static_cast<uint16_t>(std::stoi(val)));
         },
         false},
    };
}

Status DepthParameterMapper::applyParameters(
    const std::map<std::string, std::string> &iniKeyValPairs,
    bool updateDepthMap,
    std::function<void(bool, const char *, std::string)> updateCallback) {

    // Apply each parameter from the table
    for (const auto &param : m_paramTable) {
        auto it = iniKeyValPairs.find(param.key);

        if (it != iniKeyValPairs.end()) {
            // Parameter found - apply it
            Status status = param.setter(it->second);
            if (status != Status::OK) {
                LOG(WARNING) << "Could not set " << param.key;
            }

            // Update depth params map if requested
            if (updateCallback) {
                updateCallback(updateDepthMap, param.key, it->second);
            }
        } else if (param.required) {
            // Required parameter missing - log error
            LOG(ERROR) << param.key << " is required but not found";
        } else {
            // Optional parameter missing - log warning
            LOG(WARNING) << param.key
                         << " was not found in parameter list, not setting.";
        }
    }

    return Status::OK;
}
