/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 */
#include "sensor_config_helper.h"
#include "../managers/camera_configuration.h"
#include <aditof/log.h>

using namespace aditof;

SensorConfigHelper::SensorConfigHelper(
    std::shared_ptr<DepthSensorInterface> sensor, CameraConfiguration *config)
    : m_depthSensor(sensor), m_config(config) {}

std::string
SensorConfigHelper::convertBitsToSensorValue(const std::string &bits,
                                             bool isDepthOrAB) {
    if (isDepthOrAB) {
        // Depth or AB conversion
        if (bits == "16")
            return "6";
        else if (bits == "14")
            return "5";
        else if (bits == "12")
            return "4";
        else if (bits == "10")
            return "3";
        else if (bits == "8")
            return "2";
        else
            return "0";
    } else {
        // Confidence conversion
        if (bits == "8")
            return "2";
        else if (bits == "4")
            return "1";
        else
            return "0";
    }
}

Status SensorConfigHelper::configureModeDetails(
    bool isOffline, DepthSensorModeDetails &modeDetailsCache,
    bool &depthEnabled, bool &abEnabled, bool &confEnabled, bool &xyzEnabled,
    bool xyzSetViaApi, uint8_t &depthBitsPerPixel, uint8_t &abBitsPerPixel,
    uint8_t &confBitsPerPixel) {

    if (isOffline) {
        // Set enable flags based on what was actually recorded
        LOG(INFO) << "[PLAYBACK] Setting enable flags based on frameContent...";
        depthEnabled = false;
        abEnabled = false;
        confEnabled = false;
        xyzEnabled = false;

        for (const auto &content : modeDetailsCache.frameContent) {
            if (content == "depth")
                depthEnabled = true;
            else if (content == "ab")
                abEnabled = true;
            else if (content == "conf")
                confEnabled = true;
            else if (content == "xyz")
                xyzEnabled = true;
        }

        LOG(INFO) << "[PLAYBACK] Enable flags set: depth=" << depthEnabled
                  << " ab=" << abEnabled << " conf=" << confEnabled
                  << " xyz=" << xyzEnabled;

    } else {
        // Online mode: configure sensor from INI parameters
        std::string value;

        const auto &globalIniParams = m_config->getIniKeyValPairs();

        // Check for lens scatter compensation
        auto lens_scattering_it =
            globalIniParams.find("lensScatterCompensationEnabled");
        bool lensScatterEnabled =
            (lens_scattering_it != globalIniParams.end() &&
             lens_scattering_it->second == "1");

        if (lensScatterEnabled) {
            LOG(INFO) << "Lens scatter compensation enabled - sensor will "
                         "output raw frames";

            // Update INI params to disable ISP depth computation
            std::map<std::string, std::string> updatedParams = globalIniParams;
            updatedParams["partialDepthEnable"] = "1";
            m_config->setIniKeyValPairs(updatedParams);
        }

        depthEnabled = true;
        abEnabled = true;
        confEnabled = true;

        // Configure depth bits
        auto it = globalIniParams.find("bitsInPhaseOrDepth");
        if (it != globalIniParams.end()) {
            value = it->second;
            depthBitsPerPixel = std::stoi(value);
            std::string sensorValue =
                (lensScatterEnabled && value != "0") ? "0" : value;
            sensorValue = convertBitsToSensorValue(sensorValue, true);
            if (sensorValue == "0" && !lensScatterEnabled)
                depthEnabled = false;
            m_depthSensor->setControl("phaseDepthBits", sensorValue);
        } else {
            LOG(WARNING)
                << "bitsInPhaseOrDepth was not found in parameter list";
        }

        // Configure confidence bits
        it = globalIniParams.find("bitsInConf");
        if (it != globalIniParams.end()) {
            value = it->second;
            confBitsPerPixel = std::stoi(value);
            std::string sensorValue =
                (lensScatterEnabled && value != "0") ? "0" : value;
            sensorValue = convertBitsToSensorValue(sensorValue, false);
            if (sensorValue == "0" && !lensScatterEnabled)
                confEnabled = false;
            m_depthSensor->setControl("confidenceBits", sensorValue);
        } else {
            LOG(WARNING) << "bitsInConf was not found in parameter list";
        }

        // Configure AB bits
        it = globalIniParams.find("bitsInAB");
        if (it != globalIniParams.end()) {
            value = it->second;
            abEnabled = true;
            abBitsPerPixel = std::stoi(value);
            std::string sensorValue =
                (lensScatterEnabled && value != "0") ? "0" : value;
            sensorValue = convertBitsToSensorValue(sensorValue, true);
            if (sensorValue == "0" && !lensScatterEnabled)
                abEnabled = false;
            m_depthSensor->setControl("abBits", sensorValue);
        } else {
            LOG(WARNING) << "bitsInAB was not found in parameter list";
        }

        // Partial depth enable
        it = globalIniParams.find("partialDepthEnable");
        if (it != globalIniParams.end()) {
            std::string en = (it->second == "0") ? "1" : "0";
            m_depthSensor->setControl("depthEnable", en);
            m_depthSensor->setControl("abAveraging", en);
        } else {
            LOG(WARNING)
                << "partialDepthEnable was not found in parameter list";
        }

        // Input format
        it = globalIniParams.find("inputFormat");
        if (it != globalIniParams.end()) {
            value = it->second;
            m_depthSensor->setControl("inputFormat", value);
        } else {
            LOG(WARNING) << "inputFormat was not found in parameter list";
        }

        // XYZ enable (only if not set via API)
        if (!xyzSetViaApi) {
            it = globalIniParams.find("xyzEnable");
            if (it != globalIniParams.end()) {
                xyzEnabled = !(it->second == "0");
            } else {
                LOG(WARNING) << "xyzEnable was not found in parameter list";
            }
        }

        // Header size / metadata
        it = globalIniParams.find("headerSize");
        if (it != globalIniParams.end()) {
            value = it->second;
            if (std::stoi(value) == 128 && abEnabled) {
                m_config->setMetadataInAB(1);
            } else {
                m_config->setMetadataInAB(0);
            }
        } else {
            LOG(WARNING) << "headerSize was not found in parameter list";
        }

        // Rebuild frameContent based on enabled frame types
        if (!globalIniParams.empty() && !modeDetailsCache.isRawBypass) {
            modeDetailsCache.frameContent.clear();
            if (depthEnabled) {
                modeDetailsCache.frameContent.emplace_back("depth");
            }
            if (abEnabled) {
                modeDetailsCache.frameContent.emplace_back("ab");
            }
            if (confEnabled) {
                modeDetailsCache.frameContent.emplace_back("conf");
            }
            if (xyzEnabled) {
                modeDetailsCache.frameContent.emplace_back("xyz");
            }
            // Always include metadata last
            modeDetailsCache.frameContent.emplace_back("metadata");
        }
    }

    return Status::OK;
}
