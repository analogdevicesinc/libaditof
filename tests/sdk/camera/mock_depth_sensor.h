/*
 * MIT License
 *
 * Copyright (c) 2026 Analog Devices, Inc.
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
 */
#ifndef MOCK_DEPTH_SENSOR_H
#define MOCK_DEPTH_SENSOR_H

#include <aditof/adsd3500_hardware_interface.h>
#include <aditof/depth_sensor_interface.h>
#include <aditof/sensor_definitions.h>

/**
 * @brief Stub implementation of DepthSensorInterface for unit testing.
 * 
 * Provides a minimal stub of the depth sensor interface to allow testing
 * camera behavior without requiring actual hardware. All methods return
 * Status::OK or UNAVAILABLE by default with sensible stub data.
 * 
 * Note: This is a stub, not a full mock. For tests requiring specific
 * behavior, extend this class and override relevant methods.
 */
class StubDepthSensor : public aditof::DepthSensorInterface {
  public:
    StubDepthSensor() = default;
    virtual ~StubDepthSensor() = default;

    // Core sensor operations
    aditof::Status open() override { return aditof::Status::OK; }
    aditof::Status start() override { return aditof::Status::OK; }
    aditof::Status stop() override { return aditof::Status::OK; }

    // Mode operations
    aditof::Status getAvailableModes(std::vector<uint8_t> &modes) override {
        modes = {0, 1, 2, 3, 5, 6}; // Default crosby modes
        return aditof::Status::OK;
    }

    aditof::Status
    getModeDetails(const uint8_t &mode,
                   aditof::DepthSensorModeDetails &details) override {
        // Return sensible defaults for common modes
        details.modeNumber = mode;
        if (mode <= 1) {
            details.baseResolutionWidth = 1024;
            details.baseResolutionHeight = 1024;
        } else {
            details.baseResolutionWidth = 512;
            details.baseResolutionHeight = 512;
        }
        details.numberOfPhases = 0;
        details.numberOfFrequencies = 0;
        details.pixelFormatIndex = 0;
        details.frameWidthInBytes = 0;
        details.frameHeightInBytes = 0;
        details.metadataSize = 0;
        details.isPCM = 0;
        details.isRawBypass = 0;
        return aditof::Status::OK;
    }

    aditof::Status setMode(const uint8_t &mode) override {
        m_currentMode = mode;
        return aditof::Status::OK;
    }

    aditof::Status
    setMode(const aditof::DepthSensorModeDetails &mode) override {
        m_currentMode = mode.modeNumber;
        return aditof::Status::OK;
    }

    // Frame operations
    aditof::Status getFrame(uint16_t *buffer, uint32_t index = 0) override {
        return aditof::Status::UNAVAILABLE;
    }

    // Control operations
    aditof::Status
    getAvailableControls(std::vector<std::string> &controls) const override {
        controls = {"fps", "powerUp", "powerDown"};
        return aditof::Status::OK;
    }

    aditof::Status setControl(const std::string &control,
                              const std::string &value) override {
        m_controls[control] = value;
        return aditof::Status::OK;
    }

    aditof::Status getControl(const std::string &control,
                              std::string &value) const override {
        auto it = m_controls.find(control);
        if (it != m_controls.end()) {
            value = it->second;
            return aditof::Status::OK;
        }
        return aditof::Status::INVALID_ARGUMENT;
    }

    // Sensor information
    aditof::Status getDetails(aditof::SensorDetails &details) const override {
        details.id = "stub-sensor-0";
        details.connectionType = aditof::ConnectionType::ON_TARGET;
        return aditof::Status::OK;
    }

    aditof::Status getHandle(void **handle) override {
        *handle = nullptr;
        return aditof::Status::UNAVAILABLE;
    }

    aditof::Status getName(std::string &name) const override {
        name = "adsd3500";
        return aditof::Status::OK;
    }

    // Depth compute initialization and parameters
    aditof::Status initTargetDepthCompute(uint8_t *iniFile,
                                          uint16_t iniFileLength,
                                          uint8_t *calData,
                                          uint32_t calDataLength) override {
        return aditof::Status::OK;
    }

    aditof::Status
    getDepthComputeParams(std::map<std::string, std::string> &params) override {
        params = m_depthComputeParams;
        return aditof::Status::OK;
    }

    aditof::Status setDepthComputeParams(
        const std::map<std::string, std::string> &params) override {
        m_depthComputeParams = params;
        return aditof::Status::OK;
    }

    aditof::Status getIniParamsArrayForMode(int mode,
                                            std::string &iniStr) override {
        iniStr = "";
        return aditof::Status::UNAVAILABLE;
    }

  protected:
    uint8_t m_currentMode = 0;
    mutable std::map<std::string, std::string> m_controls;
    std::map<std::string, std::string> m_depthComputeParams;
};

#endif // MOCK_DEPTH_SENSOR_H
