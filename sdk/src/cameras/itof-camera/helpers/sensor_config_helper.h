/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 */
#ifndef SENSOR_CONFIG_HELPER_H
#define SENSOR_CONFIG_HELPER_H

#include <aditof/camera_definitions.h>
#include <aditof/depth_sensor_interface.h>
#include <aditof/status_definitions.h>

#include <cstdint>
#include <map>
#include <memory>
#include <string>

namespace aditof {

// Forward declarations
class CameraConfiguration;

/**
 * @brief Helper class for configuring sensor mode details and frame type enables.
 *
 * Manages conversion of INI parameters to sensor control values, handles
 * lens scatter compensation mode, and configures bit depths for depth/AB/confidence.
 */
class SensorConfigHelper {
  public:
    SensorConfigHelper(std::shared_ptr<DepthSensorInterface> sensor,
                       CameraConfiguration *config);
    ~SensorConfigHelper() = default;

    /**
     * @brief Configures sensor mode details for online or offline operation.
     * 
     * In offline mode, sets enable flags based on recorded frameContent.
     * In online mode, configures sensor controls from INI parameters (bit depths,
     * raw bypass, depth enable, etc.) and rebuilds frameContent.
     *
     * @param isOffline True if in playback mode, false for live sensor
     * @param modeDetailsCache Mode details to configure
     * @param[out] depthEnabled Depth frame enable flag
     * @param[out] abEnabled AB frame enable flag
     * @param[out] confEnabled Confidence frame enable flag
     * @param[out] xyzEnabled XYZ frame enable flag (only updated if !xyzSetViaApi)
     * @param xyzSetViaApi True if XYZ enable was set via API (takes precedence)
     * @param[out] depthBitsPerPixel Depth bit depth storage
     * @param[out] abBitsPerPixel AB bit depth storage
     * @param[out] confBitsPerPixel Confidence bit depth storage
     * @return Status::OK on success
     */
    Status configureModeDetails(bool isOffline,
                                DepthSensorModeDetails &modeDetailsCache,
                                bool &depthEnabled, bool &abEnabled,
                                bool &confEnabled, bool &xyzEnabled,
                                bool xyzSetViaApi, uint8_t &depthBitsPerPixel,
                                uint8_t &abBitsPerPixel,
                                uint8_t &confBitsPerPixel);

  private:
    std::shared_ptr<DepthSensorInterface> m_depthSensor;
    CameraConfiguration *m_config;

    // Helper to convert bit count to sensor control value
    std::string convertBitsToSensorValue(const std::string &bits,
                                         bool isDepthOrAB);
};

} // namespace aditof

#endif // SENSOR_CONFIG_HELPER_H
