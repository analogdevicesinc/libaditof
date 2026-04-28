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
#ifndef DEPTH_SENSOR_INTERFACE_H
#define DEPTH_SENSOR_INTERFACE_H

#include <aditof/frame_definitions.h>
#include <aditof/sensor_definitions.h>
#include <aditof/status_definitions.h>

// New segregated interfaces (ISP - Interface Segregation Principle)
#include <aditof/adsd3500_hardware_interface.h>
#include <aditof/playback_interface.h>
#include <aditof/recordable_interface.h>

#include <cstddef>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace aditof {

/**
 * @class DepthSensorInterface
 * @brief Core interface for depth sensors (SOLID ISP refactoring completed).
 * 
 * Provides core sensor operations: lifecycle (open/start/stop), mode configuration,
 * frame acquisition, controls, and depth compute parameters. All depth sensors
 * implement this interface.
 * 
 * **Interface Segregation (ISP):**
 * Hardware-specific, recording, and playback capabilities are now in separate interfaces:
 * - @see Adsd3500HardwareInterface for ADSD3500-specific commands
 * - @see RecordableInterface for recording capabilities
 * - @see PlaybackInterface for playback capabilities
 * 
 * Implementations should inherit only the interfaces they truly support,
 * eliminating stub methods that return Status::UNAVAILABLE.
 */
class DepthSensorInterface {
  public:
    /**
     * @brief Destructor
     */
    virtual ~DepthSensorInterface() = default;

    /**
     * @brief Open the communication channels with the hardware.
     * @return Status
     */
    virtual aditof::Status open() = 0;

    /**
     * @brief Start the streaming of data from the sensor.
     * @return Status
     */
    virtual aditof::Status start() = 0;

    /**
     * @brief Stop the sensor data stream.
     * @return Status
     */
    virtual aditof::Status stop() = 0;

    /**
     * @brief Return all modes that are supported by the sensor.
     * @param[out] modes
     * @return Status
     */
    virtual aditof::Status getAvailableModes(std::vector<uint8_t> &modes) = 0;

    /**
     * @brief Returns details of specified mode
     * @param mode
     * @param[out] details
     * @return Status
     */
    virtual aditof::Status
    getModeDetails(const uint8_t &mode,
                   aditof::DepthSensorModeDetails &details) = 0;

    /**
     * @brief Set the sensor frame mode to the given type
     * @param mode - desired mode
     * @return Status
     */
    virtual aditof::Status setMode(const uint8_t &mode) = 0;

    /**
     * @brief Set the sensor frame mode to the given type
     * @param type - frame details structure containing the frame type
     * @return Status
     */
    virtual aditof::Status
    setMode(const aditof::DepthSensorModeDetails &type) = 0;

    /**
     * @brief Request a frame from the sensor
     * @param[out] buffer - a valid location where the new frame should be stored.
     * The size of the frame is known (cached) internally and gets updated each
     * time setMode() is called.
     * @param[in] index - Frame index for offline playback mode [default: 0]
     * @return Status
     */
    virtual aditof::Status getFrame(uint16_t *buffer, uint32_t index = 0) = 0;

    /**
     * @brief Gets the sensors's list of controls
     * @param[out] controls
     * @return Status
     */
    virtual Status
    getAvailableControls(std::vector<std::string> &controls) const = 0;

    /**
     * @brief Sets a specific sensor control
     * @param[in] control - Control name
     * @param[in] value - Control value
     * @return Status
     */
    virtual Status setControl(const std::string &control,
                              const std::string &value) = 0;

    /**
     * @brief Gets the value of a specific sensor control
     * @param[in] control - Control name
     * @param[out] value - Control value
     * @return Status
     */
    virtual Status getControl(const std::string &control,
                              std::string &value) const = 0;

    /**
     * @brief Get a structure that contains information about the instance of
     * the sensor
     * @param[out] details - the variable where the sensor details should be
     * stored
     * @return Status
     */
    virtual aditof::Status getDetails(aditof::SensorDetails &details) const = 0;

    /**
     * @brief Gets a handle to be used by other devices such as Storage,
     * Temperature, etc. This handle will allow the other devices to
     * communicate remotely with the embedded target.
     * @param[out] handle - the handle which is owned by this instance
     * @return Status
     */
    virtual aditof::Status getHandle(void **handle) = 0;

    /**
     * @brief Get the name of the sensor
     * @param[out] name - the string in which the name is stored
     * @return Status
     */
    virtual aditof::Status getName(std::string &name) const = 0;

    /**
     * @brief Get the name of the sensor
     * @param[in] iniFile - iniFile content parsed as uint8_t*
     * @param[in] iniFileLength - iniFile content length
     * @param[in] calData - calibration data parsed as uint8_t*
     * @param[in] calDataLength - calibration data content length
     * @return Status
     */
    virtual aditof::Status initTargetDepthCompute(uint8_t *iniFile,
                                                  uint16_t iniFileLength,
                                                  uint8_t *calData,
                                                  uint32_t calDataLength) = 0;

    /**
     * @brief Get ini parameters for Depth Compute library
     * @param[in] params - a dictionary of parameters
     * @return Status
    */
    virtual aditof::Status
    getDepthComputeParams(std::map<std::string, std::string> &params) = 0;

    /**
     * @brief Set ini parameters for Depth Compute library
     * @param[in] params - a dictionary of parameters
     * @return Status
    */
    virtual aditof::Status
    setDepthComputeParams(const std::map<std::string, std::string> &params) = 0;

    /**
     * @brief Get ini parameters for Depth Compute library as string
     * @param[in] mode - desired mode
     * @param[out] iniStr - a string that contain ini params
     * @return Status
    */
    virtual aditof::Status getIniParamsArrayForMode(int mode,
                                                    std::string &iniStr) = 0;
};

} // namespace aditof

#endif // DEPTH_SENSOR_INTERFACE_H
