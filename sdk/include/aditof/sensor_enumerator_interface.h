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
#ifndef SENSOR_ENUMERATOR_INTERFACE_H
#define SENSOR_ENUMERATOR_INTERFACE_H

#include "aditof/depth_sensor_interface.h"
#include "aditof/status_definitions.h"

#include <memory>
#include <vector>

namespace aditof {

/**
 * @class SensorEnumeratorInterface
 * @brief Can search for sensors and retrieve sensors by category.
 */
class SensorEnumeratorInterface {
  public:
    /**
     * @brief Destructor
     */
    virtual ~SensorEnumeratorInterface() = default;

    /**
     * @brief Do a search for the available sensors.
     * @return Status
     */
    virtual aditof::Status searchSensors() = 0;

    /**
     * @brief Get the available depth sensors.
     * @param[out] depthSensors - list of found sensors
     * @return Status
     */
    virtual aditof::Status
    getDepthSensors(std::vector<std::shared_ptr<aditof::DepthSensorInterface>>
                        &depthSensors) = 0;

    /**
     * @brief Get the U-Boot version that is installed on the embedded system
     * that the camera is attached to.
     * @param[out] uBootVersion - string containing data abouth the version.
     * @return Status
     */
    virtual aditof::Status getUbootVersion(std::string &uBootVersion) const = 0;

    /**
     * @brief Get the kernel version that is installed on the embedded system
     * that the camera is attached to.
     * @param[out] kernelVersion - string containing data abouth the version.
     * @return Status
     */
    virtual aditof::Status
    getKernelVersion(std::string &kernelVersion) const = 0;

    /**
     * @brief Get the SD card image version on the embedded system that the
     * camera is attached to.
     * @param[out] sdVersion - string containing data abouth the version.
     * @return Status
     */
    virtual aditof::Status getSdVersion(std::string &sdVersion) const = 0;
};

} // namespace aditof

#endif // SENSOR_ENUMERATOR_INTERFACE_H
