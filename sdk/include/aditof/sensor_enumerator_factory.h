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
#ifndef SENSOR_ENUMERATOR_FACTORY_H
#define SENSOR_ENUMERATOR_FACTORY_H

#include "aditof/sensor_enumerator_interface.h"
#include "sdk_exports.h"

#include <memory>

namespace aditof {

/**
 * @class SensorEnumeratorFactory
 * @brief Provides the means to construct different types of sensors enumerators.
 * Based on the connection type (on target, Network), different enumerators need
 * to be used.
 */
class SDK_API SensorEnumeratorFactory {
  public:
    /**
     * @brief Factory method to create an enumerator to look for sensors on target.
     * Factory method will return null if the call is not made on target.
     * @return std::unique_ptr<SensorEnumeratorInterface>
     */
    static std::unique_ptr<SensorEnumeratorInterface>
    buildTargetSensorEnumerator();

    /**
     * Factory method to create an enumerator to look for sensors over network.
     * @return std::unique_ptr<DeviceEnumeratorInterface>
     */
    static std::unique_ptr<SensorEnumeratorInterface>
    buildNetworkSensorEnumerator(const std::string &ip);

    /**
     * Factory method to create an offline enumerator.
     * @return std::unique_ptr<DeviceEnumeratorInterface>
     */
    static std::unique_ptr<SensorEnumeratorInterface>
    buildOfflineSensorEnumerator();
};

} // namespace aditof

#endif // SENSOR_ENUMERATOR_FACTORY_H
