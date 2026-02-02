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
#include "aditof/sensor_enumerator_factory.h"

/* On target SDK will use PlatformSensorEnumerator for platform-specific device discovery,
// while optionally supporting NetworkSensorEnumerator and OfflineSensorEnumerator. */
#ifdef TARGET
#include "platform/sensor_enumerator.h"
#else
#include "connections/network/network_sensor_enumerator.h"
#endif

#ifdef HAS_OFFLINE
#include "connections/offline/offline_sensor_enumerator.h"
#endif

using namespace aditof;

std::unique_ptr<SensorEnumeratorInterface>
SensorEnumeratorFactory::buildTargetSensorEnumerator() {
#ifdef TARGET
    return std::unique_ptr<SensorEnumeratorInterface>(
        new PlatformSensorEnumerator());
#endif
    return nullptr;
}

std::unique_ptr<SensorEnumeratorInterface>
SensorEnumeratorFactory::buildNetworkSensorEnumerator(const std::string &ip) {
#ifdef HAS_NETWORK
#ifndef TARGET
    return std::unique_ptr<SensorEnumeratorInterface>(
        new NetworkSensorEnumerator(ip));
#endif
#endif
    return nullptr;
}

std::unique_ptr<SensorEnumeratorInterface>
SensorEnumeratorFactory::buildOfflineSensorEnumerator() {
#ifdef HAS_OFFLINE
    return std::unique_ptr<SensorEnumeratorInterface>(
        new OfflineSensorEnumerator());
#endif
    return nullptr;
}
