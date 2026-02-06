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

/**
 * @brief Builds a sensor enumerator for target platform devices.
 *
 * Creates and returns a platform-specific sensor enumerator that discovers
 * locally-attached ToF sensors. This is only available when building for
 * target platforms (e.g., NVIDIA Jetson, Raspberry Pi) with the TARGET flag.
 * On host builds, this returns nullptr.
 *
 * @return A unique_ptr to a PlatformSensorEnumerator on target builds,
 *         nullptr on host-only builds.
 *
 * @note This function requires the TARGET preprocessor flag to be defined.
 */
std::unique_ptr<SensorEnumeratorInterface>
SensorEnumeratorFactory::buildTargetSensorEnumerator() {
#ifdef TARGET
    return std::unique_ptr<SensorEnumeratorInterface>(
        new PlatformSensorEnumerator());
#endif
    return nullptr;
}

/**
 * @brief Builds a sensor enumerator for network-connected devices.
 *
 * Creates and returns a network sensor enumerator that discovers ToF sensors
 * accessible over the network via TCP/IP. This allows remote connection to
 * sensors running on target platforms with the network server enabled.
 * Only available when HAS_NETWORK is defined and not building for TARGET.
 *
 * @param ip The IP address of the remote target device to connect to
 *           (e.g., "192.168.1.100").
 * @return A unique_ptr to a NetworkSensorEnumerator if network support is enabled,
 *         nullptr otherwise.
 *
 * @note Requires HAS_NETWORK to be defined and TARGET to be undefined.
 */
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

/**
 * @brief Builds a sensor enumerator for offline/replay mode.
 *
 * Creates and returns an offline sensor enumerator that reads pre-recorded
 * frame data from disk instead of connecting to real hardware. This is useful
 * for development, testing, and debugging without requiring physical ToF sensors.
 * Only available when HAS_OFFLINE is defined.
 *
 * @return A unique_ptr to an OfflineSensorEnumerator if offline support is enabled,
 *         nullptr otherwise.
 *
 * @note Requires HAS_OFFLINE to be defined at build time.
 */
std::unique_ptr<SensorEnumeratorInterface>
SensorEnumeratorFactory::buildOfflineSensorEnumerator() {
#ifdef HAS_OFFLINE
    return std::unique_ptr<SensorEnumeratorInterface>(
        new OfflineSensorEnumerator());
#endif
    return nullptr;
}
