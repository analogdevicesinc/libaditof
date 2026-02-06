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
#include "offline_sensor_enumerator.h"
#include "offline_depth_sensor.h"

/**
 * @brief Constructs an OfflineSensorEnumerator object.
 *
 * Initializes the offline sensor enumerator for managing offline (playback) depth sensors.
 */
OfflineSensorEnumerator::OfflineSensorEnumerator() {}

/**
 * @brief Retrieves available offline depth sensors.
 *
 * Clears the provided vector and populates it with a single OfflineDepthSensor instance
 * for playback operations. This creates the interface for working with recorded depth data.
 *
 * @param[out] depthSensors Vector to be populated with available offline depth sensor interfaces
 *
 * @return Status::OK on success
 */
aditof::Status OfflineSensorEnumerator::getDepthSensors(
    std::vector<std::shared_ptr<aditof::DepthSensorInterface>> &depthSensors) {
    depthSensors.clear();

    depthSensors.emplace_back(std::make_shared<OfflineDepthSensor>());
    return aditof::Status::OK;
}

/**
 * @brief Searches for available offline sensors.
 *
 * This is a stub implementation for the offline enumerator. Since offline sensors
 * don't require hardware discovery, this function simply returns success.
 *
 * @return Status::OK on success
 */
aditof::Status OfflineSensorEnumerator::searchSensors() {
    return aditof::Status::OK;
}

/**
 * @brief Retrieves the U-Boot version.
 *
 * This information is not available in offline mode since there is no hardware target.
 *
 * @param[out] uBootVersion String to receive U-Boot version (not populated)
 *
 * @return Status::UNAVAILABLE (always unavailable for offline mode)
 */
aditof::Status
OfflineSensorEnumerator::getUbootVersion(std::string &uBootVersion) const {
    return aditof::Status::UNAVAILABLE;
}

/**
 * @brief Retrieves the kernel version.
 *
 * This information is not available in offline mode since there is no hardware target.
 *
 * @param[out] kernelVersion String to receive kernel version (not populated)
 *
 * @return Status::UNAVAILABLE (always unavailable for offline mode)
 */
aditof::Status
OfflineSensorEnumerator::getKernelVersion(std::string &kernelVersion) const {
    return aditof::Status::UNAVAILABLE;
}

/**
 * @brief Retrieves the SD card image version.
 *
 * This information is not available in offline mode since there is no hardware target.
 *
 * @param[out] sdVersion String to receive SD version (not populated)
 *
 * @return Status::UNAVAILABLE (always unavailable for offline mode)
 */
aditof::Status
OfflineSensorEnumerator::getSdVersion(std::string &sdVersion) const {
    return aditof::Status::UNAVAILABLE;
}
