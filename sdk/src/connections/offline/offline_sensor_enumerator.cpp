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

OfflineSensorEnumerator::OfflineSensorEnumerator() {}

aditof::Status OfflineSensorEnumerator::getDepthSensors(
    std::vector<std::shared_ptr<aditof::DepthSensorInterface>> &depthSensors) {
    depthSensors.clear();

    depthSensors.emplace_back(std::make_shared<OfflineDepthSensor>());
    return aditof::Status::OK;
}

aditof::Status OfflineSensorEnumerator::searchSensors() {
    return aditof::Status::OK;
}

aditof::Status
OfflineSensorEnumerator::getUbootVersion(std::string &uBootVersion) const {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status
OfflineSensorEnumerator::getKernelVersion(std::string &kernelVersion) const {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status
OfflineSensorEnumerator::getSdVersion(std::string &sdVersion) const {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status
OfflineSensorEnumerator::getRGBSensorStatus(bool &isAvailable,
                                            std::string &devicePath) const {
    isAvailable = false;
    devicePath = "";
    return aditof::Status::OK;
}
