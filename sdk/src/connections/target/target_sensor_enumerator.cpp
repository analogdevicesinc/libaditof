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
#include "connections/target/target_sensor_enumerator.h"
#include "connections/target/adsd3500_interrupt_notifier.h"
#include "connections/target/adsd3500_sensor.h"
#include <algorithm>
#include <fstream>

#include <aditof/log.h>

using namespace aditof;

Status TargetSensorEnumerator::getDepthSensors(
    std::vector<std::shared_ptr<DepthSensorInterface>> &depthSensors) {

    depthSensors.clear();

    for (const auto &sInfo : m_sensorsInfo) {
        switch (sInfo.sensorType) {
        case SensorType::SENSOR_ADSD3500: {
            auto sensor = std::make_shared<Adsd3500Sensor>(
                sInfo.driverPath, sInfo.subDevPath, sInfo.captureDev);
            depthSensors.emplace_back(sensor);

            auto &interruptNotifier = Adsd3500InterruptNotifier::getInstance();
            interruptNotifier.enableInterrupts(); // TO DO: refactor this!

            break;
        }
        }
    }

    return Status::OK;
}

std::string TargetSensorEnumerator::getVersionOfComponent(
    const std::string &component) const {
    std::string versionsFilePath = "/boot/sw-versions";
    std::ifstream fid;
    std::string line;
    std::string version;

    fid.open(versionsFilePath);
    if (fid.is_open()) {
        while (fid) {
            std::getline(fid, line);
            if (!line.compare(0, component.length(), component)) {
                version = line.substr(component.length());
                version.erase(std::remove(version.begin(), version.end(), '\t'),
                              version.end());
                break;
            }
        }
        fid.close();
    } else {
        LOG(ERROR) << "Failed to open file" << versionsFilePath;
    }

    return version;
}

aditof::Status
TargetSensorEnumerator::getUbootVersion(std::string &uBootVersion) const {

    uBootVersion = getVersionOfComponent("u-boot");
    if (uBootVersion.empty()) {
        LOG(ERROR) << "Could not find version for u-boot";
        return aditof::Status::INVALID_ARGUMENT;
    }

    return aditof::Status::OK;
}

aditof::Status
TargetSensorEnumerator::getKernelVersion(std::string &kernelVersion) const {

    kernelVersion = getVersionOfComponent("kernel");
    if (kernelVersion.empty()) {
        LOG(ERROR) << "Could not find version for u-boot";
        return aditof::Status::INVALID_ARGUMENT;
    }

    return aditof::Status::OK;
}

aditof::Status
TargetSensorEnumerator::getSdVersion(std::string &sdVersion) const {

    sdVersion = getVersionOfComponent("sd_img_ver");
    if (sdVersion.empty()) {
        LOG(ERROR) << "Could not find version for u-boot";
        return aditof::Status::INVALID_ARGUMENT;
    }

    return aditof::Status::OK;
}
