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
#include "sensor_enumerator.h"
#include "connections/target/adsd3500_interrupt_notifier.h"
#include "connections/target/adsd3500_sensor.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

using namespace aditof;

PlatformSensorEnumerator::PlatformSensorEnumerator() {

    auto info = m_platform.getPlatformInfo();
#ifdef USE_GLOG
    LOG(INFO) << "Initialized platform: " << info.name << " ("
              << info.architecture << ")";
#else
    LOG(INFO) << "Initialized platform: " << info.name << " ("
              << info.architecture << ")";
#endif
}

Status PlatformSensorEnumerator::searchSensors() {
    Status status;

    // Clear previous results
    m_sensorsInfo.clear();

    // Find ToF sensors
    status = m_platform.findToFSensors(m_sensorsInfo);
    if (status != Status::OK) {
#ifdef USE_GLOG
        LOG(WARNING) << "Failed to find ToF sensors";
#else
        LOG(WARNING) << "Failed to find ToF sensors";
#endif
        return status;
    }

#ifdef USE_GLOG
    LOG(INFO) << "Found " << m_sensorsInfo.size() << " ToF sensor(s)";
#else
    LOG(INFO) << "Found " << m_sensorsInfo.size() << " ToF sensor(s)";
#endif

    // Retrieve version information
    m_uBootVersion = m_platform.getBootloaderVersion();
    m_kernelVersion = m_platform.getKernelVersion();
    m_sdVersion = m_platform.getSDCardVersion();

    return Status::OK;
}

Status PlatformSensorEnumerator::getDepthSensors(
    std::vector<std::shared_ptr<DepthSensorInterface>> &depthSensors) {

    depthSensors.clear();

    for (const auto &sensorInfo : m_sensorsInfo) {
        if (sensorInfo.sensorType == platform::SensorType::SENSOR_ADSD3500) {
            auto sensor = std::make_shared<Adsd3500Sensor>(
                sensorInfo.driverPath, sensorInfo.subDevPath,
                sensorInfo.captureDev);
            depthSensors.push_back(sensor);

            // Enable interrupt support
            auto &interruptNotifier = Adsd3500InterruptNotifier::getInstance();
            interruptNotifier.enableInterrupts();

#ifdef USE_GLOG
            LOG(INFO) << "Created ADSD3500 sensor at " << sensorInfo.driverPath;
#else
            LOG(INFO) << "Created ADSD3500 sensor at " << sensorInfo.driverPath;
#endif
        }
    }

    return Status::OK;
}

Status
PlatformSensorEnumerator::getUbootVersion(std::string &uBootVersion) const {
    uBootVersion = m_uBootVersion;
    return uBootVersion.empty() ? Status::GENERIC_ERROR : Status::OK;
}

Status
PlatformSensorEnumerator::getKernelVersion(std::string &kernelVersion) const {
    kernelVersion = m_kernelVersion;
    return kernelVersion.empty() ? Status::GENERIC_ERROR : Status::OK;
}

Status PlatformSensorEnumerator::getSdVersion(std::string &sdVersion) const {
    sdVersion = m_sdVersion;
    return sdVersion.empty() ? Status::GENERIC_ERROR : Status::OK;
}
