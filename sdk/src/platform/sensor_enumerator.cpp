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

/**
 * @brief Constructor for PlatformSensorEnumerator.
 *
 * Initializes the platform abstraction layer and logs the detected platform
 * information including name and architecture.
 *
 * @note Logs platform information via glog (if available) or aditof logging.
 */
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

/**
 * @brief Searches for available ToF sensors on the platform.
 *
 * Discovers all ADSD3500 Time-of-Flight sensors connected to the system using
 * the platform abstraction layer. Retrieves bootloader, kernel, and SD card
 * version information. Results are cached internally.
 *
 * @return Status::OK on success; Status::GENERIC_ERROR if sensor discovery fails.
 *
 * @note Clears previous sensor list before searching.
 * @note Caches version information for later retrieval via getters.
 */
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

/**
 * @brief Retrieves instantiated depth sensor interfaces.
 *
 * Creates and returns DepthSensorInterface objects for all discovered ADSD3500
 * sensors. Enables interrupt support for each sensor. Must be called after
 * searchSensors() to have valid results.
 *
 * @param[out] depthSensors Vector to be populated with sensor interface pointers.
 *
 * @return Status::OK on success.
 *
 * @note Clears the output vector before populating.
 * @note Enables interrupt notifications for each ADSD3500 sensor.
 * @note Sensor discovery must be run first via searchSensors().
 */
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

/**
 * @brief Retrieves the U-Boot bootloader version string.
 *
 * Returns the cached bootloader version information discovered during sensor
 * enumeration.
 *
 * @param[out] uBootVersion String to receive the U-Boot version.
 *
 * @return Status::OK if version string is available; Status::GENERIC_ERROR if empty.
 *
 * @note Must call searchSensors() first to populate version information.
 */
Status
PlatformSensorEnumerator::getUbootVersion(std::string &uBootVersion) const {
    uBootVersion = m_uBootVersion;
    return uBootVersion.empty() ? Status::GENERIC_ERROR : Status::OK;
}

/**
 * @brief Retrieves the Linux kernel version string.
 *
 * Returns the cached kernel version information discovered during sensor
 * enumeration.
 *
 * @param[out] kernelVersion String to receive the kernel version.
 *
 * @return Status::OK if version string is available; Status::GENERIC_ERROR if empty.
 *
 * @note Must call searchSensors() first to populate version information.
 */
Status
PlatformSensorEnumerator::getKernelVersion(std::string &kernelVersion) const {
    kernelVersion = m_kernelVersion;
    return kernelVersion.empty() ? Status::GENERIC_ERROR : Status::OK;
}

/**
 * @brief Retrieves the SD card image/firmware version string.
 *
 * Returns the cached SD card version information discovered during sensor
 * enumeration.
 *
 * @param[out] sdVersion String to receive the SD card version.
 *
 * @return Status::OK if version string is available; Status::GENERIC_ERROR if empty.
 *
 * @note Must call searchSensors() first to populate version information.
 */
Status PlatformSensorEnumerator::getSdVersion(std::string &sdVersion) const {
    sdVersion = m_sdVersion;
    return sdVersion.empty() ? Status::GENERIC_ERROR : Status::OK;
}
