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
#include "system_impl.h"
#include "aditof/sensor_enumerator_factory.h"
#include "aditof/sensor_enumerator_interface.h"
#include "camera_itof.h"
#include <aditof/camera.h>
#include <aditof/log.h>
#include <algorithm>

#include "aditof/version.h"

#ifdef HAS_NETWORK
//#include <lws_config.h>
#include <zmq.hpp>
#endif

using namespace aditof;

/**
 * @brief Internal helper function to construct Camera objects from enumerated sensors.
 *
 * Takes a sensor enumerator that has already discovered depth sensors, retrieves
 * the sensor list along with version information (U-Boot, kernel, SD card), and
 * constructs a Camera object for each discovered sensor. Each Camera is an instance
 * of CameraItof configured with the depth sensor and version metadata.
 *
 * @param enumerator A unique_ptr to a SensorEnumeratorInterface that has completed
 *                   sensor discovery.
 * @param netLinkTest Optional network link test string for network-based cameras
 *                    (e.g., "network_test_param"). Defaults to empty string.
 * @return A vector of shared_ptr<Camera> objects representing all discovered cameras.
 *
 * @note This is a static helper function used internally by SystemImpl methods.
 */
static std::vector<std::shared_ptr<Camera>>
buildCameras(std::unique_ptr<SensorEnumeratorInterface> enumerator,
             const std::string &netLinkTest = {}) {

    std::vector<std::shared_ptr<Camera>> cameras;
    std::vector<std::shared_ptr<DepthSensorInterface>> depthSensors;
    std::string uboot;
    std::string kernel;
    std::string sd_ver;

    enumerator->getDepthSensors(depthSensors);
    enumerator->getUbootVersion(uboot);
    enumerator->getKernelVersion(kernel);
    enumerator->getSdVersion(sd_ver);

    for (const auto &dSensor : depthSensors) {
        std::shared_ptr<Camera> camera = std::make_shared<CameraItof>(
            dSensor, uboot, kernel, sd_ver, netLinkTest);
        cameras.emplace_back(camera);
    }

    return cameras;
}

/**
 * @brief Default constructor for SystemImpl.
 *
 * Initializes the System implementation object. The System class is responsible
 * for discovering and enumerating ToF cameras on the current platform or network.
 */
SystemImpl::SystemImpl() {}

/**
 * @brief Destructor for SystemImpl.
 *
 * Cleans up the System implementation object and releases any resources.
 */
SystemImpl::~SystemImpl() = default;

/**
 * @brief Retrieves a list of available cameras based on the specified URI.
 *
 * Discovers and enumerates ToF cameras accessible via different methods:
 * - Local/target cameras (when built with ON_TARGET flag)
 * - Network cameras (URI format: "ip:<address>[:port]")
 * - Offline/replay cameras (URI format: "offline:<path>")
 *
 * The function creates the appropriate sensor enumerator based on the URI scheme,
 * searches for sensors, and builds Camera objects for each discovered device.
 * Also logs the ZMQ version when network support is enabled.
 *
 * @param cameraList Output vector where discovered Camera objects will be stored.
 *                   The vector is cleared before populating.
 * @param uri Connection URI specifying how to discover cameras:
 *            - Empty or no prefix: local/target cameras (if ON_TARGET defined)
 *            - "ip:<address>[:port]": network cameras at the specified IP
 *            - "offline:<path>": offline replay from recorded data
 * @return Status::OK on success, Status::GENERIC_ERROR if enumeration fails or
 *         the requested mode is not supported.
 */
Status
SystemImpl::getCameraList(std::vector<std::shared_ptr<Camera>> &cameraList,
                          const std::string &uri) const {

#if HAS_NETWORK
    static bool logged = false;
    int major, minor, patch;
    zmq_version(&major, &minor, &patch);
    if (!logged) {
        LOG(INFO) << "SDK built with zmq version:" << major << "." << minor
                  << "." << patch;
        logged = true;
    }
#endif

    cameraList.clear();

    if (uri.compare(0, 3, "ip:") == 0) {
        std::string ip(uri, 3);
        return getCameraListAtIp(cameraList, ip);
    }

    std::unique_ptr<SensorEnumeratorInterface> sensorEnumerator;

#ifdef HAS_OFFLINE
    if (uri.compare(0, 8, "offline:") == 0) {
        LOG(INFO) << "Creating offline sensor.";
        sensorEnumerator =
            SensorEnumeratorFactory::buildOfflineSensorEnumerator();
        if (!sensorEnumerator) {
            LOG(ERROR) << "Could not create OfflineSensorEnumerator";
            return Status::GENERIC_ERROR;
        }

        if (sensorEnumerator) {
            Status status = sensorEnumerator->searchSensors();
            if (status == Status::OK) {
                cameraList = buildCameras(std::move(sensorEnumerator));
                return Status::OK;
            }
        }
        return Status::GENERIC_ERROR;
    }
#endif

#ifdef ADITOF_ON_TARGET
    sensorEnumerator = SensorEnumeratorFactory::buildTargetSensorEnumerator();
    if (!sensorEnumerator) {
        LOG(ERROR) << "Could not create TargetSensorEnumerator";
        return Status::GENERIC_ERROR;
    }

    if (sensorEnumerator) {
        Status status = sensorEnumerator->searchSensors();
        if (status == Status::OK) {
            cameraList = buildCameras(std::move(sensorEnumerator));
            return Status::OK;
        }
    }
    return Status::GENERIC_ERROR;
#endif

    return Status::OK;
}

/**
 * @brief Retrieves a list of cameras accessible at a specific IP address.
 *
 * Creates a network sensor enumerator to discover ToF cameras running on a remote
 * target device at the specified IP address. The IP address can optionally include
 * a network link test parameter separated by a colon (e.g., "192.168.1.100:test").
 * This method is used internally by getCameraList() when an "ip:" URI is provided.
 *
 * @param cameraList Output vector where discovered Camera objects will be stored.
 * @param ip The IP address of the remote device, optionally followed by ":" and
 *           a network link test parameter (e.g., "192.168.1.100" or
 *           "192.168.1.100:network_test").
 * @return Status::OK on success, Status::GENERIC_ERROR if the network interface is
 *         not enabled or if sensor discovery fails.
 *
 * @note Requires HAS_NETWORK to be defined at build time. Logs a warning if called
 *       when network support is disabled.
 */
Status
SystemImpl::getCameraListAtIp(std::vector<std::shared_ptr<Camera>> &cameraList,
                              const std::string &ip) const {

    LOG(INFO) << "Creating network sensor.";

    int netLinkFlag = ip.find(":");

    std::string onlyIp = ip;
    std::string netLinkTest;
    if (netLinkFlag != -1) {
        netLinkTest = ip.substr(netLinkFlag + 1, ip.size());
        onlyIp.erase(netLinkFlag);
    }
    std::unique_ptr<SensorEnumeratorInterface> sensorEnumerator =
        SensorEnumeratorFactory::buildNetworkSensorEnumerator(onlyIp);

    if (!sensorEnumerator) {
        LOG(WARNING) << "getCameraListAtIp(...) called, but network interface "
                        "is not enabled.";
        return Status::GENERIC_ERROR;
    }
    Status status = sensorEnumerator->searchSensors();
    if (status == Status::OK) {
        cameraList = buildCameras(std::move(sensorEnumerator), netLinkTest);
    }

    return status;
}
