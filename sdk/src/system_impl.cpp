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

static std::vector<std::shared_ptr<Camera>>
buildCameras(std::unique_ptr<SensorEnumeratorInterface> enumerator,
             const std::string &netLinkTest = {}) {

    std::vector<std::shared_ptr<Camera>> cameras;
    std::vector<std::shared_ptr<DepthSensorInterface>> depthSensors;
    std::string uboot;
    std::string kernel;
    std::string sd_ver;

    enumerator->getDepthSensors(depthSensors);
#ifdef NXP
    enumerator->getUbootVersion(uboot);
    enumerator->getKernelVersion(kernel);
    enumerator->getSdVersion(sd_ver);
#endif

    // Get RGB sensor enumeration status
    bool rgbAvailable = false;
    std::string rgbDevicePath = "";
    Status rgbStatus = enumerator->getRGBSensorStatus(rgbAvailable, rgbDevicePath);
    if (rgbStatus != Status::OK) {
        LOG(WARNING) << "Failed to query RGB sensor status";
    }

    for (const auto &dSensor : depthSensors) {
        std::shared_ptr<Camera> camera = std::make_shared<CameraItof>(
            dSensor, uboot, kernel, sd_ver, netLinkTest);

        // Set RGB sensor detection info on each camera
        std::shared_ptr<CameraItof> itofCamera =
            std::dynamic_pointer_cast<CameraItof>(camera);
        if (itofCamera) {
            itofCamera->setRGBSensorInfo(rgbDevicePath, rgbAvailable);
        }

        cameras.emplace_back(camera);
    }

    return cameras;
}

SystemImpl::SystemImpl() {}

SystemImpl::~SystemImpl() = default;

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

#if defined(NXP) || defined(NVIDIA)
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
