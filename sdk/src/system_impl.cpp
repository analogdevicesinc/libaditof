/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "system_impl.h"
#include "aditof/sensor_enumerator_factory.h"
#include "aditof/sensor_enumerator_interface.h"
#include "camera_itof.h"
#include <aditof/camera.h>
#include <algorithm>

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

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
    DLOG(INFO) << "Creating offline sensor.";
    sensorEnumerator = SensorEnumeratorFactory::buildOfflineSensorEnumerator();
    if (!sensorEnumerator) {
        LOG(ERROR) << "Could not create OfflineSensorEnumerator";
        return Status::GENERIC_ERROR;
    }
#elif defined(NXP) || defined(NVIDIA)
    sensorEnumerator = SensorEnumeratorFactory::buildTargetSensorEnumerator();
    if (!sensorEnumerator) {
        LOG(ERROR) << "Could not create TargetSensorEnumerator";
        return Status::GENERIC_ERROR;
    }
#else
    sensorEnumerator = SensorEnumeratorFactory::buildUsbSensorEnumerator();
    if (!sensorEnumerator) {
        LOG(ERROR) << "Could not create UsbSensorEnumerator";
        return Status::GENERIC_ERROR;
    }
#endif

    Status status = sensorEnumerator->searchSensors();
    if (status == Status::OK) {
        cameraList = buildCameras(std::move(sensorEnumerator));
    }

    return Status::OK;
}

Status
SystemImpl::getCameraListAtIp(std::vector<std::shared_ptr<Camera>> &cameraList,
                              const std::string &ip) const {
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
        LOG(ERROR) << "Network interface is not enabled."
                      "Please rebuild the SDK "
                      "with the option WITH_NETWORK=on";
        return Status::GENERIC_ERROR;
    }
    Status status = sensorEnumerator->searchSensors();
    if (status == Status::OK) {
        cameraList = buildCameras(std::move(sensorEnumerator), netLinkTest);
    }

    return status;
}
