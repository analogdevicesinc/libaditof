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
#include "connections/usb/usb_depth_sensor.h"

struct UsbDepthSensor::ImplData {};

UsbDepthSensor::UsbDepthSensor(
    const aditof::DeviceConstructionData & /*data*/) {}

UsbDepthSensor::~UsbDepthSensor() = default;

aditof::Status UsbDepthSensor::open() {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDepthSensor::start() {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDepthSensor::stop() {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status
UsbDepthSensor::getAvailableModeDetails(std::vector<std::string> &types) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status
UsbDepthSensor::getModeDetailsDetails(const std::string &frameName,
                                      aditof::DepthSensorModeDetails &details) {
    using namespace aditof;
    Status status = Status::OK;
    // TO DO

    return status;
}

aditof::Status
UsbDepthSensor::setModeDetails(const aditof::DepthSensorModeDetails &type) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDepthSensor::getFrame(uint16_t *buffer) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDepthSensor::readAfeTemp(float &temperature) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDepthSensor::readLaserTemp(float &temperature) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status
UsbDepthSensor::getDetails(aditof::SensorDetails &details) const {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDepthSensor::getHandle(void **handle) {
    using namespace aditof;
    Status status = Status::OK;

    // TO DO

    return status;
}

aditof::Status UsbDepthSensor::adsd3500_register_interrupt_callback(
    aditof::SensorInterruptCallback cb) {
    LOG(WARNING) << "Registering an interrupt callback on a USB connection "
                    "is not supported yet!";
    return aditof::Status::UNAVAILABLE;
}

aditof::Status UsbDepthSensor::getFrameProcessParams(
    std::map<std::string, std::string> &params) {
    using namespace aditof;
    Status status = Status::UNAVAILABLE;
    return status;
}

aditof::Status UsbDepthSensor::setDepthComputeParams(
    const std::map<std::string, std::string> &params) {
    using namespace aditof;
    Status status = Status::UNAVAILABLE;
    return status;
}

aditof::Status
UsbDepthSensor::setSensorConfiguration(const std::string &sensorConf) {
    // TODO: select sensor table configuration
    return aditof::Status::OK;
}

aditof::Status UsbDepthSensor::adsd3500_getInterruptandReset() {
    LOG(INFO) << "Not available!";
    return aditof::Status::OK;
}

aditof::Status UsbDepthSensor::getIniParamsArrayForMode(int mode,
                                                        std::string &iniStr) {

    return aditof::Status::OK;
}