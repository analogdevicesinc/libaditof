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
#include "network_sensor_enumerator.h"
#include "connections/network/network.h"
#include "connections/network/network_depth_sensor.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

using namespace aditof;

NetworkSensorEnumerator::NetworkSensorEnumerator(const std::string &ip)
    : m_ip(ip) {}

NetworkSensorEnumerator::~NetworkSensorEnumerator() = default;

Status NetworkSensorEnumerator::searchSensors() {
    Status status = Status::OK;

    extern std::vector<std::string> m_connectionList;
    int sensorIndex = -1, i = 0;
    for (i = 0; i < m_connectionList.size(); i++) {
        if (m_connectionList.at(i) == m_ip) {
            sensorIndex = i;
            break;
        }
    }

    if (sensorIndex == -1) {
        sensorIndex = m_connectionList.size();
        m_connectionList.emplace_back(m_ip);
    }

    LOG(INFO) << "Looking for sensors over network: " << m_ip;

    std::unique_ptr<Network> net(new Network(sensorIndex));

    if (net->ServerConnect(m_ip) != 0) {
        LOG(WARNING) << "Server Connect Failed";
        net.reset(nullptr);
        return Status::UNREACHABLE;
    }

    net->send_buff[sensorIndex].set_func_name("FindSensors");
    net->send_buff[sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    const payload::ServerResponse &msg = net->recv_buff[sensorIndex];
    const payload::SensorsInfo &pbSensorsInfo = msg.sensors_info();

    m_imageSensorsInfo = pbSensorsInfo.image_sensors().name();

    m_kernelVersion = msg.card_image_version().kernelversion();
    m_sdVersion = msg.card_image_version().sdversion();
    m_uBootVersion = msg.card_image_version().ubootversion();

    status = static_cast<Status>(net->recv_buff[sensorIndex].status());

    return status;
}

Status NetworkSensorEnumerator::getDepthSensors(
    std::vector<std::shared_ptr<DepthSensorInterface>> &depthSensors) {

    depthSensors.clear();

    auto sensor =
        std::make_shared<NetworkDepthSensor>(m_imageSensorsInfo, m_ip);
    depthSensors.emplace_back(sensor);

    return Status::OK;
}

aditof::Status
NetworkSensorEnumerator::getUbootVersion(std::string &uBootVersion) const {
    uBootVersion = m_uBootVersion;
    return aditof::Status::OK;
}

aditof::Status
NetworkSensorEnumerator::getKernelVersion(std::string &kernelVersion) const {
    kernelVersion = m_kernelVersion;
    return aditof::Status::OK;
}

aditof::Status
NetworkSensorEnumerator::getSdVersion(std::string &sdVersion) const {
    sdVersion = m_sdVersion;
    return aditof::Status::OK;
}
