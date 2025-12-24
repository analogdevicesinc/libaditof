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
#include "network_sensor_enumerator.h"
#include "connections/network/network.h"
#include "connections/network/network_depth_sensor.h"
#include <aditof/log.h>

using namespace aditof;

NetworkSensorEnumerator::NetworkSensorEnumerator(const std::string &ip)
    : m_ip(ip) {}

NetworkSensorEnumerator::~NetworkSensorEnumerator() = default;

Status NetworkSensorEnumerator::searchSensors() {
    Status status = Status::OK;

    extern std::vector<std::string> m_connectionList;
    int sensorIndex = -1;
    size_t i = 0;
    for (i = 0; i < m_connectionList.size(); i++) {
        if (m_connectionList.at(i) == m_ip) {
            sensorIndex = static_cast<int>(i);
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

aditof::Status
NetworkSensorEnumerator::getRGBSensorStatus(bool &isAvailable,
                                             std::string &devicePath) const {
    isAvailable = false;
    devicePath = "";
    return aditof::Status::OK;
}
