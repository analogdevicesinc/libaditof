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
#include "network_depth_sensor.h"
#include "connections/network/network.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
#include <chrono>
#include <unordered_map>

struct CalibrationData {
    std::string mode;
    float gain;
    float offset;
    uint16_t *cache;
};
struct NetworkDepthSensor::ImplData {
    NetworkHandle handle;
    std::string ip;
    aditof::DepthSensorModeDetails modeDetailsCache;
    std::unordered_map<std::string, CalibrationData> calibration_cache;
    bool opened;
    Network::InterruptNotificationCallback cb;
};

NetworkDepthSensor::NetworkDepthSensor(const std::string &name,
                                       const std::string &ip)
    : m_implData(new NetworkDepthSensor::ImplData), m_stopServerCheck(false),
      m_state(ST_STANDARD){

    extern std::vector<std::string> m_connectionList;
    m_sensorIndex = -1;
    for (int i = 0; i < m_connectionList.size(); i++) {
        if (m_connectionList.at(i) == ip) {
            m_sensorIndex = i;
        }
    }

    if (m_sensorIndex == -1) {
        LOG(ERROR) << "No sensors available at: " << ip;
        return;
    }

    m_implData->cb = [this]() {
        Network *net = m_implData->handle.net;

        if (!net->isServer_Connected()) {
            LOG(WARNING) << "Not connected to server";
            return;
        }

        net->send_buff[m_sensorIndex].set_func_name("GetInterrupts");
        net->send_buff[m_sensorIndex].set_expect_reply(true);

        if (net->SendCommand() != 0) {
            LOG(WARNING) << "Send Command Failed";
            return;
        }

        if (net->recv_server_data() != 0) {
            LOG(WARNING) << "Receive Data Failed";
            return;
        }

        if (net->recv_buff[m_sensorIndex].server_status() !=
            payload::ServerStatus::REQUEST_ACCEPTED) {
            LOG(WARNING) << "API execution on Target Failed";
            return;
        }

        for (int32_t i = 0;
             i < net->recv_buff[m_sensorIndex].int32_payload_size(); ++i) {
            for (auto m_interruptCallback : m_interruptCallbackMap) {
                m_interruptCallback.second(
                    (aditof::Adsd3500Status)net->recv_buff[m_sensorIndex]
                        .int32_payload(i));
            }
        }
    };

    Network *net = new Network(m_sensorIndex);
    m_implData->handle.net = net;
    m_implData->handle.net->registerInterruptCallback(m_implData->cb);
    m_implData->ip = ip;
    m_implData->opened = false;
    m_sensorDetails.connectionType = aditof::ConnectionType::NETWORK;
    m_sensorDetails.id = ip;
    m_sensorName = name;
}

NetworkDepthSensor::~NetworkDepthSensor() {
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    automaticStop();

    // If channel communication has been opened, let the server know we're hanging up
    if (m_implData->opened) {
        if (!m_implData->handle.net->isServer_Connected()) {
            LOG(WARNING) << "Not connected to server";
        }

        m_implData->handle.net->send_buff[m_sensorIndex].set_func_name(
            "HangUp");
        m_implData->handle.net->send_buff[m_sensorIndex].set_expect_reply(
            false);

        if (m_implData->handle.net->SendCommand() != 0) {
            LOG(WARNING) << "Send Command Failed";
        }

        if (!m_stopServerCheck) {
            m_stopServerCheck = true;
            m_activityCheckThread.join();
        }
    }

    delete m_implData->handle.net;

    for (auto it = m_implData->calibration_cache.begin();
         it != m_implData->calibration_cache.begin(); ++it) {
        delete[] it->second.cache;
        it->second.cache = nullptr;
    }
}

aditof::Status NetworkDepthSensor::getDepthComputeParams(
    std::map<std::string, std::string> &params) {
    using namespace aditof;
    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("GetDepthComputeParam");
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    if (status == Status::OK) {
        params["abThreshMin"] =
            (net->recv_buff[m_sensorIndex].strings_payload(0));
        params["abSumThresh"] =
            (net->recv_buff[m_sensorIndex].strings_payload(1));
        params["confThresh"] =
            (net->recv_buff[m_sensorIndex].strings_payload(2));
        params["radialThreshMin"] =
            (net->recv_buff[m_sensorIndex].strings_payload(3));
        params["radialThreshMax"] =
            (net->recv_buff[m_sensorIndex].strings_payload(4));
        params["jblfApplyFlag"] =
            (net->recv_buff[m_sensorIndex].strings_payload(5));
        params["jblfWindowSize"] =
            (net->recv_buff[m_sensorIndex].strings_payload(6));
        params["jblfGaussianSigma"] =
            (net->recv_buff[m_sensorIndex].strings_payload(7));
        params["jblfExponentialTerm"] =
            (net->recv_buff[m_sensorIndex].strings_payload(8));
        params["jblfMaxEdge"] =
            (net->recv_buff[m_sensorIndex].strings_payload(9));
        params["jblfABThreshold"] =
            (net->recv_buff[m_sensorIndex].strings_payload(10));
        params["headerSize"] =
            (net->recv_buff[m_sensorIndex].strings_payload(11));
    }

    return status;
}

aditof::Status NetworkDepthSensor::setDepthComputeParams(
    const std::map<std::string, std::string> &params) {
    using namespace aditof;
    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("SetDepthComputeParam");
    net->send_buff[m_sensorIndex].set_expect_reply(true);
    net->send_buff[m_sensorIndex].add_func_strings_param(
        params.at("abThreshMin"));
    net->send_buff[m_sensorIndex].add_func_strings_param(
        params.at("abSumThresh"));
    net->send_buff[m_sensorIndex].add_func_strings_param(
        params.at("confThresh"));
    net->send_buff[m_sensorIndex].add_func_strings_param(
        params.at("radialThreshMin"));
    net->send_buff[m_sensorIndex].add_func_strings_param(
        params.at("radialThreshMax"));
    net->send_buff[m_sensorIndex].add_func_strings_param(
        params.at("jblfApplyFlag"));
    net->send_buff[m_sensorIndex].add_func_strings_param(
        params.at("jblfWindowSize"));
    net->send_buff[m_sensorIndex].add_func_strings_param(
        params.at("jblfGaussianSigma"));
    net->send_buff[m_sensorIndex].add_func_strings_param(
        params.at("jblfExponentialTerm"));
    net->send_buff[m_sensorIndex].add_func_strings_param(
        params.at("jblfMaxEdge"));
    net->send_buff[m_sensorIndex].add_func_strings_param(
        params.at("jblfABThreshold"));

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());
    return status;
}

aditof::Status NetworkDepthSensor::open() {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (net->ServerConnect(m_implData->ip) != 0) {
        LOG(WARNING) << "Server Connect Failed";
        return Status::UNREACHABLE;
    }

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Open");
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    if (status == Status::OK) {
        m_implData->opened = true;

        // Create a new thread that periodically checks for inactivity on client-network then goes back to sleep
        m_activityCheckThread =
            std::thread(&NetworkDepthSensor::checkForServerUpdates, this);
    }

    return status;
}

aditof::Status NetworkDepthSensor::start() {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Start");
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status NetworkDepthSensor::stop() {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    LOG(INFO) << "Stopping device";

    automaticStop();

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Stop");
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status
NetworkDepthSensor::getAvailableModes(std::vector<uint8_t> &modes) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("GetAvailableModes");
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    // Cleanup array (if required) before filling it with the available types
    if (modes.size() != 0) {
        modes.clear();
    }

    for (int i = 0; i < net->recv_buff[m_sensorIndex].int32_payload_size();
         i++) {
        modes.emplace_back(net->recv_buff[m_sensorIndex].int32_payload(i));
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status
NetworkDepthSensor::getModeDetails(const uint8_t &mode,
                                   aditof::DepthSensorModeDetails &details) {
    using namespace aditof;
    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("GetModeDetails");
    net->send_buff[m_sensorIndex].add_func_int32_param(mode);
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }
    details.modeNumber = mode;
    details.pixelFormatIndex = net->recv_buff[m_sensorIndex]
                                   .depth_sensor_mode_details()
                                   .pixel_format_index();
    details.frameWidthInBytes = net->recv_buff[m_sensorIndex]
                                    .depth_sensor_mode_details()
                                    .frame_width_in_bytes();
    details.frameHeightInBytes = net->recv_buff[m_sensorIndex]
                                     .depth_sensor_mode_details()
                                     .frame_height_in_bytes();
    details.baseResolutionWidth = net->recv_buff[m_sensorIndex]
                                      .depth_sensor_mode_details()
                                      .base_resolution_width();
    details.baseResolutionHeight = net->recv_buff[m_sensorIndex]
                                       .depth_sensor_mode_details()
                                       .base_resolution_height();
    details.metadataSize = net->recv_buff[m_sensorIndex]
                               .depth_sensor_mode_details()
                               .metadata_size();
    details.isPCM =
        net->recv_buff[m_sensorIndex].depth_sensor_mode_details().is_pcm();
    details.numberOfPhases = net->recv_buff[m_sensorIndex]
                                 .depth_sensor_mode_details()
                                 .number_of_phases();
    details.frameContent.clear();
    for (int i = 0; i < net->recv_buff[m_sensorIndex]
                            .depth_sensor_mode_details()
                            .frame_content_size();
         i++) {
        details.frameContent.emplace_back(net->recv_buff[m_sensorIndex]
                                              .depth_sensor_mode_details()
                                              .frame_content(i));
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());
    return status;
}

aditof::Status NetworkDepthSensor::setMode(const uint8_t &mode) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("SetModeByIndex");
    net->send_buff[m_sensorIndex].add_func_int32_param(mode);
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status
NetworkDepthSensor::setMode(const aditof::DepthSensorModeDetails &type) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("SetMode");
    net->send_buff[m_sensorIndex].mutable_mode_details()->set_mode_number(
        type.modeNumber);
    net->send_buff[m_sensorIndex]
        .mutable_mode_details()
        ->set_pixel_format_index(type.pixelFormatIndex);
    net->send_buff[m_sensorIndex]
        .mutable_mode_details()
        ->set_frame_width_in_bytes(type.frameWidthInBytes);
    net->send_buff[m_sensorIndex]
        .mutable_mode_details()
        ->set_frame_height_in_bytes(type.frameHeightInBytes);
    net->send_buff[m_sensorIndex]
        .mutable_mode_details()
        ->set_base_resolution_width(type.baseResolutionWidth);
    net->send_buff[m_sensorIndex]
        .mutable_mode_details()
        ->set_base_resolution_height(type.baseResolutionHeight);
    net->send_buff[m_sensorIndex].mutable_mode_details()->set_is_pcm(
        type.isPCM);
    net->send_buff[m_sensorIndex].mutable_mode_details()->set_metadata_size(
        type.metadataSize);
    auto content = net->send_buff[m_sensorIndex].mutable_mode_details();
    for (int i = 0; i < type.frameContent.size(); i++) {
        content->add_frame_content(type.frameContent.at(i));
    }

    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    if (status == Status::OK) {
        m_implData->modeDetailsCache = type;
    }

    return status;
}

aditof::Status NetworkDepthSensor::getFrame(uint16_t *buffer) {
    using namespace aditof;

    if (m_state == ST_PLAYBACK) {

        return Status::OK;
    }

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("GetFrame");
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    uint32_t sz;
    if (net->SendCommand(static_cast<void *>(buffer), &sz) != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());
    if (status != Status::OK) {
        LOG(WARNING) << "getFrame() failed on target";
        return status;
    }

    if (m_state == ST_RECORD) {
        writeFrame((uint8_t*)buffer, sz);
    }

    return status;
}

aditof::Status NetworkDepthSensor::getAvailableControls(
    std::vector<std::string> &controls) const {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("GetAvailableControls");
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    if (status == Status::OK) {
        controls.clear();

        for (int i = 0;
             i < net->recv_buff[m_sensorIndex].strings_payload_size(); i++) {
            std::string controlName =
                net->recv_buff[m_sensorIndex].strings_payload(i);
            controls.push_back(controlName);
        }
    }

    return status;
}

aditof::Status NetworkDepthSensor::setControl(const std::string &control,
                                              const std::string &value) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("SetControl");
    net->send_buff[m_sensorIndex].add_func_strings_param(control);
    net->send_buff[m_sensorIndex].add_func_strings_param(value);
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status NetworkDepthSensor::getControl(const std::string &control,
                                              std::string &value) const {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("GetControl");
    net->send_buff[m_sensorIndex].add_func_strings_param(control);
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    if (status == Status::OK) {
        value = net->recv_buff[m_sensorIndex].strings_payload(0);
    }

    return status;
}

aditof::Status
NetworkDepthSensor::getDetails(aditof::SensorDetails &details) const {
    details = m_sensorDetails;
    return aditof::Status::OK;
}

aditof::Status NetworkDepthSensor::getHandle(void **handle) {
    if (m_implData->opened) {
        *handle = &m_implData->handle;
        return aditof::Status::OK;
    } else {
        *handle = nullptr;
        LOG(ERROR) << "Won't return the handle. Device hasn't been opened yet.";
        return aditof::Status::UNAVAILABLE;
    }
    return aditof::Status::OK;
}

aditof::Status NetworkDepthSensor::getName(std::string &name) const {
    name = m_sensorName;
    return aditof::Status::OK;
}

aditof::Status
NetworkDepthSensor::setHostConnectionType(std::string &connectionType) {
    LOG(INFO) << "Function used only on target!";
    return aditof::Status::OK;
}

aditof::Status NetworkDepthSensor::adsd3500_read_cmd(uint16_t cmd,
                                                     uint16_t *data,
                                                     unsigned int usDelay) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500ReadCmd");
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(cmd));
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(usDelay));
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    if (status == Status::OK) {
        *data = static_cast<uint16_t>(
            net->recv_buff[m_sensorIndex].int32_payload(0));
    }

    return status;
}

aditof::Status NetworkDepthSensor::adsd3500_write_cmd(uint16_t cmd,
                                                      uint16_t data,
                                                      unsigned int usDelay) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500WriteCmd");
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(cmd));
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(data));
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(usDelay));
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status NetworkDepthSensor::adsd3500_read_payload_cmd(
    uint32_t cmd, uint8_t *readback_data, uint16_t payload_len) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500ReadPayloadCmd");
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(cmd));
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(payload_len));
    net->send_buff[m_sensorIndex].add_func_bytes_param(readback_data,
                                                       4 * sizeof(uint8_t));
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    if (status == Status::OK) {
        memcpy(readback_data,
               net->recv_buff[m_sensorIndex].bytes_payload(0).c_str(),
               net->recv_buff[m_sensorIndex].bytes_payload(0).length());
    }

    return status;
}

aditof::Status NetworkDepthSensor::adsd3500_read_payload(uint8_t *payload,
                                                         uint16_t payload_len) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500ReadPayload");
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(payload_len));
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    if (status == Status::OK) {
        memcpy(payload, net->recv_buff[m_sensorIndex].bytes_payload(0).c_str(),
               net->recv_buff[m_sensorIndex].bytes_payload(0).length());
    }

    return status;
}

aditof::Status
NetworkDepthSensor::adsd3500_write_payload_cmd(uint32_t cmd, uint8_t *payload,
                                               uint16_t payload_len) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500WritePayloadCmd");
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(cmd));
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(payload_len));
    net->send_buff[m_sensorIndex].add_func_bytes_param(payload, payload_len);
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status
NetworkDepthSensor::adsd3500_write_payload(uint8_t *payload,
                                           uint16_t payload_len) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500WritePayload");
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(payload_len));
    net->send_buff[m_sensorIndex].add_func_bytes_param(payload, payload_len);
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status NetworkDepthSensor::adsd3500_reset() {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500Reset");
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status NetworkDepthSensor::adsd3500_register_interrupt_callback(
    aditof::SensorInterruptCallback &cb) {
    m_interruptCallbackMap.insert({&cb, cb});
    return aditof::Status::OK;
}

aditof::Status NetworkDepthSensor::adsd3500_unregister_interrupt_callback(
    aditof::SensorInterruptCallback &cb) {

    m_interruptCallbackMap.erase(&cb);

    return aditof::Status::OK;
}

aditof::Status NetworkDepthSensor::adsd3500_get_status(int &chipStatus,
                                                       int &imagerStatus) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500GetStatus");
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    chipStatus = net->recv_buff[m_sensorIndex].int32_payload(0);
    imagerStatus = net->recv_buff[m_sensorIndex].int32_payload(1);

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status NetworkDepthSensor::initTargetDepthCompute(
    uint8_t *iniFile, uint16_t iniFileLength, uint8_t *calData,
    uint16_t calDataLength) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("InitTargetDepthCompute");
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(iniFileLength));
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(calDataLength));
    net->send_buff[m_sensorIndex].add_func_bytes_param(iniFile, iniFileLength);
    net->send_buff[m_sensorIndex].add_func_bytes_param(calData, calDataLength);
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

void NetworkDepthSensor::checkForServerUpdates() {
    using namespace std::chrono;

    while (!m_stopServerCheck) {
        // get latest timestamp from Network object
        steady_clock::time_point latestActivityTimestamp =
            m_implData->handle.net->getLatestActivityTimestamp();

        // get current timestamp
        steady_clock::time_point now = steady_clock::now();

        // decide if it is required to check for server updates
        duration<double> inactivityDuration =
            duration_cast<duration<double>>(now - latestActivityTimestamp);
        if (inactivityDuration.count() > 1.0) {
            // check server for interrupts
            std::unique_lock<std::mutex> mutex_lock(
                m_implData->handle.net_mutex);
            if (m_implData->handle.net->isServer_Connected()) {
                m_implData->cb();
            } else {
                m_stopServerCheck = true;
                break;
            }
        }
        // go back to sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

aditof::Status
NetworkDepthSensor::setSensorConfiguration(const std::string &sensorConf) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("SetSensorConfiguration");
    net->send_buff[m_sensorIndex].add_func_strings_param(sensorConf);
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status
NetworkDepthSensor::getIniParamsArrayForMode(int mode, std::string &iniStr) {

    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("GetIniArray");
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(mode));
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    if (status == Status::OK) {
        iniStr = net->recv_buff[m_sensorIndex].strings_payload(0);
    }

    return status;
}

#pragma region Stream_Recording_and_Playback

#include <sys/stat.h>
#include <sys/types.h>
#ifdef _WIN32
#include <direct.h>
#endif
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <cstdlib>
#include <random>

static bool folderExists(const std::string &path) {
    struct stat info;
    return (stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR));
}

static bool createFolder(const std::string &path) {
#ifdef _WIN32
    return _mkdir(path.c_str()) == 0 || errno == EEXIST;
#else
    return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
#endif
}

static std::string generateFileName(const std::string &prefix = "aditof_",
                                    const std::string &extension = ".adcam") {
    // Get current UTC time
    std::time_t now = std::time(nullptr);
    std::tm utc_tm;
#ifdef _WIN32
    gmtime_s(&utc_tm, &now); // Windows
#else
    gmtime_r(&now, &utc_tm); // Linux/macOS
#endif

    std::ostringstream oss;

    // Format time: YYYYMMDD_HHMMSS
    oss << prefix;
    oss << std::put_time(&utc_tm, "%Y%m%d_%H%M%S");

    // Generate random 8-digit hex
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint32_t> dis(0, 0xFFFFFFFF);
    uint32_t randNum = dis(gen);
    oss << "_" << std::hex << std::setw(8) << std::setfill('0') << randNum;

    oss << extension;

    return oss.str();
}

aditof::Status NetworkDepthSensor::startRecording(std::string& fileName) {

    m_state = ST_STANDARD;
    if (!folderExists(m_folder_path)) {
        if (!createFolder(m_folder_path)) {
            LOG(ERROR) << "Failed to create folder for recordings: "
                      << m_folder_path;
            return aditof::Status::GENERIC_ERROR;
        }
    }

    fileName = m_folder_path + "/" + generateFileName();

    if (m_stream_file_out.is_open()) {
        m_stream_file_out.close();
    }

    m_frame_count = 0;

    m_stream_file_out = std::ofstream(fileName, std::ios::binary);

    m_state = ST_RECORD;

    return aditof::Status::OK;
}

aditof::Status NetworkDepthSensor::stopRecording() {
    m_state = ST_STANDARD;
    if (m_stream_file_out.is_open()) {
        m_stream_file_out.close();
        return aditof::Status::OK;
    } 
    LOG(ERROR) << "File stream is not open";
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status NetworkDepthSensor::startPlayback(const std::string filePath) {
    m_state = ST_STANDARD;

    if (m_stream_file_in.is_open()) {
        m_stream_file_in.close();
    }

    m_stream_file_in = std::ifstream(filePath, std::ios::binary);

    m_state = ST_PLAYBACK;

    return aditof::Status::OK;
}

aditof::Status NetworkDepthSensor::stopPlayback() { 
    m_state = ST_STANDARD;
    if (m_stream_file_in.is_open()) {
        m_stream_file_in.close();
        return aditof::Status::OK;
    }
    LOG(ERROR) << "File stream is not open";
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status NetworkDepthSensor::writeFrame(uint8_t *buffer,
                                              uint32_t bufferSize) {
    if (m_state != ST_RECORD) {
        LOG(ERROR) << "Not in record mode";
        return aditof::Status::GENERIC_ERROR;
    }

    try {
        if (m_stream_file_out.is_open()) {
            // Write size of buffer
            m_stream_file_out.write((char *)&bufferSize,
                                    sizeof(bufferSize));
            // Write buffer data
            m_stream_file_out.write((char *)(buffer),
                                    bufferSize);

            m_frame_count++;

            return aditof::Status::OK;
        }
    } catch (const std::ofstream::failure &e) {
        LOG(ERROR) << "File I/O exception caught: " << e.what();
        m_stream_file_out.close();
        m_state = ST_STANDARD;
    }
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status NetworkDepthSensor::readFrame(uint8_t *buffer,
                                             uint32_t &bufferSize) {

    if (m_state != ST_PLAYBACK) {
        LOG(ERROR) << "Not in playback mode";
        return aditof::Status::GENERIC_ERROR;
    }

    try {
        if (m_stream_file_in.is_open()) {
            // Read size of buffer
            uint32_t _bufferSize = 0;
            m_stream_file_in.read(reinterpret_cast<char *>(_bufferSize),
                                    sizeof(_bufferSize));

            if (bufferSize < _bufferSize) {
                LOG(ERROR) << "Buffer size is smaller than the data to be read";
                return aditof::Status::GENERIC_ERROR;
            }

            bufferSize = _bufferSize;

            // Read buffer data
            m_stream_file_in.read(reinterpret_cast<char *>(buffer),
                                    bufferSize);

            return aditof::Status::OK;
        }
    } catch (const std::ofstream::failure &e) {
        LOG(ERROR) << "File I/O exception caught: " << e.what();
        m_stream_file_in.close();
        m_state = ST_STANDARD;
    }
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status NetworkDepthSensor::automaticStop() { 
    if (m_state == ST_PLAYBACK) {
        stopPlayback();
    } else if (m_state == ST_RECORD) {
        stopRecording();
    }

    return aditof::Status::OK;
}

#pragma endregion