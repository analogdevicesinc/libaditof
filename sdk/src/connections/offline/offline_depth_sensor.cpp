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
#include "offline_depth_sensor.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
#include <chrono>
#include <unordered_map>

struct OfflineDepthSensor::ImplData {};

OfflineDepthSensor::OfflineDepthSensor() : m_state(ST_STOP), m_frame_count(0) {
    m_implData = std::make_unique<ImplData>();
}

OfflineDepthSensor::~OfflineDepthSensor() { automaticStop(); }

aditof::Status OfflineDepthSensor::getDepthComputeParams(
    std::map<std::string, std::string> &params) {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status OfflineDepthSensor::setDepthComputeParams(
    const std::map<std::string, std::string> &params) {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status OfflineDepthSensor::open() {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status OfflineDepthSensor::start() {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status OfflineDepthSensor::stop() {
    using namespace aditof;

    Status status = Status::OK;

    status = automaticStop();

    return status;
}

aditof::Status
OfflineDepthSensor::getAvailableModes(std::vector<uint8_t> &modes) {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status
OfflineDepthSensor::getModeDetails(const uint8_t &mode,
                                   aditof::DepthSensorModeDetails &details) {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status OfflineDepthSensor::setMode(const uint8_t &mode) {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status
OfflineDepthSensor::setMode(const aditof::DepthSensorModeDetails &type) {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/*
* Maybe an index of frames is needed, using vectors to store the index.
* This will make it easy to jump around.
 */
aditof::Status OfflineDepthSensor::getFrame(uint16_t *buffer, uint32_t index) {
    using namespace aditof;

    if (buffer == nullptr) {
        LOG(ERROR) << "Null buffer provided";
        return Status::INVALID_ARGUMENT;
    }

    uint32_t sz = 0;
    Status status = readFrame((uint8_t *)buffer, sz, index);

    return status;
}

aditof::Status OfflineDepthSensor::getAvailableControls(
    std::vector<std::string> &controls) const {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status OfflineDepthSensor::setControl(const std::string &control,
                                              const std::string &value) {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status OfflineDepthSensor::getControl(const std::string &control,
                                              std::string &value) const {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status
OfflineDepthSensor::getDetails(aditof::SensorDetails &details) const {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status OfflineDepthSensor::getHandle(void **handle) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status OfflineDepthSensor::getName(std::string &name) const {

    using namespace aditof;

    name = "offline";

    Status status = Status::OK;

    return status;
}

aditof::Status
OfflineDepthSensor::setHostConnectionType(std::string &connectionType) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status OfflineDepthSensor::adsd3500_read_cmd(uint16_t cmd,
                                                     uint16_t *data,
                                                     unsigned int usDelay) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status OfflineDepthSensor::adsd3500_write_cmd(uint16_t cmd,
                                                      uint16_t data,
                                                      unsigned int usDelay) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status OfflineDepthSensor::adsd3500_read_payload_cmd(
    uint32_t cmd, uint8_t *readback_data, uint16_t payload_len) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status OfflineDepthSensor::adsd3500_read_payload(uint8_t *payload,
                                                         uint16_t payload_len) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status
OfflineDepthSensor::adsd3500_write_payload_cmd(uint32_t cmd, uint8_t *payload,
                                               uint16_t payload_len) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status
OfflineDepthSensor::adsd3500_write_payload(uint8_t *payload,
                                           uint16_t payload_len) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status OfflineDepthSensor::adsd3500_reset() {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status OfflineDepthSensor::adsd3500_getInterruptandReset() {
    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::adsd3500_register_interrupt_callback(
    aditof::SensorInterruptCallback &cb) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status OfflineDepthSensor::adsd3500_unregister_interrupt_callback(
    aditof::SensorInterruptCallback &cb) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status OfflineDepthSensor::adsd3500_get_status(int &chipStatus,
                                                       int &imagerStatus) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status OfflineDepthSensor::initTargetDepthCompute(
    uint8_t *iniFile, uint16_t iniFileLength, uint8_t *calData,
    uint16_t calDataLength) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status
OfflineDepthSensor::setSensorConfiguration(const std::string &sensorConf) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

aditof::Status
OfflineDepthSensor::getIniParamsArrayForMode(int mode, std::string &iniStr) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

#pragma region Stream_Recording_and_Playback

#include <aditof/utils.h>

aditof::Status OfflineDepthSensor::startRecording(std::string &fileName,
                                                  uint8_t *parameters,
                                                  uint32_t paramSize) {

    using namespace aditof;

    Status status = Status::GENERIC_ERROR;

    return status;
}

aditof::Status OfflineDepthSensor::stopRecording() {

    using namespace aditof;

    Status status = Status::GENERIC_ERROR;

    return status;
}

aditof::Status OfflineDepthSensor::setPlaybackFile(const std::string filePath) {
    m_state = ST_STOP;

    if (m_stream_file_in.is_open()) {
        m_stream_file_in.close();
    }

    m_frameIndex.clear();
    m_stream_file_in = std::ifstream(filePath, std::ios::binary);

    if (!m_stream_file_in.is_open() || !m_stream_file_in.good()) {
        LOG(ERROR) << "Failed to open file for playback: " << filePath;
        return aditof::Status::GENERIC_ERROR;
    }

    m_state = ST_PLAYBACK;

    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::stopPlayback() {
    m_state = ST_STOP;
    m_frameIndex.clear();

    if (m_stream_file_in.is_open()) {
        m_stream_file_in.close();
        return aditof::Status::OK;
    }
    LOG(ERROR) << "File stream is not open";
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status OfflineDepthSensor::getFrameCount(uint32_t &frameCount) {

    frameCount = m_frameIndex.size();

    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::getHeader(uint8_t *buffer,
                                             uint32_t bufferSize) {

    if (m_state != ST_PLAYBACK) {
        LOG(ERROR) << "Not in playback mode";
        return aditof::Status::GENERIC_ERROR;
    }

    if (buffer == nullptr) {
        LOG(ERROR) << "Null buffer provided";
        return aditof::Status::INVALID_ARGUMENT;
    }

    try {
        if (m_stream_file_in.is_open()) {

            m_stream_file_in.seekg(0, std::ios::beg);

            if (m_stream_file_in.fail()) {
                LOG(ERROR) << "Failed to seek to beginning of file";
                return aditof::Status::GENERIC_ERROR;
            }

            uint32_t x;
            m_stream_file_in.read(reinterpret_cast<char *>(&x), sizeof(x));

            if (m_stream_file_in.eof() || m_stream_file_in.fail()) {
                LOG(ERROR) << "EOF or read error while reading header marker";
                return aditof::Status::GENERIC_ERROR;
            }

            // Read size of buffer
            uint32_t _bufferSize = 0;
            m_stream_file_in.read(reinterpret_cast<char *>(&_bufferSize),
                                  sizeof(_bufferSize));

            if (m_stream_file_in.eof() || m_stream_file_in.fail()) {
                LOG(ERROR) << "EOF or read error while reading header size";
                return aditof::Status::GENERIC_ERROR;
            }

            if (_bufferSize > bufferSize) {
                LOG(ERROR) << "Buffer too small: need " << _bufferSize
                           << " bytes, have " << bufferSize;
                return aditof::Status::INSUFFICIENT_MEMORY;
            }

            // Read buffer data
            m_stream_file_in.read(reinterpret_cast<char *>(buffer),
                                  _bufferSize);

            if (m_stream_file_in.eof() || m_stream_file_in.fail()) {
                LOG(ERROR) << "EOF or read error while reading header data";
                return aditof::Status::GENERIC_ERROR;
            }

            m_frameIndex.clear();

            std::streampos pos = m_stream_file_in.tellg();

            LOG(INFO) << "Building Index";
            while (true) {

                uint32_t _bufferSize = 0;

                pos = m_stream_file_in.tellg();

                uint32_t x;
                m_stream_file_in.read(reinterpret_cast<char *>(&x), sizeof(x));

                if (m_stream_file_in.eof()) {
                    break;
                }

                m_stream_file_in.read(reinterpret_cast<char *>(&_bufferSize),
                                      sizeof(_bufferSize));

                if (m_stream_file_in.eof()) {
                    break;
                }

                m_frameIndex.emplace_back(pos);

                m_stream_file_in.seekg(_bufferSize, std::ios::cur);
            }

            m_stream_file_in.clear();

            m_stream_file_in.seekg(pos, std::ios::beg);

            return aditof::Status::OK;
        }
    } catch (const std::ios_base::failure &e) {
        LOG(ERROR) << "File I/O exception caught: " << e.what();
        m_stream_file_in.close();
        m_frameIndex.clear();
        m_state = ST_STOP;
    }
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status OfflineDepthSensor::readFrame(uint8_t *buffer,
                                             uint32_t &bufferSize,
                                             uint32_t index) {

    LOG(INFO) << "Reading frame index: " << index;
    if (m_state != ST_PLAYBACK) {
        LOG(ERROR) << "Not in playback mode";
        return aditof::Status::GENERIC_ERROR;
    }

    if (buffer == nullptr) {
        LOG(ERROR) << "Null buffer provided";
        return aditof::Status::INVALID_ARGUMENT;
    }

    // Check if index is valid
    if (index >= m_frameIndex.size()) {
        LOG(ERROR) << "Frame index " << index
                   << " out of range (max: " << m_frameIndex.size() - 1 << ")";
        return aditof::Status::GENERIC_ERROR;
    }

    try {
        if (m_stream_file_in.is_open()) {

            m_stream_file_in.clear();

            m_stream_file_in.seekg(m_frameIndex[index], std::ios::beg);

            if (m_stream_file_in.fail()) {
                LOG(ERROR) << "Failed to seek to frame index: " << index;
                return aditof::Status::GENERIC_ERROR;
            }

            uint32_t x;
            m_stream_file_in.read(reinterpret_cast<char *>(&x), sizeof(x));

            if (m_stream_file_in.eof() || m_stream_file_in.fail()) {
                LOG(ERROR) << "EOF or read error while reading frame marker";
                return aditof::Status::GENERIC_ERROR;
            }

            // Read size of buffer
            uint32_t _bufferSize = 0;
            m_stream_file_in.read(reinterpret_cast<char *>(&_bufferSize),
                                  sizeof(_bufferSize));

            if (m_stream_file_in.eof() || m_stream_file_in.fail()) {
                LOG(ERROR) << "EOF or read error while reading buffer size";
                return aditof::Status::GENERIC_ERROR;
            }

            // Validate buffer size is sufficient
            if (bufferSize > 0 && _bufferSize > bufferSize) {
                LOG(ERROR) << "Buffer too small: need " << _bufferSize
                           << " bytes, have " << bufferSize;
                return aditof::Status::INSUFFICIENT_MEMORY;
            }

            bufferSize = _bufferSize;

            // Read buffer data
            m_stream_file_in.read(reinterpret_cast<char *>(buffer), bufferSize);

            if (m_stream_file_in.eof() || m_stream_file_in.fail()) {
                LOG(ERROR) << "EOF or read error while reading frame data";
                return aditof::Status::GENERIC_ERROR;
            }

            return aditof::Status::OK;
        }
    } catch (const std::ios_base::failure &e) {
        LOG(ERROR) << "File I/O exception caught: " << e.what();
        m_stream_file_in.close();
        m_frameIndex.clear();
        m_state = ST_STOP;
    }
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status OfflineDepthSensor::automaticStop() {
    if (m_state == ST_PLAYBACK) {
        stopPlayback();
    } else if (m_state == ST_RECORD) {
        stopRecording();
    }

    return aditof::Status::OK;
}

#pragma endregion