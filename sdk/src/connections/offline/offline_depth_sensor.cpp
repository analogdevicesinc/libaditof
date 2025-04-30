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
#include "connections/network/network.h"
#include "offline_depth_sensor.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
#include <chrono>
#include <unordered_map>


struct OfflineDepthSensor::ImplData {
};


OfflineDepthSensor::OfflineDepthSensor(std::string s) : m_state(ST_STANDARD), m_frame_count(0) {
    m_implData = std::make_unique<ImplData>();
}

OfflineDepthSensor::~OfflineDepthSensor() {

}

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

    Status status = Status::OK;

    uint32_t sz = 0;
    readFrame((uint8_t *)buffer, sz, index);

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

#include <sys/stat.h>
#include <sys/types.h>
#ifdef _WIN32
#include <direct.h>
#endif
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>

static std::vector<std::streampos> m_frameIndex;

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

aditof::Status OfflineDepthSensor::startPlayback(const std::string filePath) {
    m_state = ST_STANDARD;

    if (m_stream_file_in.is_open()) {
        m_stream_file_in.close();
    }

    m_frameIndex.clear();
    m_stream_file_in = std::ifstream(filePath, std::ios::binary);

    m_state = ST_PLAYBACK;

    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::stopPlayback() {
    m_state = ST_STANDARD;
    if (m_stream_file_in.is_open()) {
        m_stream_file_in.close();
        return aditof::Status::OK;
    }
    LOG(ERROR) << "File stream is not open";
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status OfflineDepthSensor::getHeader(uint8_t* buffer, uint32_t bufferSize) {

        if (m_state != ST_PLAYBACK) {
        LOG(ERROR) << "Not in playback mode";
        return aditof::Status::GENERIC_ERROR;
    }

    try {
        if (m_stream_file_in.is_open()) {

            m_stream_file_in.seekg(0, std::ios::beg);

            uint32_t x;
            m_stream_file_in.read(reinterpret_cast<char *>(&x), sizeof(x));

            // Read size of buffer
            uint32_t _bufferSize = 0;
            m_stream_file_in.read(reinterpret_cast<char *>(&_bufferSize),
                                  sizeof(_bufferSize));

            bufferSize = _bufferSize;

            // Read buffer data
            m_stream_file_in.read(reinterpret_cast<char *>(buffer), bufferSize);

            m_frameIndex.clear();

            std::streampos pos = m_stream_file_in.tellg();

            static uint32_t idx = 0;
            LOG(INFO) << "Building Index";
            while (true) {

                uint32_t _bufferSize = 0;

                pos = m_stream_file_in.tellg();

                m_frameIndex.emplace_back(m_stream_file_in.tellg());

                uint32_t x;
                m_stream_file_in.read(reinterpret_cast<char *>(&x), sizeof(x));

                m_stream_file_in.read(reinterpret_cast<char *>(&_bufferSize),
                                      sizeof(_bufferSize));

                if (m_stream_file_in.eof()) {
                    break;
                }

                m_stream_file_in.seekg(_bufferSize, std::ios::cur);
            }

            m_stream_file_in.clear();

            m_stream_file_in.seekg(pos, std::ios::beg);

            return aditof::Status::OK;
        }
    } catch (const std::ofstream::failure &e) {
        LOG(ERROR) << "File I/O exception caught: " << e.what();
        m_stream_file_in.close();
        m_state = ST_STANDARD;
    }
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status OfflineDepthSensor::readFrame(uint8_t *buffer,
                                             uint32_t &bufferSize, uint32_t index) {

    if (m_state != ST_PLAYBACK) {
        LOG(ERROR) << "Not in playback mode";
        return aditof::Status::GENERIC_ERROR;
    }

    try {
        if (m_stream_file_in.is_open()) {

            m_stream_file_in.clear();

            m_stream_file_in.seekg(m_frameIndex[index], std::ios::beg); 

            auto p = m_stream_file_in.tellg();

            uint32_t x;
            m_stream_file_in.read(reinterpret_cast<char *>(&x),
                                  sizeof(x));

            // Read size of buffer
            uint32_t _bufferSize = 0;
            m_stream_file_in.read(reinterpret_cast<char *>(&_bufferSize),
                                  sizeof(_bufferSize));

            bufferSize = _bufferSize;

            // Read buffer data
            m_stream_file_in.read(reinterpret_cast<char *>(buffer), bufferSize);

            return aditof::Status::OK;
        }
    } catch (const std::ofstream::failure &e) {
        LOG(ERROR) << "File I/O exception caught: " << e.what();
        m_stream_file_in.close();
        m_state = ST_STANDARD;
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