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
#include "offline_depth_sensor.h"

#include <aditof/log.h>
#include <chrono>
#include <unordered_map>

struct OfflineDepthSensor::ImplData {};

/**
 * @brief Constructs an OfflineDepthSensor object.
 *
 * Initializes the sensor in stopped state with zero frame count and allocates
 * implementation data structure.
 */
OfflineDepthSensor::OfflineDepthSensor() : m_state(ST_STOP), m_frame_count(0) {
    m_implData = std::make_unique<ImplData>();
}

/**
 * @brief Destructor for OfflineDepthSensor.
 *
 * Ensures automatic cleanup by stopping any active recording or playback.
 */
OfflineDepthSensor::~OfflineDepthSensor() { automaticStop(); }

/**
 * @brief Retrieves depth computation parameters.
 *
 * This is a stub implementation for offline sensor that returns an empty parameter map.
 *
 * @param[out] params Map to be filled with depth compute parameter key-value pairs
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::getDepthComputeParams(
    std::map<std::string, std::string> &params) {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Sets depth computation parameters.
 *
 * This is a stub implementation for offline sensor that accepts but does not use parameters.
 *
 * @param[in] params Map of depth compute parameter key-value pairs to set
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::setDepthComputeParams(
    const std::map<std::string, std::string> &params) {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Opens the offline depth sensor.
 *
 * This is a stub implementation for offline sensor that performs no hardware initialization.
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::open() {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Starts the offline depth sensor.
 *
 * This is a stub implementation for offline sensor that performs no streaming startup.
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::start() {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Stops the offline depth sensor.
 *
 * Calls automaticStop to ensure proper cleanup of recording or playback operations.
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::stop() {
    using namespace aditof;

    Status status = Status::OK;

    status = automaticStop();

    return status;
}

/**
 * @brief Retrieves available sensor modes.
 *
 * This is a stub implementation for offline sensor that returns an empty modes list.
 *
 * @param[out] modes Vector to be filled with available mode identifiers
 *
 * @return Status::OK on success
 */
aditof::Status
OfflineDepthSensor::getAvailableModes(std::vector<uint8_t> &modes) {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Retrieves detailed information for a specific sensor mode.
 *
 * This is a stub implementation for offline sensor that does not populate mode details.
 *
 * @param[in] mode Mode identifier to query
 * @param[out] details Structure to be filled with mode details
 *
 * @return Status::OK on success
 */
aditof::Status
OfflineDepthSensor::getModeDetails(const uint8_t &mode,
                                   aditof::DepthSensorModeDetails &details) {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Sets the sensor mode by mode identifier.
 *
 * This is a stub implementation for offline sensor that accepts but does not apply the mode.
 *
 * @param[in] mode Mode identifier to set
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::setMode(const uint8_t &mode) {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Sets the sensor mode using detailed mode structure.
 *
 * This is a stub implementation for offline sensor that accepts but does not apply the mode.
 *
 * @param[in] type DepthSensorModeDetails structure containing mode configuration
 *
 * @return Status::OK on success
 */
aditof::Status
OfflineDepthSensor::setMode(const aditof::DepthSensorModeDetails &type) {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Retrieves a frame from the playback file at the specified index.
 *
 * Reads a frame from the offline playback file at the given frame index. The frame
 * data is copied into the provided buffer.
 *
 * @param[out] buffer Pointer to buffer to receive frame data (must be non-null)
 * @param[in] index Frame index to read from the playback file
 *
 * @return Status::OK on success, Status::INVALID_ARGUMENT if buffer is null,
 *         Status::GENERIC_ERROR on read failure
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

/**
 * @brief Retrieves available sensor controls.
 *
 * This is a stub implementation for offline sensor that returns an empty controls list.
 *
 * @param[out] controls Vector to be filled with available control names
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::getAvailableControls(
    std::vector<std::string> &controls) const {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Sets a sensor control parameter.
 *
 * This is a stub implementation for offline sensor that accepts but does not apply controls.
 *
 * @param[in] control Name of the control parameter
 * @param[in] value Value to set for the control
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::setControl(const std::string &control,
                                              const std::string &value) {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Retrieves a sensor control parameter value.
 *
 * This is a stub implementation for offline sensor that does not populate control values.
 *
 * @param[in] control Name of the control parameter to query
 * @param[out] value String to receive the control value
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::getControl(const std::string &control,
                                              std::string &value) const {
    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Retrieves sensor details.
 *
 * This is a stub implementation for offline sensor that does not populate sensor details.
 *
 * @param[out] details Structure to be filled with sensor details
 *
 * @return Status::OK on success
 */
aditof::Status
OfflineDepthSensor::getDetails(aditof::SensorDetails &details) const {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Retrieves the sensor handle.
 *
 * This is a stub implementation for offline sensor that does not provide a handle.
 *
 * @param[out] handle Pointer to receive the sensor handle
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::getHandle(void **handle) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Retrieves the sensor name.
 *
 * Returns the name "offline" to identify this as an offline playback sensor.
 *
 * @param[out] name String to receive the sensor name
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::getName(std::string &name) const {

    using namespace aditof;

    name = "offline";

    Status status = Status::OK;

    return status;
}

/**
 * @brief Reads a command from the ADSD3500 ISP.
 *
 * This is a stub implementation for offline sensor with no hardware access.
 *
 * @param[in] cmd Command identifier to read
 * @param[out] data Pointer to receive command response data
 * @param[in] usDelay Delay in microseconds after command execution
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::adsd3500_read_cmd(uint16_t cmd,
                                                     uint16_t *data,
                                                     unsigned int usDelay) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Writes a command to the ADSD3500 ISP.
 *
 * This is a stub implementation for offline sensor with no hardware access.
 *
 * @param[in] cmd Command identifier to write
 * @param[in] data Command data to write
 * @param[in] usDelay Delay in microseconds after command execution
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::adsd3500_write_cmd(uint16_t cmd,
                                                      uint16_t data,
                                                      unsigned int usDelay) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Reads payload data from the ADSD3500 ISP after sending a command.
 *
 * This is a stub implementation for offline sensor with no hardware access.
 *
 * @param[in] cmd Command identifier to execute before reading payload
 * @param[out] readback_data Pointer to buffer to receive payload data
 * @param[in] payload_len Length of payload data to read in bytes
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::adsd3500_read_payload_cmd(
    uint32_t cmd, uint8_t *readback_data, uint16_t payload_len) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Reads payload data from the ADSD3500 ISP.
 *
 * This is a stub implementation for offline sensor with no hardware access.
 *
 * @param[out] payload Pointer to buffer to receive payload data
 * @param[in] payload_len Length of payload data to read in bytes
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::adsd3500_read_payload(uint8_t *payload,
                                                         uint16_t payload_len) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Writes payload data to the ADSD3500 ISP with a command.
 *
 * This is a stub implementation for offline sensor with no hardware access.
 *
 * @param[in] cmd Command identifier to execute with payload
 * @param[in] payload Pointer to payload data to write
 * @param[in] payload_len Length of payload data in bytes
 *
 * @return Status::OK on success
 */
aditof::Status
OfflineDepthSensor::adsd3500_write_payload_cmd(uint32_t cmd, uint8_t *payload,
                                               uint16_t payload_len) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Writes payload data to the ADSD3500 ISP.
 *
 * This is a stub implementation for offline sensor with no hardware access.
 *
 * @param[in] payload Pointer to payload data to write
 * @param[in] payload_len Length of payload data in bytes
 *
 * @return Status::OK on success
 */
aditof::Status
OfflineDepthSensor::adsd3500_write_payload(uint8_t *payload,
                                           uint16_t payload_len) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Resets the ADSD3500 ISP.
 *
 * This is a stub implementation for offline sensor with no hardware access.
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::adsd3500_reset() {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Gets interrupt status and resets the ADSD3500 ISP interrupt.
 *
 * This is a stub implementation for offline sensor with no hardware access.
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::adsd3500_getInterruptandReset() {
    return aditof::Status::OK;
}

/**
 * @brief Registers an interrupt callback for ADSD3500 ISP events.
 *
 * This is a stub implementation for offline sensor with no hardware access.
 *
 * @param[in] cb Callback function to register for interrupt events
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::adsd3500_register_interrupt_callback(
    aditof::SensorInterruptCallback &cb) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Unregisters an interrupt callback for ADSD3500 ISP events.
 *
 * This is a stub implementation for offline sensor with no hardware access.
 *
 * @param[in] cb Callback function to unregister from interrupt events
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::adsd3500_unregister_interrupt_callback(
    aditof::SensorInterruptCallback &cb) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Retrieves ADSD3500 ISP and imager status.
 *
 * This is a stub implementation for offline sensor with no hardware access.
 *
 * @param[out] chipStatus Variable to receive chip status
 * @param[out] imagerStatus Variable to receive imager status
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::adsd3500_get_status(int &chipStatus,
                                                       int &imagerStatus) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Initializes depth computation on target with INI and calibration data.
 *
 * This is a stub implementation for offline sensor with no hardware access.
 *
 * @param[in] iniFile Pointer to INI file data buffer
 * @param[in] iniFileLength Length of INI file data in bytes
 * @param[in] calData Pointer to calibration data buffer
 * @param[in] calDataLength Length of calibration data in bytes
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::initTargetDepthCompute(
    uint8_t *iniFile, uint16_t iniFileLength, uint8_t *calData,
    uint16_t calDataLength) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Sets the sensor configuration.
 *
 * This is a stub implementation for offline sensor that accepts but does not apply configuration.
 *
 * @param[in] sensorConf Sensor configuration string
 *
 * @return Status::OK on success
 */
aditof::Status
OfflineDepthSensor::setSensorConfiguration(const std::string &sensorConf) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

/**
 * @brief Retrieves INI parameters for a specific mode.
 *
 * This is a stub implementation for offline sensor that does not populate INI parameters.
 *
 * @param[in] mode Mode identifier to query
 * @param[out] iniStr String to receive INI parameters
 *
 * @return Status::OK on success
 */
aditof::Status
OfflineDepthSensor::getIniParamsArrayForMode(int mode, std::string &iniStr) {

    using namespace aditof;

    Status status = Status::OK;

    return status;
}

#pragma region Stream_Recording_and_Playback

#include <aditof/utils.h>

/**
 * @brief Starts recording frames to a file.
 *
 * This is a stub implementation that currently returns an error.
 *
 * @param[in] fileName Name of the file to record to
 * @param[in] parameters Pointer to recording parameters
 * @param[in] paramSize Size of parameters buffer in bytes
 *
 * @return Status::GENERIC_ERROR (not implemented)
 */
aditof::Status OfflineDepthSensor::startRecording(std::string &fileName,
                                                  uint8_t *parameters,
                                                  uint32_t paramSize) {

    using namespace aditof;

    Status status = Status::GENERIC_ERROR;

    return status;
}

/**
 * @brief Stops recording frames to file.
 *
 * This is a stub implementation that currently returns an error.
 *
 * @return Status::GENERIC_ERROR (not implemented)
 */
aditof::Status OfflineDepthSensor::stopRecording() {

    using namespace aditof;

    Status status = Status::GENERIC_ERROR;

    return status;
}

/**
 * @brief Sets the playback file and opens it for frame reading.
 *
 * Opens the specified file for playback and transitions the sensor to playback state.
 * Any previously open file is closed. Frame indexing is cleared and will be built
 * when getHeader is called.
 *
 * @param[in] filePath Path to the recorded file to play back
 *
 * @return Status::OK on success, Status::GENERIC_ERROR if file cannot be opened
 */
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

/**
 * @brief Stops playback and closes the playback file.
 *
 * Transitions the sensor to stopped state, clears the frame index, and closes
 * the input file stream if open.
 *
 * @return Status::OK if file was open and closed successfully,
 *         Status::GENERIC_ERROR if file stream was not open
 */
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

/**
 * @brief Retrieves the total number of frames in the playback file.
 *
 * Returns the count of frames that have been indexed from the playback file.
 * The frame index is built when getHeader is called.
 *
 * @param[out] frameCount Variable to receive the frame count
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::getFrameCount(uint32_t &frameCount) {

    frameCount = m_frameIndex.size();

    return aditof::Status::OK;
}

/**
 * @brief Reads the header from the playback file and builds frame index.
 *
 * Reads the header data from the beginning of the playback file into the provided
 * buffer, then scans through the entire file to build an index of frame positions
 * for random access. This must be called before reading frames.
 *
 * @param[out] buffer Pointer to buffer to receive header data (must be non-null)
 * @param[in] bufferSize Size of the buffer in bytes
 *
 * @return Status::OK on success, Status::INVALID_ARGUMENT if buffer is null,
 *         Status::INSUFFICIENT_MEMORY if buffer is too small,
 *         Status::GENERIC_ERROR on file I/O errors or if not in playback mode
 */
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

/**
 * @brief Reads a specific frame from the playback file by index.
 *
 * Seeks to the specified frame position in the playback file using the pre-built
 * frame index, then reads the frame data into the provided buffer. The buffer size
 * is updated with the actual frame size read.
 *
 * @param[out] buffer Pointer to buffer to receive frame data (must be non-null)
 * @param[in,out] bufferSize On input: size of buffer; on output: actual size read
 * @param[in] index Frame index to read (0-based, must be < frame count)
 *
 * @return Status::OK on success, Status::INVALID_ARGUMENT if buffer is null,
 *         Status::INSUFFICIENT_MEMORY if buffer is too small,
 *         Status::GENERIC_ERROR on file I/O errors, invalid index, or if not in playback mode
 */
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

/**
 * @brief Automatically stops recording or playback based on current state.
 *
 * Checks the current sensor state and calls stopPlayback() if in playback mode
 * or stopRecording() if in recording mode. Used for cleanup on stop() or destruction.
 *
 * @return Status::OK on success
 */
aditof::Status OfflineDepthSensor::automaticStop() {
    if (m_state == ST_PLAYBACK) {
        OfflineDepthSensor::stopPlayback();
    } else if (m_state == ST_RECORD) {
        OfflineDepthSensor::stopRecording();
    }

    return aditof::Status::OK;
}

#pragma endregion