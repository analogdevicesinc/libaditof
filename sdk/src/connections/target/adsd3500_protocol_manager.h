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
#ifndef ADSD3500_PROTOCOL_MANAGER_H
#define ADSD3500_PROTOCOL_MANAGER_H

#include <aditof/status_definitions.h>
#include <array>
#include <cstdint>
#include <mutex>

// Forward declarations
struct VideoDev;

#define ADSD3500_CTRL_PACKET_SIZE 4099

/**
 * @class Adsd3500ProtocolManager
 * @brief Manages low-level ADSD3500 chip communication protocol.
 *
 * Responsible for:
 * - Register read/write operations
 * - Payload transfer (burst mode)
 * - Command packet construction and validation
 * - V4L2 ioctl interface for chip control
 */
class Adsd3500ProtocolManager {
  public:
    /**
     * @brief Constructs protocol manager with access to hardware resources.
     *
     * @param videoDevs Pointer to V4L2 video device array
     * @param ctrlBuf Reference to control buffer for command/response packets
     * @param mutex Reference to recursive mutex for thread-safe operations
     */
    Adsd3500ProtocolManager(
        struct VideoDev *videoDevs,
        std::array<uint8_t, ADSD3500_CTRL_PACKET_SIZE> &ctrlBuf,
        std::recursive_mutex &mutex);

    ~Adsd3500ProtocolManager() = default;

    /**
     * @brief Reads a command response from ADSD3500 ISP.
     *
     * Sends a read command to the ADSD3500 via V4L2 control, waits for response,
     * and retrieves the 16-bit data value.
     *
     * @param cmd Command identifier to read
     * @param data Pointer to store read data
     * @param usDelay Delay in microseconds after command execution
     * @return Status::OK on success, Status::GENERIC_ERROR on failure
     */
    aditof::Status adsd3500_read_cmd(uint16_t cmd, uint16_t *data,
                                     unsigned int usDelay = 0);

    /**
     * @brief Writes a command to the ADSD3500 ISP.
     *
     * @param cmd Command identifier to write
     * @param data Command data to write
     * @param usDelay Delay in microseconds after command execution
     * @return Status::OK on success, Status::GENERIC_ERROR on failure
     */
    aditof::Status adsd3500_write_cmd(uint16_t cmd, uint16_t data,
                                      unsigned int usDelay = 0);

    /**
     * @brief Reads payload data from ADSD3500 after sending a command.
     *
     * Uses burst protocol with header, checksum, and error validation.
     *
     * @param cmd Command identifier to execute before reading payload
     * @param readback_data Pointer to buffer to receive payload data
     * @param payload_len Length of payload data to read in bytes
     * @return Status::OK on success, Status::GENERIC_ERROR on failure
     */
    aditof::Status adsd3500_read_payload_cmd(uint32_t cmd,
                                             uint8_t *readback_data,
                                             uint16_t payload_len);

    /**
     * @brief Reads payload data from ADSD3500 (without command prefix).
     *
     * @param payload Pointer to buffer to receive payload data
     * @param payload_len Length of payload data to read in bytes
     * @return Status::OK on success, Status::GENERIC_ERROR on failure
     */
    aditof::Status adsd3500_read_payload(uint8_t *payload,
                                         uint16_t payload_len);

    /**
     * @brief Writes payload data to ADSD3500 with command.
     *
     * Uses burst protocol with header, command, checksum, and payload.
     *
     * @param cmd Command identifier
     * @param payload Pointer to payload data to write
     * @param payload_len Length of payload data in bytes
     * @return Status::OK on success, Status::GENERIC_ERROR on failure
     */
    aditof::Status adsd3500_write_payload_cmd(uint32_t cmd, uint8_t *payload,
                                              uint16_t payload_len);

    /**
     * @brief Writes payload data to ADSD3500 (without command prefix).
     *
     * @param payload Pointer to payload data to write
     * @param payload_len Length of payload data in bytes
     * @return Status::OK on success, Status::GENERIC_ERROR on failure
     */
    aditof::Status adsd3500_write_payload(uint8_t *payload,
                                          uint16_t payload_len);

  private:
    struct VideoDev *m_videoDevs; ///< Pointer to V4L2 video device array
    std::array<uint8_t, ADSD3500_CTRL_PACKET_SIZE>
        &m_ctrlBuf; ///< Reference to control buffer
    std::recursive_mutex
        &m_mutex; ///< Reference to mutex for thread-safe protocol operations
};

#endif // ADSD3500_PROTOCOL_MANAGER_H
