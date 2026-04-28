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
#ifndef ADSD3500_COMMAND_INTERFACE_H
#define ADSD3500_COMMAND_INTERFACE_H

#include <aditof/status_definitions.h>
#include <cstdint>
#include <string>

namespace aditof {

/**
 * @brief Abstract interface for ADSD3500 chip protocol commands.
 *
 * Defines low-level command/payload read/write operations for communicating
 * with the ADSD3500 dual ISP chip via burst protocol. Enables dependency
 * inversion for testing and potential alternative communication backends.
 */
class Adsd3500CommandInterface {
  public:
    virtual ~Adsd3500CommandInterface() = default;

    /**
     * @brief Read command register from ADSD3500.
     * @param[in] cmd Command register address
     * @param[out] data Pointer to receive 16-bit register value
     * @param[in] usDelay Optional delay in microseconds before reading
     * @return Status::OK on success, error code otherwise
     */
    virtual Status readCommand(uint16_t cmd, uint16_t *data,
                               unsigned int usDelay = 0) = 0;

    /**
     * @brief Write command register to ADSD3500.
     * @param[in] cmd Command register address
     * @param[in] data 16-bit value to write
     * @param[in] usDelay Optional delay in microseconds after writing
     * @return Status::OK on success, error code otherwise
     */
    virtual Status writeCommand(uint16_t cmd, uint16_t data,
                                unsigned int usDelay = 0) = 0;

    /**
     * @brief Read payload data from ADSD3500 burst protocol.
     * @param[out] payload Buffer to receive payload data
     * @param[in] payloadSize Size of payload buffer in bytes
     * @return Status::OK on success, error code otherwise
     */
    virtual Status readPayload(uint8_t *payload, uint32_t payloadSize) = 0;

    /**
     * @brief Write payload data to ADSD3500 burst protocol.
     * @param[in] payload Buffer containing payload data
     * @param[in] payloadSize Size of payload data in bytes
     * @return Status::OK on success, error code otherwise
     */
    virtual Status writePayload(const uint8_t *payload,
                                uint32_t payloadSize) = 0;

    /**
     * @brief Get current chip status.
     * @return Chip status code
     */
    virtual int getChipStatus() const = 0;

    /**
     * @brief Get current imager status.
     * @return Imager status code
     */
    virtual int getImagerStatus() const = 0;
};

} // namespace aditof

#endif // ADSD3500_COMMAND_INTERFACE_H
