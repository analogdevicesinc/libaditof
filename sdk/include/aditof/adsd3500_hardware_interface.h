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
#ifndef ADSD3500_HARDWARE_INTERFACE_H
#define ADSD3500_HARDWARE_INTERFACE_H

#include <aditof/sensor_definitions.h>
#include <aditof/status_definitions.h>
#include <functional>

namespace aditof {

/**
 * @brief Callback for ADSD3500 sensor interrupts
 */
typedef std::function<void(Adsd3500Status)> SensorInterruptCallback;

/**
 * @class Adsd3500HardwareInterface
 * @brief ADSD3500-specific hardware command interface (ISP segregation).
 * 
 * Provides low-level access to ADSD3500 chip commands, payload operations,
 * interrupts, and status queries. Only ADSD3500-based sensors implement this.
 * Network sensors may proxy these commands to target hardware.
 * 
 * Offline and non-ADSD3500 sensors should NOT implement this interface.
 */
class Adsd3500HardwareInterface {
  public:
    /**
     * @brief Destructor
     */
    virtual ~Adsd3500HardwareInterface() = default;

    /**
     * @brief Send a read command to ADSD3500.
     * @param cmd - the command to be sent
     * @param[out] data - the variable where the read data will be stored
     * @param usDelay - the number of microseconds to wait between the host command
     * and the actual read
     * @return Status
     */
    virtual aditof::Status adsd3500_read_cmd(uint16_t cmd, uint16_t *data,
                                             unsigned int usDelay = 0) = 0;

    /**
     * @brief Send a write command to ADSD3500.
     * @param cmd - the command to be sent
     * @param data - the data to be written
     * @param usDelay - the number of microseconds to wait between the host command
     * and the actual write
     * @return Status
     */
    virtual aditof::Status adsd3500_write_cmd(uint16_t cmd, uint16_t data,
                                              unsigned int usDelay = 0) = 0;

    /**
     * @brief Send a read command to ADSD3500. This will perform a burst read making it
     *        useful for reading chunks of data.
     * @param cmd - the command to be sent
     * @param[out] readback_data - the location where the read data chunk will be stored
     * @param payload_len - the number of bytes to read
     * @return Status
     */
    virtual aditof::Status adsd3500_read_payload_cmd(uint32_t cmd,
                                                     uint8_t *readback_data,
                                                     uint16_t payload_len) = 0;

    /**
     * @brief Reads a chunk of data from ADSD3500. This will perform a burst read making it
     *        useful for reading chunks of data.
     * @param payload - the location from where to take the data chunk and read it
     * @param payload_len - the number of bytes to read
     * @return Status
     */
    virtual aditof::Status adsd3500_read_payload(uint8_t *payload,
                                                 uint16_t payload_len) = 0;

    /**
     * @brief Send a write command to ADSD3500. This will perform a burst write making it
     *        useful for writing chunks of data.
     * @param cmd - the command to be sent
     * @param payload - the location from where to take the data chunk and write it
     * @param payload_len - the number of bytes to write
     * @return Status
     */
    virtual aditof::Status adsd3500_write_payload_cmd(uint32_t cmd,
                                                      uint8_t *payload,
                                                      uint16_t payload_len) = 0;

    /**
     * @brief Send a chunk of data (payload) to ADSD3500. This will perform a burst write making it
     *        useful for writing chunks of data.
     * @param payload - the location from where to take the data chunk and write it
     * @param payload_len - the number of bytes to write
     * @return Status
     */
    virtual aditof::Status adsd3500_write_payload(uint8_t *payload,
                                                  uint16_t payload_len) = 0;

    /**
     * @brief Reset ADSD3500 chip
     * @return Status
     */
    virtual aditof::Status adsd3500_reset() = 0;

    /**
     * @brief Get the interrupt from sensor
     * @return Status
     */
    virtual aditof::Status adsd3500_getInterruptandReset() = 0;

    /**
     * @brief Register a function to be called when an ADSD3500 interrupt occurs
     * @param cb - the function to be called whenever the interrupt occurs
     * @return Status
     */
    virtual aditof::Status
    adsd3500_register_interrupt_callback(SensorInterruptCallback &cb) = 0;

    /**
     * @brief Unregister a function registered with adsd3500_register_interrupt_callback
     * @param cb - the function to be unregistered
     * @return Status
     */
    virtual aditof::Status
    adsd3500_unregister_interrupt_callback(SensorInterruptCallback &cb) = 0;

    /**
     * @brief Returns the chip status
     * @param[out] chipStatus - chip status (error) value
     * @param[out] imagerStatus - imager status (error) value
     * @return Status
     */
    virtual aditof::Status adsd3500_get_status(int &chipStatus,
                                               int &imagerStatus) = 0;
};

} // namespace aditof

#endif // ADSD3500_HARDWARE_INTERFACE_H
