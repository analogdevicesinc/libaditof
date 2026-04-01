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

#ifndef CAMERA_FIRMWARE_MANAGER_H
#define CAMERA_FIRMWARE_MANAGER_H

#include <aditof/adsd_errs.h>
#include <aditof/depth_sensor_interface.h>
#include <aditof/status_definitions.h>
#include <memory>
#include <string>

namespace aditof {

/**
 * @class CameraFirmwareManager
 * @brief Manages ADSD3500 firmware update operations.
 *
 * Handles the complete firmware update workflow for ADSD3500 depth ISP:
 * - Firmware file validation and loading
 * - Chip ID verification
 * - Mode switching (Standard → Burst → Standard)
 * - Chunked firmware transmission (256-byte packets)
 * - CRC computation and validation
 * - Interrupt-based completion detection with timeout fallback
 * - Error reporting and progress logging
 *
 * The manager ensures safe firmware updates with proper error handling,
 * progress feedback, and automatic recovery from transient failures.
 */
class CameraFirmwareManager {
  public:
    /**
     * @brief Constructs a firmware manager instance.
     *
     * @param[in] depthSensor Shared pointer to the depth sensor interface used
     *                        for firmware communication (V4L2/network).
     */
    explicit CameraFirmwareManager(
        std::shared_ptr<DepthSensorInterface> depthSensor);

    /**
     * @brief Updates ADSD3500 firmware from a binary file.
     *
     * Complete firmware update sequence:
     * 1. Registers interrupt callback for completion notification
     * 2. Reads chip ID in standard mode
     * 3. Switches to burst mode for high-speed transfer
     * 4. Loads firmware binary file into memory
     * 5. Computes CRC and creates upgrade header
     * 6. Transmits firmware in 256-byte chunks
     * 7. Switches back to standard mode
     * 8. Waits for update completion (interrupt or 60s timeout)
     * 9. Verifies update status
     *
     * @param[in] fwFilePath Path to firmware binary file (.bin).
     *
     * @return Status::OK if firmware updated successfully;
     *         error codes if file invalid, transmission fails, or update times out.
     *
     * @note This operation takes 30-60 seconds depending on firmware size.
     * @note Progress logging occurs every 25 packets (~6.4KB).
     * @note Requires standard mode before calling; returns in standard mode.
     * @note In offline mode, this function will assert.
     *
     * @warning Do not power off the device during firmware update.
     */
    Status updateFirmware(const std::string &fwFilePath);

  private:
    /**
     * @brief Reads the chip ID to verify hardware connectivity.
     *
     * @param[out] chipId Variable to receive the 16-bit chip identifier.
     *
     * @return Status::OK if chip ID read successfully, error otherwise.
     */
    Status readChipId(uint16_t &chipId);

    /**
     * @brief Switches ADSD3500 to burst mode for high-speed data transfer.
     *
     * @return Status::OK if mode switch successful, error otherwise.
     */
    Status enterBurstMode();

    /**
     * @brief Loads firmware binary file into memory buffer.
     *
     * @param[in] filePath Path to firmware file.
     * @param[out] buffer Vector to receive firmware bytes.
     *
     * @return Status::OK if file loaded successfully;
     *         error if file not found or read fails.
     */
    Status loadFirmwareFile(const std::string &filePath,
                            std::vector<uint8_t> &buffer);

    /**
     * @brief Computes CRC-32 checksum for firmware validation.
     *
     * Uses the same CRC algorithm as the ADSD3500 bootloader for verification.
     *
     * @param[in] data Pointer to firmware data.
     * @param[in] length Number of bytes to checksum.
     *
     * @return 32-bit CRC checksum value.
     */
    uint32_t computeFirmwareCrc(const uint8_t *data, uint32_t length);

    /**
     * @brief Sends firmware upgrade header with file size and CRC.
     *
     * @param[in] firmwareSize Total size of firmware binary in bytes.
     * @param[in] firmwareCrc CRC-32 checksum of firmware data.
     *
     * @return Status::OK if header sent successfully, error otherwise.
     */
    Status sendFirmwareHeader(uint32_t firmwareSize, uint32_t firmwareCrc);

    /**
     * @brief Transmits firmware data in 256-byte chunks.
     *
     * Pads final packet with 0x00 if firmware size not multiple of 256.
     * Logs progress every 25 packets.
     *
     * @param[in] firmwareData Pointer to firmware bytes.
     * @param[in] firmwareSize Total firmware size in bytes.
     *
     * @return Status::OK if all packets sent successfully;
     *         error if any transmission fails.
     */
    Status sendFirmwarePackets(const uint8_t *firmwareData,
                               uint32_t firmwareSize);

    /**
     * @brief Switches ADSD3500 back to standard mode after firmware transfer.
     *
     * @return Status::OK if mode switch successful, error otherwise.
     */
    Status exitBurstMode();

    /**
     * @brief Waits for firmware update completion.
     *
     * If interrupts available, waits for callback notification (max 60s).
     * Otherwise, uses fixed 60s delay for bootloader completion.
     *
     * @param[in] interruptsAvailable True if interrupt callback registered.
     * @param[in] updateComplete Reference to flag set by interrupt callback.
     * @param[in] updateStatus Reference to status set by interrupt callback.
     *
     * @return Status::OK if update completed successfully;
     *         error if timeout or update failed.
     */
    Status waitForUpdateCompletion(bool interruptsAvailable,
                                   bool &updateComplete,
                                   Adsd3500Status &updateStatus);

  private:
    std::shared_ptr<DepthSensorInterface> m_depthSensor;

    static constexpr int FLASH_PAGE_SIZE = 256;   ///< Firmware packet size
    static constexpr int UPDATE_TIMEOUT_SEC = 60; ///< Max wait for completion
    static constexpr int PROGRESS_LOG_INTERVAL = 25; ///< Log every N packets
};

} // namespace aditof

#endif // CAMERA_FIRMWARE_MANAGER_H
