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

#include "camera_firmware_manager.h"
#include "adsd3500_registers.h"
#include "crc.h"
#include <aditof/log.h>
#include <chrono>
#include <fstream>
#include <thread>

namespace aditof {

// Define static constexpr member for ODR-use in C++14
constexpr int CameraFirmwareManager::UPDATE_TIMEOUT_SEC;

// Firmware upgrade header structure (16 bytes)
typedef struct {
    union {
        struct {
            uint8_t id8;                // Byte 0: 0xAD (protocol ID)
            uint16_t chunk_size16;      // Bytes 1-2: 0x0100 (256 bytes)
            uint8_t cmd8;               // Byte 3: 0x04 (FW upgrade command)
            uint32_t total_size_fw32;   // Bytes 4-7: firmware size
            uint32_t crc_of_fw32;       // Bytes 8-11: CRC-32 of firmware
            uint32_t header_checksum32; // Bytes 12-15: header checksum
        };
        uint8_t cmd_header_byte[16];
    };
} cmd_header_t;

CameraFirmwareManager::CameraFirmwareManager(
    std::shared_ptr<DepthSensorInterface> depthSensor)
    : m_depthSensor(depthSensor) {}

Status CameraFirmwareManager::updateFirmware(const std::string &fwFilePath) {
    Status status = Status::OK;

    // Flags for interrupt-based completion detection
    bool updateComplete = false;
    Adsd3500Status updateStatus = Adsd3500Status::OK;

    // Register interrupt callback for firmware update completion notification
    SensorInterruptCallback callback = [&updateComplete,
                                        &updateStatus](Adsd3500Status status) {
        updateStatus = status;
        updateComplete = true;
    };

    status = m_depthSensor->adsd3500_register_interrupt_callback(callback);
    bool interruptsAvailable = (status == Status::OK);

    // Step 1: Read chip ID to verify connectivity
    uint16_t chipId = 0;
    status = readChipId(chipId);
    if (status != Status::OK) {
        return status;
    }
    LOG(INFO) << "Chip ID verified: " << chipId;

    // Step 2: Switch to burst mode for high-speed transfer
    status = enterBurstMode();
    if (status != Status::OK) {
        return status;
    }

    // Step 3: Load firmware file into memory
    std::vector<uint8_t> firmwareBuffer;
    status = loadFirmwareFile(fwFilePath, firmwareBuffer);
    if (status != Status::OK) {
        return status;
    }

    uint32_t firmwareSize = firmwareBuffer.size();
    uint8_t *firmwareData = firmwareBuffer.data();

    LOG(INFO) << "Firmware file loaded: " << firmwareSize << " bytes";

    // Step 4: Compute CRC and send header
    uint32_t firmwareCrc = computeFirmwareCrc(firmwareData, firmwareSize);
    status = sendFirmwareHeader(firmwareSize, firmwareCrc);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to send firmware header";
        return status;
    }

    // Step 5: Transmit firmware in 256-byte chunks
    status = sendFirmwarePackets(firmwareData, firmwareSize);
    if (status != Status::OK) {
        return status;
    }

    // Step 6: Switch back to standard mode
    status = exitBurstMode();
    if (status != Status::OK) {
        return status;
    }

    // Step 7: Wait for firmware update to complete
    status = waitForUpdateCompletion(interruptsAvailable, updateComplete,
                                     updateStatus);

    // Cleanup: unregister interrupt callback
    if (interruptsAvailable) {
        m_depthSensor->adsd3500_unregister_interrupt_callback(callback);
    }

    return status;
}

Status CameraFirmwareManager::readChipId(uint16_t &chipId) {
    Status status =
        m_depthSensor->adsd3500_read_cmd(ADSD3500_REG_CHIP_ID, &chipId);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to read ADSD3500 chip ID";
        return status;
    }
    return Status::OK;
}

Status CameraFirmwareManager::enterBurstMode() {
    Status status =
        m_depthSensor->adsd3500_write_cmd(ADSD3500_REG_BURST_MODE_CMD, 0x0000);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to switch to burst mode";
        return status;
    }
    LOG(INFO) << "Entered burst mode for firmware transfer";
    return Status::OK;
}

Status CameraFirmwareManager::loadFirmwareFile(const std::string &filePath,
                                               std::vector<uint8_t> &buffer) {
    std::ifstream fwFile(filePath, std::ios::binary);
    if (!fwFile.is_open()) {
        LOG(ERROR) << "Failed to open firmware file: " << filePath;
        return Status::GENERIC_ERROR;
    }

    // Read entire file into buffer
    buffer.assign(std::istreambuf_iterator<char>(fwFile), {});

    if (buffer.empty()) {
        LOG(ERROR) << "Firmware file is empty: " << filePath;
        return Status::GENERIC_ERROR;
    }

    return Status::OK;
}

uint32_t CameraFirmwareManager::computeFirmwareCrc(const uint8_t *data,
                                                   uint32_t length) {
    uint32_t crc = crcFast(data, length, true) ^ 0xFFFFFFFF;
    return ~crc;
}

Status CameraFirmwareManager::sendFirmwareHeader(uint32_t firmwareSize,
                                                 uint32_t firmwareCrc) {
    cmd_header_t header = {}; // Zero-initialize to avoid uninitialized warning
    header.id8 = 0xAD;
    header.chunk_size16 = 0x0100; // 256 bytes
    header.cmd8 = 0x04;           // FW upgrade command
    header.total_size_fw32 = firmwareSize;
    header.crc_of_fw32 = firmwareCrc;
    header.header_checksum32 = 0;

    // Compute header checksum (sum of bytes 1-7)
    for (int i = 1; i < 8; i++) {
        header.header_checksum32 += header.cmd_header_byte[i];
    }

    Status status =
        m_depthSensor->adsd3500_write_payload(header.cmd_header_byte, 16);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to send firmware upgrade header";
        return status;
    }

    LOG(INFO) << "Firmware header sent (size=" << firmwareSize << ", CRC=0x"
              << std::hex << firmwareCrc << std::dec << ")";

    return Status::OK;
}

Status CameraFirmwareManager::sendFirmwarePackets(const uint8_t *firmwareData,
                                                  uint32_t firmwareSize) {
    // Calculate number of 256-byte packets
    int packetsToSend = (firmwareSize + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;

    LOG(INFO) << "Transmitting firmware: " << packetsToSend << " packets";

    uint8_t packetBuffer[FLASH_PAGE_SIZE];

    for (int i = 0; i < packetsToSend; i++) {
        int start = FLASH_PAGE_SIZE * i;
        int end = FLASH_PAGE_SIZE * (i + 1);

        // Copy firmware data or pad with zeros
        for (int j = start; j < end; j++) {
            if (j < static_cast<int>(firmwareSize)) {
                packetBuffer[j - start] = firmwareData[j];
            } else {
                packetBuffer[j - start] = 0x00; // Padding
            }
        }

        // Send packet
        Status status = m_depthSensor->adsd3500_write_payload(packetBuffer,
                                                              FLASH_PAGE_SIZE);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to send packet " << i << " of "
                       << packetsToSend;
            return status;
        }

        // Progress logging every 25 packets (~6.4 KB)
        if (i % PROGRESS_LOG_INTERVAL == 0 && i > 0) {
            LOG(INFO) << "Sent " << i << " / " << packetsToSend << " packets ("
                      << (i * 100 / packetsToSend) << "%)";
        }
    }

    LOG(INFO) << "All " << packetsToSend << " packets transmitted successfully";
    return Status::OK;
}

Status CameraFirmwareManager::exitBurstMode() {
    // Command to switch back to standard mode
    uint8_t switchCmd[] = {0xAD, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,
                           0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    Status status = m_depthSensor->adsd3500_write_payload(
        switchCmd, sizeof(switchCmd) / sizeof(switchCmd[0]));
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to switch back to standard mode";
        return status;
    }

    LOG(INFO) << "Returned to standard mode";
    return Status::OK;
}

Status
CameraFirmwareManager::waitForUpdateCompletion(bool interruptsAvailable,
                                               bool &updateComplete,
                                               Adsd3500Status &updateStatus) {

    if (interruptsAvailable) {
        LOG(INFO) << "Waiting for ADSD3500 firmware update to complete...";

        int secondsWaited = 0;
        const int waitStep = 1; // Check every 1 second

        while (!updateComplete && secondsWaited < UPDATE_TIMEOUT_SEC) {
            if (secondsWaited > 0 && secondsWaited % 10 == 0) {
                LOG(INFO) << "Waiting... " << secondsWaited << "s elapsed";
            }
            std::this_thread::sleep_for(std::chrono::seconds(waitStep));
            secondsWaited += waitStep;
        }

        LOG(INFO) << "Firmware update completed after " << secondsWaited
                  << " seconds";

        if (!updateComplete) {
            LOG(WARNING) << "Firmware update timeout after " << secondsWaited
                         << " seconds";
            return Status::GENERIC_ERROR;
        }

        // Check interrupt status
        if (updateStatus == Adsd3500Status::OK ||
            updateStatus == Adsd3500Status::FIRMWARE_UPDATE_COMPLETE) {
            LOG(INFO) << "ADSD3500 firmware updated successfully!";
            return Status::OK;
        } else {
            LOG(ERROR) << "Firmware update failed with status: "
                       << static_cast<int>(updateStatus);
            return Status::GENERIC_ERROR;
        }
    } else {
        // No interrupt support - use fixed delay
        LOG(INFO)
            << "Interrupt support not available - waiting fixed 60 seconds";
        std::this_thread::sleep_for(std::chrono::seconds(UPDATE_TIMEOUT_SEC));
        LOG(INFO) << "ADSD3500 firmware update completed (no verification)";
        return Status::OK;
    }
}

} // namespace aditof
