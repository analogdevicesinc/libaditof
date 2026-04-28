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
#include "calibration_manager.h"
#include "crc.h"

#include <aditof/adsd3500_hardware_interface.h>
#include <aditof/log.h>
#include <cstring>
#include <fstream>
#include <vector>

namespace aditof {

// C++14 requires out-of-class definition for static constexpr members
constexpr int CalibrationManager::NR_READADSD3500CCB;

CalibrationManager::CalibrationManager(
    std::shared_ptr<DepthSensorInterface> depthSensor)
    : m_depthSensor(depthSensor), m_adsd3500Hardware(nullptr),
      m_xyzTable({nullptr, nullptr, nullptr}) {
    memset(&m_xyz_dealias_data, 0, sizeof(m_xyz_dealias_data));

    // Cast to Adsd3500HardwareInterface if sensor supports it
    m_adsd3500Hardware =
        std::dynamic_pointer_cast<Adsd3500HardwareInterface>(depthSensor);

    if (!m_adsd3500Hardware) {
        LOG(WARNING) << "Sensor does not support ADSD3500 hardware interface";
    }
}

CalibrationManager::~CalibrationManager() { cleanupXYZtables(); }

Status CalibrationManager::readCCB(std::string &ccb) {
    Status status = Status::OK;

    if (!m_adsd3500Hardware) {
        LOG(ERROR) << "ADSD3500 hardware interface not available";
        return Status::UNAVAILABLE;
    }

    uint8_t ccbHeader[16] = {0};
    ccbHeader[0] = 1;

    status = m_adsd3500Hardware->adsd3500_read_payload_cmd(0x13, ccbHeader, 16);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to get ccb command header";
        return status;
    }

    uint16_t chunkSize;
    uint32_t ccbFileSize;
    uint32_t crcOfCCB;

    memcpy(&chunkSize, ccbHeader + 1, 2);
    memcpy(&ccbFileSize, ccbHeader + 4, 4);
    memcpy(&crcOfCCB, ccbHeader + 12, 4);

    // Validate header values to prevent division by zero and unbounded allocation
    if (chunkSize == 0 || ccbFileSize == 0 || ccbFileSize > MAX_CCB_FILE_SIZE) {
        LOG(ERROR) << "Invalid CCB header: chunkSize=" << chunkSize
                   << " fileSize=" << ccbFileSize;
        return Status::GENERIC_ERROR;
    }

    uint16_t numOfChunks = ccbFileSize / chunkSize;
    std::vector<uint8_t> ccbContent(ccbFileSize); // RAII: automatic cleanup

    for (int i = 0; i < numOfChunks; i++) {
        status = m_adsd3500Hardware->adsd3500_read_payload(
            ccbContent.data() + i * chunkSize, chunkSize);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to read chunk number " << i << " out of "
                       << numOfChunks + 1 << " chunks for adsd3500!";
            return status;
        }
    }

    if (ccbFileSize % chunkSize != 0) {
        status = m_adsd3500Hardware->adsd3500_read_payload(
            ccbContent.data() + numOfChunks * chunkSize,
            ccbFileSize % chunkSize);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to read chunk number " << numOfChunks + 1
                       << " out of " << numOfChunks + 1
                       << " chunks for adsd3500!";
            return status;
        }
    }

    uint8_t switchBuf[] = {0xAD, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,
                           0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    status = m_adsd3500Hardware->adsd3500_write_payload(
        switchBuf, sizeof(switchBuf) / sizeof(switchBuf[0]));
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to switch adsd3500 to standard mode!";
        return status;
    }

    LOG(INFO) << "Successfully read ccb from adsd3500. Checking crc...";

    uint32_t computedCrc =
        crcFast(ccbContent.data(), ccbFileSize - 4, true) ^ 0xFFFFFFFF;

    if (crcOfCCB == ~computedCrc) {
        LOG(INFO) << "Crc of ccb is valid.";
    } else {
        LOG(ERROR) << "Invalid crc for ccb read from memory!";
        return Status::GENERIC_ERROR;
    }

    ccb = std::string(reinterpret_cast<char *>(ccbContent.data()),
                      ccbFileSize - 4);

    return status;
}

Status CalibrationManager::saveCCBToFile(const std::string &filepath) {
    Status status = Status::GENERIC_ERROR; // Defining with error for ccb read

    if (filepath.empty()) {
        LOG(ERROR) << "File path where CCB should be written is empty.";
        return Status::INVALID_ARGUMENT;
    }

    std::string ccbContent;
    for (int i = 0; (i < NR_READADSD3500CCB && status != Status::OK); i++) {
        LOG(INFO) << "readCCB read attempt nr :" << i;
        status = readCCB(ccbContent);
    }

    if (status != Status::OK) {
        LOG(ERROR) << "Failed to read CCB from adsd3500 module after "
                   << NR_READADSD3500CCB << " reads!";
        return Status::GENERIC_ERROR;
    }

    std::ofstream destination(filepath, std::ios::binary);
    destination << ccbContent;

    return Status::OK;
}

Status CalibrationManager::getXYZDealiasData(uint16_t mode,
                                             TofiXYZDealiasData &data) const {
    if (mode > MAX_N_MODES) {
        return Status::INVALID_ARGUMENT;
    }
    data = m_xyz_dealias_data[mode];
    return Status::OK;
}

void CalibrationManager::setXYZDealiasData(uint16_t mode,
                                           const TofiXYZDealiasData &data) {
    if (mode <= MAX_N_MODES) {
        m_xyz_dealias_data[mode] = data;
    }
}

void CalibrationManager::cleanupXYZtables() {
    if (m_xyzTable.p_x_table) {
        free((void *)m_xyzTable.p_x_table);
        m_xyzTable.p_x_table = nullptr;
    }
    if (m_xyzTable.p_y_table) {
        free((void *)m_xyzTable.p_y_table);
        m_xyzTable.p_y_table = nullptr;
    }
    if (m_xyzTable.p_z_table) {
        free((void *)m_xyzTable.p_z_table);
        m_xyzTable.p_z_table = nullptr;
    }
}

} // namespace aditof
