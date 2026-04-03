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
#include "adsd3500_controller.h"
#include "adsd3500_registers.h"
#include <aditof/log.h>
#include <cassert>
#include <sstream>

namespace aditof {

Adsd3500Controller::Adsd3500Controller(
    std::shared_ptr<DepthSensorInterface> depthSensor)
    : m_depthSensor(depthSensor) {}

// ========== Firmware Management ==========
Status Adsd3500Controller::getFirmwareVersion(std::string &fwVersion,
                                              std::string &fwHash) {
    Status status = Status::OK;

    uint8_t fwData[44] = {0};
    fwData[0] = uint8_t(1);

    status = m_depthSensor->adsd3500_read_payload_cmd(0x05, fwData, 44);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to read firmware version from ADSD3500";
        return status;
    }

    // Extract firmware version (first 4 bytes: major, minor, patch, build)
    uint8_t fwVersionArray[4];
    for (int i = 0; i < 4; i++) {
        fwVersionArray[i] = fwData[i];
    }

    std::ostringstream oss;
    oss << static_cast<int>(fwVersionArray[0]) << "."
        << static_cast<int>(fwVersionArray[1]) << "."
        << static_cast<int>(fwVersionArray[2]) << "."
        << static_cast<int>(fwVersionArray[3]);
    fwVersion = oss.str();

    // Extract Git hash (remaining 40 bytes)
    fwHash = std::string(reinterpret_cast<char *>(&fwData[4]), 40);

    return status;
}

// ========== Toggle and Sync Control ==========
Status Adsd3500Controller::setToggleMode(int mode) {
    Status status = Status::OK;

    /*mode = 2: adsd3500 fsync does not automatically toggle - Pin set as input (Slave)*/
    /*mode = 1: adsd3500 fsync automatically toggles at user specified framerate*/
    /*mode = 0: adsd3500 fsync does not automatically toggle*/

    status =
        m_depthSensor->adsd3500_write_cmd(ADSD3500_REG_FSYNC_TOGGLE_MODE, mode);
    if (status != Status::OK) {
        LOG(ERROR) << "Unable to set FSYNC Toggle mode!";
    }

    return status;
}

Status Adsd3500Controller::toggleFsync() {
    Status status =
        m_depthSensor->adsd3500_write_cmd(ADSD3500_REG_FSYNC_TOGGLE, 0x0000);
    if (status != Status::OK) {
        LOG(ERROR) << "Unable to Toggle FSYNC!";
    }
    return status;
}

// ========== Threshold Configuration ==========
Status Adsd3500Controller::setABinvalidationThreshold(int threshold) {
    return m_depthSensor->adsd3500_write_cmd(ADSD3500_REG_AB_THRESHOLD_SET,
                                             threshold);
}

Status Adsd3500Controller::getABinvalidationThreshold(int &threshold) {
    threshold = 0;
    return m_depthSensor->adsd3500_read_cmd(
        ADSD3500_REG_AB_THRESHOLD_GET,
        reinterpret_cast<uint16_t *>(&threshold));
}

Status Adsd3500Controller::setConfidenceThreshold(int threshold) {
    return m_depthSensor->adsd3500_write_cmd(
        ADSD3500_REG_CONFIDENCE_THRESHOLD_SET, threshold);
}

Status Adsd3500Controller::getConfidenceThreshold(int &threshold) {
    threshold = 0;
    return m_depthSensor->adsd3500_read_cmd(
        ADSD3500_REG_CONFIDENCE_THRESHOLD_GET,
        reinterpret_cast<uint16_t *>(&threshold));
}

Status Adsd3500Controller::setRadialThresholdMin(int threshold) {
    return m_depthSensor->adsd3500_write_cmd(
        ADSD3500_REG_RADIAL_THRESHOLD_MIN_SET, threshold);
}

Status Adsd3500Controller::getRadialThresholdMin(int &threshold) {
    threshold = 0;
    return m_depthSensor->adsd3500_read_cmd(
        ADSD3500_REG_RADIAL_THRESHOLD_MIN_GET,
        reinterpret_cast<uint16_t *>(&threshold));
}

Status Adsd3500Controller::setRadialThresholdMax(int threshold) {
    return m_depthSensor->adsd3500_write_cmd(
        ADSD3500_REG_RADIAL_THRESHOLD_MAX_SET, threshold);
}

Status Adsd3500Controller::getRadialThresholdMax(int &threshold) {
    threshold = 0;
    return m_depthSensor->adsd3500_read_cmd(
        ADSD3500_REG_RADIAL_THRESHOLD_MAX_GET,
        reinterpret_cast<uint16_t *>(&threshold));
}

// ========== JBLF (Joint Bilateral Filter) Configuration ==========
Status Adsd3500Controller::setJBLFfilterEnableState(bool enable) {
    return m_depthSensor->adsd3500_write_cmd(
        ADSD3500_REG_JBLF_ENABLE_SET, enable ? uint16_t(1) : uint16_t(0));
}

Status Adsd3500Controller::getJBLFfilterEnableState(bool &enabled) {
    uint16_t enable = 0;
    Status status =
        m_depthSensor->adsd3500_read_cmd(ADSD3500_REG_JBLF_ENABLE_GET, &enable);
    enabled = (enable == 1);
    return status;
}

Status Adsd3500Controller::setJBLFfilterSize(int size) {
    return m_depthSensor->adsd3500_write_cmd(ADSD3500_REG_JBLF_SIZE_SET, size);
}

Status Adsd3500Controller::getJBLFfilterSize(int &size) {
    size = 0;
    return m_depthSensor->adsd3500_read_cmd(
        ADSD3500_REG_JBLF_SIZE_GET, reinterpret_cast<uint16_t *>(&size));
}

Status Adsd3500Controller::setJBLFMaxEdgeThreshold(uint16_t threshold) {
    return m_depthSensor->adsd3500_write_cmd(
        ADSD3500_REG_JBLF_MAX_EDGE_THRESHOLD, threshold);
}

Status Adsd3500Controller::setJBLFABThreshold(uint16_t threshold) {
    return m_depthSensor->adsd3500_write_cmd(ADSD3500_REG_JBLF_AB_THRESHOLD,
                                             threshold);
}

Status Adsd3500Controller::setJBLFGaussianSigma(uint16_t value) {
    return m_depthSensor->adsd3500_write_cmd(
        ADSD3500_REG_JBLF_GAUSSIAN_SIGMA_SET, value);
}

Status Adsd3500Controller::getJBLFGaussianSigma(uint16_t &value) {
    return m_depthSensor->adsd3500_read_cmd(
        ADSD3500_REG_JBLF_GAUSSIAN_SIGMA_GET, &value);
}

Status Adsd3500Controller::setJBLFExponentialTerm(uint16_t value) {
    return m_depthSensor->adsd3500_write_cmd(
        ADSD3500_REG_JBLF_EXPONENTIAL_TERM_SET, value);
}

Status Adsd3500Controller::getJBLFExponentialTerm(uint16_t &value) {
    return m_depthSensor->adsd3500_read_cmd(
        ADSD3500_REG_JBLF_EXPONENTIAL_TERM_GET, &value);
}

// ========== MIPI and Communication Settings ==========
Status Adsd3500Controller::setMIPIOutputSpeed(uint16_t speed) {
    return m_depthSensor->adsd3500_write_cmd(ADSD3500_REG_MIPI_OUTPUT_SPEED_SET,
                                             speed);
}

Status Adsd3500Controller::getMIPIOutputSpeed(uint16_t &speed) {
    return m_depthSensor->adsd3500_read_cmd(ADSD3500_REG_MIPI_OUTPUT_SPEED_GET,
                                            &speed);
}

Status Adsd3500Controller::setRawBypassMode(bool enable) {
    return m_depthSensor->adsd3500_write_cmd(ADSD3500_REG_RAW_BYPASS_MODE,
                                             enable ? 1 : 0);
}

Status Adsd3500Controller::setEnableDeskewAtStreamOn(uint16_t value) {
    return m_depthSensor->adsd3500_write_cmd(ADSD3500_REG_ENABLE_DESKEW, value);
}

// ========== VCSEL Timing ==========
Status Adsd3500Controller::setVCSELDelay(uint16_t delay) {
    return m_depthSensor->adsd3500_write_cmd(ADSD3500_REG_VCSEL_DELAY_SET,
                                             delay);
}

Status Adsd3500Controller::getVCSELDelay(uint16_t &delay) {
    return m_depthSensor->adsd3500_read_cmd(ADSD3500_REG_VCSEL_DELAY_GET,
                                            &delay);
}

// ========== Frame Rate Control ==========
Status Adsd3500Controller::setFrameRate(uint16_t fps) {
    if (fps == 0) {
        fps = 10;
        LOG(WARNING) << "Using a default frame rate of " << fps;
    }

    Status status = m_depthSensor->setControl("fps", std::to_string(fps));
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to set frame rate to " << fps << " fps";
    } else {
        LOG(INFO) << "Frame rate set to " << fps << " fps";
    }
    return status;
}

Status Adsd3500Controller::getFrameRate(uint16_t &fps) {
    return m_depthSensor->adsd3500_read_cmd(ADSD3500_REG_FRAME_RATE_GET, &fps);
}

// ========== Advanced Features ==========
Status Adsd3500Controller::setEnableEdgeConfidence(uint16_t value) {
    return m_depthSensor->adsd3500_write_cmd(
        ADSD3500_REG_ENABLE_EDGE_CONFIDENCE, value);
}

Status Adsd3500Controller::setEnablePhaseInvalidation(uint16_t value) {
    return m_depthSensor->adsd3500_write_cmd(
        ADSD3500_REG_ENABLE_PHASE_INVALIDATION, value);
}

Status Adsd3500Controller::setEnableTemperatureCompensation(uint16_t value) {
    return m_depthSensor->adsd3500_write_cmd(
        ADSD3500_REG_ENABLE_TEMP_COMPENSATION, value);
}

Status Adsd3500Controller::getTemperatureCompensationStatus(uint16_t &value) {
    return m_depthSensor->adsd3500_read_cmd(
        ADSD3500_REG_TEMP_COMPENSATION_STATUS, &value);
}

Status Adsd3500Controller::setEnableMetadatainAB(uint16_t value) {
    return m_depthSensor->adsd3500_write_cmd(ADSD3500_REG_ENABLE_METADATA_SET,
                                             value);
}

Status Adsd3500Controller::getEnableMetadatainAB(uint16_t &value) {
    return m_depthSensor->adsd3500_read_cmd(ADSD3500_REG_ENABLE_METADATA_GET,
                                            &value);
}

// ========== Generic Register Access ==========
Status Adsd3500Controller::setGenericTemplate(uint16_t reg, uint16_t value) {
    return m_depthSensor->adsd3500_write_cmd(reg, value);
}

Status Adsd3500Controller::getGenericTemplate(uint16_t reg, uint16_t &value) {
    return m_depthSensor->adsd3500_read_cmd(reg, &value);
}

// ========== Status and Monitoring ==========
Status Adsd3500Controller::getStatus(int &chipStatus, int &imagerStatus) {
    return m_depthSensor->adsd3500_get_status(chipStatus, imagerStatus);
}

Status Adsd3500Controller::getImagerErrorCode(uint16_t &errcode) {
    return m_depthSensor->adsd3500_read_cmd(ADSD3500_REG_IMAGER_STATUS,
                                            &errcode);
}

Status Adsd3500Controller::getSensorTemperature(uint16_t &tmpValue,
                                                unsigned int usDelay) {
    Status status = m_depthSensor->adsd3500_read_cmd(
        ADSD3500_REG_SENSOR_TEMPERATURE, &tmpValue, usDelay);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to get sensor temperature!";
    }
    return status;
}

Status Adsd3500Controller::getLaserTemperature(uint16_t &tmpValue,
                                               unsigned int usDelay) {
    Status status = m_depthSensor->adsd3500_read_cmd(
        ADSD3500_REG_LASER_TEMPERATURE, &tmpValue, usDelay);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to get laser temperature!";
    }
    return status;
}

// ========== Calibration and Mode Management ==========
Status Adsd3500Controller::disableCCBM(bool disable) {
    uint16_t value = disable ? 1 : 0;
    Status status =
        m_depthSensor->adsd3500_write_cmd(ADSD3500_REG_DISABLE_CCBM, value);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to " << (disable ? "disable" : "enable")
                   << " CCBM module";
    }
    return status;
}

Status Adsd3500Controller::isCCBMsupported(bool &supported) {
    Status status = Status::OK;

    uint16_t chipStatusReg = 0;
    status = m_depthSensor->adsd3500_read_cmd(ADSD3500_REG_CHIP_STATUS,
                                              &chipStatusReg);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to get chip status from ADSD3500 status "
                        "register";
        return status;
    }

    // Check CCBM support bit (bit 0)
    supported = (chipStatusReg & ADSD3500_CHIP_STATUS_CCBM_SUPPORT_BIT) != 0;

    return status;
}

Status Adsd3500Controller::setEnableDynamicModeSwitching(bool en) {
    uint16_t value = en ? 1 : 0;
    Status status = m_depthSensor->adsd3500_write_cmd(
        ADSD3500_REG_ENABLE_DYNAMIC_MODE_SWITCHING, value);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to " << (en ? "enable" : "disable")
                     << " Dynamic Mode Switching";
    }
    return status;
}

Status Adsd3500Controller::setDynamicModeSwitchingSequence(
    const std::vector<std::pair<uint8_t, uint8_t>> &sequence) {
    Status status = Status::OK;

    // Maximum 12 pairs allowed by hardware
    if (sequence.size() > 12) {
        LOG(ERROR) << "Dynamic Mode Switching sequence too long (max 12 pairs)";
        return Status::INVALID_ARGUMENT;
    }

    // Pack sequence into payload: [mode0, repeat0, mode1, repeat1, ...]
    uint8_t payload[24] = {0};
    for (size_t i = 0; i < sequence.size(); i++) {
        payload[i * 2] = sequence[i].first;      // mode
        payload[i * 2 + 1] = sequence[i].second; // repeat count
    }

    // Write sequence to ADSD3500
    status = m_depthSensor->adsd3500_write_payload_cmd(
        ADSD3500_REG_DYNAMIC_MODE_SEQUENCE, payload,
        static_cast<uint16_t>(sequence.size() * 2));
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to set Dynamic Mode Switching sequence";
    }

    return status;
}

} // namespace aditof
