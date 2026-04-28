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
#ifndef ADSD3500_CONTROLLER_H
#define ADSD3500_CONTROLLER_H

#include <aditof/depth_sensor_interface.h>
#include <aditof/status_definitions.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace aditof {

/**
 * @class Adsd3500Controller
 * @brief Manages all ADSD3500 hardware control operations.
 *
 * This class encapsulates hardware-level control of the ADSD3500 Depth ISP,
 * including filter configurations, thresholds, MIPI settings, VCSEL timing,
 * temperature monitoring, and firmware management. All methods are thin
 * wrappers around DepthSensorInterface calls for direct hardware access.
 *
 * Responsibilities:
 * - JBLF (Joint Bilateral Filter) configuration
 * - Threshold management (confidence, radial, AB invalidation)
 * - MIPI output speed control
 * - VCSEL delay settings
 * - Temperature sensor readings
 * - Firmware version queries and updates
 * - Edge confidence and phase invalidation control
 * - Raw bypass mode configuration
 * - Dynamic Mode Switching (DMS) setup
 * - Generic template register access
 * - Chip status monitoring
 *
 * Usage:
 * Constructed with a DepthSensorInterface dependency; camera classes
 * delegate all ADSD3500 hardware operations to this controller.
 */
class Adsd3500Controller {
  public:
    /**
     * @brief Constructs the ADSD3500 hardware controller.
     * @param[in] depthSensor Shared pointer to the depth sensor interface
     */
    explicit Adsd3500Controller(
        std::shared_ptr<DepthSensorInterface> depthSensor);

    ~Adsd3500Controller() = default;

    // Prevent copying and assignment
    Adsd3500Controller(const Adsd3500Controller &) = delete;
    Adsd3500Controller &operator=(const Adsd3500Controller &) = delete;

    // ========== Firmware Management ==========
    /**
     * @brief Retrieves firmware version and hash.
     * @param[out] fwVersion Firmware version string
     * @param[out] fwHash Firmware hash string
     * @return Status::OK on success
     */
    Status getFirmwareVersion(std::string &fwVersion, std::string &fwHash);

    // ========== Toggle and Sync Control ==========
    /**
     * @brief Sets the toggle mode for the ADSD3500.
     * @param[in] mode Toggle mode value
     * @return Status::OK on success
     */
    Status setToggleMode(int mode);

    /**
     * @brief Toggles the FSYNC signal.
     * @return Status::OK on success
     */
    Status toggleFsync();

    // ========== Threshold Configuration ==========
    /**
     * @brief Sets AB (Active Brightness) invalidation threshold.
     * @param[in] threshold Threshold value
     * @return Status::OK on success
     */
    Status setABinvalidationThreshold(int threshold);

    /**
     * @brief Gets current AB invalidation threshold.
     * @param[out] threshold Current threshold value
     * @return Status::OK on success
     */
    Status getABinvalidationThreshold(int &threshold);

    /**
     * @brief Sets confidence threshold for depth filtering.
     * @param[in] threshold Confidence threshold (0-1023)
     * @return Status::OK on success
     */
    Status setConfidenceThreshold(int threshold);

    /**
     * @brief Gets current confidence threshold.
     * @param[out] threshold Current threshold value
     * @return Status::OK on success
     */
    Status getConfidenceThreshold(int &threshold);

    /**
     * @brief Sets minimum radial threshold.
     * @param[in] threshold Minimum radial distance threshold
     * @return Status::OK on success
     */
    Status setRadialThresholdMin(int threshold);

    /**
     * @brief Gets minimum radial threshold.
     * @param[out] threshold Current minimum threshold
     * @return Status::OK on success
     */
    Status getRadialThresholdMin(int &threshold);

    /**
     * @brief Sets maximum radial threshold.
     * @param[in] threshold Maximum radial distance threshold
     * @return Status::OK on success
     */
    Status setRadialThresholdMax(int threshold);

    /**
     * @brief Gets maximum radial threshold.
     * @param[out] threshold Current maximum threshold
     * @return Status::OK on success
     */
    Status getRadialThresholdMax(int &threshold);

    // ========== JBLF (Joint Bilateral Filter) Configuration ==========
    /**
     * @brief Enables or disables JBLF filtering.
     * @param[in] enable True to enable, false to disable
     * @return Status::OK on success
     */
    Status setJBLFfilterEnableState(bool enable);

    /**
     * @brief Gets JBLF filter enabled state.
     * @param[out] enabled Current JBLF state
     * @return Status::OK on success
     */
    Status getJBLFfilterEnableState(bool &enabled);

    /**
     * @brief Sets JBLF filter kernel size.
     * @param[in] size Filter size (typically 3, 5, or 7)
     * @return Status::OK on success
     */
    Status setJBLFfilterSize(int size);

    /**
     * @brief Gets JBLF filter size.
     * @param[out] size Current filter kernel size
     * @return Status::OK on success
     */
    Status getJBLFfilterSize(int &size);

    /**
     * @brief Sets JBLF maximum edge threshold.
     * @param[in] threshold Edge detection threshold
     * @return Status::OK on success
     */
    Status setJBLFMaxEdgeThreshold(uint16_t threshold);

    /**
     * @brief Sets JBLF AB threshold.
     * @param[in] threshold AB component threshold
     * @return Status::OK on success
     */
    Status setJBLFABThreshold(uint16_t threshold);

    /**
     * @brief Sets JBLF Gaussian sigma parameter.
     * @param[in] value Gaussian sigma value
     * @return Status::OK on success
     */
    Status setJBLFGaussianSigma(uint16_t value);

    /**
     * @brief Gets JBLF Gaussian sigma.
     * @param[out] value Current Gaussian sigma
     * @return Status::OK on success
     */
    Status getJBLFGaussianSigma(uint16_t &value);

    /**
     * @brief Sets JBLF exponential term.
     * @param[in] value Exponential coefficient
     * @return Status::OK on success
     */
    Status setJBLFExponentialTerm(uint16_t value);

    /**
     * @brief Gets JBLF exponential term.
     * @param[out] value Current exponential coefficient
     * @return Status::OK on success
     */
    Status getJBLFExponentialTerm(uint16_t &value);

    // ========== MIPI and Communication Settings ==========
    /**
     * @brief Sets MIPI output speed.
     * @param[in] speed MIPI speed configuration (e.g., 1500 for 1.5Gbps)
     * @return Status::OK on success
     */
    Status setMIPIOutputSpeed(uint16_t speed);

    /**
     * @brief Gets current MIPI output speed.
     * @param[out] speed Current MIPI speed
     * @return Status::OK on success
     */
    Status getMIPIOutputSpeed(uint16_t &speed);

    /**
     * @brief Enables raw bypass mode (disables ISP depth computation).
     * @param[in] enable True for raw sensor data, false for processed depth
     * @return Status::OK on success
     */
    Status setRawBypassMode(bool enable);

    /**
     * @brief Enables deskew operation at stream start.
     * @param[in] value 1 to enable, 0 to disable
     * @return Status::OK on success
     */
    Status setEnableDeskewAtStreamOn(uint16_t value);

    // ========== VCSEL Timing ==========
    /**
     * @brief Sets VCSEL (laser) delay in nanoseconds.
     * @param[in] delay Delay value in ns
     * @return Status::OK on success
     */
    Status setVCSELDelay(uint16_t delay);

    /**
     * @brief Gets current VCSEL delay.
     * @param[out] delay Current delay in ns
     * @return Status::OK on success
     */
    Status getVCSELDelay(uint16_t &delay);

    // ========== Frame Rate Control ==========
    /**
     * @brief Sets the frame rate in fps.
     * @param[in] fps Desired frame rate
     * @return Status::OK on success
     */
    Status setFrameRate(uint16_t fps);

    /**
     * @brief Gets current frame rate.
     * @param[out] fps Current frame rate in fps
     * @return Status::OK on success
     */
    Status getFrameRate(uint16_t &fps);

    // ========== Advanced Features ==========
    /**
     * @brief Enables edge confidence computation.
     * @param[in] value 1 to enable, 0 to disable
     * @return Status::OK on success
     */
    Status setEnableEdgeConfidence(uint16_t value);

    /**
     * @brief Enables phase invalidation filtering.
     * @param[in] value 1 to enable, 0 to disable
     * @return Status::OK on success
     */
    Status setEnablePhaseInvalidation(uint16_t value);

    /**
     * @brief Enables temperature compensation.
     * @param[in] value 1 to enable, 0 to disable
     * @return Status::OK on success
     */
    Status setEnableTemperatureCompensation(uint16_t value);

    /**
     * @brief Gets temperature compensation status.
     * @param[out] value Current compensation state
     * @return Status::OK on success
     */
    Status getTemperatureCompensationStatus(uint16_t &value);

    /**
     * @brief Enables metadata embedding in AB frame.
     * @param[in] value 1 to enable, 0 to disable
     * @return Status::OK on success
     */
    Status setEnableMetadatainAB(uint16_t value);

    /**
     * @brief Gets metadata in AB state.
     * @param[out] value Current metadata state
     * @return Status::OK on success
     */
    Status getEnableMetadatainAB(uint16_t &value);

    // ========== Generic Register Access ==========
    /**
     * @brief Sets a generic template register.
     * @param[in] reg Register address
     * @param[in] value Value to write
     * @return Status::OK on success
     */
    Status setGenericTemplate(uint16_t reg, uint16_t value);

    /**
     * @brief Reads a generic template register.
     * @param[in] reg Register address
     * @param[out] value Register value
     * @return Status::OK on success
     */
    Status getGenericTemplate(uint16_t reg, uint16_t &value);

    // ========== Status and Monitoring ==========
    /**
     * @brief Gets ADSD3500 and imager status.
     * @param[out] chipStatus ADSD3500 chip status
     * @param[out] imagerStatus Imager sensor status
     * @return Status::OK on success
     */
    Status getStatus(int &chipStatus, int &imagerStatus);

    /**
     * @brief Gets imager error code.
     * @param[out] errcode Error code from imager
     * @return Status::OK on success
     */
    Status getImagerErrorCode(uint16_t &errcode);

    /**
     * @brief Reads sensor die temperature with optional settling delay.
     * @param[out] tmpValue Temperature in sensor-specific units
     * @param[in] usDelay Optional microsecond delay for sensor settling (default: 0)
     * @return Status::OK on success
     */
    Status getSensorTemperature(uint16_t &tmpValue, unsigned int usDelay = 0);

    /**
     * @brief Reads laser (VCSEL) temperature with optional settling delay.
     * @param[out] tmpValue Temperature in sensor-specific units
     * @param[in] usDelay Optional microsecond delay for sensor settling (default: 0)
     * @return Status::OK on success
     */
    Status getLaserTemperature(uint16_t &tmpValue, unsigned int usDelay = 0);

    // ========== Calibration and Mode Management ==========
    /**
     * @brief Disables CCB (Camera Calibration Block) mode.
     * @param[in] disable True to disable CCBM
     * @return Status::OK on success
     */
    Status disableCCBM(bool disable);

    /**
     * @brief Checks if CCBM is supported by hardware.
     * @param[out] supported True if CCBM supported
     * @return Status::OK on success
     */
    Status isCCBMsupported(bool &supported);

    /**
     * @brief Enables Dynamic Mode Switching (DMS).
     * @param[in] en True to enable DMS
     * @return Status::OK on success
     */
    Status setEnableDynamicModeSwitching(bool en);

    /**
     * @brief Sets DMS sequence (mode, repeat_count pairs).
     * @param[in] sequence Vector of (mode, repeat) pairs
     * @return Status::OK on success
     */
    Status setDynamicModeSwitchingSequence(
        const std::vector<std::pair<uint8_t, uint8_t>> &sequence);

  private:
    std::shared_ptr<DepthSensorInterface> m_depthSensor; ///< Sensor interface
    std::shared_ptr<Adsd3500HardwareInterface>
        m_adsd3500Hardware; ///< ADSD3500 hardware interface
};

} // namespace aditof

#endif // ADSD3500_CONTROLLER_H
