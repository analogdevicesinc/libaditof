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
#ifndef CAMERA_H
#define CAMERA_H

#include "aditof/adsd_errs.h"
#include "camera_definitions.h"
#include "sdk_exports.h"
#include "status_definitions.h"

#include <functional>
#include <map>
#include <string>
#include <vector>

namespace aditof {

class Frame;
class DepthSensorInterface;

/**
 * @class Camera
 * @brief Manipulates the underlying camera system
 */
class SDK_API Camera {
  public:
    /**
     * @brief Destructor
     */
    virtual ~Camera() = default;

    /**
     * @brief Initialize the camera. This is required before performing any
     * operation on the camera.
     * @param configFilepath - The JSON configuration files which should be specific to the given module.
     * The expected value is a file name (including extension) or left empty.
     * @return Status
     */
    virtual Status initialize(const std::string &configFilepath = {}) = 0;

    /**
     * @brief Start the camera. This starts the streaming of data from the
     * camera.
     * @return Status
     */
    virtual Status start() = 0;

    /**
     * @brief Stop the camera. This makes the camera to stop streaming.
     * @return Status
     */
    virtual Status stop() = 0;

    /**
     * @brief Puts the camera into the given mode.
     * @param mode - The mode of the camera
     * @return Status
     */
    virtual Status setMode(const uint8_t &mode) = 0;

    /**
     * @brief Get the Depth Compute Library ini parameters
     * @param params - a dictionary of parameters
     * @return Status
    */
    virtual Status
    getFrameProcessParams(std::map<std::string, std::string> &params) = 0;

    /**
     * @brief Set the Depth Compute Library ini parameters
     * @param params - a dictionary of parameters
     * @return Status
    */
    virtual Status
    setFrameProcessParams(std::map<std::string, std::string> &params) = 0;

    /**
     * @brief Returns all the modes that are supported by the camera
     * @param[out] availableModes
     * @return Status
     */
    virtual Status
    getAvailableModes(std::vector<uint8_t> &availableModes) const = 0;

    /**
     * @brief Captures data from the camera and assigns it to the given frame.
     * @param frame - The frame to which the camera data should be assign
     * @return Status
     */
    virtual Status requestFrame(Frame *frame) = 0;

    /**
     * @brief Scale AB image with logarithmic base 10.
     * @param abBuffer - Pointer to the AB buffer
     * @param abWidth - Width (in pixels) of the AB buffer
     * @param abHeight - Height (in pixels) of the AB buffer
     * @param advanceScaling - If is true then use advance AB scaling.
     * @param useLogScaling - If is false is applied the normalization
     * between 0 and 255,
     * if is true is applied normalization between 0 and 255 and log10
     * @return Status
     */
    virtual void normalizeABBuffer(uint16_t *abBuffer, uint16_t abWidth,
                                   uint16_t abHeight, bool advanceScaling,
                                   bool useLogScaling) = 0;

    /**
     * @brief Scale AB image with logarithmic base 10 in a Frame instance.
     * @param frame - The frame of the camera
     * @param advanceScaling - If is true then use advance AB scaling.
     * @param useLogScaling - If is false is applied the normalization
     * between 0 and 255,
     * if is true is applied normalization between 0 and 255 and log10
     * @return Status
     */
    virtual aditof::Status normalizeABFrame(aditof::Frame *frame,
                                            bool advanceScaling,
                                            bool useLogScaling) = 0;

    /**
     * @brief Gets the current details of the camera
     * @param[out] details
     * @return Status
     */
    virtual Status getDetails(CameraDetails &details) const = 0;

    /**
     * @brief Gets the camera's list of controls
     * @param[out] controls
     * @return Status
     */
    virtual Status
    getAvailableControls(std::vector<std::string> &controls) const = 0;

    /**
     * @brief Sets a specific camera control
     * @param[in] control - Control name
	 * @param[in] value - Control value
     * @return Status
     */
    virtual Status setControl(const std::string &control,
                              const std::string &value) = 0;

    /**
     * @brief Gets the value of a specific camera control
     * @param[in] control - Control name
	 * @param[out] value - Control value
     * @return Status
     */
    virtual Status getControl(const std::string &control,
                              std::string &value) const = 0;

    /**
     * @brief Gets the sensor of the camera. This gives direct access
     * to low level configuration of the camera sensor.
     * @return std::shared_ptr<DepthSensorInterface>
     */
    virtual std::shared_ptr<DepthSensorInterface> getSensor() = 0;

    /**
     * @brief Enable the generation of a XYZ frame. The XYZ frame can be enabled
     * or disabled through .ini configuration file but if this method is explicitly called
     * then it will override the option in the .ini file. By default XYZ frame is disabled.
     * @param[in] enable - set to true to enable the generation of XYZ, set to false otherwise.
     * @return Status
     */
    virtual Status enableXYZframe(bool enable) = 0;

    /**
     * @brief Save the CFG content which is obtained from module memory to a given file path.
     * @param[in] filepath - A path to a file (including file name and extension) where the
     *                       CFG should be stored.
     * @return Status
     */
    virtual Status saveModuleCFG(const std::string &filepath) const = 0;

    /**
     * @brief Save the CCB content which is obtained from module memory to a given file path.
     * @param[in] filepath - A path to a file (including file name and extension) where the
     *                       CCB should be stored.
     * @return Status
     */
    virtual Status saveModuleCCB(const std::string &filepath) = 0;

    /**
     * @brief Enable or disable the depth processing on the frames received from the sensor
     * Must be called after getFrame() where the depth processing happens.
     * @param[in] enable - set to true to enable depth processing. Set to false otherwise.
     * @return Status
     */
    virtual Status enableDepthCompute(bool enable) = 0;

    /**
     * @brief Update the firmware of ADSD3500 with the content found in the specified file.
     * @param[in] fwFilePath - A path to a file (including file name and extension)
     *                         where the firmware for adsd3500 is stored.
     * @return Status
     */
    virtual Status adsd3500UpdateFirmware(const std::string &fwFilePath) = 0;

    /**
     * @brief Enables or disables FSYNC toggle for ADSD3500
     * @param[in] mode - 2 = Fsync pin set as HiZ ; 1 = Toggle at user specified framerate ; 0 = Toggle controlled via adsd3500ToggleFsync ;
     * @return Status
     */
    virtual Status adsd3500SetToggleMode(int mode) = 0;

    /**
     * @brief Toggles ADSD3500 FSYNC once if automated FSYNC is disabled
     * @return Status
     */
    virtual Status adsd3500ToggleFsync() = 0;

    /**
     * @brief Set the AB invalidation threshold
     * @param[in] threshold
     * @return Status
     */
    virtual Status adsd3500SetABinvalidationThreshold(int threshold) = 0;

    /**
     * @brief Get the AB invalidation threshold
     * @param[out] threshold
     * @return Status
     */
    virtual Status adsd3500GetABinvalidationThreshold(int &threshold) = 0;

    /**
     * @brief Set the confidence threshold
     * @param[in] threshold
     * @return Status
     */
    virtual Status adsd3500SetConfidenceThreshold(int threshold) = 0;

    /**
     * @brief Get the confidence threshold
     * @param[out] threshold
     * @return Status
     */
    virtual Status adsd3500GetConfidenceThreshold(int &threshold) = 0;

    /**
     * @brief Enable/disable the JBLF filter
     * @param[in] enable
     * @return Status
     */
    virtual Status adsd3500SetJBLFfilterEnableState(bool enable) = 0;

    /**
     * @brief Get the JBLF enabled state
     * @param[out] enabled
     * @return Status
     */
    virtual Status adsd3500GetJBLFfilterEnableState(bool &enabled) = 0;

    /**
     * @brief Set the JBLF filter size
     * @param[in] size - Supported sizes are: 3, 5, 7
     * @return Status
     */
    virtual Status adsd3500SetJBLFfilterSize(int size) = 0;

    /**
     * @brief Get the JBLF filter size
     * @param[out] size
     * @return Status
     */
    virtual Status adsd3500GetJBLFfilterSize(int &size) = 0;

    /**
     * @brief Set the radial threshold min
     * @param[in] threshold
     * @return Status
     */
    virtual Status adsd3500SetRadialThresholdMin(int threshold) = 0;

    /**
     * @brief Get the radial threshold min
     * @param[out] threshold
     * @return Status
     */
    virtual Status adsd3500GetRadialThresholdMin(int &threshold) = 0;

    /**
     * @brief Set the radial threshold max
     * @param[in] threshold
     * @return Status
     */
    virtual Status adsd3500SetRadialThresholdMax(int threshold) = 0;

    /**
     * @brief Get the radial threshold max
     * @param[out] threshold
     * @return Status
     */
    virtual Status adsd3500GetRadialThresholdMax(int &threshold) = 0;

    /**
     * @brief Get the sensor temperature
     * @param[out] tmpValue - Values in Celsius degree
     * @return Status
     */
    virtual Status adsd3500GetSensorTemperature(uint16_t &tmpValue) = 0;

    /**
     * @brief Get the laser temperature
     * @param[out] tmpValue - Values in Celsius degree
     * @return Status
     */
    virtual Status adsd3500GetLaserTemperature(uint16_t &tmpValue) = 0;

    /**
     * Get the ASDSD3500 firmware version from the ADSD3500
     * @param[out] fwVersion - the ADSD3500 firmware version
     * @param[out] fwHash - the ADSD3500 firmware git commit hash
    */
    virtual Status adsd3500GetFirmwareVersion(std::string &fwVersion,
                                              std::string &fwHash) = 0;

    /**
     * @brief Set ADSD3500 MIPI output speed
     * @param[in] speed - See "Set MIPI Output Speed" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual Status adsd3500SetMIPIOutputSpeed(uint16_t speed) = 0;

    /**
     * @brief Get ADSD3500 MIPI output speed
     * @param[out] speed - See "Get MIPI Output Speed" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual Status adsd3500GetMIPIOutputSpeed(uint16_t &speed) = 0;

    /**
     * @brief Get error code from the imager
     * @param[out] errcode - See "Get Imager Error Code" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual Status adsd3500GetImagerErrorCode(uint16_t &errcode) = 0;

    /**
     * @brief Set the delay for VCSEL - ADSD3100 imager only
     * @param[in] delay - See "Set VCSEL Delay" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual Status adsd3500SetVCSELDelay(uint16_t delay) = 0;

    /**
     * @brief Get the delay for VCSEL - ADSD3100 imager only
     * @param[out] delay - See "Get VCSEL Delay" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual Status adsd3500GetVCSELDelay(uint16_t &delay) = 0;

    /**
     * @brief Set JBLF Max Edge Threshold
     * @param[in] threshold - See "Set JBLF Max Edge Threshold" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual Status adsd3500SetJBLFMaxEdgeThreshold(uint16_t threshold) = 0;

    /**
     * @brief Get JBLF Max Edge Threshold
     * @param[out] threshold - See "Get JBLF Max Edge Threshold" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual Status adsd3500SetJBLFABThreshold(uint16_t threshold) = 0;

    /**
     * @brief Set JBLF Gaussian Sigma
     * @param[in] value - See "Set JBLF Gaussian Sigma" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual Status adsd3500SetJBLFGaussianSigma(uint16_t value) = 0;

    /**
     * @brief Get JBLF Gaussian Sigma
     * @param[out] value - See "Get JBLF Gaussian Sigma" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual Status adsd3500GetJBLFGaussianSigma(uint16_t &value) = 0;

    /**
     * @brief Set JBLF Exponential Term
     * @param[in] value - See "Set JBLF Exponential Term" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual Status adsd3500SetJBLFExponentialTerm(uint16_t value) = 0;

    /**
     * @brief Get JBLF Exponential Term
     * @param[out] value - See "Get JBLF Exponential Term" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual Status adsd3500GetJBLFExponentialTerm(uint16_t &value) = 0;

    /**
     * @brief Get Frame Rate
     * @param[out] fps - See "Get Frame Rate" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual Status adsd3500GetFrameRate(uint16_t &fps) = 0;

    /**
     * @brief Set Frame Rate
     * @param[out] fps - See "Set Frame Rate" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual Status adsd3500SetFrameRate(uint16_t fps) = 0;

    /**
     * @brief Set Enable Edge Confidence
     * @param[in] value - See "Set Enable Edge Confidence" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual Status adsd3500SetEnableEdgeConfidence(uint16_t value) = 0;

    /**
     * @brief Get Temperature Compensation Status
     * @param[out] value - See "Get Temperature Compensation Status" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual Status
    adsd3500GetTemperatureCompensationStatus(uint16_t &value) = 0;

    /**
     * @brief Set Enable Phase Invalidation
     * @param[out] value - See "Set Enable Phase Invalidation" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual Status adsd3500SetEnablePhaseInvalidation(uint16_t value) = 0;

    /**
     * @brief Set Enable Temperature Compensation
     * @param[out] value - See "Set Enable Temperature Compensation" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual Status adsd3500SetEnableTemperatureCompensation(uint16_t value) = 0;

    /**
     * @brief Set Enable Metadata in the AB frame
     * @param[in] value - See "Enable/Disable Output Metadata in AB Frame" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual aditof::Status adsd3500SetEnableMetadatainAB(uint16_t value) = 0;

    /**
     * @brief Get state of Enable Metadata in the AB frame
     * @param[out] value - See "Get Output Metadata in AB Frame status" at https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175x-adsd3500
     * @return Status
     */
    virtual aditof::Status adsd3500GetEnableMetadatainAB(uint16_t &value) = 0;

    /**
     * @brief Generic ADSD3500 function for commands not defined in the SDK (yet)
     * @param[in] reg - 16-bit ADSD3500 register
     * @param[in] value - 16-bit value to write to the register
     * @return Status
     */
    virtual Status adsd3500SetGenericTemplate(uint16_t reg, uint16_t value) = 0;

    /**
     * @brief Generic ADSD3500 function for commands not defined in the SDK (yet)
     * @param[in] reg - 16-bit ADSD3500 register
     * @param[out] value - 16-bit value read from the register
     * @return Status
     */
    virtual Status adsd3500GetGenericTemplate(uint16_t reg,
                                              uint16_t &value) = 0;
    /**
     * @brief Returns the chip status
     * @param[out] chipStatus - chip status (error) value
     * @param[out] imagerStatus - imager status (error) value
     * @return Status
     */
    virtual Status adsd3500GetStatus(int &chipStatus, int &imagerStatus) = 0;

    /**
     * @brief Enable/disable ccb as master 
     * @param[in] disable - set to: 1 - disable, 0 - enable
     * @return Status
     */
    virtual Status adsd3500DisableCCBM(bool disable) = 0;

    /**
     * @brief Check whether CCB as master is supported or not.
     * @param[out] supported - Will be set to true in case CCBM is supported
     * @return Status
     */
    virtual Status adsd3500IsCCBMsupported(bool &supported) = 0;

    /**
     * @brief Allows enabling or disabling the Dynamic Mode Switching.
     * NOTE: This must be enabled before configuring the sequence!
     * @param[in] enable - Set to true to enable and false to disable
     * @return Status
     */
    virtual aditof::Status
    adsd3500setEnableDynamicModeSwitching(bool enable) = 0;

    /**
     * @brief Configures the sequence to be captured.
     * @param[in] sequence - accepts a list of maximum 8 pairs. The first item of the pair refers to the mode
     * to be captured and the second item refers to how many times the mode should be repeated before moving
     * on to the next pair. The entire sequence will repeat over and over again until streaming is stopped.
     * The first item must be a valid mode. The modes being sequenced must have the exact same resolution.
     * The second item must be between 0 and 15. If 0 is set, the mode will be skipped (will repeat 0 times).
     * Usage example 1:
     * calling this: adsds3500setDynamicModeSwitchingSequence( { {6, 1}, {5, 2}, {2, 3}, {3, 4}, {2, 5} } );
     * will create sequence: 655222333322222655222...
     *
     * Usage example 2:
     * calling this: adsds3500setDynamicModeSwitchingSequence( { {6, 15}, {6, 15}, {6, 15}, {6, 15}, {6, 15}, {5, 1} } );
     * will create sequence: 6666(75 times)5666...
     *
     * NOTE: Only ADSD3030 supports setting how many times the mode should be repeated!
     * @return Status
     */
    virtual aditof::Status adsds3500setDynamicModeSwitchingSequence(
        const std::vector<std::pair<uint8_t, uint8_t>> &sequence) = 0;

    /**
    * @brief Read serial number from camera and update cache
    * @param[out] serialNumber - Will contain serial number
    * @param[in] useCacheValue - If it is false it will
    *  read from camera and if it is true it will
    *  return serialNumber from cache
    * @return Status
    */
    virtual Status readSerialNumber(std::string &serialNumber,
                                    bool useCacheValue = false) = 0;

    /**
     * @brief Provides the type of the imager
     * @param[out] imagerType - Will be set with the imager type
     * @return Status
     */
    virtual Status getImagerType(ImagerType &imagerType) const = 0;

    /**
     * @brief Save ini file to json format
     * @param savePathFile - Path to save json file
     * @return Status
     */
    virtual aditof::Status
    saveDepthParamsToJsonFile(const std::string &savePathFile) = 0;

    /**
     * @brief Load adsd parameters from json file.
     * Need setMode to apply
     * @param pathFileloadDepthParamsFromJsonFile - Path to load from json file
     * @param mode_in_use - Specify to read details for a specifc mode
     * @return Status
     */
    virtual aditof::Status loadDepthParamsFromJsonFile(
        const std::string &pathFileloadDepthParamsFromJsonFile,
        const int16_t mode_in_use = -1) = 0;

    /**
     * @brief Reset the ini parameters from the chip and sets the ones stored in CCB
     * @param mode - Camera mode to be reset
     * @return Status
     */
    virtual aditof::Status
    adsd3500ResetIniParamsForMode(const uint16_t mode) = 0;

    /**
     * @brief Set sensor configutation table
     * @param sensorConf - Configuration table name string like
     * e.g. standard, standardraw, custom and customraw
     * @return Status
     */
    virtual aditof::Status
    setSensorConfiguration(const std::string &sensorConf) = 0;

    /**
     * @brief Allow drop first frame
     * @param dropFrame - Drop the first frame if true
     * @return void
     */
    virtual void dropFirstFrame(bool dropFrame) = 0;
};

} // namespace aditof

#endif // CAMERA_H
