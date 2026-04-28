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
#ifndef CAMERA_ITOF_H
#define CAMERA_ITOF_H

#include "adsd3500_controller.h"
#include "calibration_manager.h"
#include "camera_configuration.h"
#include "recording_manager.h"
#include "sensor_config_helper.h"
#include "tofi/tofi_compute.h"
#include "tofi/tofi_config.h"
#include "tofi/tofi_util.h"
#include <aditof/adsd_errs.h>
#include <aditof/camera.h>
#include <aditof/depth_sensor_interface.h>
#include <map>
#include <memory>
#include <unordered_map>

#define NR_READADSD3500CCB 3

namespace aditof {
class Adsd3500Controller;
class CalibrationManager;
class CameraConfiguration;
class DepthParameterMapper; // Forward declaration to break circular dependency
class RecordingManager;
class SensorConfigHelper;
} // namespace aditof

class CameraItof : public aditof::Camera {
  public:
    CameraItof(std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
               const std::string &ubootVersion,
               const std::string &kernelVersion,
               const std::string &sdCardImageVersion,
               const std::string &netLinkTest);
    ~CameraItof();

  public: // implements Camera
    aditof::Status initialize(const std::string &configFilepath = {}) override;
    aditof::Status start() override;
    aditof::Status stop() override;
    aditof::Status setMode(const uint8_t &mode) override;
    aditof::Status
    getAvailableModes(std::vector<uint8_t> &availableModes) const override;
    aditof::Status requestFrame(aditof::Frame *frame,
                                uint32_t index = 0) override;
    aditof::Status getDetails(aditof::CameraDetails &details) const override;
    aditof::Status
    getAvailableControls(std::vector<std::string> &controls) const override;
    aditof::Status setControl(const std::string &control,
                              const std::string &value) override;
    aditof::Status getControl(const std::string &control,
                              std::string &value) const override;
    std::shared_ptr<aditof::DepthSensorInterface> getSensor() override;
    aditof::Status enableXYZframe(bool enable) override;
    aditof::Status saveModuleCFG(const std::string &filepath) const override;
    aditof::Status saveModuleCCB(const std::string &filepath) override;
    aditof::Status enableDepthCompute(bool enable) override;
    aditof::Status adsd3500UpdateFirmware(const std::string &filePath) override;
    aditof::Status adsd3500SetToggleMode(int mode) override;
    aditof::Status adsd3500ToggleFsync() override;
    aditof::Status adsd3500SetABinvalidationThreshold(int threshold) override;
    aditof::Status adsd3500GetABinvalidationThreshold(int &threshold) override;
    aditof::Status adsd3500SetConfidenceThreshold(int threshold) override;
    aditof::Status adsd3500GetConfidenceThreshold(int &threshold) override;
    aditof::Status adsd3500SetJBLFfilterEnableState(bool enable) override;
    aditof::Status adsd3500GetJBLFfilterEnableState(bool &enabled) override;
    aditof::Status adsd3500SetJBLFfilterSize(int size) override;
    aditof::Status adsd3500GetJBLFfilterSize(int &size) override;
    aditof::Status adsd3500SetRadialThresholdMin(int threshold) override;
    aditof::Status adsd3500GetRadialThresholdMin(int &threshold) override;
    aditof::Status adsd3500SetRadialThresholdMax(int threshold) override;
    aditof::Status adsd3500GetRadialThresholdMax(int &threshold) override;
    aditof::Status adsd3500GetSensorTemperature(uint16_t &tmpValue) override;
    aditof::Status adsd3500GetLaserTemperature(uint16_t &tmpValue) override;
    aditof::Status adsd3500GetFirmwareVersion(std::string &fwVersion,
                                              std::string &fwHash) override;
    aditof::Status adsd3500SetMIPIOutputSpeed(uint16_t speed) override;
    aditof::Status adsd3500GetMIPIOutputSpeed(uint16_t &speed) override;
    aditof::Status adsd3500SetRawBypassMode(bool enable);
    aditof::Status adsd3500SetEnableDeskewAtStreamOn(uint16_t value) override;
    aditof::Status adsd3500GetImagerErrorCode(uint16_t &errcode) override;
    aditof::Status adsd3500SetVCSELDelay(uint16_t delay) override;
    aditof::Status adsd3500GetVCSELDelay(uint16_t &delay) override;
    aditof::Status adsd3500SetJBLFMaxEdgeThreshold(uint16_t threshold) override;
    aditof::Status adsd3500SetJBLFABThreshold(uint16_t threshold) override;
    aditof::Status adsd3500SetJBLFGaussianSigma(uint16_t value) override;
    aditof::Status adsd3500GetJBLFGaussianSigma(uint16_t &value) override;
    aditof::Status adsd3500SetJBLFExponentialTerm(uint16_t value) override;
    aditof::Status adsd3500GetJBLFExponentialTerm(uint16_t &value) override;
    aditof::Status adsd3500GetFrameRate(uint16_t &fps) override;
    aditof::Status adsd3500SetFrameRate(uint16_t fps) override;
    aditof::Status adsd3500SetEnableEdgeConfidence(uint16_t value) override;
    aditof::Status
    adsd3500GetTemperatureCompensationStatus(uint16_t &value) override;
    aditof::Status adsd3500SetEnablePhaseInvalidation(uint16_t value) override;
    aditof::Status
    adsd3500SetEnableTemperatureCompensation(uint16_t value) override;
    aditof::Status adsd3500SetEnableMetadatainAB(uint16_t value) override;
    aditof::Status adsd3500GetEnableMetadatainAB(uint16_t &value) override;
    aditof::Status adsd3500SetGenericTemplate(uint16_t reg,
                                              uint16_t value) override;
    aditof::Status adsd3500GetGenericTemplate(uint16_t reg,
                                              uint16_t &value) override;
    aditof::Status adsd3500GetStatus(int &chipStatus,
                                     int &imagerStatus) override;
    aditof::Status adsd3500DisableCCBM(bool disable) override;
    aditof::Status adsd3500IsCCBMsupported(bool &supported) override;
    aditof::Status adsd3500setEnableDynamicModeSwitching(bool en) override;
    aditof::Status adsds3500setDynamicModeSwitchingSequence(
        const std::vector<std::pair<uint8_t, uint8_t>> &sequence) override;
    aditof::Status readSerialNumber(std::string &serialNumber,
                                    bool useCacheValue = false) override;
    aditof::Status getImagerType(aditof::ImagerType &imagerType) const override;
    aditof::Status adsd3500ResetIniParamsForMode(const uint16_t mode) override;
    aditof::Status
    saveDepthParamsToJsonFile(const std::string &savePathFile) override;

    aditof::Status
    loadDepthParamsFromJsonFile(const std::string &pathFile,
                                const int16_t mode_in_use = -1) override;

    void dropFirstFrame(bool dropFrame) override;

    aditof::Status
    getFrameProcessParams(std::map<std::string, std::string> &params) override;
    aditof::Status
    setFrameProcessParams(std::map<std::string, std::string> &params,
                          int32_t mode);

    aditof::Status
    getDepthParamtersMap(uint16_t mode,
                         std::map<std::string, std::string> &params) override;

  private:
    // DEPRECATED: Methods now delegated to CalibrationManager
    /**
     * @brief Read the CCB from adsd3500 memory and store in output variable ccb
     * @param[out] ccb - where to store the CCB content
     * @return Status
     * @see Status
     * @deprecated Use m_calibrationMgr->readCCB() instead
     */
    aditof::Status readAdsd3500CCB(std::string &ccb);

    /**
     * Configure the sensor with various settings that affect the frame type.
     */
    void configureSensorModeDetails();

    /**
     * Reads the depth process parameters from camera
     */
    aditof::Status retrieveDepthProcessParams();

    aditof::Status resetDepthProcessParams();

    /**
     * @brief Enable or disable 90-degree clockwise rotation for VGA frames.
     * Takes effect immediately and also overrides the JSON config for subsequent setMode() calls.
     * @param[in] enable - true to enable rotation, false to disable
     * @return Status
     */
    aditof::Status setRotationEnabled(bool enable) override;

    /**
     * Configure ADSD3500 with ini parameters
     * @param[in] iniKeyValPairs - ini parameteres to use
    */
    aditof::Status
    setDepthIniParams(const std::map<std::string, std::string> &iniKeyValPairs,
                      bool updateDepthMap = true);

    /**
     * @brief Delete allocated tables for X, Y, Z
     * @deprecated Use m_calibrationMgr->cleanupXYZtables() instead
     */
    void cleanupXYZtables();

    /**
     * @brief Check if string can convert to double
     * @deprecated Use m_config->isConvertibleToDouble() instead
     */
    bool isConvertibleToDouble(const std::string &str);

    aditof::Status startRecording(std::string &filePath) override;
    aditof::Status stopRecording() override;

    aditof::Status setPlaybackFile(std::string &filePath) override;
    void UpdateDepthParamsMap(bool update, const char *index,
                              std::string value);

    /**
     * @brief Auto-discovers JSON configuration file from binary path or environment variable
     * @return Path to discovered JSON file, empty string if not found
     * @deprecated Use m_config->autoDiscoverConfigFile() instead
     */
    std::string autoDiscoverConfigFile();

  private:
    using noArgCallable = std::function<aditof::Status()>;

    // Configuration and calibration managers (Phase 1 refactoring)
    std::unique_ptr<aditof::CameraConfiguration> m_config;
    std::unique_ptr<aditof::CalibrationManager> m_calibrationMgr;
    // ADSD3500 hardware controller (Phase 2A refactoring)
    std::unique_ptr<aditof::Adsd3500Controller> m_adsd3500Ctrl;
    // Recording and playback manager (Phase 2B refactoring)
    std::unique_ptr<aditof::RecordingManager> m_recordingMgr;
    // Sensor configuration helper (Phase 2C refactoring)
    std::unique_ptr<aditof::SensorConfigHelper> m_sensorConfigHelper;
    // Depth parameter mapper (Phase 2D refactoring)
    std::unique_ptr<aditof::DepthParameterMapper> m_depthParamMapper;

    aditof::CameraDetails m_details;
    std::shared_ptr<aditof::DepthSensorInterface> m_depthSensor;
    std::unordered_map<std::string, std::string> m_controls;
    std::map<std::string, noArgCallable> m_noArgCallables;
    aditof::ADSDErrors m_adsdErrors;

    bool m_devStarted;
    bool m_devStreaming;
    bool m_adsd3500Enabled;
    bool m_adsd3500_master;
    bool m_isOffline;
    std::string m_netLinkTest;

    uint8_t m_depthBitsPerPixel;
    uint8_t m_abBitsPerPixel;
    uint8_t m_confBitsPerPixel;
    bool m_depthEnabled;
    bool m_abEnabled;
    bool m_confEnabled;
    bool m_xyzEnabled;
    bool m_xyzSetViaApi;
    bool m_pcmFrame;
    std::vector<aditof::DepthSensorModeDetails> m_availableSensorModeDetails;
    std::vector<uint8_t> m_availableModes;
    aditof::DepthSensorModeDetails m_modeDetailsCache;
    std::vector<std::pair<std::string, int32_t>> m_sensor_settings;
    int16_t m_cameraFps;
    // NOTE: Configuration settings now managed by m_config:
    // - m_fsyncMode, m_mipiOutputSpeed, m_isdeskewEnabled
    // - m_enableTempCompenstation, m_enableMetaDatainAB, m_enableEdgeConfidence
    // - m_dropFirstFrame, m_configDmsSequence, m_iniKeyValPairs
    // - m_depth_params_map, m_depth_params_map_reset
    // NOTE: Calibration data now managed by m_calibrationMgr:
    // - m_rawCCBData, m_xyz_dealias_data, m_xyzTable
    std::pair<std::string, std::string> m_adsd3500FwGitHash;
    int m_adsd3500FwVersionInt;
    int m_modesVersion;
    bool m_fwUpdated;
    aditof::Adsd3500Status m_adsd3500Status;
    bool m_enableDepthCompute;
    std::string m_initConfigFilePath;
    bool m_userJsonLoaded = false;
    aditof::ImagerType m_imagerType;
    bool m_dropFrameOnce; // Per-frame state; m_dropFirstFrame moved to m_config
};

#endif // CAMERA_ITOF_H
