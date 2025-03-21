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
#ifndef CAMERA_ITOF_H
#define CAMERA_ITOF_H

#include "tofi/tofi_compute.h"
#include "tofi/tofi_config.h"
#include "tofi/tofi_util.h"
#include <aditof/adsd_errs.h>
#include <aditof/camera.h>
#include <aditof/depth_sensor_interface.h>
#include <map>
#include <unordered_map>

#define NR_READADSD3500CCB 3

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
    aditof::Status requestFrame(aditof::Frame *frame) override;
    void normalizeABBuffer(uint16_t *abBuffer, uint16_t abWidth,
                           uint16_t abHeight, bool advanceScaling,
                           bool useLogScaling) override;
    aditof::Status normalizeABFrame(aditof::Frame *frame, bool advanceScaling,
                                    bool useLogScaling) override;
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

    aditof::Status
    setSensorConfiguration(const std::string &sensorConf) override;

    void dropFirstFrame(bool dropFrame) override;

    aditof::Status
    getFrameProcessParams(std::map<std::string, std::string> &params) override;
    aditof::Status
    setFrameProcessParams(std::map<std::string, std::string> &params) override;

  private:
    /**
     * @brief Opens the CCB file passed in as part of Json file using initialize(), and loads the calibration blocks into member variable
     * @return aditof::Status
     * @see aditof::Status
     * @see <a href='../config/config_default.json'>config_default.json</a>
     */
    aditof::Status loadConfigData(void);

    /**
     * @brief Frees the calibration member variables created while loading the CCB contents
     * @return None
     */
    void freeConfigData(void);

    // Methods available only when Adsd3500 is detected as part of the entire setup

    /**
     * @brief Read the CCB from adsd3500 memory and store in output variable ccb
     * @param[out] ccb - where to store the CCB content
     * @return Status
     * @see Status
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

    /**
     * Configure ADSD3500 with ini parameters
     * @param[in] iniKeyValPairs - ini parameteres to use
    */
    aditof::Status setAdsd3500IniParams(
        const std::map<std::string, std::string> &iniKeyValPairs);

    /**
     * @brief Delete allocated tables for X, Y, Z
     */
    void cleanupXYZtables();

    /**
     * @brief Check if string can convert to double
     */
    bool isConvertibleToDouble(const std::string &str);

  private:
    using noArgCallable = std::function<aditof::Status()>;

    aditof::CameraDetails m_details;
    std::shared_ptr<aditof::DepthSensorInterface> m_depthSensor;
    std::unordered_map<std::string, std::string> m_controls;
    std::map<std::string, noArgCallable> m_noArgCallables;
    aditof::ADSDErrors m_adsdErrors;

    bool m_devStarted;
    bool m_devStreaming;
    bool m_tempSensorInitialized;
    bool m_adsd3500Enabled;
    bool m_adsd3500_master;
    bool m_isOffline;
    std::string m_netLinkTest;

    FileData m_calData = {NULL, 0};

    FileData m_depthINIData;
    std::map<std::string, FileData> m_depthINIDataMap;
    TofiXYZDealiasData m_xyz_dealias_data[MAX_N_MODES + 1];
    bool m_loadedConfigData;

    std::string m_sensorFirmwareFile;
    std::string m_ccb_calibrationFile;
    std::string m_ini_depth;
    std::map<std::string, std::string> m_ini_depth_map;
    std::map<int, std::map<std::string, std::string>> m_depth_params_map;
    bool m_abEnabled;
    uint8_t m_depthBitsPerPixel;
    uint8_t m_abBitsPerPixel;
    uint8_t m_confBitsPerPixel;
    bool m_xyzEnabled;
    bool m_xyzSetViaApi;
    bool m_pcmFrame;
    std::vector<aditof::DepthSensorModeDetails> m_availableSensorModeDetails;
    std::vector<uint8_t> m_availableModes;
    aditof::DepthSensorModeDetails m_modeDetailsCache;
    std::vector<std::pair<std::string, int32_t>> m_sensor_settings;
    int16_t m_cameraFps;
    int16_t m_fsyncMode;
    int16_t m_mipiOutputSpeed;
    int16_t m_enableTempCompenstation;
    int16_t m_enableMetaDatainAB;
    int16_t m_enableEdgeConfidence;
    std::map<std::string, std::string> m_iniKeyValPairs;
    //pair between firmware version and git hash
    std::pair<std::string, std::string> m_adsd3500FwGitHash;
    int m_adsd3500FwVersionInt;
    int m_modesVersion;
    bool m_fwUpdated;
    aditof::Adsd3500Status m_adsd3500Status;
    XYZTable m_xyzTable;
    bool m_enableDepthCompute;
    std::string m_initConfigFilePath;
    aditof::ImagerType m_imagerType;
    bool m_dropFirstFrame;
    bool m_dropFrameOnce;
    std::vector<std::pair<uint8_t, uint8_t>> m_configDmsSequence;
};

#endif // CAMERA_ITOF_H
