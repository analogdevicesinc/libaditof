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
#include "aditof/depth_sensor_interface.h"
#include "adsd3500_chip_config_manager.h"
#include "adsd3500_interrupt_manager.h"
#include "adsd3500_mode_selector.h"
#include "adsd3500_protocol_manager.h"
#include "adsd3500_recorder.h"
#include "buffer_processor.h"
#include "buffer_processor_interface.h"
#include "ini_config_manager.h"
#include "sensor-tables/ini_file_definitions.h"
#include "sensor_control_registry.h"
#include "v4l2_buffer_manager.h"
#include "v4l_buffer_access_interface.h"
#include "video_device_driver.h"
#include <atomic>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <unordered_map>

struct CcbMode {
    uint8_t UserDefinedMode;
    uint8_t CFG_mode;
    uint16_t heigth;
    uint16_t width;
    uint8_t nFreq;
    uint8_t P0Mode;
    uint8_t TempMode;
    uint8_t INIIndex;
    uint8_t default_mode;
    uint8_t isPCM;
    uint8_t noOfPhases;
    uint8_t noOfCaptures;
    uint16_t spare4;
    uint16_t spare5;
    uint16_t spare6;
    uint16_t spare7;
    uint16_t spare8;
};

struct IniTableEntry {
    uint8_t INIIndex;
    uint8_t rsvd;
    uint16_t abThreshMin;
    uint16_t confThresh;
    uint16_t radialThreshMin;
    uint16_t radialThreshMax;
    uint16_t jblfApplyFlag;
    uint16_t jblfWindowSize;
    uint16_t jblfGaussianSigma;
    uint16_t jblfExponentialTerm;
    uint16_t jblfMaxEdge;
    uint16_t jblfABThreshold;
    uint16_t spare0;
    uint16_t spare1;
    uint16_t spare2;
    uint16_t spare3;
    uint16_t spare4;
    uint16_t spare5;
    uint16_t spare6;
    uint16_t spare7;
    uint16_t spare8;
    uint16_t modeNumber;
};

enum class SensorImagerType {
    IMAGER_UNKNOWN,
    IMAGER_ADSD3100,
    IMAGER_ADSD3030,
    IMAGER_ADTF3080,
    IMAGER_ADTF3066
};

enum class CCBVersion {
    CCB_UNKNOWN,
    CCB_VERSION0,
    CCB_VERSION1,
    CCB_VERSION2,
    CCB_VERSION3
};

class Adsd3500Sensor : public aditof::DepthSensorInterface,
                       public aditof::Adsd3500HardwareInterface,
                       public aditof::RecordableInterface,
                       public aditof::V4lBufferAccessInterface,
                       public std::enable_shared_from_this<Adsd3500Sensor> {
  public:
    Adsd3500Sensor(const std::string &driverPath,
                   const std::string &driverSubPath,
                   const std::string &captureDev);
    ~Adsd3500Sensor();

  public: // implements DepthSensorInterface
    virtual aditof::Status open() override;
    virtual aditof::Status start() override;
    virtual aditof::Status stop() override;
    virtual aditof::Status
    getAvailableModes(std::vector<uint8_t> &modes) override;
    virtual aditof::Status
    getModeDetails(const uint8_t &mode,
                   aditof::DepthSensorModeDetails &details) override;
    virtual aditof::Status
    setMode(const aditof::DepthSensorModeDetails &type) override;
    virtual aditof::Status setMode(const uint8_t &mode) override;
    virtual aditof::Status getFrame(uint16_t *buffer,
                                    uint32_t index = 0) override;
    virtual aditof::Status
    getAvailableControls(std::vector<std::string> &controls) const override;
    virtual aditof::Status setControl(const std::string &control,
                                      const std::string &value) override;
    virtual aditof::Status getControl(const std::string &control,
                                      std::string &value) const override;
    virtual aditof::Status
    getDetails(aditof::SensorDetails &details) const override;
    virtual aditof::Status getHandle(void **handle) override;
    virtual aditof::Status getName(std::string &name) const override;
    virtual aditof::Status
    initTargetDepthCompute(uint8_t *iniFile, uint16_t iniFileLength,
                           uint8_t *calData, uint32_t calDataLength) override;
    virtual aditof::Status
    getDepthComputeParams(std::map<std::string, std::string> &params) override;
    virtual aditof::Status setDepthComputeParams(
        const std::map<std::string, std::string> &params) override;
    virtual aditof::Status
    getIniParamsArrayForMode(int mode, std::string &iniStr) override;

  public: // implements Adsd3500HardwareInterface
    virtual aditof::Status adsd3500_read_cmd(uint16_t cmd, uint16_t *data,
                                             unsigned int usDelay = 0) override;
    virtual aditof::Status
    adsd3500_write_cmd(uint16_t cmd, uint16_t data,
                       unsigned int usDelay = 0) override;
    virtual aditof::Status
    adsd3500_read_payload_cmd(uint32_t cmd, uint8_t *readback_data,
                              uint16_t payload_len) override;
    virtual aditof::Status adsd3500_read_payload(uint8_t *payload,
                                                 uint16_t payload_len) override;
    virtual aditof::Status
    adsd3500_write_payload_cmd(uint32_t cmd, uint8_t *payload,
                               uint16_t payload_len) override;
    virtual aditof::Status
    adsd3500_write_payload(uint8_t *payload, uint16_t payload_len) override;
    virtual aditof::Status adsd3500_reset() override;
    virtual aditof::Status adsd3500_getInterruptandReset() override;
    virtual aditof::Status adsd3500_register_interrupt_callback(
        aditof::SensorInterruptCallback &cb) override;
    virtual aditof::Status adsd3500_unregister_interrupt_callback(
        aditof::SensorInterruptCallback &cb) override;
    virtual aditof::Status adsd3500_get_status(int &chipStatus,
                                               int &imagerStatus) override;

  public: // implements RecordableInterface
    aditof::Status startRecording(std::string &fileName, uint8_t *parameters,
                                  uint32_t paramSize) override;
    aditof::Status stopRecording() override;

  public: // V4lBufferAccessInterface implementation
    virtual aditof::Status waitForBuffer() override;
    virtual aditof::Status
    dequeueInternalBuffer(struct v4l2_buffer &buf) override;
    virtual aditof::Status
    getInternalBuffer(uint8_t **buffer, uint32_t &buf_data_len,
                      const struct v4l2_buffer &buf) override;
    virtual aditof::Status
    enqueueInternalBuffer(struct v4l2_buffer &buf) override;
    virtual aditof::Status
    getDeviceFileDescriptor(int &fileDescriptor) override;

  public: // Interrupt handling
    aditof::Status adsd3500InterruptHandler(int signalValue);

  private: // Private helper methods
    aditof::Status waitForBufferPrivate(struct VideoDev *dev = nullptr);
    aditof::Status dequeueInternalBufferPrivate(struct v4l2_buffer &buf,
                                                struct VideoDev *dev = nullptr);
    aditof::Status getInternalBufferPrivate(uint8_t **buffer,
                                            uint32_t &buf_data_len,
                                            const struct v4l2_buffer &buf,
                                            struct VideoDev *dev = nullptr);
    aditof::Status enqueueInternalBufferPrivate(struct v4l2_buffer &buf,
                                                struct VideoDev *dev = nullptr);
    aditof::Status queryAdsd3500();
    aditof::Adsd3500Status convertIdToAdsd3500Status(int status);
    aditof::Status getIniParamsImpl(void *p_config_params, int params_group,
                                    const void *p_tofi_cal_config);
    aditof::Status setIniParamsImpl(void *p_config_params, int params_group,
                                    const void *p_tofi_cal_config);
    aditof::Status
    getDefaultIniParamsForMode(const std::string &imager,
                               const std::string &mode,
                               std::map<std::string, std::string> &params);
    aditof::Status
    mergeIniParams(std::vector<iniFileStruct> &iniFileStructList);
    aditof::Status convertIniParams(iniFileStruct &iniStruct,
                                    std::string &inistr);

  private: // Internal implementation details
    struct ImplData;
    std::string m_sensorName;
    aditof::SensorDetails m_sensorDetails;
    bool m_interruptAvailable;
    std::string m_driverPath;
    std::string m_driverSubPath;
    std::string m_captureDev;
    aditof::SensorControlRegistry m_controlRegistry;
    std::unique_ptr<ImplData> m_implData;
    uint8_t m_capturesPerFrame;
    bool m_firstRun;
    unsigned int m_sensorFps;
    bool m_adsd3500Queried;
    std::unordered_map<void *, aditof::SensorInterruptCallback>
        m_interruptCallbackMap;
    std::vector<aditof::DepthSensorModeDetails> m_availableModes;
    std::vector<IniTableEntry> m_ccbmINIContent;
    aditof::BufferProcessorInterface *m_bufferProcessor;
    bool m_depthComputeOnTarget;
    int m_chipStatus;
    int m_imagerStatus;
    aditof::Adsd3500Status m_adsd3500Status;
    bool m_chipResetDone;
    std::vector<iniFileStruct> m_iniFileStructList;
    aditof::Adsd3500ModeSelector m_modeSelector;
    std::string m_sensorConfiguration;
    bool m_isOpen;
    bool m_ccbmEnabled;
    std::vector<uint8_t> m_bitsInAB;
    std::vector<uint8_t> m_bitsInConf;
    uint16_t m_chipId;
    bool m_lensScatterEnabled = false;
    bool m_rotationEnabled = false;
    /**
     * @brief Target mode number for runtime bit configuration updates.
     * 
     * Thread-safe atomic variable that specifies which mode's bit depth arrays
     * (m_bitsInAB, m_bitsInConf) should be updated when abBits or confidenceBits
     * controls are set from JSON configuration before setMode() is called.
     * 
     * - Value -1: Use current mode (m_implData->modeDetails.modeNumber)
     * - Value >= 0: Update arrays for the specified mode index
     * 
     * Set via "targetModeNumber" control. Reset to -1 after setMode() completes.
     * 
     * @see setControl("targetModeNumber", value)
     * @see setControl("abBits", value)
     * @see setControl("confidenceBits", value)
     */
    std::atomic<int8_t> m_targetModeNumber{-1};

  private: // Manager instances (SOLID refactoring)
    std::recursive_mutex
        m_adsd3500_mutex; // Protects ADSD3500 command/payload operations (recursive for nested calls)
    std::unique_ptr<Adsd3500ProtocolManager>
        m_protocolManager; // Manages chip communication protocol
    std::unique_ptr<Adsd3500ChipConfigManager>
        m_chipConfigManager; // Manages chip configuration discovery
    std::unique_ptr<V4L2BufferManager>
        m_bufferManager; // Manages V4L2 buffer operations
    std::unique_ptr<IniConfigManager>
        m_iniConfigManager; // Manages INI configuration parameters
    std::unique_ptr<Adsd3500InterruptManager>
        m_interruptManager; // Manages hardware interrupt callbacks
    std::unique_ptr<aditof::VideoDeviceDriver>
        m_videoDriver; // Abstracts V4L2 operations for testability
    std::unique_ptr<aditof::VideoDeviceDriver>
        m_subdeviceDriver; // Abstracts V4L2 subdevice operations

    std::unique_ptr<aditof::Adsd3500Recorder>
        m_recorder; // Recording operations (implements RecordableInterface portion)
};