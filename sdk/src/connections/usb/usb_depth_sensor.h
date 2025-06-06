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
#ifndef USB_DEPTH_SENSOR_H
#define USB_DEPTH_SENSOR_H

#include "aditof/depth_sensor_interface.h"

#include <memory>
namespace aditof {
//TODO this is temporary and should be removed imediately after these details are read from the hardware
static const unsigned int USB_FRAME_WIDTH = 640;
static const unsigned int USB_FRAME_HEIGHT = 480;
} // namespace aditof

class UsbDepthSensor : public aditof::DepthSensorInterface {
  public:
    UsbDepthSensor(const std::string &name, const std::string &driverPath);
    ~UsbDepthSensor();

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
    virtual aditof::Status getFrame(uint16_t *buffer) override;
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
    setHostConnectionType(std::string &connectionType) override;

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
    virtual aditof::Status
    initTargetDepthCompute(uint8_t *iniFile, uint16_t iniFileLength,
                           uint8_t *calData, uint16_t calDataLength) override;
    virtual aditof::Status
    getDepthComputeParams(std::map<std::string, std::string> &params) override;
    virtual aditof::Status setDepthComputeParams(
        const std::map<std::string, std::string> &params) override;
    virtual aditof::Status
    setSensorConfiguration(const std::string &sensorConf) override;
    aditof::Status getIniParamsArrayForMode(int mode,
                                            std::string &iniStr) override;

  private:
    struct ImplData;
    std::string m_sensorName;
    aditof::SensorDetails m_sensorDetails;
    std::string m_driverPath;
    std::unique_ptr<ImplData> m_implData;
    std::vector<aditof::DepthSensorModeDetails> m_depthSensorModes;
    int m_fps;
};

#endif // USB_DEPTH_SENSOR_H
