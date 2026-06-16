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
#ifndef TARGET_SENSOR_ENUMERATOR_H
#define TARGET_SENSOR_ENUMERATOR_H

#include "aditof/sensor_definitions.h"
#include "aditof/sensor_enumerator_interface.h"

class TargetSensorEnumerator : public aditof::SensorEnumeratorInterface {
  public:
    ~TargetSensorEnumerator() = default;

  public: // implements SensorEnumeratorInterface
    virtual aditof::Status searchSensors() override;
    virtual aditof::Status
    getDepthSensors(std::vector<std::shared_ptr<aditof::DepthSensorInterface>>
                        &depthSensors) override;
    virtual aditof::Status
    getUbootVersion(std::string &uBootVersion) const override;
    virtual aditof::Status
    getKernelVersion(std::string &kernelVersion) const override;
    virtual aditof::Status getSdVersion(std::string &sdVersion) const override;

    virtual aditof::Status
    getRGBSensorStatus(bool &isAvailable,
                       std::string &devicePath) const override;

  private:
    struct RGBSensorInfo {
        std::string devicePath; // e.g., "/dev/video0"
        std::string sensorName; // e.g., "AR0234"
        bool isAvailable;
    };

    std::string getVersionOfComponent(const std::string &component) const;
    aditof::Status getRGBSensors(std::vector<RGBSensorInfo> &rgbSensors) const;
    enum class SensorType {
        SENSOR_ADSD3500 //!< ADSD3500 sensor
    };

    struct SensorInfo {
        SensorType sensorType;
        std::string driverPath;
        std::string subDevPath;
        std::string captureDev;
    };

    struct SoftwareVersions {
        std::string cardVersion;
        std::string kernelVersion;
        std::string uBootVersion;
    };

    aditof::Status searchRGBSensors();
    bool isRGBSensorAvailable(const std::string &devicePath) const;

    std::vector<SensorInfo> m_sensorsInfo;
    std::vector<RGBSensorInfo> m_rgbSensorsInfo;
    std::string m_cardImageVersion;
    std::string m_uBootVersion;
    std::string m_kernelVersion;
    std::string m_rfsVersion;
    std::string m_sdkVersion;
};

#endif // TARGET_SENSOR_ENUMERATOR_H
