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
#ifndef NETWORK_SENSOR_ENUMERATOR_H
#define NETWORK_SENSOR_ENUMERATOR_H

#include "aditof/sensor_enumerator_interface.h"

#include <string>

class NetworkSensorEnumerator : public aditof::SensorEnumeratorInterface {
  public:
    NetworkSensorEnumerator(const std::string &ip);
    ~NetworkSensorEnumerator();

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

    virtual aditof::Status getRGBSensorStatus(bool &isAvailable,
                                               std::string &devicePath) const override;

  private:
    std::string m_ip;
    std::string m_imageSensorsInfo;

    static int sensorCount;
    std::string m_uBootVersion;
    std::string m_kernelVersion;
    std::string m_sdVersion;
};

#endif // NETWORK_SENSOR_ENUMERATOR_H
