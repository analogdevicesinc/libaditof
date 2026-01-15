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
#ifndef PLATFORM_SENSOR_ENUMERATOR_H
#define PLATFORM_SENSOR_ENUMERATOR_H

#include "aditof/sensor_enumerator_interface.h"
#include "platform_impl.h"
#include <vector>

namespace aditof {

/**
 * @brief Platform-based sensor enumerator
 * 
 * This class implements the SensorEnumeratorInterface by delegating
 * platform-specific operations to the IPlatform implementation.
 * It bridges the SDK's sensor enumeration API with the platform layer.
 */
class PlatformSensorEnumerator : public SensorEnumeratorInterface {
  public:
    PlatformSensorEnumerator();
    ~PlatformSensorEnumerator() override = default;

    // SensorEnumeratorInterface implementation
    Status searchSensors() override;
    Status getDepthSensors(std::vector<std::shared_ptr<DepthSensorInterface>>
                               &depthSensors) override;
    Status getUbootVersion(std::string &uBootVersion) const override;
    Status getKernelVersion(std::string &kernelVersion) const override;
    Status getSdVersion(std::string &sdVersion) const override;

#ifdef HAS_RGB_CAMERA
    Status getRGBSensorStatus(bool &isAvailable,
                              std::string &devicePath) const override;
#endif

  private:
    platform::Platform m_platform;
    std::vector<platform::SensorInfo> m_sensorsInfo;
    std::vector<platform::RGBSensorInfo> m_rgbSensorsInfo;
    std::string m_uBootVersion;
    std::string m_kernelVersion;
    std::string m_sdVersion;
};

} // namespace aditof

#endif // PLATFORM_SENSOR_ENUMERATOR_H
