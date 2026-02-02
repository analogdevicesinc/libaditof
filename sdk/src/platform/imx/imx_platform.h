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
#ifndef IMX_PLATFORM_H
#define IMX_PLATFORM_H

#include "platform/platform_interface.h"

namespace aditof {
namespace platform {

/**
 * @brief NXP i.MX8MP platform implementation
 * 
 * This implementation provides NXP i.MX-specific device discovery
 * and system information retrieval. It handles the ISI/CSIS media pipeline
 * and NXP-specific system information.
 */
class ImxPlatform : public IPlatform {
public:
    ImxPlatform();
    ~ImxPlatform() override = default;
    
    // IPlatform implementation
    PlatformInfo getPlatformInfo() const override;
    std::string getBootloaderVersion() const override;
    std::string getKernelVersion() const override;
    std::string getSDCardVersion() const override;
    
    Status findToFSensors(std::vector<SensorInfo>& sensors) override;
    Status findRGBSensors(std::vector<RGBSensorInfo>& sensors) override;
    
    Status parseMediaPipeline(
        const std::string& mediaDevice,
        std::string& devPath,
        std::string& subdevPath,
        std::string& deviceName) override;

private:
    /**
     * @brief Read version component from system
     * @param component Component name
     * @return Version string or empty on failure
     */
    std::string getVersionOfComponent(const std::string& component) const;
};

} // namespace platform
} // namespace aditof

#endif // IMX_PLATFORM_H
