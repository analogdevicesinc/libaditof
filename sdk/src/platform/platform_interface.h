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
#ifndef PLATFORM_INTERFACE_H
#define PLATFORM_INTERFACE_H

#include <aditof/status_definitions.h>
#include <string>
#include <vector>

namespace aditof {
namespace platform {

/**
 * @brief Platform information structure
 */
struct PlatformInfo {
    std::string name;              //!< Platform name (e.g., "NVIDIA Jetson Orin Nano")
    std::string architecture;      //!< Architecture (e.g., "aarch64", "x86_64")
    std::string videoDevicePrefix; //!< Video device prefix (e.g., "vi-output", "mxc_isi")
    std::string mediaController;   //!< Media controller device (e.g., "/dev/media*")
    std::vector<std::string> capabilities; //!< Platform capabilities
};

/**
 * @brief Sensor type enumeration
 */
enum class SensorType {
    SENSOR_ADSD3500 //!< ADSD3500 ToF sensor
};

/**
 * @brief Sensor information structure
 */
struct SensorInfo {
    SensorType sensorType;      //!< Type of sensor
    std::string driverPath;     //!< V4L2 video device path
    std::string subDevPath;     //!< V4L2 subdevice path
    std::string captureDev;     //!< Capture device path
};

/**
 * @brief RGB sensor information structure
 */
struct RGBSensorInfo {
    std::string devicePath;     //!< RGB camera device path
    std::string sensorName;     //!< RGB sensor name
};

/**
 * @brief Abstract platform interface
 * 
 * This interface defines platform-specific operations for device discovery
 * and system information retrieval. Each supported SoC platform (NVIDIA, NXP, etc.)
 * provides its own implementation.
 */
class IPlatform {
public:
    virtual ~IPlatform() = default;
    
    /**
     * @brief Get platform identification information
     * @return Platform information structure
     */
    virtual PlatformInfo getPlatformInfo() const = 0;
    
    /**
     * @brief Get bootloader version
     * @return Bootloader version string
     */
    virtual std::string getBootloaderVersion() const = 0;
    
    /**
     * @brief Get kernel version
     * @return Kernel version string
     */
    virtual std::string getKernelVersion() const = 0;
    
    /**
     * @brief Get SD card image version
     * @return SD card version string
     */
    virtual std::string getSDCardVersion() const = 0;
    
    /**
     * @brief Find available ToF sensors
     * @param sensors Output vector of discovered sensors
     * @return Status of operation
     */
    virtual Status findToFSensors(std::vector<SensorInfo>& sensors) = 0;
    
    /**
     * @brief Find available RGB sensors
     * @param sensors Output vector of discovered RGB sensors
     * @return Status of operation
     */
    virtual Status findRGBSensors(std::vector<RGBSensorInfo>& sensors) = 0;
    
    /**
     * @brief Parse media pipeline topology
     * @param mediaDevice Media controller device path
     * @param devPath Output video device path
     * @param subdevPath Output subdevice path
     * @param deviceName Output device name
     * @return Status of operation
     */
    virtual Status parseMediaPipeline(
        const std::string& mediaDevice,
        std::string& devPath,
        std::string& subdevPath,
        std::string& deviceName) = 0;
};

} // namespace platform
} // namespace aditof

#endif // PLATFORM_INTERFACE_H
