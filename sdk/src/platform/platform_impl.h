/*
 * MIT License
 *
 * Copyright (c) 2025 Analog Devices, Inc.
 */
#ifndef PLATFORM_IMPL_H
#define PLATFORM_IMPL_H

#include <aditof/status_definitions.h>
#include <string>
#include <vector>

namespace aditof {
namespace platform {

/**
 * @brief Platform information structure
 */
struct PlatformInfo {
    std::string name;
    std::string architecture;
    std::string videoDevicePrefix;
    std::string mediaController;
};

/**
 * @brief Sensor type enumeration
 */
enum class SensorType { SENSOR_ADSD3500 };

/**
 * @brief Sensor information structure
 */
struct SensorInfo {
    SensorType sensorType;
    std::string driverPath;
    std::string subDevPath;
    std::string captureDev;
};

/**
 * @brief Unified platform implementation (configured via CMake)
 */
class Platform {
  public:
    Platform();
    ~Platform() = default;

    PlatformInfo getPlatformInfo() const;
    std::string getBootloaderVersion() const;
    std::string getKernelVersion() const;
    std::string getSDCardVersion() const;

    Status findToFSensors(std::vector<SensorInfo> &sensors);

    /**
     * @brief Reset the ToF sensor via GPIO
     * @param[in] waitForInterrupt If true, wait for sensor interrupt callback
     * @param[in,out] resetDone Pointer to flag set by interrupt handler when reset completes
     * @param[in] timeoutSeconds Maximum seconds to wait for interrupt (default: 10)
     * @return Status::OK on success, error status otherwise
     * @note Platform-specific implementation (RPI/NVIDIA/NXP)
     */
    Status resetSensor(bool waitForInterrupt = false, bool *resetDone = nullptr,
                       int timeoutSeconds = 10);

    /**
     * @brief Get singleton instance of Platform
     * @return Reference to the global Platform instance
     */
    static Platform &getInstance();

  private:
    std::string getVersionOfComponent(const std::string &component) const;

    Status parseMediaPipeline(const std::string &mediaDevice,
                              std::string &devPath, std::string &subdevPath,
                              std::string &deviceName);
};

} // namespace platform
} // namespace aditof

#endif // PLATFORM_IMPL_H
