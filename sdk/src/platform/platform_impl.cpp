/*
 * MIT License
 *
 * Copyright (c) 2025 Analog Devices, Inc.
 */
#include "platform_impl.h"
#include "platform_config.h"

#ifdef RPI
#include "raspberrypi/rpi_media_config.h"
#endif

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <linux/videodev2.h>
#include <sstream>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

#ifdef PLATFORM_NEEDS_MEDIA_CONFIG
#include "media_config.h"
#endif

// Include GPIO helper for platforms that need character device access
#ifndef RPI
#include "connections/target/gpio.h"
#endif

// Helper macro for stringification
#define XSTR(x) STR(x)
#define STR(x) #x

namespace aditof {
namespace platform {

/**
 * @brief Constructor for Platform.
 *
 * Initializes the platform abstraction layer and logs the platform name.
 */
Platform::Platform() {
    LOG(INFO) << "Initializing platform: " << PLATFORM_NAME;
}

/**
 * @brief Returns the singleton Platform instance.
 *
 * Creates and returns a static instance of the Platform class on first call.
 * Subsequent calls return the same instance.
 *
 * @return Reference to the singleton Platform instance.
 */
Platform &Platform::getInstance() {
    static Platform instance;
    return instance;
}

/**
 * @brief Retrieves platform metadata including name, architecture, and device paths.
 *
 * Returns a structure containing platform identification information and
 * hardware-specific device naming conventions.
 *
 * @return PlatformInfo structure with name, architecture, video prefix, and media controller.
 */
PlatformInfo Platform::getPlatformInfo() const {
    PlatformInfo info;
    info.name = PLATFORM_NAME;
    info.architecture = PLATFORM_ARCH;
    info.videoDevicePrefix = PLATFORM_VIDEO_PREFIX;
    info.mediaController = PLATFORM_MEDIA_CONTROLLER;

    return info;
}

/**
 * @brief Retrieves the U-Boot bootloader version string.
 *
 * Returns the bootloader version from the device. Only available on NXP platforms.
 *
 * @return Bootloader version string; empty string on non-NXP platforms.
 */
std::string Platform::getBootloaderVersion() const {
#ifdef NXP
    return getVersionOfComponent("u-boot");
#else
    return ""; // Only applicable for NXP
#endif
}

/**
 * @brief Retrieves the Linux kernel version string.
 *
 * Reads and returns the kernel version from /proc/version on NXP platforms.
 *
 * @return Kernel version string; "unknown" on read error, empty string on non-NXP platforms.
 */
std::string Platform::getKernelVersion() const {
#ifdef NXP
    std::ifstream versionFile("/proc/version");
    if (versionFile.is_open()) {
        std::string version;
        std::getline(versionFile, version);
        versionFile.close();
        return version;
    }
    return "unknown";
#else
    return ""; // Only applicable for NXP
#endif
}

/**
 * @brief Retrieves the SD card image/firmware version string.
 *
 * Returns the SD card image version information. Only available on NXP platforms.
 *
 * @return SD card version string; empty string on non-NXP platforms.
 */
std::string Platform::getSDCardVersion() const {
#ifdef NXP
    return getVersionOfComponent("sd_img_ver");
#else
    return ""; // Only applicable for NXP
#endif
}

/**
 * @brief Retrieves the V4L2 chip configuration control ID.
 *
 * Returns the platform-specific V4L2 control ID for ISP chip configuration commands.
 *
 * @return V4L2 control ID for chip configuration.
 */
uint32_t Platform::getV4L2ChipConfigControlId() const {
#ifdef NXP
    return 0x9819e1;
#else
    return 0x9819d1;
#endif
}

/**
 * @brief Retrieves the V4L2 mode selection control ID.
 *
 * Returns the platform-specific V4L2 control ID for sensor mode switching commands.
 *
 * @return V4L2 control ID for mode selection.
 */
uint32_t Platform::getV4L2ModeControlId() const {
#ifdef NXP
    return 0x9819e0;
#else
    return 0x9819d0;
#endif
}

/**
 * @brief Retrieves the V4L2 AB averaging control ID.
 *
 * Returns the platform-specific V4L2 control ID for AB (amplitude/brightness)
 * frame averaging configuration.
 *
 * @return V4L2 control ID for AB averaging.
 */
uint32_t Platform::getV4L2AbAvgControlId() const {
#ifdef NXP
    return 0x9819e5;
#else
    return 0x9819d5;
#endif
}

/**
 * @brief Retrieves the V4L2 depth enable control ID.
 *
 * Returns the platform-specific V4L2 control ID for enabling/disabling
 * depth frame computation.
 *
 * @return V4L2 control ID for depth enable.
 */
uint32_t Platform::getV4L2DepthEnControlId() const {
#ifdef NXP
    return 0x9819e6;
#else
    return 0x9819d6;
#endif
}

/**
 * @brief Retrieves the V4L2 phase depth bits control ID.
 *
 * Returns the platform-specific V4L2 control ID for configuring the bit depth
 * of phase/depth data.
 *
 * @return V4L2 control ID for phase depth bits.
 */
uint32_t Platform::getV4L2PhaseDepthBitsControlId() const {
#ifdef NXP
    return 0x9819e2;
#else
    return 0x9819d2;
#endif
}

/**
 * @brief Retrieves the V4L2 AB bits control ID.
 *
 * Returns the platform-specific V4L2 control ID for configuring the bit depth
 * of AB (amplitude/brightness) data.
 *
 * @return V4L2 control ID for AB bits.
 */
uint32_t Platform::getV4L2AbBitsControlId() const {
#ifdef NXP
    return 0x9819e3;
#else
    return 0x9819d3;
#endif
}

/**
 * @brief Retrieves the V4L2 confidence bits control ID.
 *
 * Returns the platform-specific V4L2 control ID for configuring the bit depth
 * of confidence data.
 *
 * @return V4L2 control ID for confidence bits.
 */
uint32_t Platform::getV4L2ConfidenceBitsControlId() const {
#ifdef NXP
    return 0x9819e4;
#else
    return 0x9819d4;
#endif
}

/**
 * @brief Retrieves the V4L2 pixel format for 8-bit raw sensor data.
 *
 * Returns the platform-specific V4L2 pixel format code for raw 8-bit Bayer pattern data.
 *
 * @return V4L2 pixel format code for 8-bit data.
 */
uint32_t Platform::getV4L2PixelFormat8bit() const {
#ifdef NXP
    return V4L2_PIX_FMT_SBGGR8;
#else
    return V4L2_PIX_FMT_SRGGB8;
#endif
}

/**
 * @brief Calculates the required buffer size for video frames.
 *
 * Computes the memory size needed for V4L2 video buffers based on frame dimensions.
 * On NVIDIA platforms, adds extra alignment padding; on other platforms, computes
 * exact frame size.
 *
 * @param[in] widthInBytes Frame width in bytes.
 * @param[in] heightInBytes Frame height in lines.
 *
 * @return Required buffer size in bytes.
 *
 * @note NVIDIA platforms require extra line for alignment.
 */
size_t Platform::calculateBufferSize(int widthInBytes,
                                     int heightInBytes) const {
#ifdef NVIDIA
    // NVIDIA requires extra line for alignment
    return static_cast<size_t>(widthInBytes) * heightInBytes + widthInBytes;
#else
    return static_cast<size_t>(widthInBytes) * heightInBytes;
#endif
}

/**
 * @brief Discovers all ADSD3500 Time-of-Flight sensors connected to the system.
 *
 * Scans /dev directory for media device nodes and parses their media pipeline
 * to identify ADSD3500 depth sensors. Caches device paths (video device,
 * subdevice, and capture device) for each discovered sensor.
 *
 * @param[out] sensors Vector to be populated with discovered sensor information.
 *
 * @return Status::OK if at least one sensor is found; Status::GENERIC_ERROR if
 *         no sensors found or media device enumeration fails.
 *
 * @note Clears the output vector before populating.
 * @note Requires media device support in the kernel.
 */
Status Platform::findToFSensors(std::vector<SensorInfo> &sensors) {
    sensors.clear();

    DIR *dir = opendir("/dev");
    if (!dir) {
        LOG(ERROR) << "Failed to open /dev directory";
        return Status::GENERIC_ERROR;
    }

    struct dirent *entry;
    std::vector<std::string> mediaDevices;

    while ((entry = readdir(dir)) != nullptr) {
        if (strncmp(entry->d_name, "media", 5) == 0) {
            mediaDevices.push_back(std::string("/dev/") + entry->d_name);
        }
    }
    closedir(dir);

    std::sort(mediaDevices.begin(), mediaDevices.end());

    for (const auto &mediaDevice : mediaDevices) {
        std::string devPath, subdevPath, deviceName;
        Status status =
            parseMediaPipeline(mediaDevice, devPath, subdevPath, deviceName);

        if (status == Status::OK &&
            deviceName.find(PLATFORM_CAPTURE_DEVICE) != std::string::npos) {
            SensorInfo info;
            info.sensorType = SensorType::SENSOR_ADSD3500;
            info.driverPath = devPath;
            info.subDevPath = subdevPath;
            info.captureDev = deviceName;
            sensors.push_back(info);

            LOG(INFO) << "Found ADSD3500: video=" << devPath
                      << " subdev=" << subdevPath;
        }
    }

    return sensors.empty() ? Status::GENERIC_ERROR : Status::OK;
}

/**
 * @brief Retrieves the MIPI output speed setting for the platform.
 *
 * Returns the configured MIPI CSI-2 data rate in Gbps. On NVIDIA Jetson,
 * defaults to 2.5 Gbps.
 *
 * @return MIPI speed setting (1 for 2.5 Gbps on NVIDIA; -1 if not configured).
 */
int Platform::getMipiOutputSpeed() const {
#ifdef NVIDIA
    return 1; // 2.5 Gbps for NVIDIA Jetson
#else
    return -1; // Not configured for other platforms
#endif
}

/**
 * @brief Retrieves the deskew enable status for the platform.
 *
 * Returns whether deskew (geometric correction for sensor misalignment) is
 * enabled on this platform. On NVIDIA Jetson, deskew is enabled.
 *
 * @return 1 if deskew is enabled (NVIDIA); -1 if not configured for this platform.
 */
int Platform::getDeskewEnabled() const {
#ifdef NVIDIA
    return 1; // Enable deskew for NVIDIA Jetson
#else
    return -1; // Not configured for other platforms
#endif
}

/**
 * @brief Configures the media pipeline for video device and sensor.
 *
 * Sets up the hardware media controller pipeline to match the specified video
 * device, frame dimensions, and bit depth. Platform-specific implementations
 * handle configuration via media controller entities and properties.
 *
 * @param[in] videoDevice Path to the video device (e.g., "/dev/video0").
 * @param[in] width Frame width in pixels.
 * @param[in] height Frame height in lines.
 * @param[in] bitDepth Sensor raw data bit depth (8 or 12).
 *
 * @return Status::OK on successful configuration; Status::GENERIC_ERROR on failure.
 *
 * @note Raspberry Pi uses RP1 CFE pipeline; other platforms skip configuration.
 */
Status Platform::configureMediaPipeline(const std::string &videoDevice,
                                        int width, int height, int bitDepth) {
#ifdef RPI
    // Raspberry Pi: Configure RP1 CFE media pipeline
    std::string mediaDevice = rpi::detectRpiMediaDevice();
    std::string sensorEntity = rpi::detectSensorEntity(mediaDevice);

    bool configured = rpi::configureMediaPipeline(
        mediaDevice, videoDevice, sensorEntity, width, height, bitDepth);

    return configured ? Status::OK : Status::GENERIC_ERROR;
#else
    // Other platforms don't need media pipeline configuration
    return Status::OK;
#endif
}

/**
 * @brief Parses the media pipeline to extract device paths and names.
 *
 * Uses media-ctl to query the media device entity hierarchy and extracts
 * the video device, subdevice, and capture device entity names for the
 * ADSD3500 sensor.
 *
 * @param[in] mediaDevice Path to the media device (e.g., "/dev/media0").
 * @param[out] devPath Video device path (e.g., "/dev/video0").
 * @param[out] subdevPath Subdevice path (e.g., "/dev/v4l-subdev0").
 * @param[out] deviceName Capture entity name (e.g., "rp1-cfe-csi2_ch0").
 *
 * @return Status::OK if all paths are found; Status::GENERIC_ERROR otherwise.
 *
 * @note Requires media-ctl utility to be installed and accessible.
 */
Status Platform::parseMediaPipeline(const std::string &mediaDevice,
                                    std::string &devPath,
                                    std::string &subdevPath,
                                    std::string &deviceName) {

    char cmd[512];
    snprintf(cmd, sizeof(cmd), "media-ctl -d %s -p 2>/dev/null",
             mediaDevice.c_str());

    FILE *pipe = popen(cmd, "r");
    if (!pipe) {
        return Status::GENERIC_ERROR;
    }

    char buffer[512];
    std::string output;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        output += buffer;
    }
    pclose(pipe);

    // Parse output for video device and subdevice paths
    // Strategy: Find capture device entity, then scan subsequent lines for device nodes
    size_t pos = 0;
    bool foundCapture = false;
    bool foundAdsd3500 = false;

    while ((pos = output.find("- entity", pos)) != std::string::npos) {
        size_t lineEnd = output.find('\n', pos);
        size_t nextEntity = output.find("- entity", pos + 1);
        if (nextEntity == std::string::npos) {
            nextEntity = output.length();
        }

        std::string entityBlock = output.substr(pos, nextEntity - pos);
        std::string firstLine = output.substr(pos, lineEnd - pos);

        // Look for capture device (e.g., "rp1-cfe-csi2_ch0" for RPI)
        if (firstLine.find(PLATFORM_CAPTURE_DEVICE) != std::string::npos) {
            deviceName = PLATFORM_CAPTURE_DEVICE;
            foundCapture = true;

            // Search for /dev/video in the entity block (subsequent lines)
            size_t devPos = entityBlock.find("/dev/video");
            if (devPos != std::string::npos) {
                size_t devEnd = entityBlock.find_first_of(" \t\n)", devPos);
                devPath = entityBlock.substr(devPos, devEnd - devPos);
            }
        }

        // Look for ADSD3500 subdevice
        if (firstLine.find("adsd3500") != std::string::npos) {
            foundAdsd3500 = true;

            // Search for /dev/v4l-subdev in the entity block
            size_t subdevPos = entityBlock.find("/dev/v4l-subdev");
            if (subdevPos != std::string::npos) {
                size_t subdevEnd =
                    entityBlock.find_first_of(" \t\n)", subdevPos);
                subdevPath =
                    entityBlock.substr(subdevPos, subdevEnd - subdevPos);
            }
        }

        // Exit early if we found both
        if (foundCapture && foundAdsd3500 && !devPath.empty() &&
            !subdevPath.empty()) {
            break;
        }

        pos = nextEntity;
    }

    return (!devPath.empty() && !subdevPath.empty()) ? Status::OK
                                                     : Status::GENERIC_ERROR;
}

/**
 * @brief Retrieves a version string for a platform component.
 *
 * Reads the version information from a version file stored in /opt/adi/ directory.
 * Commonly used to retrieve bootloader, SD image, and kernel version information.
 *
 * @param[in] component Component identifier (e.g., "u-boot", "sd_img_ver").
 *
 * @return Version string if file exists and is readable; "unknown" otherwise.
 *
 * @note Version files are stored at /opt/adi/{component}_ver.
 */
std::string
Platform::getVersionOfComponent(const std::string &component) const {
    std::string versionFile = "/opt/adi/" + component + "_ver";
    std::ifstream file(versionFile);

    if (file.is_open()) {
        std::string version;
        std::getline(file, version);
        file.close();
        return version;
    }

    return "unknown";
}

/**
 * @brief Resets the ADSD3500 sensor via GPIO control.
 *
 * Performs a hardware reset of the sensor by toggling the reset GPIO pin.
 * Can optionally wait for interrupt-driven reset completion. Supports both
 * named GPIO paths and numeric GPIO pins depending on platform configuration.
 *
 * @param[in] waitForInterrupt Whether to wait for interrupt-driven reset confirmation.
 * @param[in,out] resetDone Pointer to boolean flag indicating reset completion
 *                          (set by interrupt handler if waitForInterrupt=true).
 * @param[in] timeoutSeconds Timeout in seconds for reset operation.
 *
 * @return Status::OK on successful reset; Status::GENERIC_ERROR on failure.
 *
 * @note Raspberry Pi uses GPIO via sysfs or debugfs; other platforms may differ.
 * @note Reset pin name/number is specified via PLATFORM_RESET_GPIO defines.
 */
Status Platform::resetSensor(bool waitForInterrupt, bool *resetDone,
                             int timeoutSeconds) {
    LOG(INFO) << "Resetting sensor via GPIO";

#if defined(RPI)
    // Raspberry Pi: Use named GPIO if provided, otherwise resolve via debugfs
    char gpio_name[32] = {0};
    bool useNumeric = true;

    if (strlen(PLATFORM_RESET_GPIO) > 0) {
        // Use provided GPIO name
        strncpy(gpio_name, PLATFORM_RESET_GPIO, sizeof(gpio_name) - 1);
        useNumeric = false;
    } else {
        // Use debugfs to find actual GPIO number
        FILE *fp = popen(
            "cat /sys/kernel/debug/gpio 2>/dev/null | grep -i GPIO" XSTR(
                PLATFORM_RESET_GPIO_PIN) " | sed -E 's/.*gpio-([0-9]+).*/\\1/'",
            "r");
        if (!fp) {
            LOG(ERROR) << "Could not access GPIO debug interface";
            return Status::GENERIC_ERROR;
        }

        char gpio_num[16] = {0};
        if (fgets(gpio_num, sizeof(gpio_num), fp)) {
            strncpy(gpio_name, gpio_num, sizeof(gpio_name) - 1);
            // Remove trailing newline
            gpio_name[strcspn(gpio_name, "\n")] = 0;
        }
        pclose(fp);

        if (strlen(gpio_name) == 0 || atoi(gpio_name) <= 0) {
            LOG(WARNING) << "Could not find GPIO pin "
                         << PLATFORM_RESET_GPIO_PIN << ", reset skipped";
            return Status::GENERIC_ERROR;
        }
        useNumeric = true;
    }

    // Toggle GPIO via sysfs
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "echo 0 > /sys/class/gpio/%s%s/value",
             useNumeric ? "gpio" : "", gpio_name);
    if (system(cmd) != 0) {
        LOG(ERROR) << "Failed to set GPIO" << gpio_name << " to 0";
        return Status::GENERIC_ERROR;
    }
    usleep(PLATFORM_RESET_PULSE_US);

    snprintf(cmd, sizeof(cmd), "echo 1 > /sys/class/gpio/%s%s/value",
             useNumeric ? "gpio" : "", gpio_name);
    if (system(cmd) != 0) {
        LOG(ERROR) << "Failed to set GPIO" << gpio_name << " to 1";
        return Status::GENERIC_ERROR;
    }

    if (waitForInterrupt && resetDone) {
        LOG(INFO) << "Waiting for sensor to reset";
        int secondsWaited = 0;
        while (!(*resetDone) && secondsWaited < timeoutSeconds) {
            LOG(INFO) << ".";
            std::this_thread::sleep_for(std::chrono::seconds(1));
            secondsWaited++;
        }
        LOG(INFO) << "Waited: " << secondsWaited << " seconds";
    } else {
        usleep(PLATFORM_RESET_DELAY_US);
    }

    LOG(INFO) << "Sensor reset complete via GPIO" << gpio_name;

#elif defined(NVIDIA)
    // NVIDIA: Try sysfs named GPIO first, fall back to gpiochip
    struct stat st;
    char gpio_sysfs_path[256];
    snprintf(gpio_sysfs_path, sizeof(gpio_sysfs_path),
             "/sys/class/gpio/%s/value", PLATFORM_RESET_GPIO);

    if (stat(gpio_sysfs_path, &st) == 0) {
        // Use sysfs named GPIO
        char cmd[256];
        snprintf(cmd, sizeof(cmd), "echo 0 > /sys/class/gpio/%s/value",
                 PLATFORM_RESET_GPIO);
        if (system(cmd) != 0) {
            LOG(ERROR) << "Failed to set GPIO " << PLATFORM_RESET_GPIO
                       << " to 0";
            return Status::GENERIC_ERROR;
        }
        usleep(PLATFORM_RESET_PULSE_US);

        snprintf(cmd, sizeof(cmd), "echo 1 > /sys/class/gpio/%s/value",
                 PLATFORM_RESET_GPIO);
        if (system(cmd) != 0) {
            LOG(ERROR) << "Failed to set GPIO " << PLATFORM_RESET_GPIO
                       << " to 1";
            return Status::GENERIC_ERROR;
        }

        if (waitForInterrupt && resetDone) {
            LOG(INFO) << "Waiting for sensor to reset";
            int secondsWaited = 0;
            while (!(*resetDone) && secondsWaited < timeoutSeconds) {
                LOG(INFO) << ".";
                std::this_thread::sleep_for(std::chrono::seconds(1));
                secondsWaited++;
            }
            LOG(INFO) << "Waited: " << secondsWaited << " seconds";
        } else {
            usleep(PLATFORM_RESET_DELAY_US);
        }
    } else {
        // Fall back to gpiochip character device
        // Note: Requires numeric GPIO offset; PLATFORM_RESET_GPIO must be defined
        LOG(ERROR)
            << "NVIDIA sysfs GPIO not found and no numeric fallback available";
        return Status::GENERIC_ERROR;
    }

    LOG(INFO) << "Sensor reset complete";

#elif defined(NXP)
    // NXP: Use sysfs GPIO (named or numeric)
    char gpio_name[32];
    bool useNumeric = false;
    if (strlen(PLATFORM_RESET_GPIO) > 0) {
        snprintf(gpio_name, sizeof(gpio_name), "%s", PLATFORM_RESET_GPIO);
        // Check if it's a numeric string or named GPIO
        useNumeric = (gpio_name[0] >= '0' && gpio_name[0] <= '9');
    } else {
        LOG(ERROR) << "PLATFORM_RESET_GPIO not defined for NXP";
        return Status::GENERIC_ERROR;
    }

    char cmd[256];
    snprintf(cmd, sizeof(cmd), "echo 0 > /sys/class/gpio/%s%s/value",
             useNumeric ? "gpio" : "", gpio_name);
    if (system(cmd) != 0) {
        LOG(ERROR) << "Failed to set GPIO" << gpio_name << " to 0";
        return Status::GENERIC_ERROR;
    }
    usleep(PLATFORM_RESET_PULSE_US);

    snprintf(cmd, sizeof(cmd), "echo 1 > /sys/class/gpio/%s%s/value",
             useNumeric ? "gpio" : "", gpio_name);
    if (system(cmd) != 0) {
        LOG(ERROR) << "Failed to set GPIO" << gpio_name << " to 1";
        return Status::GENERIC_ERROR;
    }

    if (waitForInterrupt && resetDone) {
        LOG(INFO) << "Waiting for sensor to reset";
        int secondsWaited = 0;
        while (!(*resetDone) && secondsWaited < timeoutSeconds) {
            LOG(INFO) << ".";
            std::this_thread::sleep_for(std::chrono::seconds(1));
            secondsWaited++;
        }
        LOG(INFO) << "Waited: " << secondsWaited << " seconds";
    } else {
        usleep(PLATFORM_RESET_DELAY_US);
    }

    LOG(INFO) << "Sensor reset complete via GPIO" << gpio_name;

#else
    LOG(WARNING) << "Platform reset not implemented";
    return Status::UNAVAILABLE;
#endif

    return Status::OK;
}

} // namespace platform
} // namespace aditof
