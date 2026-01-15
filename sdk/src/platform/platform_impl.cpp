/*
 * MIT License
 *
 * Copyright (c) 2025 Analog Devices, Inc.
 */
#include "platform_impl.h"
#include "platform_config.h"

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

Platform::Platform() {
    LOG(INFO) << "Initializing platform: " << PLATFORM_NAME;
}

Platform &Platform::getInstance() {
    static Platform instance;
    return instance;
}

PlatformInfo Platform::getPlatformInfo() const {
    PlatformInfo info;
    info.name = PLATFORM_NAME;
    info.architecture = PLATFORM_ARCH;
    info.videoDevicePrefix = PLATFORM_VIDEO_PREFIX;
    info.mediaController = PLATFORM_MEDIA_CONTROLLER;

    return info;
}

std::string Platform::getBootloaderVersion() const {
    return getVersionOfComponent("u-boot");
}

std::string Platform::getKernelVersion() const {
    std::ifstream versionFile("/proc/version");
    if (versionFile.is_open()) {
        std::string version;
        std::getline(versionFile, version);
        versionFile.close();
        return version;
    }
    return "unknown";
}

std::string Platform::getSDCardVersion() const {
    return getVersionOfComponent("sd_img_ver");
}

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

Status Platform::findRGBSensors(std::vector<RGBSensorInfo> &sensors) {
    sensors.clear();
#ifdef PLATFORM_HAS_RGB
    // RGB sensor discovery implementation
    // Left for future implementation
#endif
    return Status::OK;
}

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

Status Platform::resetSensor(bool waitForInterrupt, bool *resetDone,
                             int timeoutSeconds) {
    LOG(INFO) << "Resetting sensor via GPIO";

#if defined(RPI)
    // Raspberry Pi: Use named GPIO if provided, otherwise resolve via debugfs
    char gpio_name[32] = {0};
    bool useNumeric = true;

    if (strlen(PLATFORM_RESET_GPIO_NAME) > 0) {
        // Use provided GPIO name
        strncpy(gpio_name, PLATFORM_RESET_GPIO_NAME, sizeof(gpio_name) - 1);
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
             "/sys/class/gpio/%s/value", PLATFORM_RESET_GPIO_NAME);

    if (stat(gpio_sysfs_path, &st) == 0) {
        // Use sysfs named GPIO
        char cmd[256];
        snprintf(cmd, sizeof(cmd), "echo 0 > /sys/class/gpio/%s/value",
                 PLATFORM_RESET_GPIO_NAME);
        if (system(cmd) != 0) {
            LOG(ERROR) << "Failed to set GPIO " << PLATFORM_RESET_GPIO_NAME
                       << " to 0";
            return Status::GENERIC_ERROR;
        }
        usleep(PLATFORM_RESET_PULSE_US);

        snprintf(cmd, sizeof(cmd), "echo 1 > /sys/class/gpio/%s/value",
                 PLATFORM_RESET_GPIO_NAME);
        if (system(cmd) != 0) {
            LOG(ERROR) << "Failed to set GPIO " << PLATFORM_RESET_GPIO_NAME
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
        // Note: Requires numeric GPIO offset; PLATFORM_RESET_GPIO_NAME must be defined
        LOG(ERROR)
            << "NVIDIA sysfs GPIO not found and no numeric fallback available";
        return Status::GENERIC_ERROR;
    }

    LOG(INFO) << "Sensor reset complete";

#elif defined(NXP)
    // NXP: Use sysfs GPIO (named or numeric)
    char gpio_name[32];
    bool useNumeric = false;
    if (strlen(PLATFORM_RESET_GPIO_NAME) > 0) {
        snprintf(gpio_name, sizeof(gpio_name), "%s", PLATFORM_RESET_GPIO_NAME);
        // Check if it's a numeric string or named GPIO
        useNumeric = (gpio_name[0] >= '0' && gpio_name[0] <= '9');
    } else {
        LOG(ERROR) << "PLATFORM_RESET_GPIO_NAME not defined for NXP";
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
