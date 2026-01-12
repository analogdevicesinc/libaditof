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
#include "rpi_platform.h"
#include "rpi_media_config.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <sys/stat.h>
#include <unistd.h>

namespace aditof {
namespace platform {

// Raspberry Pi RP1 Camera Front End capture device identifier
static const char* CAPTURE_DEVICE_NAME = "rp1-cfe";

RpiPlatform::RpiPlatform() {
#ifdef USE_GLOG
    LOG(INFO) << "Initializing Raspberry Pi platform";
#else
    LOG(INFO) << "Initializing Raspberry Pi platform";
#endif
}

PlatformInfo RpiPlatform::getPlatformInfo() const {
    PlatformInfo info;
    info.name = "Raspberry Pi 5";
    info.architecture = "aarch64";
    info.videoDevicePrefix = "rp1-cfe";  // RP1 Camera Front End
    info.mediaController = "/dev/media";
    info.capabilities = {"HARDWARE_DEPTH"};
    
    return info;
}

std::string RpiPlatform::getBootloaderVersion() const {
    return getVersionOfComponent("u-boot");
}

std::string RpiPlatform::getKernelVersion() const {
    // Read kernel version from /proc/version
    std::ifstream versionFile("/proc/version");
    if (versionFile.is_open()) {
        std::string version;
        std::getline(versionFile, version);
        versionFile.close();
        return version;
    }
    return "unknown";
}

std::string RpiPlatform::getSDCardVersion() const {
    return getVersionOfComponent("sd_img_ver");
}

std::string RpiPlatform::getVersionOfComponent(
    const std::string& component) const {
    
    // Check common version file locations for Raspberry Pi
    std::vector<std::string> versionFiles = {
        "/boot/sw-versions",
        "/boot/firmware/sw-versions"
    };
    
    for (const auto& filePath : versionFiles) {
        std::ifstream fid(filePath);
        if (!fid.is_open()) {
            continue;
        }
        
        std::string line;
        std::string version;
        
        while (std::getline(fid, line)) {
            if (line.compare(0, component.length(), component) == 0) {
                version = line.substr(component.length());
                version.erase(std::remove(version.begin(), version.end(), '\t'),
                             version.end());
                break;
            }
        }
        fid.close();
        
        if (!version.empty()) {
            return version;
        }
    }
    
    return "";
}

Status RpiPlatform::parseMediaPipeline(
    const std::string& mediaDevice,
    std::string& devPath,
    std::string& subdevPath,
    std::string& deviceName) {
    
    // Run media-ctl to get the video processing pipes
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "media-ctl -d %s --print-dot 2>/dev/null", 
             mediaDevice.c_str());
    
    FILE* fp = popen(cmd, "r");
    if (!fp) {
#ifdef USE_GLOG
        LOG(WARNING) << "Error running media-ctl on " << mediaDevice;
#else
        LOG(WARNING) << "Error running media-ctl on " << mediaDevice;
#endif
        return Status::GENERIC_ERROR;
    }

    // Read the media-ctl output stream
    char* buf = static_cast<char*>(malloc(128 * 1024));
    size_t size = 0;
    while (!feof(fp)) {
        size_t sz = fread(&buf[size], 1, 1, fp);
        size += sz;
    }
    pclose(fp);
    buf[size] = '\0';

    // Search media-ctl output for device/subdevice name
    std::string str(buf);
    free(buf);

    if (str.empty()) {
        return Status::GENERIC_ERROR;
    }

    // Look for ADSD3500 entity
    size_t pos = str.find("adsd3500");
    if (pos == std::string::npos) {
        return Status::GENERIC_ERROR;
    }
    
    deviceName = "adsd3500";
    
    // Find the subdevice path after the entity name
    // Format: "adsd3500 10-0038\n/dev/v4l-subdev2"
    size_t subdevStart = str.find("/dev/v4l-subdev", pos);
    if (subdevStart == std::string::npos) {
        return Status::GENERIC_ERROR;
    }
    
    // Find the end of the subdev path (space, quote, or newline)
    size_t subdevEnd = str.find_first_of(" \"\n", subdevStart);
    if (subdevEnd == std::string::npos) {
        subdevEnd = subdevStart + strlen("/dev/v4l-subdevX");
    }
    
    subdevPath = str.substr(subdevStart, subdevEnd - subdevStart);
    
    if (subdevPath.empty()) {
        return Status::GENERIC_ERROR;
    }
    
    // For Raspberry Pi, we need to find the video capture device
    // The adsd3500 sensor connects to rp1-cfe-csi2_ch0 (/dev/video0)
    // For now, use a simple heuristic: /dev/video0 is typically the main capture
    devPath = "/dev/video0";
    
    // TODO: Parse media topology more robustly to find the exact video node
    // connected to the sensor via the CSI pipeline
    
    return Status::OK;
}

Status RpiPlatform::findToFSensors(std::vector<SensorInfo>& sensors) {
    sensors.clear();
    
#ifdef USE_GLOG
    LOG(INFO) << "Looking for ToF sensors on Raspberry Pi";
#else
    LOG(INFO) << "Looking for ToF sensors on Raspberry Pi";
#endif

    // Check /dev/media0 through /dev/media3 for ADSD3500 entity
    const std::string entityName = "adsd3500";
    
    for (int i = 0; i < 4; ++i) {
        std::string mediaDevice = "/dev/media" + std::to_string(i);
        
#ifdef USE_GLOG
        DLOG(INFO) << "Checking " << mediaDevice << " for ToF camera";
#else
        DLOG(INFO) << "Checking " << mediaDevice << " for ToF camera";
#endif

        std::string devPath;
        std::string subdevPath;
        std::string deviceName;

        Status status = parseMediaPipeline(mediaDevice, devPath, subdevPath, deviceName);
        if (status != Status::OK) {
            continue;
        }

        if (devPath.empty() || subdevPath.empty()) {
            continue;
        }

#ifdef USE_GLOG
        LOG(INFO) << "Found ToF camera: " << deviceName 
                  << " at subdev=" << subdevPath 
                  << " driver=" << devPath;
#else
        LOG(INFO) << "Found ToF camera: " << deviceName 
                  << " at subdev=" << subdevPath 
                  << " driver=" << devPath;
#endif

        SensorInfo sInfo;
        if (deviceName == "adsd3500") {
            sInfo.sensorType = SensorType::SENSOR_ADSD3500;
        }

        sInfo.driverPath = devPath;
        sInfo.subDevPath = subdevPath;
        sInfo.captureDev = CAPTURE_DEVICE_NAME;
        sensors.push_back(sInfo);
    }

    if (sensors.empty()) {
#ifdef USE_GLOG
        LOG(WARNING) << "No ToF sensors found on any media device";
#else
        LOG(WARNING) << "No ToF sensors found on any media device";
#endif
    }

    return Status::OK;
}

Status RpiPlatform::configureStreamFormat(
    const std::string& mediaDevice,
    const std::string& videoDevice,
    const std::string& sensorEntity,
    int width,
    int height,
    int bitDepth) {
    
    bool success = rpi::configureMediaPipeline(
        mediaDevice, videoDevice, sensorEntity, width, height, bitDepth);
    
    return success ? Status::OK : Status::GENERIC_ERROR;
}

Status RpiPlatform::findRGBSensors(std::vector<RGBSensorInfo>& sensors) {
    sensors.clear();
    
    // Raspberry Pi RGB camera support can be added here
    return Status::OK;
}

} // namespace platform
} // namespace aditof
