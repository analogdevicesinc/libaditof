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
#include "imx_platform.h"

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

// NXP i.MX8MP capture device identifier
static const char* CAPTURE_DEVICE_NAME = "mxc-isi-cap";

ImxPlatform::ImxPlatform() {
#ifdef USE_GLOG
    LOG(INFO) << "Initializing NXP i.MX platform";
#else
    LOG(INFO) << "Initializing NXP i.MX platform";
#endif
}

PlatformInfo ImxPlatform::getPlatformInfo() const {
    PlatformInfo info;
    info.name = "NXP i.MX8MP";
    info.architecture = "aarch64";
    info.videoDevicePrefix = "mxc_isi";
    info.mediaController = "/dev/media";
    info.capabilities = {"HARDWARE_DEPTH"};
    
    return info;
}

std::string ImxPlatform::getBootloaderVersion() const {
    return getVersionOfComponent("u-boot");
}

std::string ImxPlatform::getKernelVersion() const {
    return getVersionOfComponent("kernel");
}

std::string ImxPlatform::getSDCardVersion() const {
    return getVersionOfComponent("sd_img_ver");
}

std::string ImxPlatform::getVersionOfComponent(
    const std::string& component) const {
    
    // NXP i.MX may use different version file locations
    // Check common locations
    std::vector<std::string> versionFiles = {
        "/boot/sw-versions",
        "/sys/firmware/devicetree/base/model"
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
    
#ifdef USE_GLOG
    DLOG(WARNING) << "Could not find version for " << component;
#else
    DLOG(WARNING) << "Could not find version for " << component;
#endif
    
    return "";
}

Status ImxPlatform::parseMediaPipeline(
    const std::string& mediaDevice,
    std::string& devPath,
    std::string& subdevPath,
    std::string& deviceName) {
    
    // Run media-ctl to get the video processing pipes
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "media-ctl -d %s --print-dot", mediaDevice.c_str());
    
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

    // Look for NXP ISI capture entity
    size_t pos = str.find("mxc_isi.0.capture");
    if (pos != std::string::npos) {
        devPath = str.substr(pos + strlen("mxc_isi.0.capture") + 2,
                            strlen("/dev/mediaX"));
    } else {
        return Status::GENERIC_ERROR;
    }

    // Look for ADSD3500 subdevice
    if (str.find("adsd3500") != std::string::npos) {
        deviceName = "adsd3500";
        pos = str.find("adsd3500");
        subdevPath = str.substr(pos + strlen("adsd3500") + 9,
                               strlen("/dev/v4l-subdevX"));
    } else {
        return Status::GENERIC_ERROR;
    }
    
    return Status::OK;
}

Status ImxPlatform::findToFSensors(std::vector<SensorInfo>& sensors) {
    sensors.clear();
    
#ifdef USE_GLOG
    LOG(INFO) << "Looking for ToF sensors on NXP i.MX";
#else
    LOG(INFO) << "Looking for ToF sensors on NXP i.MX";
#endif

    // Find all video device paths
    std::vector<std::string> mediaPaths;
    const std::string mediaDir("/dev/");
    const std::string mediaBaseName("media");

    DIR* dirp = opendir(mediaDir.c_str());
    if (!dirp) {
#ifdef USE_GLOG
        LOG(ERROR) << "Failed to open " << mediaDir;
#else
        LOG(ERROR) << "Failed to open " << mediaDir;
#endif
        return Status::GENERIC_ERROR;
    }

    struct dirent* dp;
    while ((dp = readdir(dirp))) {
        if (strncmp(dp->d_name, mediaBaseName.c_str(), mediaBaseName.length()) == 0) {
            std::string fullPath = mediaDir + std::string(dp->d_name);
            mediaPaths.push_back(fullPath);
        }
    }
    closedir(dirp);

    // Identify any eligible time of flight cameras
    for (const auto& media : mediaPaths) {
#ifdef USE_GLOG
        DLOG(INFO) << "Checking " << media << " for eligible ToF camera";
#else
        DLOG(INFO) << "Checking " << media << " for eligible ToF camera";
#endif

        std::string devPath;
        std::string subdevPath;
        std::string deviceName;

        Status status = parseMediaPipeline(media, devPath, subdevPath, deviceName);
        if (status != Status::OK) {
#ifdef USE_GLOG
            DLOG(WARNING) << "Failed to parse media pipeline at " << media;
#else
            DLOG(WARNING) << "Failed to parse media pipeline at " << media;
#endif
            continue;
        }

        if (devPath.empty() || subdevPath.empty()) {
            continue;
        }

#ifdef USE_GLOG
        LOG(INFO) << "Found ToF camera: " << deviceName << " at " << devPath;
#else
        LOG(INFO) << "Found ToF camera: " << deviceName << " at " << devPath;
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

    return Status::OK;
}

Status ImxPlatform::findRGBSensors(std::vector<RGBSensorInfo>& sensors) {
    sensors.clear();
    
    // NXP i.MX doesn't have RGB camera support in current implementation
    return Status::OK;
}

} // namespace platform
} // namespace aditof
