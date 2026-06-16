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
#include "connections/target/target_sensor_enumerator.h"
#include "target_definitions.h"

#include <dirent.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>

using namespace aditof;

namespace local {

aditof::Status findDevicePathsAtVideo(const std::string &video,
                                      std::string &dev_path,
                                      std::string &subdev_path,
                                      std::string &device_name) {
    using namespace aditof;
    using namespace std;

    char *buf;
    int size = 0;

    /* Run media-ctl to get the video processing pipes */
    char cmd[64];
    sprintf(cmd, "media-ctl -d %s --print-dot", video.c_str());
    FILE *fp = popen(cmd, "r");
    if (!fp) {
        LOG(WARNING) << "Error running media-ctl";
        return Status::GENERIC_ERROR;
    }

    /* Read the media-ctl output stream */
    buf = (char *)malloc(128 * 1024);
    while (!feof(fp)) {
        auto sz = fread(&buf[size], 1, 1, fp);
        size += sz;
    }
    pclose(fp);
    buf[size] = '\0';

    /* Search command media-ctl for device/subdevice name */
    string str(buf);
    free(buf);

    size_t pos = str.find("vi-output, adsd3500");
    if (pos != string::npos) {
        dev_path = str.substr(pos + strlen("vi-output, adsd3500") + 9,
                              strlen("/dev/mediaX"));
    } else {
        return Status::GENERIC_ERROR;
    }

    if (str.find("adsd3500") != string::npos) {
        device_name = "adsd3500";
        pos = str.find("adsd3500");
        subdev_path = str.substr(pos + strlen("adsd3500") + 9,
                                 strlen("/dev/v4l-subdevX"));
    } else {
        return Status::GENERIC_ERROR;
    }
    return Status::OK;
}

}; // namespace local

Status TargetSensorEnumerator::searchSensors() {
    Status status = Status::OK;

    LOG(INFO) << "Looking for sensors on the target";

    // Find all video device paths
    std::vector<std::string> videoPaths;
    const std::string videoDirPath("/dev/");
    const std::string videoBaseName("media");
    std::string deviceName;

    DIR *dirp = opendir(videoDirPath.c_str());
    struct dirent *dp;
    while ((dp = readdir(dirp))) {
        if (!strncmp(dp->d_name, videoBaseName.c_str(),
                     videoBaseName.length())) {
            std::string fullvideoPath = videoDirPath + std::string(dp->d_name);
            videoPaths.emplace_back(fullvideoPath);
        }
    }
    closedir(dirp);

    // Identify any eligible time of flight cameras
    for (const auto &video : videoPaths) {
        DLOG(INFO) << "Looking at: " << video << " for an eligible TOF camera";

        std::string devPath;
        std::string subdevPath;

        status = local::findDevicePathsAtVideo(video, devPath, subdevPath,
                                               deviceName);
        if (status != Status::OK) {
            LOG(WARNING) << "failed to find device paths at video: " << video;
            return status;
        }

        if (devPath.empty() || subdevPath.empty()) {
            continue;
        }

        DLOG(INFO) << "Considering: " << video << " an eligible TOF camera";

        SensorInfo sInfo;

        if (deviceName == "adsd3500") {
            sInfo.sensorType = SensorType::SENSOR_ADSD3500;
        }

        sInfo.driverPath = devPath;
        sInfo.subDevPath = subdevPath;
        sInfo.captureDev = CAPTURE_DEVICE_NAME;
        m_sensorsInfo.emplace_back(sInfo);
    }

#ifdef HAS_RGB_CAMERA
    // Also search for RGB sensors
    status = searchRGBSensors();
    if (status != Status::OK) {
        LOG(WARNING) << "RGB sensor enumeration failed, continuing without RGB";
        // Don't return error - RGB is optional
        status = Status::OK;
    }
#endif

    return status;
}

#ifdef HAS_RGB_CAMERA
bool TargetSensorEnumerator::isRGBSensorAvailable(
    const std::string &devicePath) const {
    // Check if device exists and can be accessed
    struct stat buffer;
    if (stat(devicePath.c_str(), &buffer) != 0) {
        return false;
    }

    // Try to open the device to verify it's accessible
    int fd = open(devicePath.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        return false;
    }

    // Use V4L2 to query the device capabilities
    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
        close(fd);
        return false;
    }

    // Check if it's a video capture device
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        close(fd);
        return false;
    }

    // Verify it's NOT the ToF sensor (adsd3500)
    // The RGB sensor should be "arducam-jvar" based on media-ctl output
    std::string cardName = reinterpret_cast<const char *>(cap.card);
    LOG(INFO) << "Device " << devicePath << " card name: " << cardName;

    // Reject if it's the ADSD3500 ToF sensor
    if (cardName.find("adsd3500") != std::string::npos) {
        close(fd);
        LOG(INFO) << "Skipping " << devicePath << " - it's the ToF sensor";
        return false;
    }

    close(fd);

    // If it's a video capture device and not adsd3500, assume it's RGB
    return true;
}

Status TargetSensorEnumerator::searchRGBSensors() {
    LOG(INFO) << "Looking for RGB sensors on the target";

    m_rgbSensorsInfo.clear();

    // ========================================================================
    // RGB Sensor Auto-Discovery Strategy:
    // Iteratively scan /dev/video0-9 for RGB sensors
    // Filter out ADSD3500 ToF sensors by checking V4L2 card name
    // Use first non-ToF video capture device found
    // ========================================================================

    LOG(INFO) << "Enumerating all /dev/video* devices to find RGB sensor...";

    // Try video devices 0-9 (typical range on Jetson)
    for (int videoNum = 0; videoNum < 10; ++videoNum) {
        std::string devicePath = "/dev/video" + std::to_string(videoNum);

        // Check if device exists
        struct stat st;
        if (stat(devicePath.c_str(), &st) != 0) {
            continue; // Device doesn't exist
        }

        LOG(INFO) << "Checking " << devicePath << "...";

        if (isRGBSensorAvailable(devicePath)) {
            // Found an RGB sensor - get detailed info
            int fd = open(devicePath.c_str(), O_RDWR | O_NONBLOCK);
            std::string sensorName = "AR0234";

            if (fd >= 0) {
                struct v4l2_capability cap;
                if (ioctl(fd, VIDIOC_QUERYCAP, &cap) >= 0) {
                    sensorName = reinterpret_cast<const char *>(cap.card);
                    if (sensorName.find("arducam") != std::string::npos) {
                        sensorName = "AR0234 (arducam-jvar)";
                    }
                }
                close(fd);
            }

            RGBSensorInfo rgbInfo;
            rgbInfo.devicePath = devicePath;
            rgbInfo.sensorName = sensorName;
            rgbInfo.isAvailable = true;

            m_rgbSensorsInfo.emplace_back(rgbInfo);
            LOG(INFO) << "Found RGB sensor: " << rgbInfo.sensorName << " at "
                      << rgbInfo.devicePath;

            // Found first RGB sensor - stop searching
            return Status::OK;
        }
    }

    LOG(INFO) << "No RGB sensor detected after scanning all video devices";

    return Status::OK;
}

Status TargetSensorEnumerator::getRGBSensors(
    std::vector<RGBSensorInfo> &rgbSensors) const {
    rgbSensors = m_rgbSensorsInfo;
    return Status::OK;
}
#endif

Status
TargetSensorEnumerator::getRGBSensorStatus(bool &isAvailable,
                                           std::string &devicePath) const {
#ifdef HAS_RGB_CAMERA
    if (!m_rgbSensorsInfo.empty() && m_rgbSensorsInfo[0].isAvailable) {
        isAvailable = true;
        devicePath = m_rgbSensorsInfo[0].devicePath;
    } else {
        isAvailable = false;
        devicePath = "";
    }
#else
    isAvailable = false;
    devicePath = "";
#endif
    return Status::OK;
}