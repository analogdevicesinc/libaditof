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

#include <aditof/log.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

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
        std::string str1("vi-output, adsd3500");
        std::string str2("/dev/mediaX");
        dev_path = str.substr(pos + str1.length() + 9, str2.length());
    } else {
        return Status::GENERIC_ERROR;
    }

    if (str.find("adsd3500") != string::npos) {
        std::string str1("/dev/v4l-subdevX");
        device_name = "adsd3500";
        pos = str.find("adsd3500");
        subdev_path = str.substr(pos + device_name.length() + 9, str1.length());
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
        std::string currentEntry(dp->d_name);
        if (currentEntry.compare(0, videoBaseName.length(), videoBaseName) ==
            0) {
            std::string fullvideoPath = videoDirPath + currentEntry;
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

    return status;
}