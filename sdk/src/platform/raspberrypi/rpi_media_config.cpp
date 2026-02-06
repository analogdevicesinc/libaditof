/*
 * MIT License
 *
 * Copyright (c) 2025 Analog Devices, Inc.
 */
#include "rpi_media_config.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

#include <array>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <unistd.h>
#include <vector>

namespace aditof {
namespace platform {
namespace rpi {

static std::string execCommand(const std::string &cmd) {
    std::array<char, 256> buffer;
    std::string result;

    FILE *pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        return "";
    }

    while (fgets(buffer.data(), buffer.size(), pipe)) {
        result += buffer.data();
    }

    pclose(pipe);
    return result;
}

std::string detectRpiMediaDevice() {
    // Scan all media devices to find the RP1 CFE controller
    for (int i = 0; i < 10; i++) {
        std::string mediaDevice = "/dev/media" + std::to_string(i);
        std::string cmd = "media-ctl -d " + mediaDevice +
                          " -p 2>/dev/null | grep -q 'driver.*rp1-cfe'";

        if (system(cmd.c_str()) == 0) {
#ifdef USE_GLOG
            LOG(INFO) << "Detected RP1 CFE media device: " << mediaDevice;
#else
            LOG(INFO) << "Detected RP1 CFE media device: " << mediaDevice;
#endif
            return mediaDevice;
        }
    }

#ifdef USE_GLOG
    LOG(WARNING)
        << "Could not detect RP1 CFE media device, using default /dev/media3";
#else
    LOG(WARNING)
        << "Could not detect RP1 CFE media device, using default /dev/media3";
#endif
    return "/dev/media3"; // Fallback
}

std::string detectSensorEntity(const std::string &mediaDevice) {
    // Query media topology to find adsd3500 sensor entity
    std::string cmd = "media-ctl -d " + mediaDevice +
                      " -p 2>/dev/null | grep -oP 'entity.*adsd3500 "
                      "[0-9]+-[0-9a-f]+' | head -1 | sed 's/^.*: //'";
    std::string result = execCommand(cmd);

    // Remove trailing whitespace/newlines
    size_t end = result.find_last_not_of(" \t\n\r");
    if (end != std::string::npos) {
        result = result.substr(0, end + 1);
    }

    if (!result.empty()) {
#ifdef USE_GLOG
        LOG(INFO) << "Detected sensor entity: " << result;
#else
        LOG(INFO) << "Detected sensor entity: " << result;
#endif
        return result;
    }

#ifdef USE_GLOG
    LOG(WARNING)
        << "Could not detect sensor entity, using default 'adsd3500 10-0038'";
#else
    LOG(WARNING)
        << "Could not detect sensor entity, using default 'adsd3500 10-0038'";
#endif
    return "adsd3500 10-0038"; // Fallback
}

bool configureMediaPipeline(const std::string &mediaDevice,
                            const std::string &videoDevice,
                            const std::string &sensorEntity, int width,
                            int height, int bitDepth) {

#ifdef USE_GLOG
    LOG(INFO) << "Configuring RP1 CFE media pipeline: " << width << "x"
              << height << " @ " << bitDepth << "-bit";
#else
    LOG(INFO) << "Configuring RP1 CFE media pipeline: " << width << "x"
              << height << " @ " << bitDepth << "-bit";
#endif

    // Determine media bus code and pixel format based on bit depth
    std::string mbusCode;
    std::vector<std::string> pixelFormats;

    if (bitDepth == 12) {
        mbusCode = "SRGGB12_1X12";
        pixelFormats = {"pRAA", "RG12", "BG12", "RGGB"}; // Try in order
    } else if (bitDepth == 8) {
        mbusCode = "SRGGB8_1X8";
        pixelFormats = {"RGGB", "BA81", "GRBG", "GBRG"};
    } else {
#ifdef USE_GLOG
        LOG(ERROR) << "Unsupported bit depth: " << bitDepth;
#else
        LOG(ERROR) << "Unsupported bit depth: " << bitDepth;
#endif
        return false;
    }

    const std::string csi2Entity = "csi2";
    const std::string formatSpec =
        "field:none colorspace:raw xfer:none ycbcr:601 quantization:full-range";

    // Step 1: Reset media links
#ifdef USE_GLOG
    DLOG(INFO) << "Resetting media links...";
#else
    DLOG(INFO) << "Resetting media links...";
#endif
    std::string cmd = "media-ctl -d " + mediaDevice + " -r 2>/dev/null";
    execCommand(cmd);
    usleep(500000); // 0.5s delay

    // Step 2: Configure sensor format
#ifdef USE_GLOG
    DLOG(INFO) << "Configuring sensor format...";
#else
    DLOG(INFO) << "Configuring sensor format...";
#endif
    std::ostringstream oss;
    oss << "media-ctl -d " << mediaDevice << " -V '\"" << sensorEntity << "\":0"
        << "[fmt:" << mbusCode << "/" << width << "x" << height << " "
        << formatSpec << "]' 2>/dev/null";
    execCommand(oss.str());

    // Step 3: Configure CSI2 sink pad (pad 0)
#ifdef USE_GLOG
    DLOG(INFO) << "Configuring CSI2 sink pad...";
#else
    DLOG(INFO) << "Configuring CSI2 sink pad...";
#endif
    oss.str("");
    oss << "media-ctl -d " << mediaDevice << " -V '\"" << csi2Entity << "\":0"
        << "[fmt:" << mbusCode << "/" << width << "x" << height << " "
        << formatSpec << "]' 2>/dev/null";
    execCommand(oss.str());

    // Step 4: Configure CSI2 source pad (pad 4)
#ifdef USE_GLOG
    DLOG(INFO) << "Configuring CSI2 source pad...";
#else
    DLOG(INFO) << "Configuring CSI2 source pad...";
#endif
    oss.str("");
    oss << "media-ctl -d " << mediaDevice << " -V '\"" << csi2Entity << "\":4"
        << "[fmt:" << mbusCode << "/" << width << "x" << height << " "
        << formatSpec << "]' 2>/dev/null";
    execCommand(oss.str());

    // Step 5: Re-establish media links
#ifdef USE_GLOG
    DLOG(INFO) << "Re-establishing media links...";
#else
    DLOG(INFO) << "Re-establishing media links...";
#endif
    oss.str("");
    oss << "media-ctl -d " << mediaDevice << " -l '\"" << sensorEntity
        << "\":0->\"" << csi2Entity << "\":0[1]' 2>/dev/null";
    execCommand(oss.str());

    oss.str("");
    oss << "media-ctl -d " << mediaDevice << " -l '\"" << csi2Entity
        << "\":4->\"rp1-cfe-csi2_ch0\":0[1]' 2>/dev/null";
    execCommand(oss.str());

    // Step 6: Configure video device format
    // Try each pixel format until one succeeds
#ifdef USE_GLOG
    DLOG(INFO) << "Configuring video device format...";
#else
    DLOG(INFO) << "Configuring video device format...";
#endif

    bool formatSet = false;
    std::string finalPixFmt;

    for (const auto &pixfmt : pixelFormats) {
        oss.str("");
        oss << "v4l2-ctl -d " << videoDevice << " --set-fmt-video="
            << "width=" << width << ","
            << "height=" << height << ","
            << "pixelformat=" << pixfmt << ","
            << "field=none,"
            << "colorspace=raw"
            << " 2>/dev/null";

        std::string result = execCommand(oss.str());
        if (result.find("error") == std::string::npos &&
            result.find("Error") == std::string::npos) {
            formatSet = true;
            finalPixFmt = pixfmt;
#ifdef USE_GLOG
            LOG(INFO) << "Successfully set pixel format: " << pixfmt;
#else
            LOG(INFO) << "Successfully set pixel format: " << pixfmt;
#endif
            break;
        }
    }

    if (!formatSet) {
#ifdef USE_GLOG
        LOG(ERROR) << "Failed to set any pixel format for video device";
#else
        LOG(ERROR) << "Failed to set any pixel format for video device";
#endif
        return false;
    }

    // Verify configuration
    oss.str("");
    oss << "v4l2-ctl -d " << videoDevice << " --get-fmt-video 2>/dev/null";
    std::string fmtInfo = execCommand(oss.str());

#ifdef USE_GLOG
    DLOG(INFO) << "Video device format: " << fmtInfo;
#else
    DLOG(INFO) << "Video device format: " << fmtInfo;
#endif

#ifdef USE_GLOG
    LOG(INFO) << "RP1 CFE media pipeline configured successfully";
#else
    LOG(INFO) << "RP1 CFE media pipeline configured successfully";
#endif

    return true;
}

} // namespace rpi
} // namespace platform
} // namespace aditof
