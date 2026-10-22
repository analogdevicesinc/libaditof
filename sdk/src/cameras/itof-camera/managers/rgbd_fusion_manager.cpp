/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Analog Devices, Inc.
 * All rights reserved.
 */
#include "rgbd_fusion_manager.h"

#include <aditof/log.h>

#include <cstring>

using namespace aditof;

Status RgbdFusionManager::loadCalibration(const uint8_t *data,
                                          std::size_t size) {
    reset();
    Status status = m_coreg.loadCalibrationFrom160Bytes(data, size);
    if (status != Status::OK) {
        LOG(WARNING) << "RGBD fusion: calibration not loaded";
    }
    return status;
}

void RgbdFusionManager::reset() {
    m_coreg = RGBDCoregistration();
    m_warnedUnavailable = false;
}

Status RgbdFusionManager::colorize(Frame *frame, bool rgbValid) {
    uint16_t *colorRaw = nullptr;
    FrameDataDetails colorDetails;
    if (frame->getData("xyz_color", &colorRaw) != Status::OK || !colorRaw ||
        frame->getDataDetails("xyz_color", colorDetails) != Status::OK) {
        return Status::UNAVAILABLE;
    }
    uint8_t *color = reinterpret_cast<uint8_t *>(colorRaw);

    uint16_t *xyzRaw = nullptr;
    uint16_t *rgbRaw = nullptr;
    FrameDataDetails xyzDetails, rgbDetails;
    const bool haveInputs =
        isReady() && frame->getData("xyz", &xyzRaw) == Status::OK && xyzRaw &&
        frame->getData("rgb", &rgbRaw) == Status::OK && rgbRaw &&
        frame->getDataDetails("xyz", xyzDetails) == Status::OK &&
        frame->getDataDetails("rgb", rgbDetails) == Status::OK &&
        xyzDetails.width == colorDetails.width &&
        xyzDetails.height == colorDetails.height;
    if (!haveInputs) {
        if (!m_warnedUnavailable) {
            LOG(WARNING) << "RGBD fusion: no calibration or frame data, "
                            "xyz_color left empty";
            m_warnedUnavailable = true;
        }
        std::memset(color, 0, colorDetails.bytesCount);
        return Status::UNAVAILABLE;
    }

    int16_t *xyz = reinterpret_cast<int16_t *>(xyzRaw);
    const std::size_t numPoints =
        static_cast<std::size_t>(xyzDetails.width) * xyzDetails.height;
    Status status = m_coreg.colorizePointCloud(
        xyz, numPoints, reinterpret_cast<const uint8_t *>(rgbRaw),
        rgbDetails.width, rgbDetails.height, rgbDetails.width * 3u, color);
    if (status != Status::OK) {
        std::memset(color, 0, colorDetails.bytesCount);
        return status;
    }

    const uint8_t valid = static_cast<uint8_t>(XYZColorStatus::Valid);
    const uint8_t noColor = static_cast<uint8_t>(XYZColorStatus::NoColor);
    for (std::size_t i = 0; i < numPoints; ++i) {
        uint8_t *out = color + 4 * i;
        if (!rgbValid && out[3] == valid) {
            out[0] = out[1] = out[2] = 0;
            out[3] = noColor;
        }
        if (m_overlapOnly && out[3] != valid && out[3] != noColor) {
            xyz[3 * i] = xyz[3 * i + 1] = xyz[3 * i + 2] = 0;
        }
    }
    return Status::OK;
}
