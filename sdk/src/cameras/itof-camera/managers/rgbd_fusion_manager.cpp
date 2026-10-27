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

void RgbdFusionManager::setRgbRotated180(bool rotated) {
    m_rgbRotated180 = rotated;
    m_coreg.setImageRotated180(rotated);
    m_rgbMap.clear();
}

void RgbdFusionManager::reset() {
    m_coreg = RGBDCoregistration();
    m_coreg.setImageRotated180(m_rgbRotated180);
    m_warnedUnavailable = false;
    m_rgbMap.clear();
}

void RgbdFusionManager::setPixelRays(std::vector<int16_t> rays) {
    m_rays = std::move(rays);
    m_rgbMap.clear();
}

Status RgbdFusionManager::fuse(Frame *frame, int16_t *xyz, const uint8_t *bgr,
                               uint32_t bgrWidth, uint32_t bgrHeight,
                               uint32_t bgrStride) {
    uint16_t *rgbRaw = nullptr;
    uint16_t *colorRaw = nullptr;
    FrameDataDetails rgbDetails, colorDetails;
    const bool haveRgb = frame->getData("rgb", &rgbRaw) == Status::OK &&
                         rgbRaw &&
                         frame->getDataDetails("rgb", rgbDetails) == Status::OK;
    const bool haveColor =
        frame->getData("xyz_color", &colorRaw) == Status::OK && colorRaw &&
        frame->getDataDetails("xyz_color", colorDetails) == Status::OK;
    uint8_t *rgbOut = haveRgb ? reinterpret_cast<uint8_t *>(rgbRaw) : nullptr;
    uint8_t *color =
        haveColor ? reinterpret_cast<uint8_t *>(colorRaw) : nullptr;
    const bool needPoints = xyz && color;
    if (!rgbOut && !needPoints) {
        if (color) {
            std::memset(color, 0, colorDetails.bytesCount);
        }
        return Status::OK;
    }

    if (!isReady()) {
        if (!m_warnedUnavailable) {
            LOG(WARNING) << "RGBD fusion: no calibration, rgb and xyz_color "
                            "left empty";
            m_warnedUnavailable = true;
        }
        if (rgbOut) {
            std::memset(rgbOut, 0, rgbDetails.bytesCount);
        }
        if (color) {
            std::memset(color, 0, colorDetails.bytesCount);
        }
        return Status::UNAVAILABLE;
    }

    Status status = Status::OK;
    if (rgbOut) {
        const std::size_t numPixels =
            static_cast<std::size_t>(rgbDetails.width) * rgbDetails.height;
        if (bgr && (m_rgbMap.size() != numPixels * 2 ||
                    m_rgbMapWidth != bgrWidth || m_rgbMapHeight != bgrHeight)) {
            m_rgbMap.clear();
            if (m_rays.size() == numPixels * 3) {
                m_rgbMap.resize(numPixels * 2);
                if (m_coreg.mapToRGB(m_rays.data(), numPixels, bgrWidth,
                                     bgrHeight,
                                     m_rgbMap.data()) != Status::OK) {
                    m_rgbMap.clear();
                }
                m_rgbMapWidth = bgrWidth;
                m_rgbMapHeight = bgrHeight;
            }
        }
        if (bgr && !m_rgbMap.empty()) {
            status = RGBDCoregistration::remapRGB(m_rgbMap.data(), numPixels,
                                                  bgr, bgrWidth, bgrHeight,
                                                  bgrStride, rgbOut);
        }
        if (!bgr || m_rgbMap.empty() || status != Status::OK) {
            std::memset(rgbOut, 0, rgbDetails.bytesCount);
        }
    }

    if (!needPoints) {
        return status;
    }
    const std::size_t numPoints =
        static_cast<std::size_t>(colorDetails.width) * colorDetails.height;
    Status colorStatus = m_coreg.colorizePointCloud(
        xyz, numPoints, bgr, bgrWidth, bgrHeight, bgrStride, color);
    if (colorStatus != Status::OK) {
        std::memset(color, 0, colorDetails.bytesCount);
        return colorStatus;
    }

    if (m_overlapOnly) {
        const uint8_t outOfView =
            static_cast<uint8_t>(XYZColorStatus::OutOfView);
        for (std::size_t i = 0; i < numPoints; ++i) {
            if (color[4 * i + 3] == outOfView) {
                xyz[3 * i] = xyz[3 * i + 1] = xyz[3 * i + 2] = 0;
            }
        }
    }
    return status;
}
