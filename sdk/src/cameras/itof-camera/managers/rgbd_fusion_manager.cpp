/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Analog Devices, Inc.
 * All rights reserved.
 */
#include "rgbd_fusion_manager.h"

#include <aditof/log.h>

#include <algorithm>
#include <cmath>
#include <cstring>

using namespace aditof;

namespace {
// The "rgb" view projects every ToF ray at this one depth.
constexpr float VIEW_DEPTH_MM = 1000.0f;
} // namespace

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
}

void RgbdFusionManager::reset() {
    m_coreg = RGBDCoregistration();
    m_coreg.setImageRotated180(m_rgbRotated180);
    m_warnedUnavailable = false;
    m_planeXyzRays = nullptr;
    m_viewLookupRays = nullptr;
}

RgbdFusionManager::Outputs RgbdFusionManager::getOutputs(Frame *frame) {
    Outputs out;
    uint16_t *rgbRaw = nullptr;
    uint16_t *colorRaw = nullptr;
    FrameDataDetails rgbDetails, colorDetails;
    if (frame->getData("rgb", &rgbRaw) == Status::OK && rgbRaw &&
        frame->getDataDetails("rgb", rgbDetails) == Status::OK) {
        out.rgb = reinterpret_cast<uint8_t *>(rgbRaw);
        out.rgbBytes = rgbDetails.bytesCount;
        out.width = rgbDetails.width;
        out.height = rgbDetails.height;
        out.numPoints =
            static_cast<std::size_t>(rgbDetails.width) * rgbDetails.height;
    }
    if (frame->getData("xyz_color", &colorRaw) == Status::OK && colorRaw &&
        frame->getDataDetails("xyz_color", colorDetails) == Status::OK) {
        out.color = reinterpret_cast<uint8_t *>(colorRaw);
        out.colorBytes = colorDetails.bytesCount;
        if (!out.rgb) {
            out.width = colorDetails.width;
            out.height = colorDetails.height;
            out.numPoints = static_cast<std::size_t>(colorDetails.width) *
                            colorDetails.height;
        }
    }
    return out;
}

void RgbdFusionManager::clearOutputs(const Outputs &out) {
    if (out.rgb) {
        std::memset(out.rgb, 0, out.rgbBytes);
    }
    if (out.color) {
        std::memset(out.color, 0, out.colorBytes);
    }
}

uint8_t *RgbdFusionManager::colorBuffer(const Outputs &out) {
    if (out.color) {
        return out.color;
    }
    m_colorScratch.resize(out.numPoints * 4);
    return m_colorScratch.data();
}

void RgbdFusionManager::zeroOutOfView(const Outputs &out, int16_t *xyz,
                                      const uint8_t *rgba) {
    if (!out.color || !xyz) {
        return;
    }
    const uint8_t outOfView = static_cast<uint8_t>(XYZColorStatus::OutOfView);
    for (std::size_t i = 0; i < out.numPoints; ++i) {
        if (rgba[4 * i + 3] == outOfView) {
            xyz[3 * i] = xyz[3 * i + 1] = xyz[3 * i + 2] = 0;
        }
    }
}

const int16_t *RgbdFusionManager::planeXyz(const XYZTable *rays,
                                           std::size_t numPoints) {
    if (!rays || !rays->p_x_table || !rays->p_y_table || !rays->p_z_table) {
        return nullptr;
    }
    if (m_planeXyzRays == rays->p_x_table &&
        m_planeXyz.size() == numPoints * 3) {
        return m_planeXyz.data();
    }
    m_planeXyz.resize(numPoints * 3);
    const float d = VIEW_DEPTH_MM;
    for (std::size_t i = 0; i < numPoints; ++i) {
        m_planeXyz[3 * i] =
            static_cast<int16_t>(std::lround(rays->p_x_table[i] * d));
        m_planeXyz[3 * i + 1] =
            static_cast<int16_t>(std::lround(rays->p_y_table[i] * d));
        m_planeXyz[3 * i + 2] =
            static_cast<int16_t>(std::lround(rays->p_z_table[i] * d));
    }
    m_planeXyzRays = rays->p_x_table;
    return m_planeXyz.data();
}

Status RgbdFusionManager::fuse(Frame *frame, int16_t *xyz, const XYZTable *rays,
                               const uint8_t *bgr, uint32_t bgrWidth,
                               uint32_t bgrHeight, uint32_t bgrStride) {
    const Outputs out = getOutputs(frame);
    if (!out.rgb && !out.color) {
        return Status::OK;
    }

    const int16_t *plane = out.rgb ? planeXyz(rays, out.numPoints) : nullptr;
    if (!isReady() || (out.rgb && !plane) || (out.color && !xyz)) {
        if (!m_warnedUnavailable) {
            LOG(WARNING) << "RGBD fusion: no calibration, XYZ tables or xyz, "
                            "rgb and xyz_color left empty";
            m_warnedUnavailable = true;
        }
        clearOutputs(out);
        return Status::UNAVAILABLE;
    }

    if (out.rgb && !bgr) {
        std::memset(out.rgb, 0, out.rgbBytes);
    } else if (out.rgb) {
        // Fixed per mode and RGB image size, so built once.
        RGBDCoregistration::RGBLookup &lookup = m_viewLookup;
        if (m_viewLookupRays != m_planeXyzRays || lookup.rgbWidth != bgrWidth ||
            lookup.rgbHeight != bgrHeight || lookup.stride != bgrStride ||
            lookup.offset.size() != out.numPoints) {
            Status status = m_coreg.buildRgbLookup(
                plane, out.numPoints, bgrWidth, bgrHeight, bgrStride, lookup);
            if (status != Status::OK) {
                clearOutputs(out);
                return status;
            }
            m_viewLookupRays = m_planeXyzRays;
        }
        RGBDCoregistration::sampleRgbLookup(lookup, bgr, out.rgb);
    }

    if (out.color) {
        Status status = m_coreg.colorizeVisiblePoints(
            xyz, out.width, out.height, bgr, bgrWidth, bgrHeight, bgrStride,
            out.color);
        if (status != Status::OK) {
            clearOutputs(out);
            return status;
        }
        zeroOutOfView(out, xyz, out.color);
    }
    return Status::OK;
}

Status RgbdFusionManager::applyRecordedRgb(Frame *frame, int16_t *xyz,
                                           const uint8_t *bgr,
                                           std::size_t numPoints,
                                           uint32_t rgbWidth,
                                           uint32_t rgbHeight) {
    const Outputs out = getOutputs(frame);
    if (!out.rgb && !out.color) {
        return Status::OK;
    }
    if (!bgr || numPoints != out.numPoints) {
        LOG(WARNING) << "RGBD fusion: recorded rgb does not match frame ("
                     << numPoints << " vs " << out.numPoints << " pixels)";
        clearOutputs(out);
        return Status::INVALID_ARGUMENT;
    }

    uint8_t *rgba = colorBuffer(out);
    // No image: gives each point's status, NoColor where it is in view.
    if (!isReady() || !xyz ||
        m_coreg.colorizeVisiblePoints(xyz, out.width, out.height, nullptr,
                                      rgbWidth, rgbHeight, 0,
                                      rgba) != Status::OK) {
        if (!m_warnedUnavailable) {
            LOG(WARNING) << "RGBD fusion: no calibration or no xyz, "
                            "xyz_color left empty";
            m_warnedUnavailable = true;
        }
        clearOutputs(out);
        if (out.rgb) {
            std::memcpy(out.rgb, bgr, out.numPoints * 3);
        }
        return Status::UNAVAILABLE;
    }

    // Live xyz differs slightly from playback xyz, so a recorded colour
    // decides the status; only black and hidden points keep the recomputed one.
    const uint8_t occluded = static_cast<uint8_t>(XYZColorStatus::Occluded);
    for (std::size_t i = 0; i < out.numPoints; ++i) {
        const uint8_t *src = bgr + 3 * i;
        uint8_t *p = rgba + 4 * i;
        if ((src[0] | src[1] | src[2]) && p[3] != occluded) {
            p[0] = src[2];
            p[1] = src[1];
            p[2] = src[0];
            p[3] = static_cast<uint8_t>(XYZColorStatus::Valid);
        }
    }
    if (out.rgb) {
        std::memcpy(out.rgb, bgr, out.numPoints * 3);
    }
    zeroOutOfView(out, xyz, rgba);
    return Status::OK;
}
