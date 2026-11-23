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
// Depth frames measured at each start before the reference depth is fixed.
constexpr uint32_t REFERENCE_DEPTH_FRAMES = 5;
constexpr std::size_t DEPTH_SAMPLE_STEP = 16;
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
    m_referenceFrames = 0;
    m_depthSamples.clear();
    m_planeXyzDepthMm = 0.0f;
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
        out.numPoints =
            static_cast<std::size_t>(rgbDetails.width) * rgbDetails.height;
    }
    if (frame->getData("xyz_color", &colorRaw) == Status::OK && colorRaw &&
        frame->getDataDetails("xyz_color", colorDetails) == Status::OK) {
        out.color = reinterpret_cast<uint8_t *>(colorRaw);
        out.colorBytes = colorDetails.bytesCount;
        if (!out.rgb) {
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

void RgbdFusionManager::writeRgb(const Outputs &out, const uint8_t *rgba) {
    if (!out.rgb) {
        return;
    }
    for (std::size_t i = 0; i < out.numPoints; ++i) {
        out.rgb[3 * i] = rgba[4 * i + 2];
        out.rgb[3 * i + 1] = rgba[4 * i + 1];
        out.rgb[3 * i + 2] = rgba[4 * i];
    }
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

const int16_t *RgbdFusionManager::planeXyz(Frame *frame, const XYZTable *rays,
                                           std::size_t numPoints) {
    if (!rays || !rays->p_x_table || !rays->p_y_table || !rays->p_z_table) {
        return nullptr;
    }
    uint16_t *depth = nullptr;
    FrameDataDetails details;
    if (m_referenceFrames < REFERENCE_DEPTH_FRAMES &&
        frame->getData("depth", &depth) == Status::OK && depth &&
        frame->getDataDetails("depth", details) == Status::OK &&
        static_cast<std::size_t>(details.width) * details.height == numPoints) {
        const std::size_t before = m_depthSamples.size();
        for (std::size_t i = 0; i < numPoints; i += DEPTH_SAMPLE_STEP) {
            if (depth[i]) {
                m_depthSamples.push_back(depth[i]);
            }
        }
        if (m_depthSamples.size() > before) {
            ++m_referenceFrames;
            auto mid = m_depthSamples.begin() + m_depthSamples.size() / 2;
            std::nth_element(m_depthSamples.begin(), mid, m_depthSamples.end());
            m_planeDepthMm = *mid;
            if (m_referenceFrames == REFERENCE_DEPTH_FRAMES) {
                LOG(INFO) << "RGB view: reference depth " << m_planeDepthMm
                          << " mm (median of the first "
                          << REFERENCE_DEPTH_FRAMES
                          << " frames), fixed until the next start";
                m_depthSamples.clear();
                m_depthSamples.shrink_to_fit();
            }
        }
    }

    if (m_planeXyzDepthMm == m_planeDepthMm &&
        m_planeXyzRays == rays->p_x_table &&
        m_planeXyz.size() == numPoints * 3) {
        return m_planeXyz.data();
    }
    m_planeXyz.resize(numPoints * 3);
    const float d = m_planeDepthMm;
    for (std::size_t i = 0; i < numPoints; ++i) {
        m_planeXyz[3 * i] =
            static_cast<int16_t>(std::lround(rays->p_x_table[i] * d));
        m_planeXyz[3 * i + 1] =
            static_cast<int16_t>(std::lround(rays->p_y_table[i] * d));
        m_planeXyz[3 * i + 2] =
            static_cast<int16_t>(std::lround(rays->p_z_table[i] * d));
    }
    m_planeXyzDepthMm = d;
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

    const int16_t *plane = planeXyz(frame, rays, out.numPoints);
    if (!isReady() || !plane) {
        if (!m_warnedUnavailable) {
            LOG(WARNING) << "RGBD fusion: no calibration or XYZ tables, "
                            "rgb and xyz_color left empty";
            m_warnedUnavailable = true;
        }
        clearOutputs(out);
        return Status::UNAVAILABLE;
    }

    uint8_t *rgba = colorBuffer(out);
    Status status = m_coreg.colorizePointCloud(
        plane, out.numPoints, bgr, bgrWidth, bgrHeight, bgrStride, rgba);
    if (status != Status::OK) {
        clearOutputs(out);
        return status;
    }
    writeRgb(out, rgba);

    if (out.color && xyz) {
        const uint8_t noDepth = static_cast<uint8_t>(XYZColorStatus::NoDepth);
        for (std::size_t i = 0; i < out.numPoints; ++i) {
            if (xyz[3 * i + 2] == 0) {
                uint8_t *p = rgba + 4 * i;
                p[0] = p[1] = p[2] = 0;
                p[3] = noDepth;
            }
        }
        zeroOutOfView(out, xyz, rgba);
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
        m_coreg.colorizePointCloud(xyz, out.numPoints, nullptr, rgbWidth,
                                   rgbHeight, 0, rgba) != Status::OK) {
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
    // decides the status; only black pixels keep the recomputed one.
    for (std::size_t i = 0; i < out.numPoints; ++i) {
        const uint8_t *src = bgr + 3 * i;
        if (src[0] | src[1] | src[2]) {
            uint8_t *p = rgba + 4 * i;
            p[0] = src[2];
            p[1] = src[1];
            p[2] = src[0];
            p[3] = static_cast<uint8_t>(XYZColorStatus::Valid);
        }
    }
    writeRgb(out, rgba);
    zeroOutOfView(out, xyz, rgba);
    return Status::OK;
}
