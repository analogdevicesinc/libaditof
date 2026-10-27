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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "aditof/rgbd_coregistration.h"
#include "aditof/log.h"

#include <algorithm>
#include <cmath>
#include <cstring>

static constexpr float METRES_TO_MM = 1000.0f;

namespace aditof {

// ---------------------------------------------------------------------------
// Invert rgb2tof extrinsics to tof2rgb.
// Source: P_tof = R_src * P_rgb + t_src  (t_src in mm)
// Result: P_rgb = R_out * P_tof + t_out  (t_out in mm)
//   R_out = R_src^T
//   t_out = -R_src^T * t_src
// ---------------------------------------------------------------------------
static StereoExtrinsics invertRgb2Tof(const float R_src[9],
                                      const float t_src[3]) {
    StereoExtrinsics out;
    // Transpose
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out.R[r * 3 + c] = R_src[c * 3 + r];
    // t_tof2rgb = -R^T * t_rgb2tof
    for (int r = 0; r < 3; ++r) {
        out.t[r] = 0.0f;
        for (int c = 0; c < 3; ++c)
            out.t[r] -= out.R[r * 3 + c] * t_src[c];
    }
    return out;
}

// ===========================================================================
// RGBDCoregistration implementation
// ===========================================================================

RGBDCoregistration::RGBDCoregistration() {}

Status RGBDCoregistration::setCalibration(const RGBDCalibration &calib) {
    m_calib = calib;
    m_calibLoaded = true;
    return Status::OK;
}

// ---------------------------------------------------------------------------
// Load from the 160-byte (40-float) GET_RGBD_CALIBRATION_DATA (0x30) response.
//
// Layout (all IEEE-754 LE float32, matching firmware doc):
//   [0 –13] ToF intrinsics   (active mode)
//   [14–27] RGB intrinsics   (flash chunk 0x61)
//   [28–39] Extrinsics rgb2tof: r11..r33, t1..t3  (translation in metres)
// ---------------------------------------------------------------------------
Status RGBDCoregistration::loadCalibrationFrom160Bytes(const uint8_t *data,
                                                       std::size_t size) {
    static constexpr std::size_t EXPECTED = 160u;
    if (!data || size < EXPECTED) {
        LOG(ERROR) << "RGBD 0x30 response too small: expected " << EXPECTED
                   << " bytes, got " << size;
        return Status::INVALID_ARGUMENT;
    }

    // Decode 40 little-endian float32 values
    float f[40];
    for (int i = 0; i < 40; ++i) {
        std::memcpy(&f[i], data + i * 4, 4);
    }

    // Floats 0-13: ToF intrinsics (supplied by firmware for the active mode)
    m_calib.tof.fx = f[0];
    m_calib.tof.fy = f[1];
    m_calib.tof.cx = f[2];
    m_calib.tof.cy = f[3];
    m_calib.tof.codx = f[4];
    m_calib.tof.cody = f[5];
    m_calib.tof.k1 = f[6];
    m_calib.tof.k2 = f[7];
    m_calib.tof.k3 = f[8];
    m_calib.tof.k4 = f[9];
    m_calib.tof.k5 = f[10];
    m_calib.tof.k6 = f[11];
    m_calib.tof.p2 = f[12];
    m_calib.tof.p1 = f[13];

    // Floats 14-27: RGB intrinsics (from flash)
    m_calib.rgb.fx = f[14];
    m_calib.rgb.fy = f[15];
    m_calib.rgb.cx = f[16];
    m_calib.rgb.cy = f[17];
    m_calib.rgb.codx = f[18];
    m_calib.rgb.cody = f[19];
    m_calib.rgb.k1 = f[20];
    m_calib.rgb.k2 = f[21];
    m_calib.rgb.k3 = f[22];
    m_calib.rgb.k4 = f[23];
    m_calib.rgb.k5 = f[24];
    m_calib.rgb.k6 = f[25];
    m_calib.rgb.p2 = f[26];
    m_calib.rgb.p1 = f[27];

    // Floats 28-39: extrinsics rgb2tof — rotation (28-36) + translation (37-39) in metres
    float R_src[9], t_src_mm[3];
    for (int i = 0; i < 9; ++i)
        R_src[i] = f[28 + i];
    t_src_mm[0] = f[37] * METRES_TO_MM;
    t_src_mm[1] = f[38] * METRES_TO_MM;
    t_src_mm[2] = f[39] * METRES_TO_MM;

    m_calib.extrinsics = invertRgb2Tof(R_src, t_src_mm);

    m_calibLoaded = true;
    LOG(INFO) << "RGBD calibration loaded from chip (0x30 response)"
              << "  ToF fx=" << m_calib.tof.fx << "  RGB fx=" << m_calib.rgb.fx;
    return Status::OK;
}

// ---------------------------------------------------------------------------
// Project a normalised 3-D direction (X/Z, Y/Z) through the RGB lens model
// and return pixel coordinates.
// ---------------------------------------------------------------------------
void RGBDCoregistration::projectToRGB(float x_norm, float y_norm, float &u_px,
                                      float &v_px) const {
    const auto &K = m_calib.rgb;
    // Apply centre-of-distortion offset before computing r²
    float xp = x_norm - K.codx;
    float yp = y_norm - K.cody;
    float r2 = xp * xp + yp * yp;
    float r4 = r2 * r2;
    float r6 = r4 * r2;

    float numer = 1.0f + K.k1 * r2 + K.k2 * r4 + K.k3 * r6;
    float denom = 1.0f + K.k4 * r2 + K.k5 * r4 + K.k6 * r6;
    float radial = (std::abs(denom) > 1e-6f) ? (numer / denom) : numer;

    float x_dist = xp * radial + K.codx + 2.0f * K.p1 * xp * yp +
                   K.p2 * (r2 + 2.0f * xp * xp);
    float y_dist = yp * radial + K.cody + K.p1 * (r2 + 2.0f * yp * yp) +
                   2.0f * K.p2 * xp * yp;

    u_px = K.fx * x_dist + K.cx;
    v_px = K.fy * y_dist + K.cy;
}

float RGBDCoregistration::maxRay2(uint32_t rgbWidth, uint32_t rgbHeight) const {
    // The rational lens model can fold far off-image rays back into the
    // image; rays well beyond the image corners are rejected first.
    const auto &K = m_calib.rgb;
    const float edgeX =
        std::max(K.cx, static_cast<float>(rgbWidth) - K.cx) / K.fx;
    const float edgeY =
        std::max(K.cy, static_cast<float>(rgbHeight) - K.cy) / K.fy;
    return 2.0f * (edgeX * edgeX + edgeY * edgeY);
}

bool RGBDCoregistration::projectPoint(const int16_t *p, float maxU, float maxV,
                                      float maxRay2, float &u, float &v) const {
    if (p[2] == 0) {
        return false;
    }
    const float *R = m_calib.extrinsics.R;
    const float *t = m_calib.extrinsics.t;
    const float X = p[0], Y = p[1], Z = p[2];
    const float Xr = R[0] * X + R[1] * Y + R[2] * Z + t[0];
    const float Yr = R[3] * X + R[4] * Y + R[5] * Z + t[1];
    const float Zr = R[6] * X + R[7] * Y + R[8] * Z + t[2];
    if (Zr <= 0.0f) {
        return false;
    }
    const float invZ = 1.0f / Zr;
    const float xn = Xr * invZ;
    const float yn = Yr * invZ;
    if (xn * xn + yn * yn > maxRay2) {
        return false;
    }
    projectToRGB(xn, yn, u, v);
    if (!(u >= 0.0f && u <= maxU && v >= 0.0f && v <= maxV)) {
        return false;
    }
    if (m_rotated180) {
        u = maxU - u;
        v = maxV - v;
    }
    return true;
}

// Bilinear sample of a B, G, R image at (u, v) inside the image.
static void sampleBilinear(const uint8_t *bgr, uint32_t width, uint32_t height,
                           uint32_t stride, float u, float v, uint8_t out[3]) {
    const int x0 = static_cast<int>(u);
    const int y0 = static_cast<int>(v);
    const int x1 = std::min(x0 + 1, static_cast<int>(width) - 1);
    const int y1 = std::min(y0 + 1, static_cast<int>(height) - 1);
    // Weights in 1/256 steps
    const uint32_t wx = static_cast<uint32_t>((u - x0) * 256.0f);
    const uint32_t wy = static_cast<uint32_t>((v - y0) * 256.0f);
    const uint8_t *row0 = bgr + static_cast<std::size_t>(y0) * stride;
    const uint8_t *row1 = bgr + static_cast<std::size_t>(y1) * stride;
    const uint8_t *p00 = row0 + x0 * 3, *p01 = row0 + x1 * 3;
    const uint8_t *p10 = row1 + x0 * 3, *p11 = row1 + x1 * 3;
    for (int c = 0; c < 3; ++c) {
        const uint32_t top = p00[c] * (256 - wx) + p01[c] * wx;
        const uint32_t bottom = p10[c] * (256 - wx) + p11[c] * wx;
        out[c] = static_cast<uint8_t>(
            (top * (256 - wy) + bottom * wy + 32768) >> 16);
    }
}

Status RGBDCoregistration::colorizePointCloud(
    const int16_t *xyz, std::size_t numPoints, const uint8_t *bgr,
    uint32_t rgbWidth, uint32_t rgbHeight, uint32_t rgbStride,
    uint8_t *rgba) const {

    if (!m_calibLoaded) {
        LOG(ERROR) << "RGBDCoregistration: calibration not loaded";
        return Status::UNAVAILABLE;
    }
    if (!xyz || !rgba || rgbWidth < 2 || rgbHeight < 2 ||
        (bgr && rgbStride < rgbWidth * 3u)) {
        return Status::INVALID_ARGUMENT;
    }

    const float maxU = static_cast<float>(rgbWidth - 1);
    const float maxV = static_cast<float>(rgbHeight - 1);
    const float rayLimit2 = maxRay2(rgbWidth, rgbHeight);

    for (std::size_t i = 0; i < numPoints; ++i) {
        const int16_t *p = xyz + 3 * i;
        uint8_t *out = rgba + 4 * i;
        out[0] = out[1] = out[2] = 0;
        float u, v;
        if (p[2] == 0) {
            out[3] = static_cast<uint8_t>(XYZColorStatus::NoDepth);
        } else if (!projectPoint(p, maxU, maxV, rayLimit2, u, v)) {
            out[3] = static_cast<uint8_t>(XYZColorStatus::OutOfView);
        } else if (!bgr) {
            out[3] = static_cast<uint8_t>(XYZColorStatus::NoColor);
        } else {
            uint8_t px[3];
            sampleBilinear(bgr, rgbWidth, rgbHeight, rgbStride, u, v, px);
            out[0] = px[2];
            out[1] = px[1];
            out[2] = px[0];
            out[3] = static_cast<uint8_t>(XYZColorStatus::Valid);
        }
    }

    return Status::OK;
}

Status RGBDCoregistration::mapToRGB(const int16_t *xyz, std::size_t numPoints,
                                    uint32_t rgbWidth, uint32_t rgbHeight,
                                    float *uv) const {
    if (!m_calibLoaded) {
        LOG(ERROR) << "RGBDCoregistration: calibration not loaded";
        return Status::UNAVAILABLE;
    }
    if (!xyz || !uv || rgbWidth < 2 || rgbHeight < 2) {
        return Status::INVALID_ARGUMENT;
    }

    const float maxU = static_cast<float>(rgbWidth - 1);
    const float maxV = static_cast<float>(rgbHeight - 1);
    const float rayLimit2 = maxRay2(rgbWidth, rgbHeight);
    for (std::size_t i = 0; i < numPoints; ++i) {
        float u, v;
        if (projectPoint(xyz + 3 * i, maxU, maxV, rayLimit2, u, v)) {
            uv[2 * i] = u;
            uv[2 * i + 1] = v;
        } else {
            uv[2 * i] = uv[2 * i + 1] = -1.0f;
        }
    }
    return Status::OK;
}

Status RGBDCoregistration::remapRGB(const float *uv, std::size_t numPoints,
                                    const uint8_t *bgr, uint32_t rgbWidth,
                                    uint32_t rgbHeight, uint32_t rgbStride,
                                    uint8_t *bgrOut) {
    if (!uv || !bgr || !bgrOut || rgbWidth < 2 || rgbHeight < 2 ||
        rgbStride < rgbWidth * 3u) {
        return Status::INVALID_ARGUMENT;
    }
    for (std::size_t i = 0; i < numPoints; ++i) {
        uint8_t *out = bgrOut + 3 * i;
        if (uv[2 * i] < 0.0f) {
            out[0] = out[1] = out[2] = 0;
        } else {
            sampleBilinear(bgr, rgbWidth, rgbHeight, rgbStride, uv[2 * i],
                           uv[2 * i + 1], out);
        }
    }
    return Status::OK;
}

} // namespace aditof
