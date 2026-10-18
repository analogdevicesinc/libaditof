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

void RGBDCoregistration::setToFIntrinsics(
    const CameraIntrinsicsCalib &tofCalib) {
    m_calib.tof = tofCalib;
}

// ---------------------------------------------------------------------------
// Load from the 160-byte (40-float) GET_RGBD_CALIBRATION_DATA (0x30) response.
//
// Layout (all IEEE-754 LE float32, matching firmware doc):
//   [0 –13] ToF intrinsics   (firmware, per-mode)
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
// Undistort a normalised ToF image point: invert the rational Brown-Conrady
// model (OpenCV convention, same as cv::undistortPoints) by fixed-point
// iteration. The coefficients describe ideal -> distorted, so the observed
// pixel must be inverted, not run through the forward model.
//
//   radial = (1 + k1*r2 + k2*r4 + k3*r6) / (1 + k4*r2 + k5*r4 + k6*r6)
//   x_d = radial*x + 2*p1*x*y + p2*(r2 + 2*x^2)
//   y_d = radial*y + p1*(r2 + 2*y^2) + 2*p2*x*y
// ---------------------------------------------------------------------------
void RGBDCoregistration::undistortToFPoint(float x_norm, float y_norm,
                                           float &x_undist,
                                           float &y_undist) const {
    const auto &K = m_calib.tof;
    constexpr int kIterations = 5; // cv::undistortPoints default

    const float x0 = x_norm - K.codx;
    const float y0 = y_norm - K.cody;
    float xp = x0;
    float yp = y0;

    for (int i = 0; i < kIterations; ++i) {
        const float r2 = xp * xp + yp * yp;
        const float r4 = r2 * r2;
        const float r6 = r4 * r2;
        const float numer = 1.0f + K.k1 * r2 + K.k2 * r4 + K.k3 * r6;
        const float denom = 1.0f + K.k4 * r2 + K.k5 * r4 + K.k6 * r6;
        const float radial = numer / denom;
        if (!(radial > 0.0f)) {
            // Diverged (same guard as cv::undistortPoints): keep the raw point
            xp = x0;
            yp = y0;
            break;
        }
        const float dx = 2.0f * K.p1 * xp * yp + K.p2 * (r2 + 2.0f * xp * xp);
        const float dy = K.p1 * (r2 + 2.0f * yp * yp) + 2.0f * K.p2 * xp * yp;
        xp = (x0 - dx) / radial;
        yp = (y0 - dy) / radial;
    }

    x_undist = xp + K.codx;
    y_undist = yp + K.cody;
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

// ---------------------------------------------------------------------------
// Build the per-ToF-pixel lookup table into the RGB image: rgb_u/rgb_v get
// the RGB pixel each valid ToF pixel (d > 0) projects to, or -1 when the
// pixel has no depth, is behind the RGB camera or lands outside the image.
// Pixels are mapped independently, so a point hidden from the RGB camera
// still gets the colour of whatever is in front of it.
// ---------------------------------------------------------------------------
Status RGBDCoregistration::buildToFToRGBMap(const uint16_t *depth_mm,
                                            uint32_t tofWidth,
                                            uint32_t tofHeight, int32_t *rgb_u,
                                            int32_t *rgb_v, uint32_t rgbWidth,
                                            uint32_t rgbHeight) const {

    if (!m_calibLoaded) {
        LOG(ERROR) << "RGBDCoregistration: calibration not loaded";
        return Status::UNAVAILABLE;
    }
    if (!depth_mm || !rgb_u || !rgb_v) {
        return Status::INVALID_ARGUMENT;
    }

    const std::size_t nPixels = static_cast<std::size_t>(tofWidth) * tofHeight;
    std::fill(rgb_u, rgb_u + nPixels, -1);
    std::fill(rgb_v, rgb_v + nPixels, -1);

    const auto &tofK = m_calib.tof;
    const float *R = m_calib.extrinsics.R;
    const float *t = m_calib.extrinsics.t;
    const int iRgbW = static_cast<int>(rgbWidth);
    const int iRgbH = static_cast<int>(rgbHeight);

    for (uint32_t row = 0; row < tofHeight; ++row) {
        for (uint32_t col = 0; col < tofWidth; ++col) {
            const uint32_t idx = row * tofWidth + col;
            const uint16_t d = depth_mm[idx];
            if (d == 0)
                continue;

            float x_norm = (static_cast<float>(col) - tofK.cx) / tofK.fx;
            float y_norm = (static_cast<float>(row) - tofK.cy) / tofK.fy;

            float x_u, y_u;
            undistortToFPoint(x_norm, y_norm, x_u, y_u);

            // d is radial depth — convert to Z-depth before back-projection
            float ray_len = std::sqrt(x_u * x_u + y_u * y_u + 1.0f);
            float Zf = static_cast<float>(d) / ray_len;
            float Xf = x_u * Zf;
            float Yf = y_u * Zf;

            float Xr = R[0] * Xf + R[1] * Yf + R[2] * Zf + t[0];
            float Yr = R[3] * Xf + R[4] * Yf + R[5] * Zf + t[1];
            float Zr = R[6] * Xf + R[7] * Yf + R[8] * Zf + t[2];

            if (Zr <= 0.0f)
                continue;

            float u_px, v_px;
            projectToRGB(Xr / Zr, Yr / Zr, u_px, v_px);

            int u = static_cast<int>(u_px + 0.5f);
            int v = static_cast<int>(v_px + 0.5f);

            if (u < 0 || u >= iRgbW || v < 0 || v >= iRgbH)
                continue;

            rgb_u[idx] = u;
            rgb_v[idx] = v;
        }
    }

    return Status::OK;
}

} // namespace aditof
