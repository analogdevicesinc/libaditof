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

#if defined(__aarch64__)
#include <arm_neon.h>
#define RGBD_NEON 1
#elif defined(__SSE2__) || defined(_M_X64)
#include <emmintrin.h>
#define RGBD_SSE2 1
#endif

static constexpr float METRES_TO_MM = 1000.0f;

namespace aditof {

#if defined(RGBD_NEON)
using F4 = float32x4_t;
using M4 = uint32x4_t;
static inline F4 f4(float a) { return vdupq_n_f32(a); }
static inline F4 add(F4 a, F4 b) { return vaddq_f32(a, b); }
static inline F4 sub(F4 a, F4 b) { return vsubq_f32(a, b); }
static inline F4 mul(F4 a, F4 b) { return vmulq_f32(a, b); }
static inline F4 div(F4 a, F4 b) { return vdivq_f32(a, b); }
static inline F4 absF(F4 a) { return vabsq_f32(a); }
static inline M4 gt(F4 a, F4 b) { return vcgtq_f32(a, b); }
static inline M4 ge(F4 a, F4 b) { return vcgeq_f32(a, b); }
static inline M4 le(F4 a, F4 b) { return vcleq_f32(a, b); }
static inline M4 andM(M4 a, M4 b) { return vandq_u32(a, b); }
static inline F4 select(M4 m, F4 a, F4 b) { return vbslq_f32(m, a, b); }
static inline void store(float *p, F4 a) { vst1q_f32(p, a); }
static inline int maskBits(M4 m) {
    uint32_t lanes[4];
    vst1q_u32(lanes, m);
    return (lanes[0] & 1) | (lanes[1] & 2) | (lanes[2] & 4) | (lanes[3] & 8);
}
static inline void loadXyz4(const int16_t *p, F4 &x, F4 &y, F4 &z) {
    const int16x4x3_t v = vld3_s16(p);
    x = vcvtq_f32_s32(vmovl_s16(v.val[0]));
    y = vcvtq_f32_s32(vmovl_s16(v.val[1]));
    z = vcvtq_f32_s32(vmovl_s16(v.val[2]));
}
#elif defined(RGBD_SSE2)
using F4 = __m128;
using M4 = __m128;
static inline F4 f4(float a) { return _mm_set1_ps(a); }
static inline F4 add(F4 a, F4 b) { return _mm_add_ps(a, b); }
static inline F4 sub(F4 a, F4 b) { return _mm_sub_ps(a, b); }
static inline F4 mul(F4 a, F4 b) { return _mm_mul_ps(a, b); }
static inline F4 div(F4 a, F4 b) { return _mm_div_ps(a, b); }
static inline F4 absF(F4 a) { return _mm_andnot_ps(_mm_set1_ps(-0.0f), a); }
static inline M4 gt(F4 a, F4 b) { return _mm_cmpgt_ps(a, b); }
static inline M4 ge(F4 a, F4 b) { return _mm_cmpge_ps(a, b); }
static inline M4 le(F4 a, F4 b) { return _mm_cmple_ps(a, b); }
static inline M4 andM(M4 a, M4 b) { return _mm_and_ps(a, b); }
static inline F4 select(M4 m, F4 a, F4 b) {
    return _mm_or_ps(_mm_and_ps(m, a), _mm_andnot_ps(m, b));
}
static inline void store(float *p, F4 a) { _mm_storeu_ps(p, a); }
static inline int maskBits(M4 m) { return _mm_movemask_ps(m); }
static inline void loadXyz4(const int16_t *p, F4 &x, F4 &y, F4 &z) {
    x = _mm_setr_ps(p[0], p[3], p[6], p[9]);
    y = _mm_setr_ps(p[1], p[4], p[7], p[10]);
    z = _mm_setr_ps(p[2], p[5], p[8], p[11]);
}
#endif

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

bool RGBDCoregistration::projectPoint(const int16_t *p, float maxU, float maxV,
                                      float &u, float &v) const {
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
    projectToRGB(Xr * invZ, Yr * invZ, u, v);
    if (!(u >= 0.0f && u <= maxU && v >= 0.0f && v <= maxV)) {
        return false;
    }
    if (m_rotated180) {
        u = maxU - u;
        v = maxV - v;
    }
    return true;
}

void RGBDCoregistration::projectPoints(const int16_t *xyz, std::size_t count,
                                       float maxU, float maxV, float *u,
                                       float *v, uint8_t *status) const {
    const uint8_t noDepth = static_cast<uint8_t>(XYZColorStatus::NoDepth);
    const uint8_t outOfView = static_cast<uint8_t>(XYZColorStatus::OutOfView);
    const uint8_t inView = static_cast<uint8_t>(XYZColorStatus::Valid);
    std::size_t i = 0;

#if defined(RGBD_NEON) || defined(RGBD_SSE2)
    const float *R = m_calib.extrinsics.R;
    const float *t = m_calib.extrinsics.t;
    const auto &K = m_calib.rgb;
    const F4 r0 = f4(R[0]), r1 = f4(R[1]), r2 = f4(R[2]);
    const F4 r3 = f4(R[3]), r4 = f4(R[4]), r5 = f4(R[5]);
    const F4 r6 = f4(R[6]), r7 = f4(R[7]), r8 = f4(R[8]);
    const F4 t0 = f4(t[0]), t1 = f4(t[1]), t2 = f4(t[2]);
    const F4 codx = f4(K.codx), cody = f4(K.cody);
    const F4 k1 = f4(K.k1), k2 = f4(K.k2), k3 = f4(K.k3);
    const F4 k4 = f4(K.k4), k5 = f4(K.k5), k6 = f4(K.k6);
    const F4 p1 = f4(K.p1), p2 = f4(K.p2);
    const F4 twoP1 = f4(2.0f * K.p1), twoP2 = f4(2.0f * K.p2);
    const F4 fx = f4(K.fx), fy = f4(K.fy), cx = f4(K.cx), cy = f4(K.cy);
    const F4 zero = f4(0.0f), one = f4(1.0f), two = f4(2.0f);
    const F4 eps = f4(1e-6f), vMaxU = f4(maxU), vMaxV = f4(maxV);

    for (; i + 4 <= count; i += 4) {
        const int16_t *p = xyz + 3 * i;
        F4 X, Y, Z;
        loadXyz4(p, X, Y, Z);
        const F4 Xr = add(add(add(mul(r0, X), mul(r1, Y)), mul(r2, Z)), t0);
        const F4 Yr = add(add(add(mul(r3, X), mul(r4, Y)), mul(r5, Z)), t1);
        const F4 Zr = add(add(add(mul(r6, X), mul(r7, Y)), mul(r8, Z)), t2);
        // Lanes with Zr <= 0 give inf/NaN here and are masked out below.
        const F4 invZ = div(one, Zr);
        const F4 xp = sub(mul(Xr, invZ), codx);
        const F4 yp = sub(mul(Yr, invZ), cody);
        const F4 rr2 = add(mul(xp, xp), mul(yp, yp));
        const F4 rr4 = mul(rr2, rr2);
        const F4 rr6 = mul(rr4, rr2);
        const F4 numer =
            add(add(add(one, mul(k1, rr2)), mul(k2, rr4)), mul(k3, rr6));
        const F4 denom =
            add(add(add(one, mul(k4, rr2)), mul(k5, rr4)), mul(k6, rr6));
        const F4 radial =
            select(gt(absF(denom), eps), div(numer, denom), numer);
        const F4 xpyp = mul(xp, yp);
        const F4 xd = add(add(add(mul(xp, radial), codx), mul(twoP1, xpyp)),
                          mul(p2, add(rr2, mul(two, mul(xp, xp)))));
        const F4 yd = add(add(add(mul(yp, radial), cody),
                              mul(p1, add(rr2, mul(two, mul(yp, yp))))),
                          mul(twoP2, xpyp));
        F4 uu = add(mul(fx, xd), cx);
        F4 vv = add(mul(fy, yd), cy);
        const M4 ok =
            andM(andM(gt(Zr, zero), andM(ge(uu, zero), le(uu, vMaxU))),
                 andM(ge(vv, zero), le(vv, vMaxV)));
        if (m_rotated180) {
            uu = sub(vMaxU, uu);
            vv = sub(vMaxV, vv);
        }
        store(u + i, uu);
        store(v + i, vv);
        const int bits = maskBits(ok);
        for (int k = 0; k < 4; ++k) {
            status[i + k] = p[3 * k + 2] == 0
                                ? noDepth
                                : ((bits >> k) & 1 ? inView : outOfView);
        }
    }
#endif

    for (; i < count; ++i) {
        const int16_t *p = xyz + 3 * i;
        if (p[2] == 0) {
            status[i] = noDepth;
        } else {
            status[i] =
                projectPoint(p, maxU, maxV, u[i], v[i]) ? inView : outOfView;
        }
    }
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

// Mean of the n x n pixels centred on (u, v); needs width, height >= n.
static void sampleBox(const uint8_t *bgr, uint32_t width, uint32_t height,
                      uint32_t stride, float u, float v, int n,
                      uint8_t out[3]) {
    const float half = 0.5f * static_cast<float>(n - 1);
    const int x0 =
        std::min(std::max(static_cast<int>(std::lround(u - half)), 0),
                 static_cast<int>(width) - n);
    const int y0 =
        std::min(std::max(static_cast<int>(std::lround(v - half)), 0),
                 static_cast<int>(height) - n);
    uint32_t sum[3] = {0, 0, 0};

#if defined(RGBD_NEON) || defined(RGBD_SSE2)
    // 16-byte row loads: n <= 5 pixels per row, and the last image row only
    // when the load stays inside it.
    if (n * 3 <= 16 && (static_cast<uint32_t>(y0 + n) < height ||
                        static_cast<uint32_t>(x0) * 3u + 16u <= width * 3u)) {
        uint16_t cols[16];
        const uint8_t *p = bgr + static_cast<std::size_t>(y0) * stride + x0 * 3;
#if defined(RGBD_NEON)
        uint16x8_t lo = vdupq_n_u16(0), hi = vdupq_n_u16(0);
        for (int y = 0; y < n; ++y, p += stride) {
            const uint8x16_t row = vld1q_u8(p);
            lo = vaddw_u8(lo, vget_low_u8(row));
            hi = vaddw_high_u8(hi, row);
        }
        vst1q_u16(cols, lo);
        vst1q_u16(cols + 8, hi);
#else
        const __m128i zero = _mm_setzero_si128();
        __m128i lo = zero, hi = zero;
        for (int y = 0; y < n; ++y, p += stride) {
            const __m128i row =
                _mm_loadu_si128(reinterpret_cast<const __m128i *>(p));
            lo = _mm_add_epi16(lo, _mm_unpacklo_epi8(row, zero));
            hi = _mm_add_epi16(hi, _mm_unpackhi_epi8(row, zero));
        }
        _mm_storeu_si128(reinterpret_cast<__m128i *>(cols), lo);
        _mm_storeu_si128(reinterpret_cast<__m128i *>(cols + 8), hi);
#endif
        for (int x = 0; x < n; ++x) {
            sum[0] += cols[3 * x];
            sum[1] += cols[3 * x + 1];
            sum[2] += cols[3 * x + 2];
        }
    } else
#endif
    {
        for (int y = y0; y < y0 + n; ++y) {
            const uint8_t *p =
                bgr + static_cast<std::size_t>(y) * stride + x0 * 3;
            for (int x = 0; x < n; ++x, p += 3) {
                sum[0] += p[0];
                sum[1] += p[1];
                sum[2] += p[2];
            }
        }
    }
    const uint32_t count = static_cast<uint32_t>(n * n);
    for (int c = 0; c < 3; ++c) {
        out[c] = static_cast<uint8_t>((sum[c] + count / 2) / count);
    }
}

int RGBDCoregistration::rgbPixelsPerToFPixel() const {
    static constexpr int MAX_BOX = 8;
    const auto &tof = m_calib.tof;
    const auto &rgb = m_calib.rgb;
    if (tof.fx <= 0.0f || tof.fy <= 0.0f || rgb.fx <= 0.0f || rgb.fy <= 0.0f) {
        return 1;
    }
    // Geometric mean: with ToF rotation the ToF axes are swapped against RGB.
    const float ratio = std::sqrt((rgb.fx * rgb.fy) / (tof.fx * tof.fy));
    return std::min(std::max(static_cast<int>(std::lround(ratio)), 1), MAX_BOX);
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
    // Bilinear sampling already reads up to 2 x 2 RGB pixels per ToF pixel.
    int box = rgbPixelsPerToFPixel();
    if (box < 3 || static_cast<uint32_t>(box) > rgbWidth ||
        static_cast<uint32_t>(box) > rgbHeight) {
        box = 0;
    }

    static constexpr std::size_t CHUNK = 256;
    float u[CHUNK], v[CHUNK];
    uint8_t status[CHUNK];
    const uint8_t inView = static_cast<uint8_t>(XYZColorStatus::Valid);

    for (std::size_t base = 0; base < numPoints; base += CHUNK) {
        const std::size_t count = std::min(CHUNK, numPoints - base);
        projectPoints(xyz + 3 * base, count, maxU, maxV, u, v, status);
        for (std::size_t k = 0; k < count; ++k) {
            uint8_t *out = rgba + 4 * (base + k);
            out[0] = out[1] = out[2] = 0;
            if (status[k] != inView) {
                out[3] = status[k];
            } else if (!bgr) {
                out[3] = static_cast<uint8_t>(XYZColorStatus::NoColor);
            } else {
                uint8_t px[3];
                if (box) {
                    sampleBox(bgr, rgbWidth, rgbHeight, rgbStride, u[k], v[k],
                              box, px);
                } else {
                    sampleBilinear(bgr, rgbWidth, rgbHeight, rgbStride, u[k],
                                   v[k], px);
                }
                out[0] = px[2];
                out[1] = px[1];
                out[2] = px[0];
                out[3] = inView;
            }
        }
    }

    return Status::OK;
}

} // namespace aditof
