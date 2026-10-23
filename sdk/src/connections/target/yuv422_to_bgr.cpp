/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Analog Devices, Inc.
 * All rights reserved.
 */
#include "yuv422_to_bgr.h"

#ifdef HAS_RGB_CAMERA

#include <cmath>
#include <vector>

#if defined(__ARM_NEON) || defined(__aarch64__)
#include <arm_neon.h>
#define YUV_USE_NEON 1
#endif

namespace aditof {

namespace {

constexpr int kShift = 16;

// Scalar path, Q16.
struct Coefficients {
    int32_t y;
    int32_t yOffset;
    int32_t rv;
    int32_t gu;
    int32_t gv;
    int32_t bu;
};

struct Factors {
    double y, rv, gu, gv, bu;
    int yOffset;
};

Factors makeFactors(RGBYuvMatrix matrix, bool fullRange) {
    const bool bt709 = matrix == RGBYuvMatrix::BT709;
    // Limited range: Y spans 16-235 and U/V 16-240.
    const double yScale = fullRange ? 1.0 : 255.0 / 219.0;
    const double cScale = fullRange ? 1.0 : 255.0 / 224.0;
    return {yScale,
            (bt709 ? 1.5748 : 1.402) * cScale,
            (bt709 ? 0.187324 : 0.344136) * cScale,
            (bt709 ? 0.468124 : 0.714136) * cScale,
            (bt709 ? 1.8556 : 1.772) * cScale,
            fullRange ? 0 : 16};
}

int32_t toFixed(double value, int shift) {
    return static_cast<int32_t>(std::lround(value * (1 << shift)));
}

Coefficients makeCoefficients(const Factors &f) {
    return {toFixed(f.y, kShift),  f.yOffset,
            toFixed(f.rv, kShift), toFixed(f.gu, kShift),
            toFixed(f.gv, kShift), toFixed(f.bu, kShift)};
}

inline uint8_t toByte(int32_t fixed) {
    const int32_t value = (fixed + (1 << (kShift - 1))) >> kShift;
    return static_cast<uint8_t>(value < 0 ? 0 : (value > 255 ? 255 : value));
}

template <bool kUyvy>
void convertPairsScalar(const uint8_t *src, uint8_t *dst, uint32_t pairs,
                        const Coefficients &c) {
    for (uint32_t i = 0; i < pairs; ++i, src += 4, dst += 6) {
        const int32_t y0 = kUyvy ? src[1] : src[0];
        const int32_t u = (kUyvy ? src[0] : src[1]) - 128;
        const int32_t y1 = kUyvy ? src[3] : src[2];
        const int32_t v = (kUyvy ? src[2] : src[3]) - 128;
        const int32_t r = c.rv * v;
        const int32_t g = -c.gu * u - c.gv * v;
        const int32_t b = c.bu * u;
        const int32_t l0 = c.y * (y0 - c.yOffset);
        const int32_t l1 = c.y * (y1 - c.yOffset);
        dst[0] = toByte(l0 + b);
        dst[1] = toByte(l0 + g);
        dst[2] = toByte(l0 + r);
        dst[3] = toByte(l1 + b);
        dst[4] = toByte(l1 + g);
        dst[5] = toByte(l1 + r);
    }
}

#ifdef YUV_USE_NEON
// Chroma terms in Q6 and luma in Q7 keep every product within int16; the sums
// saturate only where the result clips to 0 or 255 anyway.
struct NeonCoefficients {
    uint8_t yQ7;
    uint16_t yOffsetQ7;
    int16_t rv, gu, gv, bu;
};

NeonCoefficients makeNeonCoefficients(const Factors &f) {
    const int32_t yQ7 = toFixed(f.y, 7);
    return {static_cast<uint8_t>(yQ7),
            static_cast<uint16_t>(yQ7 * f.yOffset),
            static_cast<int16_t>(toFixed(f.rv, 6)),
            static_cast<int16_t>(toFixed(f.gu, 6)),
            static_cast<int16_t>(toFixed(f.gv, 6)),
            static_cast<int16_t>(toFixed(f.bu, 6))};
}

// Eight pixel pairs to sixteen BGR pixels.
inline void convertEightPairsNeon(uint8x8_t y0, uint8x8_t y1, uint8x8_t u8,
                                  uint8x8_t v8, uint8_t *dst,
                                  const NeonCoefficients &k) {
    const uint8x8_t bias = vdup_n_u8(128);
    const int16x8_t u = vreinterpretq_s16_u16(vsubl_u8(u8, bias));
    const int16x8_t v = vreinterpretq_s16_u16(vsubl_u8(v8, bias));
    const int16x8_t rC = vmulq_n_s16(v, k.rv);
    const int16x8_t gC = vnegq_s16(vmlaq_n_s16(vmulq_n_s16(u, k.gu), v, k.gv));
    const int16x8_t bC = vmulq_n_s16(u, k.bu);

    const uint8x8_t yScale = vdup_n_u8(k.yQ7);
    const uint16x8_t yOffset = vdupq_n_u16(k.yOffsetQ7);
    // Halving subtract keeps luma below the offset negative and gives Q6.
    const int16x8_t l0 =
        vreinterpretq_s16_u16(vhsubq_u16(vmull_u8(y0, yScale), yOffset));
    const int16x8_t l1 =
        vreinterpretq_s16_u16(vhsubq_u16(vmull_u8(y1, yScale), yOffset));

    const uint8x8x2_t b = vzip_u8(vqrshrun_n_s16(vqaddq_s16(l0, bC), 6),
                                  vqrshrun_n_s16(vqaddq_s16(l1, bC), 6));
    const uint8x8x2_t g = vzip_u8(vqrshrun_n_s16(vqaddq_s16(l0, gC), 6),
                                  vqrshrun_n_s16(vqaddq_s16(l1, gC), 6));
    const uint8x8x2_t r = vzip_u8(vqrshrun_n_s16(vqaddq_s16(l0, rC), 6),
                                  vqrshrun_n_s16(vqaddq_s16(l1, rC), 6));
    uint8x16x3_t out;
    out.val[0] = vcombine_u8(b.val[0], b.val[1]);
    out.val[1] = vcombine_u8(g.val[0], g.val[1]);
    out.val[2] = vcombine_u8(r.val[0], r.val[1]);
    vst3q_u8(dst, out);
}

// Sixteen pixel pairs (64 source bytes, 96 output bytes) per step, straight
// through the row so the reads and writes stay sequential.
template <bool kUyvy>
uint32_t convertPairsNeon(const uint8_t *src, uint8_t *dst, uint32_t pairs,
                          const NeonCoefficients &k) {
    const uint32_t blocks = pairs / 16;
    for (uint32_t i = 0; i < blocks; ++i, src += 64, dst += 96) {
        const uint8x16x4_t p = vld4q_u8(src);
        const uint8x16_t y0 = kUyvy ? p.val[1] : p.val[0];
        const uint8x16_t u = kUyvy ? p.val[0] : p.val[1];
        const uint8x16_t y1 = kUyvy ? p.val[3] : p.val[2];
        const uint8x16_t v = kUyvy ? p.val[2] : p.val[3];
        convertEightPairsNeon(vget_low_u8(y0), vget_low_u8(y1), vget_low_u8(u),
                              vget_low_u8(v), dst, k);
        convertEightPairsNeon(vget_high_u8(y0), vget_high_u8(y1),
                              vget_high_u8(u), vget_high_u8(v), dst + 48, k);
    }
    return blocks * 16;
}
#endif

template <bool kUyvy>
void convertFrame(const uint8_t *src, size_t srcStride, uint8_t *dst,
                  uint32_t width, uint32_t height, const Factors &factors) {
    const Coefficients c = makeCoefficients(factors);
#ifdef YUV_USE_NEON
    const NeonCoefficients k = makeNeonCoefficients(factors);
#endif
    const uint32_t pairs = width / 2;
    const size_t dstStride = static_cast<size_t>(width) * 3;
    for (uint32_t row = 0; row < height;
         ++row, src += srcStride, dst += dstStride) {
        uint32_t done = 0;
#ifdef YUV_USE_NEON
        done = convertPairsNeon<kUyvy>(src, dst, pairs, k);
#endif
        convertPairsScalar<kUyvy>(src + done * 4, dst + done * 6, pairs - done,
                                  c);
    }
}

} // namespace

bool convertYuv422ToBgr(RGBFrame &frame, RGBYuvMatrix matrix, bool fullRange) {
    if (frame.format == RGBPixelFormat::BGR) {
        return true;
    }
    if (frame.format != RGBPixelFormat::UYVY &&
        frame.format != RGBPixelFormat::YUYV) {
        return false;
    }
    const uint32_t width = frame.width;
    const uint32_t height = frame.height;
    const size_t srcStride = frame.stride ? frame.stride : width * 2u;
    if (width == 0 || height == 0 || width % 2 != 0 || srcStride < width * 2u ||
        frame.data.size() < srcStride * (height - 1) + width * 2u) {
        return false;
    }

    const Factors factors = makeFactors(matrix, fullRange);
    std::vector<uint8_t> bgr(static_cast<size_t>(width) * height * 3);
    if (frame.format == RGBPixelFormat::UYVY) {
        convertFrame<true>(frame.data.data(), srcStride, bgr.data(), width,
                           height, factors);
    } else {
        convertFrame<false>(frame.data.data(), srcStride, bgr.data(), width,
                            height, factors);
    }

    frame.data.swap(bgr);
    frame.format = RGBPixelFormat::BGR;
    frame.stride = width * 3;
    return true;
}

} // namespace aditof

#endif // HAS_RGB_CAMERA
