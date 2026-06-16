/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, Analog Devices, Inc.
 * All rights reserved.
 */

#include "nv12_to_rgb.h"
#include <aditof/log.h>
#include <algorithm>
#include <cstring>

#if defined(__ARM_NEON) || defined(__aarch64__)
#include <arm_neon.h>
#elif defined(__AVX2__)
#include <immintrin.h>
#endif

namespace aditof {

Status convertNV12toBGR(const uint8_t *nv12_data, size_t nv12_size,
                        uint8_t *bgr_data, int width, int height) {
    // Input validation
    if (nv12_data == nullptr || bgr_data == nullptr) {
        LOG(ERROR) << "convertNV12toBGR: Null input or output pointer";
        return Status::INVALID_ARGUMENT;
    }

    if (width <= 0 || height <= 0 || (width & 1) || (height & 1)) {
        LOG(ERROR) << "convertNV12toBGR: Invalid dimensions " << width << "x"
                   << height << " (must be positive and even)";
        return Status::INVALID_ARGUMENT;
    }

    const size_t expected_nv12_size =
        static_cast<size_t>(width * height * 3 / 2);
    if (nv12_size < expected_nv12_size) {
        LOG(ERROR) << "convertNV12toBGR: NV12 buffer too small (" << nv12_size
                   << " < " << expected_nv12_size << ")";
        return Status::INVALID_ARGUMENT;
    }

    try {
        const int y_size = width * height;
        const uint8_t *y_plane = nv12_data;
        const uint8_t *uv_plane = nv12_data + y_size;

#if defined(__ARM_NEON) || defined(__aarch64__)
        // NEON optimized path for ARM/Jetson (processes 8 pixels at once)

        // BT.709 coefficients scaled by 256 for integer math
        const int16_t coeff_rv = 403;  // 1.5748 * 256
        const int16_t coeff_gu = -48;  // -0.18732 * 256
        const int16_t coeff_gv = -120; // -0.46812 * 256
        const int16_t coeff_bu = 475;  // 1.8556 * 256

        const int16x8_t v_128 = vdupq_n_s16(128);
        const int16x8_t v_zero = vdupq_n_s16(0);
        const int16x8_t v_255 = vdupq_n_s16(255);

        // Process two rows at a time (UV is shared across 2x2 blocks)
        for (int y = 0; y < height; y += 2) {
            const uint8_t *y_row0 = y_plane + y * width;
            const uint8_t *y_row1 = y_row0 + width;
            const uint8_t *uv_row = uv_plane + (y / 2) * width;

            uint8_t *bgr_row0 = bgr_data + y * width * 3;
            uint8_t *bgr_row1 = bgr_row0 + width * 3;

            for (int x = 0; x < width; x += 8) {
                // Load 8 Y values from each row
                uint8x8_t y0_u8 = vld1_u8(y_row0 + x);
                uint8x8_t y1_u8 = vld1_u8(y_row1 + x);

                // Load 4 UV pairs (8 bytes total for 8 horizontal pixels)
                uint8x8x2_t uv_pairs = vld2_u8(uv_row + x);
                uint8x8_t u_u8 = uv_pairs.val[0]; // U values
                uint8x8_t v_u8 = uv_pairs.val[1]; // V values

                // Duplicate UV horizontally (each UV covers 2 pixels)
                uint8x8_t u_dup = vzip1_u8(u_u8, u_u8);
                uint8x8_t v_dup = vzip1_u8(v_u8, v_u8);

                // Convert to signed 16-bit and subtract 128 from UV
                int16x8_t y0_s16 = vreinterpretq_s16_u16(vmovl_u8(y0_u8));
                int16x8_t y1_s16 = vreinterpretq_s16_u16(vmovl_u8(y1_u8));
                int16x8_t u_s16 =
                    vsubq_s16(vreinterpretq_s16_u16(vmovl_u8(u_dup)), v_128);
                int16x8_t v_s16 =
                    vsubq_s16(vreinterpretq_s16_u16(vmovl_u8(v_dup)), v_128);

                // Process row 0
                // R = Y + 1.5748 * (V - 128)
                int16x8_t r0 = vaddq_s16(
                    y0_s16, vshrq_n_s16(vmulq_n_s16(v_s16, coeff_rv), 8));
                // G = Y - 0.18732 * (U - 128) - 0.46812 * (V - 128)
                int16x8_t g0 = vaddq_s16(
                    y0_s16, vshrq_n_s16(vaddq_s16(vmulq_n_s16(u_s16, coeff_gu),
                                                  vmulq_n_s16(v_s16, coeff_gv)),
                                        8));
                // B = Y + 1.8556 * (U - 128)
                int16x8_t b0 = vaddq_s16(
                    y0_s16, vshrq_n_s16(vmulq_n_s16(u_s16, coeff_bu), 8));

                // Process row 1 (same UV values)
                int16x8_t r1 = vaddq_s16(
                    y1_s16, vshrq_n_s16(vmulq_n_s16(v_s16, coeff_rv), 8));
                int16x8_t g1 = vaddq_s16(
                    y1_s16, vshrq_n_s16(vaddq_s16(vmulq_n_s16(u_s16, coeff_gu),
                                                  vmulq_n_s16(v_s16, coeff_gv)),
                                        8));
                int16x8_t b1 = vaddq_s16(
                    y1_s16, vshrq_n_s16(vmulq_n_s16(u_s16, coeff_bu), 8));

                // Clamp to [0, 255]
                r0 = vmaxq_s16(v_zero, vminq_s16(v_255, r0));
                g0 = vmaxq_s16(v_zero, vminq_s16(v_255, g0));
                b0 = vmaxq_s16(v_zero, vminq_s16(v_255, b0));
                r1 = vmaxq_s16(v_zero, vminq_s16(v_255, r1));
                g1 = vmaxq_s16(v_zero, vminq_s16(v_255, g1));
                b1 = vmaxq_s16(v_zero, vminq_s16(v_255, b1));

                // Convert back to uint8
                uint8x8_t r0_u8 = vmovn_u16(vreinterpretq_u16_s16(r0));
                uint8x8_t g0_u8 = vmovn_u16(vreinterpretq_u16_s16(g0));
                uint8x8_t b0_u8 = vmovn_u16(vreinterpretq_u16_s16(b0));
                uint8x8_t r1_u8 = vmovn_u16(vreinterpretq_u16_s16(r1));
                uint8x8_t g1_u8 = vmovn_u16(vreinterpretq_u16_s16(g1));
                uint8x8_t b1_u8 = vmovn_u16(vreinterpretq_u16_s16(b1));

                // Interleave BGR and store (OpenGL expects BGR order)
                uint8x8x3_t bgr0;
                bgr0.val[0] = b0_u8;
                bgr0.val[1] = g0_u8;
                bgr0.val[2] = r0_u8;
                vst3_u8(bgr_row0 + x * 3, bgr0);

                uint8x8x3_t bgr1;
                bgr1.val[0] = b1_u8;
                bgr1.val[1] = g1_u8;
                bgr1.val[2] = r1_u8;
                vst3_u8(bgr_row1 + x * 3, bgr1);
            }
        }

#elif defined(__AVX2__)
        // AVX2 optimized path for x86 (processes 16 pixels at once)

        const __m256i coeff_rv = _mm256_set1_epi16(403);  // 1.5748 * 256
        const __m256i coeff_gu = _mm256_set1_epi16(-48);  // -0.18732 * 256
        const __m256i coeff_gv = _mm256_set1_epi16(-120); // -0.46812 * 256
        const __m256i coeff_bu = _mm256_set1_epi16(475);  // 1.8556 * 256

        const __m256i v_128 = _mm256_set1_epi16(128);
        const __m256i v_zero = _mm256_setzero_si256();
        const __m256i v_255 = _mm256_set1_epi16(255);

        for (int y = 0; y < height; y += 2) {
            const uint8_t *y_row0 = y_plane + y * width;
            const uint8_t *y_row1 = y_row0 + width;
            const uint8_t *uv_row = uv_plane + (y / 2) * width;

            uint8_t *bgr_row0 = bgr_data + y * width * 3;
            uint8_t *bgr_row1 = bgr_row0 + width * 3;

            for (int x = 0; x < width; x += 16) {
                // Load 16 Y values
                __m128i y0_128 = _mm_loadu_si128((__m128i *)(y_row0 + x));
                __m128i y1_128 = _mm_loadu_si128((__m128i *)(y_row1 + x));
                __m256i y0_256 = _mm256_cvtepu8_epi16(y0_128);
                __m256i y1_256 = _mm256_cvtepu8_epi16(y1_128);

                // Load and deinterleave UV
                __m128i uv_128 = _mm_loadu_si128((__m128i *)(uv_row + x));
                __m128i u_128 = _mm_shuffle_epi8(
                    uv_128, _mm_set_epi8(-1, -1, -1, -1, -1, -1, -1, -1, 14, 12,
                                         10, 8, 6, 4, 2, 0));
                __m128i v_128 = _mm_shuffle_epi8(
                    uv_128, _mm_set_epi8(-1, -1, -1, -1, -1, -1, -1, -1, 15, 13,
                                         11, 9, 7, 5, 3, 1));

                // Duplicate UV and convert to 16-bit
                __m128i u_dup = _mm_unpacklo_epi8(u_128, u_128);
                __m128i v_dup = _mm_unpacklo_epi8(v_128, v_128);
                __m256i u_256 =
                    _mm256_sub_epi16(_mm256_cvtepu8_epi16(u_dup), v_128);
                __m256i v_256 =
                    _mm256_sub_epi16(_mm256_cvtepu8_epi16(v_dup), v_128);

                // Color conversion for row 0
                __m256i r0 = _mm256_add_epi16(
                    y0_256,
                    _mm256_srai_epi16(_mm256_mullo_epi16(v_256, coeff_rv), 8));
                __m256i g0 = _mm256_add_epi16(
                    y0_256,
                    _mm256_srai_epi16(
                        _mm256_add_epi16(_mm256_mullo_epi16(u_256, coeff_gu),
                                         _mm256_mullo_epi16(v_256, coeff_gv)),
                        8));
                __m256i b0 = _mm256_add_epi16(
                    y0_256,
                    _mm256_srai_epi16(_mm256_mullo_epi16(u_256, coeff_bu), 8));

                // Color conversion for row 1
                __m256i r1 = _mm256_add_epi16(
                    y1_256,
                    _mm256_srai_epi16(_mm256_mullo_epi16(v_256, coeff_rv), 8));
                __m256i g1 = _mm256_add_epi16(
                    y1_256,
                    _mm256_srai_epi16(
                        _mm256_add_epi16(_mm256_mullo_epi16(u_256, coeff_gu),
                                         _mm256_mullo_epi16(v_256, coeff_gv)),
                        8));
                __m256i b1 = _mm256_add_epi16(
                    y1_256,
                    _mm256_srai_epi16(_mm256_mullo_epi16(u_256, coeff_bu), 8));

                // Clamp to [0, 255]
                r0 = _mm256_max_epi16(v_zero, _mm256_min_epi16(v_255, r0));
                g0 = _mm256_max_epi16(v_zero, _mm256_min_epi16(v_255, g0));
                b0 = _mm256_max_epi16(v_zero, _mm256_min_epi16(v_255, b0));
                r1 = _mm256_max_epi16(v_zero, _mm256_min_epi16(v_255, r1));
                g1 = _mm256_max_epi16(v_zero, _mm256_min_epi16(v_255, g1));
                b1 = _mm256_max_epi16(v_zero, _mm256_min_epi16(v_255, b1));

                // Pack to uint8
                __m128i r0_u8 =
                    _mm_packus_epi16(_mm256_castsi256_si128(r0),
                                     _mm256_extracti128_si256(r0, 1));
                __m128i g0_u8 =
                    _mm_packus_epi16(_mm256_castsi256_si128(g0),
                                     _mm256_extracti128_si256(g0, 1));
                __m128i b0_u8 =
                    _mm_packus_epi16(_mm256_castsi256_si128(b0),
                                     _mm256_extracti128_si256(b0, 1));
                __m128i r1_u8 =
                    _mm_packus_epi16(_mm256_castsi256_si128(r1),
                                     _mm256_extracti128_si256(r1, 1));
                __m128i g1_u8 =
                    _mm_packus_epi16(_mm256_castsi256_si128(g1),
                                     _mm256_extracti128_si256(g1, 1));
                __m128i b1_u8 =
                    _mm_packus_epi16(_mm256_castsi256_si128(b1),
                                     _mm256_extracti128_si256(b1, 1));

                // Interleave BGR - scalar store for correctness
                for (int i = 0; i < 16; i++) {
                    bgr_row0[(x + i) * 3 + 0] = ((uint8_t *)&b0_u8)[i];
                    bgr_row0[(x + i) * 3 + 1] = ((uint8_t *)&g0_u8)[i];
                    bgr_row0[(x + i) * 3 + 2] = ((uint8_t *)&r0_u8)[i];
                    bgr_row1[(x + i) * 3 + 0] = ((uint8_t *)&b1_u8)[i];
                    bgr_row1[(x + i) * 3 + 1] = ((uint8_t *)&g1_u8)[i];
                    bgr_row1[(x + i) * 3 + 2] = ((uint8_t *)&r1_u8)[i];
                }
            }
        }

#else
        // Optimized scalar fallback - process 2 rows at once
        const int coeff_rv = 403;  // 1.5748 * 256
        const int coeff_gu = -48;  // -0.18732 * 256
        const int coeff_gv = -120; // -0.46812 * 256
        const int coeff_bu = 475;  // 1.8556 * 256

        for (int y = 0; y < height; y += 2) {
            const uint8_t *y_row0 = y_plane + y * width;
            const uint8_t *y_row1 = y_row0 + width;
            const uint8_t *uv_row = uv_plane + (y / 2) * width;

            uint8_t *bgr_row0 = bgr_data + y * width * 3;
            uint8_t *bgr_row1 = bgr_row0 + width * 3;

            for (int x = 0; x < width; x += 2) {
                // Load one UV pair for 2x2 block
                int u_val = static_cast<int>(uv_row[x]) - 128;
                int v_val = static_cast<int>(uv_row[x + 1]) - 128;

                // Precompute chroma contributions
                int cr = (coeff_rv * v_val) >> 8;
                int cg = ((coeff_gu * u_val) + (coeff_gv * v_val)) >> 8;
                int cb = (coeff_bu * u_val) >> 8;

                // Process 2x2 block
                for (int dy = 0; dy < 2; dy++) {
                    const uint8_t *y_row = (dy == 0) ? y_row0 : y_row1;
                    uint8_t *bgr_row = (dy == 0) ? bgr_row0 : bgr_row1;

                    for (int dx = 0; dx < 2; dx++) {
                        int y_val = y_row[x + dx];

                        // Apply color transform
                        int r = y_val + cr;
                        int g = y_val + cg;
                        int b = y_val + cb;

                        // Clamp and store as BGR
                        int idx = (x + dx) * 3;
                        bgr_row[idx + 0] =
                            static_cast<uint8_t>(std::max(0, std::min(255, b)));
                        bgr_row[idx + 1] =
                            static_cast<uint8_t>(std::max(0, std::min(255, g)));
                        bgr_row[idx + 2] =
                            static_cast<uint8_t>(std::max(0, std::min(255, r)));
                    }
                }
            }
        }
#endif

        return Status::OK;

    } catch (const std::exception &e) {
        LOG(ERROR) << "convertNV12toBGR: Exception during conversion: "
                   << e.what();
        return Status::GENERIC_ERROR;
    }
}

Status convertNV12toBGR(const std::vector<uint8_t> &nv12_data,
                        std::vector<uint8_t> &bgr_data, int width, int height) {
    // Validate dimensions
    if (width <= 0 || height <= 0 || (width & 1) || (height & 1)) {
        LOG(ERROR) << "convertNV12toBGR: Invalid dimensions";
        return Status::INVALID_ARGUMENT;
    }

    // Allocate output buffer (3 bytes per pixel for BGR)
    size_t bgr_size = static_cast<size_t>(width * height * 3);
    bgr_data.resize(bgr_size);

    // Call the main conversion function
    return convertNV12toBGR(nv12_data.data(), nv12_data.size(), bgr_data.data(),
                            width, height);
}

} // namespace aditof
