/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, Analog Devices, Inc.
 * All rights reserved.
 */

#ifndef NV12_TO_RGB_H
#define NV12_TO_RGB_H

#include <aditof/status_definitions.h>
#include <cstdint>
#include <vector>

namespace aditof {

/**
 * @brief Convert NV12 (YUV420 semi-planar) format to BGR format (OpenGL compatible)
 * 
 * NV12 Format (YUV420 Semi-Planar):
 * - Y plane: Contains luminance values for all pixels (width × height bytes)
 * - UV plane: Contains interleaved chrominance pairs ((width/2) × (height/2) * 2 bytes)
 *   Each U/V pair covers a 2x2 block of Y pixels
 * 
 * Color Conversion (ITU-R BT.709 - HDTV Standard):
 * R = Y + 1.5748 * (V - 128)
 * G = Y - 0.18732 * (U - 128) - 0.46812 * (V - 128)
 * B = Y + 1.8556 * (U - 128)
 * 
 * Output format: BGR (3 bytes per pixel) for OpenGL/OpenCV compatibility
 * 
 * SIMD Optimizations:
 * - ARM NEON (Jetson): Processes 8 pixels per iteration
 * - x86 AVX2: Processes 16 pixels per iteration  
 * - Scalar fallback: Optimized 2x2 block processing
 * 
 * @param[in] nv12_data Input NV12 data buffer (Y plane + UV plane)
 * @param[in] nv12_size Size of NV12 buffer in bytes (width * height * 1.5)
 * @param[out] bgr_data Output BGR data buffer (must be pre-allocated: width * height * 3 bytes)
 * @param[in] width Frame width in pixels (must be even)
 * @param[in] height Frame height in pixels (must be even)
 * @return Status::OK on success, Status::INVALID_ARGUMENT on invalid input,
 *         Status::GENERIC_ERROR on conversion failure
 */
Status convertNV12toBGR(const uint8_t *nv12_data, size_t nv12_size,
                        uint8_t *bgr_data, int width, int height);

/**
 * @brief Convert NV12 format to BGR and store in std::vector (convenience wrapper)
 * 
 * @param[in] nv12_data Input NV12 data vector
 * @param[out] bgr_data Output BGR data vector (will be resized automatically)
 * @param[in] width Frame width in pixels (must be even)
 * @param[in] height Frame height in pixels (must be even)
 * @return Status::OK on success, error status otherwise
 */
Status convertNV12toBGR(const std::vector<uint8_t> &nv12_data,
                        std::vector<uint8_t> &bgr_data, int width, int height);

} // namespace aditof

#endif // NV12_TO_RGB_H
