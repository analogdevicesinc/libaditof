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
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef DEPTH_COMPUTE_LIBRARY_H
#define DEPTH_COMPUTE_LIBRARY_H

#include <aditof/camera_definitions.h>
#include <aditof/status_definitions.h>

namespace aditof {

/**
 * @brief Runtime choice of the depth compute library (libtofi_compute.so +
 * libtofi_config.so) per depth algorithm.
 *
 * TEMPORARY (two depth compute libraries): remove this loader, and link the
 * library again, once one library has every algorithm.
 *
 * For now lens scatter compensation has its own build of the library, found
 * in a "lens_scatter" folder next to the standard one (or
 * $ADITOF_TOFI_LENS_SCATTER_LIBDIR); the standard one is next to libaditof
 * (or $ADITOF_TOFI_LIBDIR). Both are loaded side by side; each compute
 * context keeps using the library that created it. Once one library has
 * every algorithm, an empty lens scatter folder makes both use it.
 */
namespace depth_compute_library {

/**
 * @brief Library used by the next InitTofiConfig*()/InitTofiCompute() calls.
 * @return Status::UNAVAILABLE if no library supporting algorithm loads
 */
Status select(DepthAlgorithm algorithm);

/**
 * @brief Whether a library supporting algorithm loads.
 */
bool supports(DepthAlgorithm algorithm);

/**
 * @brief Address of a symbol of the selected library, or nullptr.
 */
void *symbol(const char *name);

} // namespace depth_compute_library
} // namespace aditof

#endif // DEPTH_COMPUTE_LIBRARY_H
