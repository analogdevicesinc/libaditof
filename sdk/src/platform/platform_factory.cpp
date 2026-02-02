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
#include "platform_factory.h"

// Platform-specific implementations will be included based on compile flags
#ifdef NVIDIA
#include "nvidia/nvidia_platform.h"
#elif defined(NXP)
#include "imx/imx_platform.h"
#elif defined(RPI)
#include "raspberrypi/rpi_platform.h"
#else
#error "No platform defined! Use -DNVIDIA=ON, -DNXP=ON, or -DRPI=ON"
#endif

namespace aditof {
namespace platform {

std::unique_ptr<IPlatform> PlatformFactory::create() {
#ifdef NVIDIA
    return std::make_unique<NvidiaPlatform>();
#elif defined(NXP)
    return std::make_unique<ImxPlatform>();
#elif defined(RPI)
    return std::make_unique<RpiPlatform>();
#else
#error "No platform defined! Use -DNVIDIA=ON, -DNXP=ON, or -DRPI=ON"
#endif
}

} // namespace platform
} // namespace aditof
