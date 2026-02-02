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
#ifndef PLATFORM_FACTORY_H
#define PLATFORM_FACTORY_H

#include "platform_interface.h"
#include <memory>

namespace aditof {
namespace platform {

/**
 * @brief Factory for creating platform instances
 * 
 * The factory selects the appropriate platform implementation at compile time
 * based on CMake flags (NVIDIA, NXP, etc.). This ensures zero runtime overhead
 * for platform selection.
 */
class PlatformFactory {
public:
    /**
     * @brief Create platform instance for current target
     * @return Unique pointer to platform implementation
     */
    static std::unique_ptr<IPlatform> create();
    
private:
    PlatformFactory() = delete;
    ~PlatformFactory() = delete;
};

} // namespace platform
} // namespace aditof

#endif // PLATFORM_FACTORY_H
