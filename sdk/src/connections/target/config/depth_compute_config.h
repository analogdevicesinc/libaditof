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
#ifndef DEPTH_COMPUTE_CONFIG_H
#define DEPTH_COMPUTE_CONFIG_H

#include <cstdint>

namespace aditof {

/**
 * @brief Configuration object for depth computation settings.
 *
 * Replaces global variable `depthComputeOpenSourceEnabled` to improve:
 * - Testability (can inject different configurations)
 * - Thread safety (encapsulated state)
 * - Maintainability (clear ownership and lifetime)
 *
 * This class encapsulates depth computation configuration that determines
 * whether open-source or proprietary depth compute libraries are used.
 */
class DepthComputeConfig {
  public:
    DepthComputeConfig() : m_openSourceEnabled(false) {}
    ~DepthComputeConfig() = default;

    /**
     * @brief Sets whether open-source depth compute is enabled.
     *
     * @param enabled True to use open-source implementation, false for proprietary
     */
    void setOpenSourceEnabled(bool enabled) { m_openSourceEnabled = enabled; }

    /**
     * @brief Gets whether open-source depth compute is enabled.
     *
     * @return True if open-source implementation is enabled
     */
    bool isOpenSourceEnabled() const { return m_openSourceEnabled; }

    /**
     * @brief Gets the status as a uint8_t (legacy compatibility).
     *
     * @return 1 if open-source enabled, 0 otherwise
     */
    uint8_t getStatus() const { return m_openSourceEnabled ? 1 : 0; }

    /**
     * @brief Sets the status from a uint8_t (legacy compatibility).
     *
     * @param status 1 to enable open-source, 0 to disable
     */
    void setStatus(uint8_t status) { m_openSourceEnabled = (status != 0); }

  private:
    bool m_openSourceEnabled;
};

} // namespace aditof

#endif // DEPTH_COMPUTE_CONFIG_H
