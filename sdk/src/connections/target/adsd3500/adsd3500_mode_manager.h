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
#ifndef ADSD3500_MODE_MANAGER_H
#define ADSD3500_MODE_MANAGER_H

#include <aditof/depth_sensor_interface.h>
#include <aditof/status_definitions.h>
#include <cstdint>
#include <memory>
#include <vector>

namespace aditof {

class VideoDeviceDriver;
class BufferProcessorInterface;
class SensorControlRegistry;
class Adsd3500CommandInterface;

/**
 * @brief Manages ADSD3500 frame mode configuration and switching.
 *
 * Handles mode-specific initialization, buffer allocation, V4L2 format setup,
 * and mode switching logic. Encapsulates mode management complexity away from
 * main sensor class (SRP compliance).
 */
class Adsd3500ModeManager {
  public:
    Adsd3500ModeManager();
    ~Adsd3500ModeManager();

    /**
     * @brief Set dependencies required for mode operations.
     * @param[in] videoDriver Video device driver for format configuration
     * @param[in] bufferProcessor Buffer processor for pipeline setup
     * @param[in] controlRegistry Control registry for reading configuration
     * @param[in] commandInterface Command interface for chip communication
     */
    void setDependencies(VideoDeviceDriver *videoDriver,
                         BufferProcessorInterface *bufferProcessor,
                         SensorControlRegistry *controlRegistry,
                         Adsd3500CommandInterface *commandInterface);

    /**
     * @brief Configure sensor to specified frame mode.
     * @param[in] mode Mode number to activate (0-6 for MP/QMP modes)
     * @param[in] availableModes List of available mode details
     * @return Status::OK on success, error code otherwise
     */
    Status setMode(uint8_t mode,
                   const std::vector<DepthSensorModeDetails> &availableModes);

    /**
     * @brief Get current mode details.
     * @param[out] details Structure to receive current mode information
     * @return Status::OK on success, error code otherwise
     */
    Status getCurrentModeDetails(DepthSensorModeDetails &details) const;

    /**
     * @brief Get current mode number.
     * @return Current mode number, or 0xFF if no mode set
     */
    uint8_t getCurrentMode() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace aditof

#endif // ADSD3500_MODE_MANAGER_H
