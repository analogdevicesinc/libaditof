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
#ifndef ADSD3500_DEVICE_H
#define ADSD3500_DEVICE_H

#include <aditof/status_definitions.h>
#include <memory>
#include <string>

namespace aditof {

class VideoDeviceDriver;
class Adsd3500CommandInterface;
class BufferProcessorInterface;

/**
 * @brief Manages ADSD3500 device lifecycle operations.
 *
 * Handles device initialization, V4L2 device open/close, capability queries,
 * and resource allocation/deallocation. Separates device management concerns
 * from configuration and operational logic (SRP compliance).
 */
class Adsd3500Device {
  public:
    Adsd3500Device(const std::string &videoDevicePath,
                   const std::string &subdevicePath);
    ~Adsd3500Device();

    /**
     * @brief Open and initialize V4L2 video and subdevice.
     * @param[in] commandInterface Command interface for chip queries
     * @param[in] bufferProcessor Buffer processor to initialize with device
     * @return Status::OK on success, error code otherwise
     */
    Status open(Adsd3500CommandInterface *commandInterface,
                BufferProcessorInterface *bufferProcessor);

    /**
     * @brief Close and release V4L2 device resources.
     * @return Status::OK on success, error code otherwise
     */
    Status close();

    /**
     * @brief Check if device is currently open.
     * @return true if device is open, false otherwise
     */
    bool isOpen() const;

    /**
     * @brief Get video device driver for frame capture operations.
     * @return Pointer to video device driver, nullptr if not open
     */
    VideoDeviceDriver *getVideoDriver() const;

    /**
     * @brief Get subdevice driver for control operations.
     * @return Pointer to subdevice driver, nullptr if not open
     */
    VideoDeviceDriver *getSubdeviceDriver() const;

    /**
     * @brief Get chip ID read during initialization.
     * @return 16-bit chip ID value
     */
    uint16_t getChipId() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace aditof

#endif // ADSD3500_DEVICE_H
