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
#ifndef VIDEO_DEVICE_DRIVER_H
#define VIDEO_DEVICE_DRIVER_H

#include <aditof/status_definitions.h>
#include <cstdint>
#include <string>

namespace aditof {

/**
 * @brief Buffer metadata for video capture operations.
 *
 * Abstracts platform-specific buffer details (e.g., V4L2 buffer).
 */
struct VideoBufferInfo {
    unsigned int index;     /**< Buffer index in device queue */
    unsigned int bytesUsed; /**< Actual data size in buffer */
    uint64_t timestamp;     /**< Capture timestamp (microseconds) */
    unsigned int sequence;  /**< Frame sequence number */
    void *mappedAddress;    /**< Pointer to mapped buffer memory */
    unsigned int length;    /**< Total buffer size */
    unsigned int offset;    /**< Buffer offset for mmap */
};

/**
 * @brief Video format configuration.
 *
 * Describes pixel format and frame dimensions for video capture.
 */
struct VideoFormat {
    unsigned int pixelFormat;  /**< Pixel format (e.g., V4L2_PIX_FMT_SRGGB12) */
    unsigned int width;        /**< Frame width in pixels */
    unsigned int height;       /**< Frame height in pixels */
    unsigned int bytesPerLine; /**< Stride in bytes */
    unsigned int sizeImage;    /**< Total image size in bytes */
};

/**
 * @brief Abstract interface for video device operations.
 *
 * Abstracts platform-specific video capture APIs (V4L2, DirectShow, etc.)
 * to enable:
 * - Hardware-independent testing (mock implementations)
 * - Platform portability (V4L2 on Linux, alternative drivers elsewhere)
 * - Cleaner separation of concerns
 *
 * Following Dependency Inversion Principle: sensor code depends on this
 * interface, not on Linux V4L2 directly.
 */
class VideoDeviceDriver {
  public:
    virtual ~VideoDeviceDriver() = default;

    /**
     * @brief Opens the video device.
     *
     * @param devicePath Path to device (e.g., "/dev/video0")
     * @param flags Open flags (e.g., O_RDWR | O_NONBLOCK)
     * @return Status::OK on success
     */
    virtual Status open(const std::string &devicePath, int flags) = 0;

    /**
     * @brief Adopts an already-opened file descriptor.
     *
     * Allows the driver to work with a device that was opened externally.
     * Useful when device opening is managed by legacy code.
     *
     * @param fd Opened file descriptor
     * @param devicePath Device path for logging/identification
     * @return Status::OK on success
     */
    virtual Status adoptFileDescriptor(int fd,
                                       const std::string &devicePath) = 0;

    /**
     * @brief Closes the video device.
     *
     * @return Status::OK on success
     */
    virtual Status close() = 0;

    /**
     * @brief Checks if device is open.
     *
     * @return true if device is open
     */
    virtual bool isOpen() const = 0;

    /**
     * @brief Sets a device control parameter.
     *
     * @param controlId Control identifier
     * @param value Control value
     * @return Status::OK on success
     */
    virtual Status setControl(unsigned int controlId, int value) = 0;

    /**
     * @brief Gets a device control parameter.
     *
     * @param controlId Control identifier
     * @param value Output parameter for control value
     * @return Status::OK on success
     */
    virtual Status getControl(unsigned int controlId, int &value) const = 0;

    /**
     * @brief Sets the video format.
     *
     * @param format Desired video format
     * @return Status::OK on success
     */
    virtual Status setFormat(const VideoFormat &format) = 0;

    /**
     * @brief Gets the current video format.
     *
     * @param format Output parameter for current format
     * @return Status::OK on success
     */
    virtual Status getFormat(VideoFormat &format) const = 0;

    /**
     * @brief Requests video buffers from the driver.
     *
     * @param count Number of buffers to allocate
     * @param memoryType Memory type (e.g., MMAP, USERPTR)
     * @return Status::OK on success
     */
    virtual Status requestBuffers(unsigned int count,
                                  unsigned int memoryType) = 0;

    /**
     * @brief Queries buffer information.
     *
     * @param index Buffer index
     * @param info Output parameter for buffer information
     * @return Status::OK on success
     */
    virtual Status queryBuffer(unsigned int index, VideoBufferInfo &info) = 0;

    /**
     * @brief Enqueues a buffer for capture.
     *
     * @param index Buffer index
     * @return Status::OK on success
     */
    virtual Status queueBuffer(unsigned int index) = 0;

    /**
     * @brief Dequeues a captured buffer.
     *
     * @param info Output parameter for dequeued buffer information
     * @return Status::OK on success, Status::BUSY if no buffer ready
     */
    virtual Status dequeueBuffer(VideoBufferInfo &info) = 0;

    /**
     * @brief Starts video streaming.
     *
     * @return Status::OK on success
     */
    virtual Status streamOn() = 0;

    /**
     * @brief Stops video streaming.
     *
     * @return Status::OK on success
     */
    virtual Status streamOff() = 0;

    /**
     * @brief Waits for buffer to become available.
     *
     * @param timeoutMs Timeout in milliseconds (0 = no wait, -1 = infinite)
     * @return Status::OK if buffer ready, Status::BUSY if timeout
     */
    virtual Status waitForBuffer(int timeoutMs) = 0;

    /**
     * @brief Gets the file descriptor for polling/select.
     *
     * @return File descriptor, or -1 if not applicable
     */
    virtual int getFileDescriptor() const = 0;

    /**
     * @brief Performs a custom ioctl command.
     *
     * Provides low-level access for platform-specific operations.
     *
     * @param request ioctl request code
     * @param arg Pointer to argument structure
     * @return Status::OK on success
     */
    virtual Status ioctl(unsigned long request, void *arg) = 0;
};

} // namespace aditof

#endif // VIDEO_DEVICE_DRIVER_H
