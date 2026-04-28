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
#ifndef V4L2_BUFFER_MANAGER_H
#define V4L2_BUFFER_MANAGER_H

#include <aditof/status_definitions.h>
#include <cstdint>
#include <linux/videodev2.h>

// Forward declarations
struct VideoDev;

/**
 * @class V4L2BufferManager
 * @brief Manages V4L2 video buffer queue operations.
 *
 * Responsible for:
 * - Buffer dequeue/enqueue operations
 * - Buffer availability polling (select)
 * - Buffer memory access
 * - File descriptor management
 */
class V4L2BufferManager {
  public:
    /**
     * @brief Constructs V4L2 buffer manager with access to video devices.
     *
     * @param videoDevs Pointer to V4L2 video device array
     * @param numVideoDevs Number of video devices in array
     */
    V4L2BufferManager(struct VideoDev *videoDevs, uint8_t numVideoDevs);

    ~V4L2BufferManager() = default;

    /**
     * @brief Waits for a buffer to become available.
     *
     * Uses select() to block until a buffer is ready to dequeue.
     *
     * @param dev Pointer to VideoDev (uses first device if nullptr)
     * @return Status::OK if buffer ready, Status::GENERIC_ERROR on timeout/error
     */
    aditof::Status waitForBuffer(struct VideoDev *dev = nullptr);

    /**
     * @brief Dequeues a filled buffer from the driver.
     *
     * Executes VIDIOC_DQBUF ioctl to retrieve a buffer filled by hardware.
     *
     * @param buf v4l2_buffer structure to receive dequeued buffer info
     * @param dev Pointer to VideoDev (uses first device if nullptr)
     * @return Status::OK on success, Status::GENERIC_ERROR on ioctl failure
     */
    aditof::Status dequeueInternalBuffer(struct v4l2_buffer &buf,
                                         struct VideoDev *dev = nullptr);

    /**
     * @brief Retrieves pointer and size of a dequeued buffer.
     *
     * Returns mmap'd memory region for the given buffer index.
     *
     * @param buffer Pointer to receive buffer memory address
     * @param buf_data_len Variable to receive buffer data length in bytes
     * @param buf v4l2_buffer structure from dequeue operation
     * @param dev Pointer to VideoDev (uses first device if nullptr)
     * @return Status::OK on success
     */
    aditof::Status getInternalBuffer(uint8_t **buffer, uint32_t &buf_data_len,
                                     const struct v4l2_buffer &buf,
                                     struct VideoDev *dev = nullptr);

    /**
     * @brief Re-queues a processed buffer to the driver.
     *
     * Executes VIDIOC_QBUF ioctl to return buffer to driver for refilling.
     *
     * @param buf v4l2_buffer structure to re-queue
     * @param dev Pointer to VideoDev (uses first device if nullptr)
     * @return Status::OK on success, Status::GENERIC_ERROR on ioctl failure
     */
    aditof::Status enqueueInternalBuffer(struct v4l2_buffer &buf,
                                         struct VideoDev *dev = nullptr);

    /**
     * @brief Retrieves the video device file descriptor.
     *
     * @param fileDescriptor Variable to receive the file descriptor
     * @return Status::OK on success, Status::INVALID_ARGUMENT if device not initialized
     */
    aditof::Status getDeviceFileDescriptor(int &fileDescriptor);

  private:
    struct VideoDev *m_videoDevs; ///< Pointer to V4L2 video device array
    uint8_t m_numVideoDevs;       ///< Number of video devices
};

#endif // V4L2_BUFFER_MANAGER_H
