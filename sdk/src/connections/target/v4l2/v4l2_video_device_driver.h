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
#ifndef V4L2_VIDEO_DEVICE_DRIVER_H
#define V4L2_VIDEO_DEVICE_DRIVER_H

#include "video_device_driver.h"
#include <linux/videodev2.h>

namespace aditof {

/**
 * @brief V4L2 (Video4Linux2) implementation of VideoDeviceDriver.
 *
 * Provides Linux-specific video capture using the V4L2 API.
 * Wraps V4L2 ioctl calls with error handling and status conversion.
 */
class V4L2VideoDeviceDriver : public VideoDeviceDriver {
  public:
    V4L2VideoDeviceDriver();
    ~V4L2VideoDeviceDriver() override;

    // VideoDeviceDriver interface implementation
    Status open(const std::string &devicePath, int flags) override;
    Status adoptFileDescriptor(int fd, const std::string &devicePath) override;
    Status close() override;
    bool isOpen() const override;

    Status setControl(unsigned int controlId, int value) override;
    Status getControl(unsigned int controlId, int &value) const override;

    Status setFormat(const VideoFormat &format) override;
    Status getFormat(VideoFormat &format) const override;

    Status requestBuffers(unsigned int count, unsigned int memoryType) override;
    Status queryBuffer(unsigned int index, VideoBufferInfo &info) override;
    Status queueBuffer(unsigned int index) override;
    Status dequeueBuffer(VideoBufferInfo &info) override;

    Status streamOn() override;
    Status streamOff() override;

    Status waitForBuffer(int timeoutMs) override;
    int getFileDescriptor() const override;
    Status ioctl(unsigned long request, void *arg) override;

    Status validateDeviceCapabilities(const std::string &devicePath,
                                      const std::string &expectedCardName,
                                      unsigned int &bufferType) override;

  private:
    /**
     * @brief Internal ioctl wrapper with retry on EINTR.
     *
     * @param request ioctl request code
     * @param arg Pointer to argument structure
     * @return 0 on success, -1 on error (errno set)
     */
    int xioctl(unsigned long request, void *arg) const;

    /**
     * @brief Converts VideoFormat to v4l2_format.
     *
     * @param format Input format
     * @param v4l2fmt Output V4L2 format structure
     */
    void toV4L2Format(const VideoFormat &format,
                      struct v4l2_format &v4l2fmt) const;

    /**
     * @brief Converts v4l2_format to VideoFormat.
     *
     * @param v4l2fmt Input V4L2 format structure
     * @param format Output format
     */
    void fromV4L2Format(const struct v4l2_format &v4l2fmt,
                        VideoFormat &format) const;

    int m_fd;                   /**< File descriptor for video device */
    v4l2_buf_type m_bufferType; /**< V4L2 buffer type (capture/mplane) */
    std::string m_devicePath;   /**< Path to video device */
};

} // namespace aditof

#endif // V4L2_VIDEO_DEVICE_DRIVER_H
