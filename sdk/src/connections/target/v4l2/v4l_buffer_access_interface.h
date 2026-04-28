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
#ifndef V4L_BUFFER_ACCESS_INTERFACE_H
#define V4L_BUFFER_ACCESS_INTERFACE_H

#include <aditof/status_definitions.h>
#include <cstddef>
#include <cstdint>
#include <linux/videodev2.h>

namespace aditof {

/**
 * @class V4lBufferAccessInterface
 * @brief Interface for operations on v4l buffer such as enquing and dequeing
 */
class V4lBufferAccessInterface {
  public:
    virtual ~V4lBufferAccessInterface() = default;

    virtual Status enqueueInternalBuffer(struct v4l2_buffer &buf) = 0;

    virtual Status dequeueInternalBuffer(struct v4l2_buffer &buf) = 0;

    virtual Status getInternalBuffer(uint8_t **buffer, uint32_t &buf_data_len,
                                     const struct v4l2_buffer &buf) = 0;

    virtual Status getDeviceFileDescriptor(int &fileDescriptor) = 0;

    virtual Status waitForBuffer() = 0;
};

} // namespace aditof

#endif // V4L_BUFFER_ACCESS_INTERFACE_H
