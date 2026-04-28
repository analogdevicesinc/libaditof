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
#include "v4l2_video_device_driver.h"
#include <aditof/log.h>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>

namespace aditof {

V4L2VideoDeviceDriver::V4L2VideoDeviceDriver()
    : m_fd(-1), m_bufferType(V4L2_BUF_TYPE_VIDEO_CAPTURE) {}

V4L2VideoDeviceDriver::~V4L2VideoDeviceDriver() {
    if (isOpen()) {
        close();
    }
}

Status V4L2VideoDeviceDriver::open(const std::string &devicePath, int flags) {
    if (isOpen()) {
        LOG(WARNING) << "Device already open: " << m_devicePath;
        return Status::OK;
    }

    m_fd = ::open(devicePath.c_str(), flags);
    if (m_fd == -1) {
        LOG(ERROR) << "Failed to open device " << devicePath << ": "
                   << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    m_devicePath = devicePath;
    LOG(INFO) << "Opened video device: " << devicePath << " (fd=" << m_fd
              << ")";
    return Status::OK;
}

Status
V4L2VideoDeviceDriver::adoptFileDescriptor(int fd,
                                           const std::string &devicePath) {
    if (fd < 0) {
        LOG(ERROR) << "Invalid file descriptor: " << fd;
        return Status::INVALID_ARGUMENT;
    }

    if (isOpen()) {
        LOG(WARNING) << "Driver already has an open device, closing first";
        close();
    }

    m_fd = fd;
    m_devicePath = devicePath;
    LOG(INFO) << "Adopted file descriptor " << fd
              << " for device: " << devicePath;
    return Status::OK;
}

Status V4L2VideoDeviceDriver::close() {
    if (!isOpen()) {
        return Status::OK;
    }

    if (::close(m_fd) == -1 && errno != EBADF) {
        // Suppress EBADF - file descriptor may have been closed elsewhere
        LOG(WARNING) << "Error closing device " << m_devicePath << ": "
                     << strerror(errno);
        // Continue with cleanup even if close fails
    }

    m_fd = -1;
    m_devicePath.clear();
    return Status::OK;
}

bool V4L2VideoDeviceDriver::isOpen() const { return m_fd >= 0; }

Status V4L2VideoDeviceDriver::setControl(unsigned int controlId, int value) {
    if (!isOpen()) {
        LOG(ERROR) << "Device not open";
        return Status::GENERIC_ERROR;
    }

    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = controlId;
    ctrl.value = value;

    if (xioctl(VIDIOC_S_CTRL, &ctrl) == -1) {
        LOG(ERROR) << "Failed to set control 0x" << std::hex << controlId
                   << " to " << std::dec << value << ": " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    return Status::OK;
}

Status V4L2VideoDeviceDriver::getControl(unsigned int controlId,
                                         int &value) const {
    if (!isOpen()) {
        LOG(ERROR) << "Device not open";
        return Status::GENERIC_ERROR;
    }

    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = controlId;

    if (xioctl(VIDIOC_G_CTRL, &ctrl) == -1) {
        LOG(ERROR) << "Failed to get control 0x" << std::hex << controlId
                   << ": " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    value = ctrl.value;
    return Status::OK;
}

Status V4L2VideoDeviceDriver::setFormat(const VideoFormat &format) {
    if (!isOpen()) {
        LOG(ERROR) << "Device not open";
        return Status::GENERIC_ERROR;
    }

    struct v4l2_format v4l2fmt;
    memset(&v4l2fmt, 0, sizeof(v4l2fmt));
    toV4L2Format(format, v4l2fmt);

    if (xioctl(VIDIOC_S_FMT, &v4l2fmt) == -1) {
        LOG(ERROR) << "Failed to set format: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    return Status::OK;
}

Status V4L2VideoDeviceDriver::getFormat(VideoFormat &format) const {
    if (!isOpen()) {
        LOG(ERROR) << "Device not open";
        return Status::GENERIC_ERROR;
    }

    struct v4l2_format v4l2fmt;
    memset(&v4l2fmt, 0, sizeof(v4l2fmt));
    v4l2fmt.type = m_bufferType;

    if (xioctl(VIDIOC_G_FMT, &v4l2fmt) == -1) {
        LOG(ERROR) << "Failed to get format: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    fromV4L2Format(v4l2fmt, format);
    return Status::OK;
}

Status V4L2VideoDeviceDriver::requestBuffers(unsigned int count,
                                             unsigned int memoryType) {
    if (!isOpen()) {
        LOG(ERROR) << "Device not open";
        return Status::GENERIC_ERROR;
    }

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = count;
    req.type = m_bufferType;
    req.memory = memoryType;

    if (xioctl(VIDIOC_REQBUFS, &req) == -1) {
        LOG(ERROR) << "Failed to request buffers: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    LOG(INFO) << "Requested " << count << " buffers, driver allocated "
              << req.count;
    return Status::OK;
}

Status V4L2VideoDeviceDriver::queryBuffer(unsigned int index,
                                          VideoBufferInfo &info) {
    if (!isOpen()) {
        LOG(ERROR) << "Device not open";
        return Status::GENERIC_ERROR;
    }

    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = m_bufferType;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;

    // For multiplanar, need to set up planes structure
    struct v4l2_plane planes[8];
    if (m_bufferType == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        memset(planes, 0, sizeof(planes));
        buf.m.planes = planes;
        buf.length = 1;
    }

    if (xioctl(VIDIOC_QUERYBUF, &buf) == -1) {
        LOG(ERROR) << "Failed to query buffer " << index << ": "
                   << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    info.index = buf.index;
    info.bytesUsed = buf.bytesused;
    info.timestamp = static_cast<uint64_t>(buf.timestamp.tv_sec) * 1000000ULL +
                     static_cast<uint64_t>(buf.timestamp.tv_usec);
    info.sequence = buf.sequence;
    info.mappedAddress = nullptr; // Filled in by caller after mmap

    // Extract length and offset based on buffer type
    if (m_bufferType == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
        info.length = buf.length;
        info.offset = buf.m.offset;
    } else {
        info.length = buf.m.planes[0].length;
        info.offset = buf.m.planes[0].m.mem_offset;
    }

    return Status::OK;
}

Status V4L2VideoDeviceDriver::queueBuffer(unsigned int index) {
    if (!isOpen()) {
        LOG(ERROR) << "Device not open";
        return Status::GENERIC_ERROR;
    }

    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = m_bufferType;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;

    if (xioctl(VIDIOC_QBUF, &buf) == -1) {
        LOG(ERROR) << "Failed to queue buffer " << index << ": "
                   << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    return Status::OK;
}

Status V4L2VideoDeviceDriver::dequeueBuffer(VideoBufferInfo &info) {
    if (!isOpen()) {
        LOG(ERROR) << "Device not open";
        return Status::GENERIC_ERROR;
    }

    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = m_bufferType;
    buf.memory = V4L2_MEMORY_MMAP;

    if (xioctl(VIDIOC_DQBUF, &buf) == -1) {
        if (errno == EAGAIN) {
            return Status::BUSY; // No buffer ready
        }
        LOG(ERROR) << "Failed to dequeue buffer: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    info.index = buf.index;
    info.bytesUsed = buf.bytesused;
    info.timestamp = static_cast<uint64_t>(buf.timestamp.tv_sec) * 1000000ULL +
                     static_cast<uint64_t>(buf.timestamp.tv_usec);
    info.sequence = buf.sequence;
    info.length = buf.length;

    return Status::OK;
}

Status V4L2VideoDeviceDriver::streamOn() {
    if (!isOpen()) {
        LOG(ERROR) << "Device not open";
        return Status::GENERIC_ERROR;
    }

    if (xioctl(VIDIOC_STREAMON, &m_bufferType) == -1) {
        LOG(ERROR) << "Failed to start streaming: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    LOG(INFO) << "Started streaming on " << m_devicePath;
    return Status::OK;
}

Status V4L2VideoDeviceDriver::streamOff() {
    if (!isOpen()) {
        LOG(ERROR) << "Device not open";
        return Status::GENERIC_ERROR;
    }

    if (xioctl(VIDIOC_STREAMOFF, &m_bufferType) == -1) {
        LOG(ERROR) << "Failed to stop streaming: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    LOG(INFO) << "Stopped streaming on " << m_devicePath;
    return Status::OK;
}

Status V4L2VideoDeviceDriver::waitForBuffer(int timeoutMs) {
    if (!isOpen()) {
        LOG(ERROR) << "Device not open";
        return Status::GENERIC_ERROR;
    }

    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(m_fd, &fds);

    if (timeoutMs >= 0) {
        tv.tv_sec = timeoutMs / 1000;
        tv.tv_usec = (timeoutMs % 1000) * 1000;
        r = select(m_fd + 1, &fds, nullptr, nullptr, &tv);
    } else {
        r = select(m_fd + 1, &fds, nullptr, nullptr, nullptr);
    }

    if (r == -1) {
        if (errno == EINTR) {
            return Status::BUSY; // Interrupted by signal
        }
        LOG(ERROR) << "select() error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    if (r == 0) {
        return Status::BUSY; // Timeout
    }

    return Status::OK;
}

int V4L2VideoDeviceDriver::getFileDescriptor() const { return m_fd; }

Status V4L2VideoDeviceDriver::ioctl(unsigned long request, void *arg) {
    if (!isOpen()) {
        LOG(ERROR) << "Device not open";
        return Status::GENERIC_ERROR;
    }

    if (xioctl(request, arg) == -1) {
        LOG(ERROR) << "ioctl 0x" << std::hex << request
                   << " failed: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    return Status::OK;
}

int V4L2VideoDeviceDriver::xioctl(unsigned long request, void *arg) const {
    int r;
    int tries = 3;

    do {
        r = ::ioctl(m_fd, request, arg);
    } while (--tries > 0 && r == -1 && errno == EINTR);

    return r;
}

void V4L2VideoDeviceDriver::toV4L2Format(const VideoFormat &format,
                                         struct v4l2_format &v4l2fmt) const {
    v4l2fmt.type = m_bufferType;
    v4l2fmt.fmt.pix.width = format.width;
    v4l2fmt.fmt.pix.height = format.height;
    v4l2fmt.fmt.pix.pixelformat = format.pixelFormat;
    v4l2fmt.fmt.pix.field = V4L2_FIELD_NONE;
    v4l2fmt.fmt.pix.bytesperline = format.bytesPerLine;
    v4l2fmt.fmt.pix.sizeimage = format.sizeImage;
}

void V4L2VideoDeviceDriver::fromV4L2Format(const struct v4l2_format &v4l2fmt,
                                           VideoFormat &format) const {
    format.pixelFormat = v4l2fmt.fmt.pix.pixelformat;
    format.width = v4l2fmt.fmt.pix.width;
    format.height = v4l2fmt.fmt.pix.height;
    format.bytesPerLine = v4l2fmt.fmt.pix.bytesperline;
    format.sizeImage = v4l2fmt.fmt.pix.sizeimage;
}

} // namespace aditof
