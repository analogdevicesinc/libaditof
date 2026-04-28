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
#include "adsd3500_device.h"
#include "adsd3500_command_interface.h"
#include "buffer_processor_interface.h"
#include "v4l2_video_device_driver.h"
#include "video_device_driver.h"
#include <aditof/log.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace aditof {

struct Adsd3500Device::Impl {
    std::string videoDevicePath;
    std::string subdevicePath;
    std::unique_ptr<VideoDeviceDriver> videoDriver;
    std::unique_ptr<VideoDeviceDriver> subdeviceDriver;
    bool isDeviceOpen;
    uint16_t chipId;
    int videoFd;
    int subdeviceFd;

    Impl(const std::string &video, const std::string &subdev)
        : videoDevicePath(video), subdevicePath(subdev), isDeviceOpen(false),
          chipId(0), videoFd(-1), subdeviceFd(-1) {}

    ~Impl() {
        if (videoFd >= 0) {
            ::close(videoFd);
        }
        if (subdeviceFd >= 0) {
            ::close(subdeviceFd);
        }
    }
};

Adsd3500Device::Adsd3500Device(const std::string &videoDevicePath,
                               const std::string &subdevicePath)
    : m_impl(new Impl(videoDevicePath, subdevicePath)) {}

Adsd3500Device::~Adsd3500Device() { close(); }

Status Adsd3500Device::open(Adsd3500CommandInterface *commandInterface,
                            BufferProcessorInterface *bufferProcessor) {
    if (m_impl->isDeviceOpen) {
        LOG(WARNING) << "Device already open";
        return Status::OK;
    }

    LOG(INFO) << "Opening ADSD3500 device: video=" << m_impl->videoDevicePath
              << ", subdev=" << m_impl->subdevicePath;

    // Open video device
    struct stat st;
    if (stat(m_impl->videoDevicePath.c_str(), &st) == -1) {
        LOG(ERROR) << "Cannot identify video device: "
                   << m_impl->videoDevicePath;
        return Status::GENERIC_ERROR;
    }

    if (!S_ISCHR(st.st_mode)) {
        LOG(ERROR) << "Video device is not a valid device: "
                   << m_impl->videoDevicePath;
        return Status::GENERIC_ERROR;
    }

    m_impl->videoFd =
        ::open(m_impl->videoDevicePath.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (m_impl->videoFd == -1) {
        LOG(ERROR) << "Cannot open video device: " << m_impl->videoDevicePath
                   << " errno: " << errno;
        return Status::GENERIC_ERROR;
    }

    // Query capabilities
    struct v4l2_capability cap;
    if (ioctl(m_impl->videoFd, VIDIOC_QUERYCAP, &cap) == -1) {
        LOG(ERROR) << "VIDIOC_QUERYCAP failed on " << m_impl->videoDevicePath;
        ::close(m_impl->videoFd);
        m_impl->videoFd = -1;
        return Status::GENERIC_ERROR;
    }

    if (!(cap.capabilities &
          (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE))) {
        LOG(ERROR) << m_impl->videoDevicePath
                   << " is not a video capture device";
        ::close(m_impl->videoFd);
        m_impl->videoFd = -1;
        return Status::GENERIC_ERROR;
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        LOG(ERROR) << m_impl->videoDevicePath
                   << " does not support streaming I/O";
        ::close(m_impl->videoFd);
        m_impl->videoFd = -1;
        return Status::GENERIC_ERROR;
    }

    // Open subdevice
    if (stat(m_impl->subdevicePath.c_str(), &st) == -1) {
        LOG(ERROR) << "Cannot identify subdevice: " << m_impl->subdevicePath;
        ::close(m_impl->videoFd);
        m_impl->videoFd = -1;
        return Status::GENERIC_ERROR;
    }

    if (!S_ISCHR(st.st_mode)) {
        LOG(ERROR) << "Subdevice is not a valid device: "
                   << m_impl->subdevicePath;
        ::close(m_impl->videoFd);
        m_impl->videoFd = -1;
        return Status::GENERIC_ERROR;
    }

    m_impl->subdeviceFd =
        ::open(m_impl->subdevicePath.c_str(), O_RDWR | O_NONBLOCK);
    if (m_impl->subdeviceFd == -1) {
        LOG(ERROR) << "Cannot open subdevice: " << m_impl->subdevicePath
                   << " errno: " << errno;
        ::close(m_impl->videoFd);
        m_impl->videoFd = -1;
        return Status::GENERIC_ERROR;
    }

    // Initialize video device driver
    m_impl->videoDriver = std::make_unique<V4L2VideoDeviceDriver>();
    Status status = m_impl->videoDriver->adoptFileDescriptor(
        m_impl->videoFd, m_impl->videoDevicePath);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to adopt file descriptor for VideoDeviceDriver";
        ::close(m_impl->videoFd);
        ::close(m_impl->subdeviceFd);
        m_impl->videoFd = -1;
        m_impl->subdeviceFd = -1;
        return Status::GENERIC_ERROR;
    }

    // Initialize subdevice driver
    m_impl->subdeviceDriver = std::make_unique<V4L2VideoDeviceDriver>();
    status = m_impl->subdeviceDriver->adoptFileDescriptor(
        m_impl->subdeviceFd, m_impl->subdevicePath);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to adopt file descriptor for SubdeviceDriver";
        ::close(m_impl->videoFd);
        ::close(m_impl->subdeviceFd);
        m_impl->videoFd = -1;
        m_impl->subdeviceFd = -1;
        return Status::GENERIC_ERROR;
    }

    m_impl->isDeviceOpen = true;
    LOG(INFO) << "ADSD3500 device opened successfully";

    return Status::OK;
}

Status Adsd3500Device::close() {
    if (!m_impl->isDeviceOpen) {
        return Status::OK;
    }

    LOG(INFO) << "Closing ADSD3500 device";

    m_impl->videoDriver.reset();
    m_impl->subdeviceDriver.reset();

    if (m_impl->videoFd >= 0) {
        ::close(m_impl->videoFd);
        m_impl->videoFd = -1;
    }

    if (m_impl->subdeviceFd >= 0) {
        ::close(m_impl->subdeviceFd);
        m_impl->subdeviceFd = -1;
    }

    m_impl->isDeviceOpen = false;
    LOG(INFO) << "ADSD3500 device closed";

    return Status::OK;
}

bool Adsd3500Device::isOpen() const { return m_impl->isDeviceOpen; }

VideoDeviceDriver *Adsd3500Device::getVideoDriver() const {
    return m_impl->videoDriver.get();
}

VideoDeviceDriver *Adsd3500Device::getSubdeviceDriver() const {
    return m_impl->subdeviceDriver.get();
}

uint16_t Adsd3500Device::getChipId() const { return m_impl->chipId; }

} // namespace aditof
