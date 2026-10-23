/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Analog Devices, Inc.
 * All rights reserved.
 */
#include "rgb_v4l2_yuv_backend.h"

#ifdef HAS_RGB_CAMERA

#include "rgb_camera_contract.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

namespace aditof {

namespace {

constexpr uint32_t kBufferCount = 4;

int xioctl(int fd, unsigned long request, void *arg) {
    int ret;
    do {
        ret = ioctl(fd, request, arg);
    } while (ret == -1 && errno == EINTR);
    return ret;
}

uint32_t toFourcc(RGBPixelFormat format) {
    return format == RGBPixelFormat::YUYV ? V4L2_PIX_FMT_YUYV
                                          : V4L2_PIX_FMT_UYVY;
}

bool fromFourcc(uint32_t fourcc, RGBPixelFormat &format) {
    if (fourcc == V4L2_PIX_FMT_UYVY) {
        format = RGBPixelFormat::UYVY;
        return true;
    }
    if (fourcc == V4L2_PIX_FMT_YUYV) {
        format = RGBPixelFormat::YUYV;
        return true;
    }
    return false;
}

const char *formatName(RGBPixelFormat format) {
    return format == RGBPixelFormat::YUYV ? "YUYV" : "UYVY";
}

} // namespace

RGBV4L2YuvBackend::~RGBV4L2YuvBackend() {
    stop();
    closeDevice();
}

bool RGBV4L2YuvBackend::supportsYuv422(const std::string &devicePath) {
    const int fd = ::open(devicePath.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        return false;
    }
    bool found = false;
    v4l2_fmtdesc desc;
    std::memset(&desc, 0, sizeof(desc));
    desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    while (!found && xioctl(fd, VIDIOC_ENUM_FMT, &desc) == 0) {
        found = desc.pixelformat == V4L2_PIX_FMT_UYVY ||
                desc.pixelformat == V4L2_PIX_FMT_YUYV;
        ++desc.index;
    }
    ::close(fd);
    return found;
}

bool RGBV4L2YuvBackend::initialize(RGBSensorConfig &config) {
    closeDevice();

    m_fd = ::open(config.devicePath.c_str(), O_RDWR | O_NONBLOCK);
    if (m_fd < 0) {
        LOG(ERROR) << "V4L2 YUV: cannot open " << config.devicePath << ": "
                   << std::strerror(errno);
        return false;
    }

    v4l2_capability cap;
    if (xioctl(m_fd, VIDIOC_QUERYCAP, &cap) < 0 ||
        !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
        !(cap.capabilities & V4L2_CAP_STREAMING)) {
        LOG(ERROR) << "V4L2 YUV: " << config.devicePath
                   << " is not a streaming capture device";
        closeDevice();
        return false;
    }

    // The camera firmware fixes the frame size; keep the driver's.
    v4l2_format fmt;
    std::memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(m_fd, VIDIOC_G_FMT, &fmt) < 0) {
        LOG(ERROR) << "V4L2 YUV: VIDIOC_G_FMT failed: " << std::strerror(errno);
        closeDevice();
        return false;
    }
    fmt.fmt.pix.pixelformat = toFourcc(config.yuvFormat);
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(m_fd, VIDIOC_S_FMT, &fmt) < 0) {
        LOG(ERROR) << "V4L2 YUV: VIDIOC_S_FMT failed: " << std::strerror(errno);
        closeDevice();
        return false;
    }

    RGBPixelFormat format;
    const uint32_t width = fmt.fmt.pix.width;
    const uint32_t height = fmt.fmt.pix.height;
    const uint32_t bytesPerLine = fmt.fmt.pix.bytesperline;
    if (!fromFourcc(fmt.fmt.pix.pixelformat, format) || width == 0 ||
        height == 0 || bytesPerLine < width * 2 ||
        bytesPerLine < GW5_ANCILLARY_DATA_SIZE) {
        LOG(ERROR) << "V4L2 YUV: driver did not accept YUV422 (" << width
                   << "x" << height << ", " << bytesPerLine
                   << " bytes per line)";
        closeDevice();
        return false;
    }
    if (format != config.yuvFormat) {
        LOG(WARNING) << "V4L2 YUV: asked for " << formatName(config.yuvFormat)
                     << ", driver delivers " << formatName(format);
    }

    const size_t imageBytes =
        static_cast<size_t>(bytesPerLine) * (height - 1) + width * 2;
    switch (config.embeddedData) {
    case RGBEmbeddedDataLocation::BUFFER_START:
        m_embeddedOffset = 0;
        m_imageOffset = bytesPerLine;
        m_minFrameBytes = m_imageOffset + imageBytes;
        break;
    case RGBEmbeddedDataLocation::BUFFER_END:
        m_imageOffset = 0;
        m_embeddedOffset = static_cast<size_t>(bytesPerLine) * height;
        m_minFrameBytes = m_embeddedOffset + GW5_ANCILLARY_DATA_SIZE;
        break;
    case RGBEmbeddedDataLocation::NONE:
    default:
        m_imageOffset = 0;
        m_embeddedOffset = 0;
        m_minFrameBytes = imageBytes;
        break;
    }

    m_format = format;
    m_bytesPerLine = bytesPerLine;
    config.width = static_cast<int>(width);
    config.height = static_cast<int>(height);
    m_config = config;

    LOG(INFO) << "V4L2 YUV: " << config.devicePath << " " << formatName(format)
              << " " << width << "x" << height << ", " << bytesPerLine
              << " bytes per line, image size " << fmt.fmt.pix.sizeimage;
    return true;
}

bool RGBV4L2YuvBackend::start() {
    if (m_isRunning) {
        return true;
    }
    if (m_fd < 0) {
        LOG(ERROR) << "V4L2 YUV: start() before initialize()";
        return false;
    }

    v4l2_requestbuffers req;
    std::memset(&req, 0, sizeof(req));
    req.count = kBufferCount;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(m_fd, VIDIOC_REQBUFS, &req) < 0 || req.count < 2) {
        LOG(ERROR) << "V4L2 YUV: VIDIOC_REQBUFS failed";
        stop();
        return false;
    }

    m_buffers.assign(req.count, MappedBuffer());
    for (uint32_t i = 0; i < req.count; ++i) {
        v4l2_buffer buf;
        std::memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (xioctl(m_fd, VIDIOC_QUERYBUF, &buf) < 0) {
            LOG(ERROR) << "V4L2 YUV: VIDIOC_QUERYBUF failed";
            stop();
            return false;
        }
        if (buf.length < m_minFrameBytes) {
            LOG(ERROR) << "V4L2 YUV: buffer of " << buf.length
                       << " bytes cannot hold " << m_minFrameBytes
                       << " (image and embedded data)";
            stop();
            return false;
        }
        void *mapped = mmap(nullptr, buf.length, PROT_READ, MAP_SHARED, m_fd,
                            buf.m.offset);
        if (mapped == MAP_FAILED) {
            LOG(ERROR) << "V4L2 YUV: mmap failed: " << std::strerror(errno);
            stop();
            return false;
        }
        m_buffers[i].start = mapped;
        m_buffers[i].length = buf.length;
        if (xioctl(m_fd, VIDIOC_QBUF, &buf) < 0) {
            LOG(ERROR) << "V4L2 YUV: VIDIOC_QBUF failed";
            stop();
            return false;
        }
    }

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(m_fd, VIDIOC_STREAMON, &type) < 0) {
        LOG(ERROR) << "V4L2 YUV: VIDIOC_STREAMON failed: "
                   << std::strerror(errno);
        stop();
        return false;
    }

    m_haveSequence = false;
    m_frameCount = 0;
    m_missedFrames = 0;
    m_badFrames = 0;
    m_isRunning = true;

    // Frames from before the depth trigger would have no depth partner.
    v4l2_buffer buf;
    while (dequeueBuffer(buf, 0)) {
        requeueBuffer(buf);
    }
    LOG(INFO) << "V4L2 YUV: streaming, waiting for the depth trigger";
    return true;
}

bool RGBV4L2YuvBackend::stop() {
    if (m_fd >= 0) {
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(m_fd, VIDIOC_STREAMOFF, &type);
    }
    m_isRunning = false;
    releaseBuffers();
    return true;
}

void RGBV4L2YuvBackend::releaseBuffers() {
    for (auto &buffer : m_buffers) {
        if (buffer.start) {
            munmap(buffer.start, buffer.length);
        }
    }
    m_buffers.clear();
    if (m_fd >= 0) {
        v4l2_requestbuffers req;
        std::memset(&req, 0, sizeof(req));
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        xioctl(m_fd, VIDIOC_REQBUFS, &req);
    }
}

void RGBV4L2YuvBackend::closeDevice() {
    releaseBuffers();
    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
    }
}

bool RGBV4L2YuvBackend::dequeueBuffer(v4l2_buffer &buf, uint32_t timeoutMs) {
    if (!m_isRunning || m_fd < 0) {
        return false;
    }
    pollfd pfd;
    pfd.fd = m_fd;
    pfd.events = POLLIN;
    pfd.revents = 0;
    if (poll(&pfd, 1, static_cast<int>(timeoutMs)) <= 0 ||
        !(pfd.revents & POLLIN)) {
        return false;
    }
    std::memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (xioctl(m_fd, VIDIOC_DQBUF, &buf) < 0) {
        if (errno != EAGAIN) {
            LOG(ERROR) << "V4L2 YUV: VIDIOC_DQBUF failed: "
                       << std::strerror(errno);
        }
        return false;
    }
    return true;
}

bool RGBV4L2YuvBackend::requeueBuffer(v4l2_buffer &buf) {
    if (buf.index < m_buffers.size() && xioctl(m_fd, VIDIOC_QBUF, &buf) < 0) {
        LOG(ERROR) << "V4L2 YUV: VIDIOC_QBUF failed: " << std::strerror(errno);
        return false;
    }
    return true;
}

bool RGBV4L2YuvBackend::getFrame(RGBFrame &frame, uint32_t timeoutMs) {
    v4l2_buffer buf;
    if (!dequeueBuffer(buf, timeoutMs)) {
        return false;
    }
    if (buf.index >= m_buffers.size() || (buf.flags & V4L2_BUF_FLAG_ERROR) ||
        buf.bytesused < m_minFrameBytes) {
        ++m_badFrames;
        LOG(WARNING) << "V4L2 YUV: bad frame (flags 0x" << std::hex
                     << buf.flags << std::dec << ", " << buf.bytesused
                     << " bytes), skipped";
        requeueBuffer(buf);
        return false;
    }

    // VIDIOC_QBUF rewrites buf, so take its sequence and time first.
    const uint32_t sequence = buf.sequence;
    const uint64_t monotonicUs =
        (buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
                V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC
            ? static_cast<uint64_t>(buf.timestamp.tv_sec) * 1000000ULL +
                  static_cast<uint64_t>(buf.timestamp.tv_usec)
            : 0;
    if (m_haveSequence && sequence != m_lastSequence + 1) {
        m_missedFrames += sequence - m_lastSequence - 1;
        LOG(WARNING) << "V4L2 YUV: sequence jumped from " << m_lastSequence
                     << " to " << sequence;
    }
    m_haveSequence = true;
    m_lastSequence = sequence;

    const uint32_t width = static_cast<uint32_t>(m_config.width);
    const uint32_t height = static_cast<uint32_t>(m_config.height);
    const size_t rowBytes = static_cast<size_t>(width) * 2;
    const uint8_t *base =
        static_cast<const uint8_t *>(m_buffers[buf.index].start);
    const uint8_t *image = base + m_imageOffset;
    frame.data.resize(rowBytes * height);
    if (m_bytesPerLine == rowBytes) {
        std::memcpy(frame.data.data(), image, rowBytes * height);
    } else {
        for (uint32_t row = 0; row < height; ++row) {
            std::memcpy(frame.data.data() + row * rowBytes,
                        image + static_cast<size_t>(row) * m_bytesPerLine,
                        rowBytes);
        }
    }
    if (m_config.embeddedData != RGBEmbeddedDataLocation::NONE) {
        frame.embeddedData.assign(base + m_embeddedOffset,
                                  base + m_embeddedOffset +
                                      GW5_ANCILLARY_DATA_SIZE);
    }
    if (!requeueBuffer(buf)) {
        return false;
    }

    if (m_config.embeddedData == RGBEmbeddedDataLocation::NONE) {
        RgbFrameMetadata metadata;
        metadata.outputFrameCount = sequence;
        metadata.inputFrameCount = sequence;
        metadata.outputTimestampMs = monotonicUs / 1000;
        metadata.inputTimestampMs = metadata.outputTimestampMs;
        writeGw5AncillaryData(metadata, frame.embeddedData);
    }

    frame.width = width;
    frame.height = height;
    frame.stride = static_cast<uint32_t>(rowBytes);
    frame.format = m_format;
    frame.sequence = sequence;
    frame.monotonicUs = monotonicUs;
    ++m_frameCount;
    return true;
}

std::string RGBV4L2YuvBackend::getStatistics() const {
    std::ostringstream stats;
    stats << "Backend: V4L2 YUV422 (" << formatName(m_format) << ")\n";
    stats << "Resolution: " << m_config.width << "x" << m_config.height
          << "\n";
    stats << "Frames: " << m_frameCount.load() << "\n";
    stats << "Missed (sequence gaps): " << m_missedFrames.load() << "\n";
    stats << "Bad frames: " << m_badFrames.load() << "\n";
    return stats.str();
}

} // namespace aditof

#endif // HAS_RGB_CAMERA
