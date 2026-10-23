/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Analog Devices, Inc.
 * All rights reserved.
 */
#include "rgb_v4l2_frame_grabber_backend.h"

#ifdef HAS_RGB_CAMERA

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

namespace aditof {

namespace {

constexpr uint32_t kBufferCount = 4;
constexpr int kArmTimeoutMs = 2000;
constexpr double kGamma = 2.2;
constexpr unsigned kMaxConvertThreads = 3;

// White balance is tracked every few frames and smoothed.
constexpr uint64_t kAutoInterval = 4;
constexpr uint16_t kClipRaw = 1000u << 6; // counted as clipped
constexpr double kMinFlickerGain = 0.7;
constexpr double kMaxFlickerGain = 1.4;
constexpr double kLevelAverageAlpha = 0.05;

// Exposure is searched once per start, before the depth stream runs.
constexpr double kTargetLevel = 0.25;  // linear mid-gray
constexpr double kFullScale = 65472.0; // 10-bit sample in the upper bits
constexpr double kMaxClippedFraction = 0.03;
constexpr int32_t kStartExposureUs = 3000;
constexpr int32_t kMinExposureUs = 100;
constexpr int32_t kMaxExposureUs = 16667; // limit of the sensor control
constexpr int kExposureSteps = 6;
constexpr uint32_t kBurstWaitMs = 1000; // for the first frame after a write
constexpr uint32_t kQuietMs = 300;      // gap that ends a burst of frames

int xioctl(int fd, unsigned long request, void *arg) {
    int ret;
    do {
        ret = ioctl(fd, request, arg);
    } while (ret == -1 && errno == EINTR);
    return ret;
}

// The driver names are like "Bypass Mode"; v4l2-ctl shows them as bypass_mode.
std::string controlVarName(const unsigned char *name) {
    std::string var;
    bool underscore = false;
    for (const unsigned char *p = name; *p; ++p) {
        if (std::isalnum(*p)) {
            if (underscore) {
                var += '_';
            }
            underscore = false;
            var += static_cast<char>(std::tolower(*p));
        } else if (!var.empty()) {
            underscore = true;
        }
    }
    return var;
}

} // namespace

RGBV4L2FrameGrabberBackend::RGBV4L2FrameGrabberBackend() {}

RGBV4L2FrameGrabberBackend::~RGBV4L2FrameGrabberBackend() {
    stop();
    closeDevice();
}

bool RGBV4L2FrameGrabberBackend::findControl(const char *name,
                                             v4l2_query_ext_ctrl &found) {
    std::memset(&found, 0, sizeof(found));
    found.id = V4L2_CTRL_FLAG_NEXT_CTRL;
    while (xioctl(m_fd, VIDIOC_QUERY_EXT_CTRL, &found) == 0) {
        if (!(found.flags & V4L2_CTRL_FLAG_DISABLED) &&
            controlVarName(
                reinterpret_cast<const unsigned char *>(found.name)) == name) {
            return true;
        }
        found.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }
    return false;
}

// Exposure is a 64-bit control, which VIDIOC_S_CTRL and VIDIOC_G_CTRL reject.
bool RGBV4L2FrameGrabberBackend::setControlByName(const char *name,
                                                  int32_t value) {
    v4l2_query_ext_ctrl query;
    if (!findControl(name, query)) {
        return false;
    }
    v4l2_ext_control control;
    std::memset(&control, 0, sizeof(control));
    control.id = query.id;
    if (query.type == V4L2_CTRL_TYPE_INTEGER64) {
        control.value64 = value;
    } else {
        control.value = value;
    }
    v4l2_ext_controls controls;
    std::memset(&controls, 0, sizeof(controls));
    controls.ctrl_class = V4L2_CTRL_ID2CLASS(query.id);
    controls.count = 1;
    controls.controls = &control;
    return xioctl(m_fd, VIDIOC_S_EXT_CTRLS, &controls) == 0;
}

bool RGBV4L2FrameGrabberBackend::getControlByName(const char *name,
                                                  int64_t &value) {
    v4l2_query_ext_ctrl query;
    if (!findControl(name, query)) {
        return false;
    }
    v4l2_ext_control control;
    std::memset(&control, 0, sizeof(control));
    control.id = query.id;
    v4l2_ext_controls controls;
    std::memset(&controls, 0, sizeof(controls));
    controls.ctrl_class = V4L2_CTRL_ID2CLASS(query.id);
    controls.count = 1;
    controls.controls = &control;
    if (xioctl(m_fd, VIDIOC_G_EXT_CTRLS, &controls) != 0) {
        return false;
    }
    value = query.type == V4L2_CTRL_TYPE_INTEGER64 ? control.value64
                                                   : control.value;
    return true;
}

bool RGBV4L2FrameGrabberBackend::initialize(const RGBSensorConfig &config) {
    closeDevice();
    m_config = config;

    m_fd = ::open(m_config.devicePath.c_str(), O_RDWR | O_NONBLOCK);
    if (m_fd < 0) {
        LOG(ERROR) << "V4L2 RGB: cannot open " << m_config.devicePath << ": "
                   << std::strerror(errno);
        return false;
    }

    v4l2_capability cap;
    if (xioctl(m_fd, VIDIOC_QUERYCAP, &cap) < 0 ||
        !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
        !(cap.capabilities & V4L2_CAP_STREAMING)) {
        LOG(ERROR) << "V4L2 RGB: " << m_config.devicePath
                   << " is not a streaming capture device";
        closeDevice();
        return false;
    }

    // Raw capture delivers frames only with the Argus bypass mode off, and the
    // capture timeout has to be off so the sensor can wait for its trigger.
    if (!setControlByName("bypass_mode", 0)) {
        LOG(WARNING) << "V4L2 RGB: could not set bypass_mode=0";
    }
    if (!setControlByName("override_capture_timeout_ms", -1)) {
        LOG(WARNING) << "V4L2 RGB: could not set override_capture_timeout_ms";
    }

    v4l2_format fmt;
    std::memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = m_config.width;
    fmt.fmt.pix.height = m_config.height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SGRBG10; // 'BA10'
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(m_fd, VIDIOC_S_FMT, &fmt) < 0) {
        LOG(ERROR) << "V4L2 RGB: VIDIOC_S_FMT failed: " << std::strerror(errno);
        closeDevice();
        return false;
    }
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_SGRBG10 ||
        fmt.fmt.pix.width != static_cast<uint32_t>(m_config.width) ||
        fmt.fmt.pix.height != static_cast<uint32_t>(m_config.height) ||
        fmt.fmt.pix.bytesperline < fmt.fmt.pix.width * 2u) {
        LOG(ERROR) << "V4L2 RGB: driver did not accept BA10 " << m_config.width
                   << "x" << m_config.height;
        closeDevice();
        return false;
    }
    m_stridePx = fmt.fmt.pix.bytesperline / 2;
    m_minFrameBytes =
        static_cast<size_t>(fmt.fmt.pix.bytesperline) * (m_config.height - 1) +
        static_cast<size_t>(m_config.width) * 2;

    LOG(INFO) << "V4L2 RGB: " << m_config.devicePath << " BA10 "
              << m_config.width << "x" << m_config.height << ", "
              << fmt.fmt.pix.bytesperline << " bytes per line";
    return true;
}

bool RGBV4L2FrameGrabberBackend::start() {
    if (m_isRunning) {
        return true;
    }
    if (m_fd < 0) {
        LOG(ERROR) << "V4L2 RGB: start() before initialize()";
        return false;
    }

    v4l2_requestbuffers req;
    std::memset(&req, 0, sizeof(req));
    req.count = kBufferCount;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(m_fd, VIDIOC_REQBUFS, &req) < 0 || req.count < 2) {
        LOG(ERROR) << "V4L2 RGB: VIDIOC_REQBUFS failed";
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
            LOG(ERROR) << "V4L2 RGB: VIDIOC_QUERYBUF failed";
            stop();
            return false;
        }
        void *mapped = mmap(nullptr, buf.length, PROT_READ, MAP_SHARED, m_fd,
                            buf.m.offset);
        if (mapped == MAP_FAILED) {
            LOG(ERROR) << "V4L2 RGB: mmap failed: " << std::strerror(errno);
            stop();
            return false;
        }
        m_buffers[i].start = mapped;
        m_buffers[i].length = buf.length;
        if (xioctl(m_fd, VIDIOC_QBUF, &buf) < 0) {
            LOG(ERROR) << "V4L2 RGB: VIDIOC_QBUF failed";
            stop();
            return false;
        }
    }

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(m_fd, VIDIOC_STREAMON, &type) < 0) {
        LOG(ERROR) << "V4L2 RGB: VIDIOC_STREAMON failed: "
                   << std::strerror(errno);
        stop();
        return false;
    }

    m_lutReady = false;
    m_gainR = 1.0;
    m_gainB = 1.0;
    m_flickerGain = 1.0;
    m_levelAvg = 0;
    m_haveSequence = false;
    m_frameCount = 0;
    m_missedFrames = 0;

    // Only tells that the sensor is armed; settleExposure() then reads every
    // frame that came before the depth trigger.
    pollfd pfd;
    pfd.fd = m_fd;
    pfd.events = POLLIN;
    pfd.revents = 0;
    if (poll(&pfd, 1, kArmTimeoutMs) <= 0) {
        LOG(ERROR) << "V4L2 RGB: no frame from the sensor within "
                   << kArmTimeoutMs << " ms of STREAMON";
        stop();
        return false;
    }

    m_isRunning = true;
    settleExposure();
    LOG(INFO) << "V4L2 RGB: streaming, sensor armed";
    return true;
}

bool RGBV4L2FrameGrabberBackend::stop() {
    if (m_fd >= 0) {
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(m_fd, VIDIOC_STREAMOFF, &type);
    }
    m_isRunning = false;
    releaseBuffers();
    return true;
}

void RGBV4L2FrameGrabberBackend::releaseBuffers() {
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

void RGBV4L2FrameGrabberBackend::closeDevice() {
    releaseBuffers();
    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
    }
}

bool RGBV4L2FrameGrabberBackend::dequeueBuffer(v4l2_buffer &buf,
                                               uint32_t timeoutMs) {
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
            LOG(ERROR) << "V4L2 RGB: VIDIOC_DQBUF failed: "
                       << std::strerror(errno);
        }
        return false;
    }
    return true;
}

bool RGBV4L2FrameGrabberBackend::acceptBuffer(const v4l2_buffer &buf) {
    if (buf.index >= m_buffers.size() || (buf.flags & V4L2_BUF_FLAG_ERROR) ||
        buf.bytesused < m_minFrameBytes) {
        LOG(WARNING) << "V4L2 RGB: bad frame (flags 0x" << std::hex << buf.flags
                     << std::dec << ", " << buf.bytesused << " bytes), skipped";
        return false;
    }
    if (m_haveSequence && buf.sequence != m_lastSequence + 1) {
        m_missedFrames += buf.sequence - m_lastSequence - 1;
        LOG(WARNING) << "V4L2 RGB: sequence jumped from " << m_lastSequence
                     << " to " << buf.sequence;
    }
    m_haveSequence = true;
    m_lastSequence = buf.sequence;
    return true;
}

bool RGBV4L2FrameGrabberBackend::requeueBuffer(v4l2_buffer &buf) {
    if (buf.index < m_buffers.size() && xioctl(m_fd, VIDIOC_QBUF, &buf) < 0) {
        LOG(ERROR) << "V4L2 RGB: VIDIOC_QBUF failed: " << std::strerror(errno);
        return false;
    }
    return true;
}

bool RGBV4L2FrameGrabberBackend::getFrame(RGBFrame &frame, uint32_t timeoutMs) {
    v4l2_buffer buf;
    if (!dequeueBuffer(buf, timeoutMs)) {
        return false;
    }
    if (!acceptBuffer(buf)) {
        requeueBuffer(buf);
        return false;
    }

    // The mapped buffer is not cached: demosaicing straight from it takes
    // about 20 times longer than from a copy.
    m_rawCopy.resize(static_cast<size_t>(m_stridePx) * m_config.height);
    std::memcpy(m_rawCopy.data(), m_buffers[buf.index].start, m_minFrameBytes);
    if (!requeueBuffer(buf)) {
        return false;
    }

    convertToBgr(m_rawCopy.data(), frame);
    frame.timestamp = static_cast<uint64_t>(buf.timestamp.tv_sec) * 1000000ULL +
                      static_cast<uint64_t>(buf.timestamp.tv_usec);
    ++m_frameCount;
    return true;
}

RGBV4L2FrameGrabberBackend::SceneStats
RGBV4L2FrameGrabberBackend::measureScene(const uint16_t *raw) const {
    // Sparse sample of the Bayer cells; clipped cells would skew the white
    // balance and the level, so they are only counted.
    SceneStats stats;
    size_t total = 0;
    size_t clipped = 0;
    for (uint32_t y = 0; y + 1 < static_cast<uint32_t>(m_config.height);
         y += 8) {
        const uint16_t *row0 = raw + static_cast<size_t>(y) * m_stridePx;
        const uint16_t *row1 = row0 + m_stridePx;
        for (uint32_t x = 0; x + 1 < static_cast<uint32_t>(m_config.width);
             x += 8) {
            const uint16_t g0 = row0[x], g1 = row1[x + 1];
            const uint16_t r = row0[x + 1], b = row1[x];
            ++total;
            if (std::max(std::max(g0, g1), std::max(r, b)) >= kClipRaw) {
                ++clipped;
                continue;
            }
            stats.g += (g0 + g1) * 0.5;
            stats.r += r;
            stats.b += b;
            ++stats.cells;
        }
    }
    stats.clippedFraction =
        total ? static_cast<double>(clipped) / static_cast<double>(total) : 0.0;
    return stats;
}

bool RGBV4L2FrameGrabberBackend::measureLastFrame(SceneStats &stats,
                                                  uint32_t firstWaitMs) {
    // Reads every queued frame and the burst that follows, and measures the
    // last one. The queue is empty afterwards.
    bool measured = false;
    uint32_t waitMs = firstWaitMs;
    v4l2_buffer buf;
    while (dequeueBuffer(buf, waitMs)) {
        waitMs = kQuietMs;
        if (acceptBuffer(buf)) {
            m_rawCopy.resize(static_cast<size_t>(m_stridePx) * m_config.height);
            std::memcpy(m_rawCopy.data(), m_buffers[buf.index].start,
                        m_minFrameBytes);
            measured = true;
        }
        requeueBuffer(buf);
    }
    if (measured) {
        stats = measureScene(m_rawCopy.data());
    }
    return measured;
}

void RGBV4L2FrameGrabberBackend::settleExposure() {
    // Nothing triggers the sensor yet, so every frame read here has no depth
    // partner and is thrown away. Writing exposure makes the module send a
    // burst of frames, which is why it is done only now and never while the
    // depth stream runs: the extra frames would break the pairing.
    const auto begin = std::chrono::steady_clock::now();
    int32_t exposure = kStartExposureUs;
    int64_t current = 0;
    if (getControlByName("exposure", current) && current == exposure) {
        ++exposure; // an unchanged value never reaches the module
    }

    bool drained = false;
    int32_t previousExposure = 0;
    double previousLevel = 0;
    for (int step = 0; step < kExposureSteps; ++step) {
        drained = false;
        SceneStats stats;
        if (!setControlByName("exposure", exposure)) {
            LOG(WARNING) << "V4L2 RGB: could not set exposure";
            break;
        }
        // Only the first write after STREAMON sends a burst of frames; later
        // ones are applied silently, so they cannot be measured.
        const bool measured =
            measureLastFrame(stats, step == 0 ? kBurstWaitMs : kQuietMs);
        drained = true;
        if (!measured) {
            if (step == 0) {
                LOG(WARNING) << "V4L2 RGB: no frame after the exposure write";
            }
            break;
        }

        const double level =
            stats.cells
                ? stats.g / static_cast<double>(stats.cells) / kFullScale
                : 1.0;
        LOG(INFO) << "V4L2 RGB: exposure " << exposure << " us gives level "
                  << level << ", " << stats.clippedFraction * 100
                  << "% clipped";
        if (previousExposure > 0 && stats.clippedFraction < 0.5 &&
            std::fabs(static_cast<double>(exposure) / previousExposure - 1) >
                0.5 &&
            std::fabs(level / std::max(previousLevel, 1e-4) - 1) < 0.1) {
            LOG(WARNING) << "V4L2 RGB: the exposure control has no effect";
            break;
        }
        previousExposure = exposure;
        previousLevel = level;

        double ratio = kTargetLevel / std::max(level, 1e-4);
        if (stats.clippedFraction > kMaxClippedFraction) {
            ratio = std::min(ratio, 0.6);
        }
        if (std::fabs(ratio - 1.0) < 0.15) {
            break;
        }
        const int32_t next = std::max(
            kMinExposureUs,
            std::min(kMaxExposureUs,
                     static_cast<int32_t>(std::lround(
                         exposure * std::max(0.25, std::min(4.0, ratio))))));
        if (next == exposure) {
            break;
        }
        exposure = next;
    }

    if (!drained) {
        v4l2_buffer buf;
        while (dequeueBuffer(buf, kQuietMs)) {
            requeueBuffer(buf);
        }
    }
    LOG(INFO) << "V4L2 RGB: exposure settled at " << exposure << " us in "
              << std::chrono::duration<double, std::milli>(
                     std::chrono::steady_clock::now() - begin)
                     .count()
              << " ms, frame queue empty";
}

void RGBV4L2FrameGrabberBackend::rebuildColorTables() {
    const double gains[3] = {m_gainB * m_flickerGain, m_flickerGain,
                             m_gainR * m_flickerGain}; // B, G, R
    for (int c = 0; c < 3; ++c) {
        for (int i = 0; i < 1024; ++i) {
            const double linear = std::min(1.0, i / 1023.0 * gains[c]);
            m_lut[c][i] = static_cast<uint8_t>(
                std::lround(255.0 * std::pow(linear, 1.0 / kGamma)));
        }
    }
    m_lutReady = true;
}

void RGBV4L2FrameGrabberBackend::trackColor(const uint16_t *raw) {
    const SceneStats stats = measureScene(raw);
    const bool tuneFrame = !m_lutReady || m_frameCount % kAutoInterval == 0;

    // Mains light flickers at 100/120 Hz and the frame rate aliases that into a
    // frame-to-frame change of the whole (global shutter) image. Scale each
    // frame to the running average level; scene changes are slower than that.
    double flicker = 1.0;
    if (stats.cells > 0 && stats.g > 0) {
        const double level = stats.g / static_cast<double>(stats.cells);
        if (m_levelAvg <= 0) {
            m_levelAvg = level;
        }
        flicker = std::max(kMinFlickerGain,
                           std::min(kMaxFlickerGain, m_levelAvg / level));
        m_levelAvg += kLevelAverageAlpha * (level - m_levelAvg);
    }
    m_flickerGain = flicker;

    // Gray-world white balance, smoothed so it never steps visibly.
    if (tuneFrame && stats.cells > 0 && stats.r > 0 && stats.b > 0) {
        const double alpha = m_lutReady ? 0.25 : 1.0;
        m_gainR += alpha * (stats.g / stats.r - m_gainR);
        m_gainB += alpha * (stats.g / stats.b - m_gainB);
    }
    rebuildColorTables();
}

// Bilinear demosaic of the GRBG pattern (G R / B G); neighbours that fall
// outside the image are mirrored so they keep the color of the missing one.
void RGBV4L2FrameGrabberBackend::demosaicRows(const uint16_t *raw, uint8_t *bgr,
                                              uint32_t firstRow,
                                              uint32_t endRow) const {
    const uint32_t w = static_cast<uint32_t>(m_config.width);
    const uint32_t h = static_cast<uint32_t>(m_config.height);
    const uint8_t *lutB = m_lut[0].data();
    const uint8_t *lutG = m_lut[1].data();
    const uint8_t *lutR = m_lut[2].data();

    for (uint32_t y = firstRow; y < endRow; ++y) {
        const uint16_t *cur = raw + static_cast<size_t>(y) * m_stridePx;
        const uint16_t *up =
            raw + static_cast<size_t>(y > 0 ? y - 1 : y + 1) * m_stridePx;
        const uint16_t *dn =
            raw + static_cast<size_t>(y + 1 < h ? y + 1 : y - 1) * m_stridePx;
        uint8_t *out = bgr + static_cast<size_t>(y) * w * 3;
        const bool redRow = (y & 1) == 0;

        for (uint32_t x = 0; x < w; ++x) {
            const uint32_t xl = x > 0 ? x - 1 : x + 1;
            const uint32_t xr = x + 1 < w ? x + 1 : x - 1;
            const bool oddCol = (x & 1) != 0;
            unsigned r, g, b;
            if (redRow && oddCol) { // red site
                r = cur[x];
                g = (up[x] + dn[x] + cur[xl] + cur[xr]) >> 2;
                b = (up[xl] + up[xr] + dn[xl] + dn[xr]) >> 2;
            } else if (!redRow && !oddCol) { // blue site
                b = cur[x];
                g = (up[x] + dn[x] + cur[xl] + cur[xr]) >> 2;
                r = (up[xl] + up[xr] + dn[xl] + dn[xr]) >> 2;
            } else if (redRow) { // green site, red row: R left/right, B up/down
                g = cur[x];
                r = (cur[xl] + cur[xr]) >> 1;
                b = (up[x] + dn[x]) >> 1;
            } else { // green site, blue row: B left/right, R up/down
                g = cur[x];
                b = (cur[xl] + cur[xr]) >> 1;
                r = (up[x] + dn[x]) >> 1;
            }
            // 10-bit samples sit in the upper bits of 16-bit words.
            out[3 * x] = lutB[b >> 6];
            out[3 * x + 1] = lutG[g >> 6];
            out[3 * x + 2] = lutR[r >> 6];
        }
    }
}

void RGBV4L2FrameGrabberBackend::convertToBgr(const uint16_t *raw,
                                              RGBFrame &frame) {
    trackColor(raw);

    const uint32_t w = static_cast<uint32_t>(m_config.width);
    const uint32_t h = static_cast<uint32_t>(m_config.height);
    frame.width = w;
    frame.height = h;
    frame.format = RGBPixelFormat::BGR;
    frame.data.resize(static_cast<size_t>(w) * h * 3);
    uint8_t *bgr = frame.data.data();

    unsigned threads = std::thread::hardware_concurrency();
    threads = std::max(1u, std::min(threads, kMaxConvertThreads));
    const uint32_t rowsPerThread = (h + threads - 1) / threads;

    std::vector<std::thread> workers;
    uint32_t row = 0;
    for (unsigned t = 0; t + 1 < threads && row < h; ++t) {
        const uint32_t end = std::min(h, row + rowsPerThread);
        workers.emplace_back(
            [this, raw, bgr, row, end]() { demosaicRows(raw, bgr, row, end); });
        row = end;
    }
    demosaicRows(raw, bgr, row, h);
    for (auto &worker : workers) {
        worker.join();
    }
}

std::string RGBV4L2FrameGrabberBackend::getStatistics() const {
    std::ostringstream stats;
    stats << "Backend: V4L2\n";
    stats << "Resolution: " << m_config.width << "x" << m_config.height << "\n";
    stats << "Device Path: " << m_config.devicePath << "\n";
    stats << "Total Frames: " << m_frameCount.load() << "\n";
    stats << "Missed Frames: " << m_missedFrames.load() << "\n";
    stats << "Running: " << (m_isRunning ? "Yes" : "No");
    return stats.str();
}

} // namespace aditof

#endif // HAS_RGB_CAMERA
