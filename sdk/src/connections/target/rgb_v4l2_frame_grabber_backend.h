/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Analog Devices, Inc.
 * All rights reserved.
 */
#ifndef RGB_V4L2_FRAME_GRABBER_BACKEND_H
#define RGB_V4L2_FRAME_GRABBER_BACKEND_H

#if defined(HAS_RGB_CAMERA) && defined(HAS_V4L2_BACKEND)

#include "aditof/ar0234_sensor.h"
#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

struct v4l2_buffer;

namespace aditof {

/**
 * @class RGBV4L2FrameGrabberBackend
 * @brief RGB camera backend that reads the raw Bayer stream straight from V4L2
 *
 * The sensor (BA10, 10-bit GRBG in 16-bit words) is read with mmap buffers and
 * converted to BGR here: bilinear demosaic, gray-world white balance measured
 * on the first frame of each start, and a gamma of 2.2. No Argus, no GStreamer.
 *
 * initialize() opens the device and sets controls and format, so it belongs in
 * setMode(); start() and stop() only stream, and the device stays open until
 * the backend is destroyed or initialized again.
 *
 * The sensor is triggered by the depth stream. start() returns only after the
 * sensor has delivered its first frame, i.e. when it is waiting for triggers,
 * so the depth stream can be started right after it. That first frame (seq 0)
 * has no depth partner; the consumer is expected to drop it.
 */
class RGBV4L2FrameGrabberBackend : public RGBBackend_Internal {
  public:
    RGBV4L2FrameGrabberBackend();
    ~RGBV4L2FrameGrabberBackend() override;

    RGBBackend getBackendType() const override { return RGBBackend::V4L2; }
    std::string getBackendName() const override { return "V4L2"; }

    bool initialize(const RGBSensorConfig &config) override;
    bool start() override;
    bool stop() override;
    bool isRunning() const override { return m_isRunning; }
    bool getFrame(RGBFrame &frame, uint32_t timeoutMs = 1000) override;
    bool discardFrame(uint32_t timeoutMs = 1000) override;
    std::string getStatistics() const override;

  private:
    struct MappedBuffer {
        void *start = nullptr;
        size_t length = 0;
    };

    bool setControlByName(const char *name, int32_t value);
    bool dequeueBuffer(v4l2_buffer &buf, uint32_t timeoutMs);
    bool acceptBuffer(const v4l2_buffer &buf);
    bool requeueBuffer(v4l2_buffer &buf);
    void buildColorTables(const uint16_t *raw);
    void demosaicRows(const uint16_t *raw, uint8_t *bgr, uint32_t firstRow,
                      uint32_t endRow) const;
    void convertToBgr(const uint16_t *raw, RGBFrame &frame);
    void releaseBuffers();
    void closeDevice();

    RGBSensorConfig m_config;
    int m_fd = -1;
    uint32_t m_stridePx = 0;    ///< bytesperline / 2
    size_t m_minFrameBytes = 0; ///< smallest valid payload of one frame
    std::vector<MappedBuffer> m_buffers;
    std::vector<uint16_t>
        m_rawCopy; ///< cached copy of the frame being converted

    std::atomic<bool> m_isRunning{false};
    std::atomic<uint64_t> m_frameCount{0};
    std::atomic<uint64_t> m_missedFrames{0};
    bool m_haveSequence = false;
    uint32_t m_lastSequence = 0;

    /// 10-bit sample to 8-bit output, per channel (B, G, R), with gain and gamma.
    std::array<std::array<uint8_t, 1024>, 3> m_lut;
    bool m_lutReady = false;
};

} // namespace aditof

#endif // HAS_RGB_CAMERA && HAS_V4L2_BACKEND

#endif // RGB_V4L2_FRAME_GRABBER_BACKEND_H
