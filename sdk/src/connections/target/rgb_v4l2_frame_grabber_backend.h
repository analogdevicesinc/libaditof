/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Analog Devices, Inc.
 * All rights reserved.
 */
#ifndef RGB_V4L2_FRAME_GRABBER_BACKEND_H
#define RGB_V4L2_FRAME_GRABBER_BACKEND_H

#ifdef HAS_RGB_CAMERA

#include "aditof/ar0234_sensor.h"
#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

struct v4l2_buffer;
struct v4l2_query_ext_ctrl;

namespace aditof {

/**
 * @class RGBV4L2FrameGrabberBackend
 * @brief RGB camera backend that reads the raw Bayer stream straight from V4L2
 *
 * The sensor (BA10, 10-bit GRBG in 16-bit words) is read with mmap buffers and
 * converted to BGR here: bilinear demosaic, gray-world white balance and a
 * gamma of 2.2. Without the Argus ISP there is no auto exposure, so start()
 * searches for one exposure before the depth stream runs, and white balance
 * and the frame-to-frame level (light flicker) are tracked from the Bayer data.
 * The sensor is never written to while the depth stream runs: a control write
 * makes it emit extra frames that would break the pairing.
 *
 * initialize() opens the device and sets controls and format, so it belongs in
 * setMode(); start() and stop() only stream, and the device stays open until
 * the backend is destroyed or initialized again.
 *
 * The sensor is triggered by the depth stream. start() returns once the sensor
 * is armed and has been read empty, so the depth stream can be started right
 * after it and every frame that follows has a depth partner.
 */
class RGBV4L2FrameGrabberBackend : public RGBBackend_Internal {
  public:
    RGBV4L2FrameGrabberBackend();
    ~RGBV4L2FrameGrabberBackend() override;

    RGBBackend getBackendType() const override { return RGBBackend::V4L2; }
    std::string getBackendName() const override { return "V4L2"; }

    bool initialize(RGBSensorConfig &config) override;
    bool start() override;
    bool stop() override;
    bool isRunning() const override { return m_isRunning; }
    bool getFrame(RGBFrame &frame, uint32_t timeoutMs = 1000) override;
    std::string getStatistics() const override;

  private:
    struct MappedBuffer {
        void *start = nullptr;
        size_t length = 0;
    };

    /// Sums over the sampled Bayer cells that are not clipped.
    struct SceneStats {
        double r = 0, g = 0, b = 0;
        size_t cells = 0;
        double clippedFraction = 0;
    };

    bool findControl(const char *name, v4l2_query_ext_ctrl &found);
    bool setControlByName(const char *name, int32_t value);
    bool getControlByName(const char *name, int64_t &value);
    void settleExposure();
    bool measureLastFrame(SceneStats &stats, uint32_t firstWaitMs);
    SceneStats measureScene(const uint16_t *raw) const;
    void rebuildColorTables();
    void trackColor(const uint16_t *raw);
    bool dequeueBuffer(v4l2_buffer &buf, uint32_t timeoutMs);
    bool acceptBuffer(const v4l2_buffer &buf);
    bool requeueBuffer(v4l2_buffer &buf);
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
    double m_gainR = 1.0; ///< white balance gains, tracked while streaming
    double m_gainB = 1.0;
    double m_flickerGain = 1.0; ///< per-frame level correction
    double m_levelAvg = 0;      ///< running average of the green level
};

} // namespace aditof

#endif // HAS_RGB_CAMERA

#endif // RGB_V4L2_FRAME_GRABBER_BACKEND_H
