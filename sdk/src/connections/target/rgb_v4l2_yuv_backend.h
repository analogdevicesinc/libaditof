/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Analog Devices, Inc.
 * All rights reserved.
 */
#ifndef RGB_V4L2_YUV_BACKEND_H
#define RGB_V4L2_YUV_BACKEND_H

#ifdef HAS_RGB_CAMERA

#include "aditof/ar0234_sensor.h"
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

struct v4l2_buffer;

namespace aditof {

/**
 * @class RGBV4L2YuvBackend
 * @brief RGB backend for the AR0234 + GW5200 camera: YUV422 straight from V4L2
 *
 * The GW5200 ISP does demosaic, exposure and white balance, so frames are
 * copied as delivered (UYVY or YUYV, packed rows). The frame size is whatever
 * the camera firmware outputs; initialize() reports it back in the config.
 *
 * The camera is triggered by the depth stream, so start() only arms it; no
 * frame arrives until depth streaming starts. The GW5200 embedded data is
 * copied into RGBFrame::embeddedData from the location given in the config;
 * with RGBEmbeddedDataLocation::NONE a stand-in packet carrying the V4L2
 * sequence is written instead, so frame-number pairing still works.
 */
class RGBV4L2YuvBackend : public RGBBackend_Internal {
  public:
    RGBV4L2YuvBackend() = default;
    ~RGBV4L2YuvBackend() override;

    /// True if the device offers UYVY or YUYV capture.
    static bool supportsYuv422(const std::string &devicePath);

    RGBBackend getBackendType() const override { return RGBBackend::V4L2_YUV; }
    std::string getBackendName() const override { return "V4L2 YUV422"; }

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

    bool dequeueBuffer(v4l2_buffer &buf, uint32_t timeoutMs);
    bool requeueBuffer(v4l2_buffer &buf);
    void releaseBuffers();
    void closeDevice();

    RGBSensorConfig m_config;
    int m_fd = -1;
    RGBPixelFormat m_format = RGBPixelFormat::UYVY;
    uint32_t m_bytesPerLine = 0;
    size_t m_imageOffset = 0;    ///< bytes before the first image row
    size_t m_embeddedOffset = 0; ///< bytes before the embedded data
    size_t m_minFrameBytes = 0;  ///< smallest valid payload of one frame
    std::vector<MappedBuffer> m_buffers;

    std::atomic<bool> m_isRunning{false};
    std::atomic<uint64_t> m_frameCount{0};
    std::atomic<uint64_t> m_missedFrames{0};
    std::atomic<uint64_t> m_badFrames{0};
    bool m_haveSequence = false;
    uint32_t m_lastSequence = 0;
};

} // namespace aditof

#endif // HAS_RGB_CAMERA

#endif // RGB_V4L2_YUV_BACKEND_H
