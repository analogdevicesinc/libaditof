/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, Analog Devices, Inc.
 * All rights reserved.
 */
#ifndef GSTREAMER_FRAME_GRABBER_H
#define GSTREAMER_FRAME_GRABBER_H

#if defined(HAS_RGB_CAMERA) && defined(HAS_GSTREAMER_BACKEND)

#include "aditof/ar0234_sensor.h"
#include <aditof/status_definitions.h>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <mutex>
#include <thread>
#include <vector>

namespace aditof {

/**
 * @struct GStreamerConfig
 * @brief Configuration parameters for GStreamer pipeline
 * @deprecated Use AR0234SensorConfig instead for new code
 */
struct GStreamerConfig {
    int sensorId;           // Camera sensor ID (0, 1, etc.)
    int mode;               // Camera mode (0, 1, 2 from DTS)
    int width;              // Frame width
    int height;             // Frame height
    int framerate;          // Target framerate
    std::string format;     // Pixel format (e.g., "BGRx", "RGB")
    std::string devicePath; // V4L2 device path (e.g., "/dev/video0")

    GStreamerConfig()
        : sensorId(0), mode(0), width(1920), height(1200), framerate(60),
          format("BGRx"), devicePath("/dev/video0") {}
};

/**
 * @class GStreamerFrameGrabber
 * @brief GStreamer-based backend for RGB camera
 * 
 * This class implements the RGBBackend_Internal interface using GStreamer
 * pipeline with nvarguscamerasrc for capturing frames from RGB sensor.
 * 
 * Applications should use RGBSensor class, not this directly.
 * To add V4L2 or nvargus backends, create new classes implementing RGBBackend_Internal.
 */
class GStreamerFrameGrabber : public RGBBackend_Internal {
  public:
    GStreamerFrameGrabber();
    ~GStreamerFrameGrabber() override;

    // RGBBackend_Internal interface implementation
    RGBBackend getBackendType() const override { return RGBBackend::GSTREAMER; }
    std::string getBackendName() const override { return "GStreamer"; }
    /**
     * @brief Initialize the GStreamer backend with sensor configuration
     * @param[in] config RGB sensor configuration parameters
     * @return true if initialization succeeded, false otherwise
     */
    bool initialize(const RGBSensorConfig &config) override;
    /**
     * @brief Start the GStreamer pipeline and begin frame capture
     * @return true if pipeline started successfully, false otherwise
     */
    bool start() override;
    /**
     * @brief Stop the GStreamer pipeline
     * @return true if pipeline stopped successfully, false otherwise
     */
    bool stop() override;
    bool isRunning() const override { return m_isRunning; }
    /**
     * @brief Capture an RGB frame from the pipeline
     * @param[out] frame Output frame structure to populate
     * @param[in] timeoutMs Timeout in milliseconds (default 1000)
     * @return true if frame captured successfully, false on timeout or error
     */
    bool getFrame(RGBFrame &frame, uint32_t timeoutMs = 1000) override;
    /**
     * @brief Get pipeline statistics (frame count, errors, etc.)
     * @return Statistics string
     */
    std::string getStatistics() const override;

    // Internal implementation methods (used by public interface)
    /**
     * @brief Initialize GStreamer library (call once per application)
     * @return Status
     */
    static Status initializeGStreamer();

    /**
     * @brief Create and configure the GStreamer pipeline
     * @param config Pipeline configuration
     * @return Status
     */
    Status createPipeline(const GStreamerConfig &config);

    /**
     * @brief Start the pipeline and begin capturing frames
     * @return Status
     */
    Status startPipeline();

    /**
     * @brief Stop the pipeline
     * @return Status
     */
    Status stopPipeline();

  private:
    // GStreamer elements
    GstElement *m_pipeline;
    GstElement *m_source;
    GstElement *m_converter;
    GstElement *m_appsink;
    GstBus *m_bus;

    // Configuration
    GStreamerConfig m_config;

    // State management
    std::atomic<bool> m_isRunning;
    std::atomic<bool> m_shouldExit;
    std::atomic<uint64_t> m_frameCount;

    // Bus monitoring thread
    std::thread m_busWatchThread;

    // Private methods
    void busWatchThreadFunc();
    Status destroyPipeline();
};

} // namespace aditof

#endif // HAS_RGB_CAMERA
#endif // GSTREAMER_FRAME_GRABBER_H
