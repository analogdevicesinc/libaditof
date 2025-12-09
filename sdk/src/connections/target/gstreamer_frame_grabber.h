/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, Analog Devices, Inc.
 * All rights reserved.
 */
#ifndef GSTREAMER_FRAME_GRABBER_H
#define GSTREAMER_FRAME_GRABBER_H

#if defined(HAS_RGB_CAMERA) && defined(HAS_GSTREAMER_BACKEND)

#include "ar0234_sensor.h"
#include <aditof/status_definitions.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <vector>
#include <cstdint>

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
    
    GStreamerConfig() 
        : sensorId(0)
        , mode(0)
        , width(1920)
        , height(1200)
        , framerate(60)
        , format("BGRx")
    {}
};

/**
 * @class GStreamerFrameGrabber
 * @brief GStreamer-based backend for AR0234 camera
 * 
 * This class implements the AR0234Backend_Internal interface using GStreamer
 * pipeline with nvarguscamerasrc for capturing frames from AR0234 sensor.
 * 
 * Applications should use AR0234Sensor class, not this directly.
 * To add V4L2 or nvargus backends, create new classes implementing AR0234Backend_Internal.
 */
class GStreamerFrameGrabber : public AR0234Backend_Internal {
public:
    GStreamerFrameGrabber();
    ~GStreamerFrameGrabber() override;

    // AR0234Backend_Internal interface implementation
    AR0234Backend getBackendType() const override { return AR0234Backend::GSTREAMER; }
    std::string getBackendName() const override { return "GStreamer"; }
    bool initialize(const AR0234SensorConfig& config) override;
    bool start() override;
    bool stop() override;
    bool isRunning() const override { return m_isRunning; }
    bool getFrame(AR0234Frame& frame, uint32_t timeoutMs = 1000) override;
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
    Status createPipeline(const GStreamerConfig& config);

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
    GstElement* m_pipeline;
    GstElement* m_source;
    GstElement* m_converter;
    GstElement* m_appsink;
    GstBus* m_bus;

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
