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

    // Legacy API (kept for backward compatibility - deprecated)
    /**
     * @brief Initialize GStreamer library (call once per application)
     * @return Status
     */
    static Status initializeGStreamer();

    /**
     * @brief Create and configure the GStreamer pipeline
     * @param config Pipeline configuration
     * @return Status
     * @deprecated Use initialize() with AR0234SensorConfig instead
     */
    Status createPipeline(const GStreamerConfig& config);

    /**
     * @brief Start the pipeline and begin capturing frames
     * @return Status
     * @deprecated Use start() instead
     */
    Status startPipeline();

    /**
     * @brief Stop the pipeline
     * @return Status
     * @deprecated Use stop() instead
     */
    Status stopPipeline();

    /**
     * @brief Get the latest captured frame (non-blocking)
     * @param[out] buffer Buffer to copy frame data into
     * @param[in,out] bufferSize Size of buffer (in), actual bytes copied (out)
     * @param[out] timestamp Frame timestamp (optional)
     * @return Status
     * @deprecated Use getFrame() instead
     */
    Status getLatestFrame(uint8_t* buffer, size_t& bufferSize, uint64_t* timestamp = nullptr);

    /**
     * @brief Wait for and get the next frame (blocking with timeout)
     * @param[out] buffer Buffer to copy frame data into
     * @param[in,out] bufferSize Size of buffer (in), actual bytes copied (out)
     * @param timeoutMs Timeout in milliseconds
     * @param[out] timestamp Frame timestamp (optional)
     * @return Status
     * @deprecated Use getFrame() instead
     */
    Status waitForFrame(uint8_t* buffer, size_t& bufferSize, int timeoutMs = 2000, uint64_t* timestamp = nullptr);

    /**
     * @brief Get the configured frame width
     * @return Frame width in pixels
     */
    int getWidth() const { return m_config.width; }

    /**
     * @brief Get the configured frame height
     * @return Frame height in pixels
     */
    int getHeight() const { return m_config.height; }

    /**
     * @brief Get the configured framerate
     * @return Framerate in fps
     */
    int getFramerate() const { return m_config.framerate; }

    /**
     * @brief Get total number of frames captured
     * @return Frame count
     */
    uint64_t getFrameCount() const { return m_frameCount; }

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

    // Frame buffer management
    std::vector<uint8_t> m_currentFrameBuffer;
    std::mutex m_frameMutex;
    std::condition_variable m_frameCondition;
    bool m_newFrameAvailable;
    uint64_t m_lastFrameTimestamp;

    // Bus monitoring thread
    std::thread m_busWatchThread;

    // Private methods
    void busWatchThreadFunc();
    Status destroyPipeline();
    
    /**
     * @brief Callback for new frame from appsink
     * Called by GStreamer when new frame is available
     */
    static GstFlowReturn onNewSample(GstElement* sink, gpointer userData);
    
    /**
     * @brief Handle new sample from GStreamer
     */
    GstFlowReturn handleNewSample(GstSample* sample);
};

} // namespace aditof

#endif // HAS_RGB_CAMERA
#endif // GSTREAMER_FRAME_GRABBER_H
