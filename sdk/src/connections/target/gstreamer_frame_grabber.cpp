/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, Analog Devices, Inc.
 * All rights reserved.
 */
#include "gstreamer_frame_grabber.h"

#if defined(HAS_RGB_CAMERA) && defined(HAS_GSTREAMER_BACKEND)

#include <iostream>
#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>

#include <aditof/log.h>

namespace aditof {

// Static initialization flag for GStreamer
static bool s_gstreamerInitialized = false;
static std::mutex s_gstInitMutex;

GStreamerFrameGrabber::GStreamerFrameGrabber()
    : m_pipeline(nullptr)
    , m_source(nullptr)
    , m_converter(nullptr)
    , m_appsink(nullptr)
    , m_bus(nullptr)
    , m_isRunning(false)
    , m_shouldExit(false)
    , m_frameCount(0)
{
}

GStreamerFrameGrabber::~GStreamerFrameGrabber() {
    if (m_isRunning) {
        stopPipeline();
    }
    destroyPipeline();
}

Status GStreamerFrameGrabber::initializeGStreamer() {
    std::lock_guard<std::mutex> lock(s_gstInitMutex);
    
    if (s_gstreamerInitialized) {
        return Status::OK;
    }

    GError* error = nullptr;
    if (!gst_init_check(nullptr, nullptr, &error)) {
        if (error) {
            LOG(ERROR) << "Failed to initialize GStreamer: " << error->message;
            g_error_free(error);
        }
        return Status::GENERIC_ERROR;
    }

    s_gstreamerInitialized = true;
    
    LOG(INFO) << "GStreamer initialized successfully";
    
    return Status::OK;
}

Status GStreamerFrameGrabber::createPipeline(const GStreamerConfig& config) {
    if (m_pipeline) {
        destroyPipeline();
    }

    m_config = config;

    // Initialize GStreamer if not already done
    if (initializeGStreamer() != Status::OK) {
        return Status::GENERIC_ERROR;
    }

    // Create pipeline elements
    m_pipeline = gst_pipeline_new("rgb-camera-pipeline");
    m_source = gst_element_factory_make("nvarguscamerasrc", "camera-source");
    m_converter = gst_element_factory_make("nvvidconv", "converter");
    m_appsink = gst_element_factory_make("appsink", "app-sink");

    if (!m_pipeline || !m_source || !m_converter || !m_appsink) {
        LOG(ERROR) << "Failed to create GStreamer elements";
        destroyPipeline();
        return Status::GENERIC_ERROR;
    }

    // Configure nvarguscamerasrc
    g_object_set(G_OBJECT(m_source),
                 "sensor-id", config.sensorId,
                 "sensor-mode", config.mode,
                 nullptr);

    // Configure appsink for PULL model (not callback/signal model)
    // This eliminates condition variable overhead and allows direct frame pulling
    g_object_set(G_OBJECT(m_appsink),
                 "emit-signals", FALSE,     // Disable callbacks - use pull model instead
                 "max-buffers", 10,         // Increase buffer pool to prevent frame drops
                 "drop", FALSE,             // Don't drop frames
                 "sync", FALSE,              // Don't throttle to real-time (capture as fast as possible)
                 nullptr);


    // Set caps for source output (NVMM format from Argus)
    std::string sourceCaps = "video/x-raw(memory:NVMM),width=" + 
                             std::to_string(config.width) + 
                             ",height=" + std::to_string(config.height) + 
                             ",framerate=" + std::to_string(config.framerate) + "/1";
    
    GstCaps* caps1 = gst_caps_from_string(sourceCaps.c_str());
    
    // Set caps for converter output (standard format)
    std::string converterCaps = "video/x-raw,format=" + config.format;
    GstCaps* caps2 = gst_caps_from_string(converterCaps.c_str());
    
    // Build the pipeline
    gst_bin_add_many(GST_BIN(m_pipeline), m_source, m_converter, m_appsink, nullptr);
    
    // Link elements with caps
    if (!gst_element_link_filtered(m_source, m_converter, caps1)) {
        LOG(ERROR) << "Failed to link source to converter";
        gst_caps_unref(caps1);
        gst_caps_unref(caps2);
        destroyPipeline();
        return Status::GENERIC_ERROR;
    }
    
    if (!gst_element_link_filtered(m_converter, m_appsink, caps2)) {
        LOG(ERROR) << "Failed to link converter to appsink";
        gst_caps_unref(caps1);
        gst_caps_unref(caps2);
        destroyPipeline();
        return Status::GENERIC_ERROR;
    }
    
    gst_caps_unref(caps1);
    gst_caps_unref(caps2);

    // Note: Not using callbacks (emit-signals=FALSE) - using pull model instead
    // This eliminates callback overhead and condition variable latency

    // Get bus for monitoring
    m_bus = gst_element_get_bus(m_pipeline);

    LOG(INFO) << "GStreamer pipeline created: " << config.width << "x" 
              << config.height << "@" << config.framerate << "fps";

    return Status::OK;
}

Status GStreamerFrameGrabber::startPipeline() {
    if (!m_pipeline) {
        LOG(ERROR) << "Pipeline not created. Call createPipeline() first";
        return Status::GENERIC_ERROR;
    }

    if (m_isRunning) {
        LOG(WARNING) << "Pipeline already running";
        return Status::OK;
    }

    // Set pipeline to PLAYING state
    GstStateChangeReturn ret = gst_element_set_state(m_pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        LOG(ERROR) << "Failed to start pipeline";
        return Status::GENERIC_ERROR;
    }

    // Wait for pipeline to reach PLAYING state (important for pull model)
    GstState state;
    ret = gst_element_get_state(m_pipeline, &state, nullptr, 10 * GST_SECOND);
    if (ret == GST_STATE_CHANGE_FAILURE || state != GST_STATE_PLAYING) {
        LOG(ERROR) << "Pipeline failed to reach PLAYING state";
        return Status::GENERIC_ERROR;
    }

    m_isRunning = true;
    m_shouldExit = false;
    m_frameCount = 0;

    // Start bus watch thread
    m_busWatchThread = std::thread(&GStreamerFrameGrabber::busWatchThreadFunc, this);

    LOG(INFO) << "GStreamer pipeline started and PLAYING";

    return Status::OK;
}

Status GStreamerFrameGrabber::stopPipeline() {
    if (!m_isRunning) {
        return Status::OK;
    }

    LOG(INFO) << "Stopping GStreamer pipeline gracefully...";

    m_shouldExit = true;

    // Stop the pipeline gracefully: PLAYING → PAUSED → READY → NULL
    if (m_pipeline) {
        // Set timeout for state changes (1 second)
        GstClockTime timeout = 1 * GST_SECOND;

        // PLAYING → PAUSED
        GstStateChangeReturn ret = gst_element_set_state(m_pipeline, GST_STATE_PAUSED);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            LOG(WARNING) << "Failed to pause pipeline, forcing stop";
        } else {
            gst_element_get_state(m_pipeline, nullptr, nullptr, timeout);
        }

        // PAUSED → READY (releases resources)
        ret = gst_element_set_state(m_pipeline, GST_STATE_READY);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            LOG(WARNING) << "Failed to set pipeline to READY";
        } else {
            gst_element_get_state(m_pipeline, nullptr, nullptr, timeout);
        }

        // READY → NULL (cleanup)
        ret = gst_element_set_state(m_pipeline, GST_STATE_NULL);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            LOG(ERROR) << "Failed to set pipeline to NULL";
        } else {
            gst_element_get_state(m_pipeline, nullptr, nullptr, timeout);
        }
    }

    // Wait for bus watch thread to exit
    if (m_busWatchThread.joinable()) {
        m_busWatchThread.join();
    }

    m_isRunning = false;

    //LOG(INFO) << "GStreamer pipeline stopped. Total frames: " << m_frameCount.load();

    return Status::OK;
}

void GStreamerFrameGrabber::busWatchThreadFunc() {
    while (!m_shouldExit && m_isRunning) {
        GstMessage* msg = gst_bus_timed_pop_filtered(
            m_bus, 
            100 * GST_MSECOND,
            (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_WARNING)
        );

        if (msg != nullptr) {
            GError* err;
            gchar* debug_info;

            switch (GST_MESSAGE_TYPE(msg)) {
                case GST_MESSAGE_ERROR:
                    gst_message_parse_error(msg, &err, &debug_info);
                    LOG(ERROR) << "GStreamer error: " << err->message;
                    LOG(ERROR) << "Debug info: " << (debug_info ? debug_info : "none");
                    g_error_free(err);
                    g_free(debug_info);
                    m_shouldExit = true;
                    break;

                case GST_MESSAGE_WARNING:
                    gst_message_parse_warning(msg, &err, &debug_info);
                    LOG(WARNING) << "GStreamer warning: " << err->message;
                    g_error_free(err);
                    g_free(debug_info);
                    break;

                case GST_MESSAGE_EOS:
                    LOG(INFO) << "GStreamer: End of stream";
                    m_shouldExit = true;
                    break;

                default:
                    break;
            }

            gst_message_unref(msg);
        }
    }
}

Status GStreamerFrameGrabber::destroyPipeline() {
    if (m_bus) {
        gst_object_unref(m_bus);
        m_bus = nullptr;
    }

    if (m_pipeline) {
        gst_element_set_state(m_pipeline, GST_STATE_NULL);
        gst_object_unref(m_pipeline);
        m_pipeline = nullptr;
        m_source = nullptr;
        m_converter = nullptr;
        m_appsink = nullptr;
    }

    return Status::OK;
}

// AR0234Backend_Internal interface implementation

bool GStreamerFrameGrabber::initialize(const RGBSensorConfig& config) {
    // Convert AR0234SensorConfig to GStreamerConfig
    GStreamerConfig gstConfig;
    gstConfig.width = config.width;
    gstConfig.height = config.height;
    gstConfig.framerate = config.fps;
    gstConfig.sensorId = config.sensorId;
    gstConfig.mode = config.mode;
    gstConfig.format = "NV12"; // Use NV12 for efficient memory (1.5 bytes/pixel)
    
    Status status = createPipeline(gstConfig);
    return (status == Status::OK);
}

bool GStreamerFrameGrabber::start() {
    Status status = startPipeline();
    return (status == Status::OK);
}

bool GStreamerFrameGrabber::stop() {
    Status status = stopPipeline();
    return (status == Status::OK);
}

bool GStreamerFrameGrabber::getFrame(RGBFrame& frame, uint32_t timeoutMs) {
    // OPTIMIZED: Direct pull from appsink (no callbacks, no condition variables)

    if (!m_appsink || !m_isRunning) {
        return false;
    }
    
    // Check pipeline state
    GstState state;
    gst_element_get_state(m_pipeline, &state, nullptr, 0);
    if (state != GST_STATE_PLAYING) {
        return false;
    }

    // Check for EOS (end of stream)
    GstAppSink* appSink = GST_APP_SINK(m_appsink);
    if (gst_app_sink_is_eos(appSink)) {
        LOG(WARNING) << "Pipeline in EOS state - no more frames available";
        return false;
    }

    // Check bus for errors before attempting to pull
    GstMessage* msg = gst_bus_pop_filtered(m_bus, GST_MESSAGE_ERROR);
    if (msg) {
        GError* err;
        gchar* debug_info;
        gst_message_parse_error(msg, &err, &debug_info);
        LOG(ERROR) << "GStreamer error before frame pull: " << err->message;
        if (debug_info) {
            LOG(ERROR) << "Debug info: " << debug_info;
            g_free(debug_info);
        }
        g_error_free(err);
        gst_message_unref(msg);
        return false;
    }

    // Directly pull sample from appsink with timeout (non-blocking)
    GstClockTime timeout = timeoutMs * GST_MSECOND;
    GstSample* sample = gst_app_sink_try_pull_sample(appSink, timeout);

    if (!sample) {
        // Timeout or no sample available
        return false;
    }

    // Get buffer from sample
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        gst_sample_unref(sample);
        return false;
    }

    // Map buffer for reading (zero-copy access)
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return false;
    }

    // Copy frame data to AR0234Frame
    frame.data.assign(map.data, map.data + map.size);
    frame.width = m_config.width;
    frame.height = m_config.height;
    frame.timestamp = GST_BUFFER_PTS(buffer);

    // Cleanup
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    // Update frame counter
    m_frameCount++;

    return true;
}

std::string GStreamerFrameGrabber::getStatistics() const {
    std::ostringstream stats;
    stats << "Backend: GStreamer\n";
    stats << "Resolution: " << m_config.width << "x" << m_config.height << "\n";
    stats << "FPS: " << m_config.framerate << "\n";
    stats << "Format: " << m_config.format << "\n";
    stats << "Sensor ID: " << m_config.sensorId << "\n";
    stats << "Total Frames: " << m_frameCount.load() << "\n";
    stats << "Running: " << (m_isRunning ? "Yes" : "No");
    return stats.str();
}

} // namespace aditof

#endif // HAS_RGB_CAMERA
