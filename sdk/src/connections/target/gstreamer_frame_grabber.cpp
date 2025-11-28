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
    , m_newFrameAvailable(false)
    , m_lastFrameTimestamp(0)
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

    // Configure appsink
    g_object_set(G_OBJECT(m_appsink),
                 "emit-signals", TRUE,
                 "max-buffers", 1,
                 "drop", TRUE,
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

    // Connect callback for new frames
    g_signal_connect(m_appsink, "new-sample", G_CALLBACK(onNewSample), this);

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

    m_isRunning = true;
    m_shouldExit = false;
    m_frameCount = 0;

    // Start bus watch thread
    m_busWatchThread = std::thread(&GStreamerFrameGrabber::busWatchThreadFunc, this);

    LOG(INFO) << "GStreamer pipeline started";

    return Status::OK;
}

Status GStreamerFrameGrabber::stopPipeline() {
    if (!m_isRunning) {
        return Status::OK;
    }

    m_shouldExit = true;
    m_frameCondition.notify_all();

    // Stop the pipeline
    if (m_pipeline) {
        gst_element_set_state(m_pipeline, GST_STATE_NULL);
    }

    // Wait for bus watch thread to exit
    if (m_busWatchThread.joinable()) {
        m_busWatchThread.join();
    }

    m_isRunning = false;

    LOG(INFO) << "GStreamer pipeline stopped. Total frames: " << m_frameCount.load();

    return Status::OK;
}

Status GStreamerFrameGrabber::waitForFrame(uint8_t* buffer, size_t& bufferSize, 
                                           int timeoutMs, uint64_t* timestamp) {
    if (!buffer) {
        return Status::INVALID_ARGUMENT;
    }

    std::unique_lock<std::mutex> lock(m_frameMutex);

    // Wait for new frame with timeout
    auto timeout = std::chrono::milliseconds(timeoutMs);
    if (!m_frameCondition.wait_for(lock, timeout, [this] { 
        return m_newFrameAvailable || m_shouldExit; 
    })) {
        return Status::UNREACHABLE; // Timeout
    }

    if (m_shouldExit) {
        return Status::GENERIC_ERROR;
    }

    if (m_currentFrameBuffer.empty()) {
        return Status::UNAVAILABLE;
    }

    size_t frameSize = m_currentFrameBuffer.size();
    if (bufferSize < frameSize) {
        bufferSize = frameSize;
        return Status::INVALID_ARGUMENT;
    }

    std::memcpy(buffer, m_currentFrameBuffer.data(), frameSize);
    bufferSize = frameSize;

    if (timestamp) {
        *timestamp = m_lastFrameTimestamp;
    }

    m_newFrameAvailable = false;

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

GstFlowReturn GStreamerFrameGrabber::onNewSample(GstElement* sink, gpointer userData) {
    GStreamerFrameGrabber* grabber = static_cast<GStreamerFrameGrabber*>(userData);
    
    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (sample) {
        GstFlowReturn ret = grabber->handleNewSample(sample);
        gst_sample_unref(sample);
        return ret;
    }
    
    return GST_FLOW_ERROR;
}

GstFlowReturn GStreamerFrameGrabber::handleNewSample(GstSample* sample) {
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        return GST_FLOW_ERROR;
    }

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        return GST_FLOW_ERROR;
    }

    {
        std::lock_guard<std::mutex> lock(m_frameMutex);
        
        // Copy frame data to internal buffer
        m_currentFrameBuffer.assign(map.data, map.data + map.size);
        m_lastFrameTimestamp = GST_BUFFER_PTS(buffer);
        m_newFrameAvailable = true;
        m_frameCount++;
    }

    gst_buffer_unmap(buffer, &map);
    
    // Notify waiting threads
    m_frameCondition.notify_all();

    return GST_FLOW_OK;
}

// AR0234Backend_Internal interface implementation

bool GStreamerFrameGrabber::initialize(const AR0234SensorConfig& config) {
    // Convert AR0234SensorConfig to GStreamerConfig
    GStreamerConfig gstConfig;
    gstConfig.width = config.width;
    gstConfig.height = config.height;
    gstConfig.framerate = config.fps;
    gstConfig.sensorId = config.sensorId;
    gstConfig.mode = config.mode;
    gstConfig.format = "BGRx"; // AR0234 uses BGRx format
    
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

bool GStreamerFrameGrabber::getFrame(AR0234Frame& frame, uint32_t timeoutMs) {
    // Allocate buffer for frame data
    size_t expectedSize = m_config.width * m_config.height * 4; // BGRx = 4 bytes per pixel
    std::vector<uint8_t> buffer(expectedSize);
    size_t bufferSize = expectedSize;
    uint64_t timestamp = 0;
    
    // Use existing waitForFrame method
    Status status = waitForFrame(buffer.data(), bufferSize, timeoutMs, &timestamp);
    
    if (status != Status::OK) {
        return false;
    }
    
    // Fill AR0234Frame structure
    frame.data = std::move(buffer);
    frame.width = m_config.width;
    frame.height = m_config.height;
    frame.timestamp = timestamp;
    
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
