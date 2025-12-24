/**
 * RGB Camera Sensor Implementation
 * 
 * Copyright (c) 2025 Analog Devices, Inc.
 */

#include "aditof/ar0234_sensor.h"

#ifdef HAS_RGB_CAMERA

// Include available backend implementations
#ifdef HAS_GSTREAMER_BACKEND
#include "gstreamer_frame_grabber.h"
#endif

#ifdef HAS_V4L2_BACKEND
// #include "v4l2_frame_grabber.h"  // Future implementation
#endif

#ifdef HAS_NVARGUS_BACKEND
// #include "nvargus_frame_grabber.h"  // Future implementation
#endif

#include <iostream>
#include <sstream>

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

namespace aditof {

// ============================================================================
// Internal Backend Factory Functions (used only by RGBSensor)
// ============================================================================

static std::unique_ptr<RGBBackend_Internal> createBackend() {
    // Priority order: GStreamer > V4L2 > nvargus
#ifdef HAS_GSTREAMER_BACKEND
    return std::make_unique<GStreamerFrameGrabber>();
#elif defined(HAS_V4L2_BACKEND)
    // return std::make_unique<V4L2FrameGrabber>();  // Future implementation
    return nullptr;
#elif defined(HAS_NVARGUS_BACKEND)
    // return std::make_unique<NVArgusFrameGrabber>();  // Future implementation
    return nullptr;
#else
    return nullptr;
#endif
}

static std::string getDefaultBackendName() {
#ifdef HAS_GSTREAMER_BACKEND
    return "GStreamer";
#elif defined(HAS_V4L2_BACKEND)
    return "V4L2";
#elif defined(HAS_NVARGUS_BACKEND)
    return "NVIDIA Argus";
#else
    return "Unknown";
#endif
}

// ============================================================================
// AR0234Sensor Implementation
// ============================================================================

RGBSensor::RGBSensor()
    : m_backend(nullptr)
    , m_isOpen(false)
    , m_frameCount(0)
{
    LOG(INFO) << "RGBSensor created";
}

RGBSensor::~RGBSensor() {
    if (m_isOpen) {
        close();
    }
    LOG(INFO) << "RGBSensor destroyed";
}

Status RGBSensor::open(const RGBSensorConfig& config) {
    if (m_isOpen) {
        LOG(WARNING) << "AR0234Sensor already open";
        return Status::BUSY;
    }
    
    m_config = config;
    
    // Create backend using internal factory function
    m_backend = createBackend();
    if (!m_backend) {
        LOG(ERROR) << "Failed to create RGB camera backend - no backend available";
        return Status::GENERIC_ERROR;
    }
    
    LOG(INFO) << "AR0234Sensor using backend: " << m_backend->getBackendName();
    
    // Initialize backend directly with AR0234 config
    if (!m_backend->initialize(config)) {
        LOG(ERROR) << "Failed to initialize AR0234 sensor backend";
        m_backend.reset();
        return Status::GENERIC_ERROR;
    }
    
    m_isOpen = true;
    m_frameCount = 0;
    
    LOG(INFO) << "AR0234Sensor opened successfully: " 
              << config.width << "x" << config.height << "@" << config.fps << "fps";
    
    return Status::OK;
}

Status RGBSensor::close() {
    if (!m_isOpen) {
        return Status::OK;
    }
    
    // Stop if still capturing
    if (isCapturing()) {
        stop();
    }
    
    // Release backend
    m_backend.reset();
    m_isOpen = false;
    
    LOG(INFO) << "AR0234Sensor closed";
    
    return Status::OK;
}

Status RGBSensor::start() {
    if (!m_isOpen) {
        LOG(ERROR) << "AR0234Sensor not open";
        return Status::UNAVAILABLE;
    }
    
    if (isCapturing()) {
        LOG(WARNING) << "AR0234Sensor already capturing";
        return Status::BUSY;
    }
    
    if (!m_backend->start()) {
        LOG(ERROR) << "Failed to start AR0234 sensor capture";
        return Status::GENERIC_ERROR;
    }
    
    LOG(INFO) << "AR0234Sensor started capturing";
    
    return Status::OK;
}

Status RGBSensor::stop() {
    if (!m_isOpen) {
        return Status::UNAVAILABLE;
    }
    
    if (!isCapturing()) {
        return Status::OK;
    }
    
    if (!m_backend->stop()) {
        LOG(ERROR) << "Failed to stop AR0234 sensor capture";
        return Status::GENERIC_ERROR;
    }
    
    LOG(INFO) << "AR0234Sensor stopped capturing";
    
    return Status::OK;
}

Status RGBSensor::getFrame(RGBFrame& frame, uint32_t timeoutMs) {
    if (!m_isOpen) {
        LOG(ERROR) << "AR0234Sensor not open";
        return Status::UNAVAILABLE;
    }
    
    if (!isCapturing()) {
        LOG(ERROR) << "AR0234Sensor not capturing";
        return Status::UNAVAILABLE;
    }
    
    // Get frame directly from backend (already uses AR0234Frame)
    if (!m_backend->getFrame(frame, timeoutMs)) {
        return Status::GENERIC_ERROR;
    }
    
    m_frameCount++;
    
    return Status::OK;
}

bool RGBSensor::isCapturing() const {
    return m_isOpen && m_backend && m_backend->isRunning();
}

std::string RGBSensor::getBackendName() const {
    if (!m_backend) {
        return "None";
    }
    return m_backend->getBackendName();
}

std::string RGBSensor::getStatistics() const {
    if (!m_backend) {
        return "RGBSensor: Not initialized";
    }
    
    std::ostringstream stats;
    stats << "RGB Camera Sensor\n";
    stats << "-------------------------\n";
    stats << "Status: " << (m_isOpen ? "Open" : "Closed") << "\n";
    stats << "Capturing: " << (isCapturing() ? "Yes" : "No") << "\n";
    stats << "Backend: " << getBackendName() << "\n";
    stats << "Resolution: " << m_config.width << "x" << m_config.height << "\n";
    stats << "FPS: " << m_config.fps << "\n";
    stats << "Sensor ID: " << m_config.sensorId << "\n";
    stats << "Frames Captured: " << m_frameCount << "\n";
    stats << "\nBackend Details:\n";
    stats << m_backend->getStatistics();
    
    return stats.str();
}

} // namespace aditof

#endif // HAS_RGB_CAMERA
