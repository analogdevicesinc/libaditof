/**
 * RGB Camera Sensor Implementation
 * 
 * Copyright (c) 2025 Analog Devices, Inc.
 */

#include "aditof/ar0234_sensor.h"

#ifdef HAS_RGB_CAMERA

#include "rgb_camera_contract.h"
#include "rgb_v4l2_frame_grabber_backend.h"
#include "rgb_v4l2_yuv_backend.h"

#include <chrono>
#include <sstream>

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

namespace aditof {

// ============================================================================
// RGBSensor Implementation
// ============================================================================

RGBSensor::RGBSensor()
    : m_backend(nullptr), m_isOpen(false), m_backendInitialized(false),
      m_frameCount(0) {
    LOG(INFO) << "RGBSensor created";
}

RGBSensor::~RGBSensor() {
    if (m_isOpen) {
        close();
    }
    LOG(INFO) << "RGBSensor destroyed";
}

Status RGBSensor::open(const RGBSensorConfig &config) {
    if (m_isOpen) {
        LOG(WARNING) << "RGBSensor already open";
        return Status::BUSY;
    }

    m_config = config;

    if (m_config.devicePath.empty()) {
        m_config.devicePath = "/dev/video0";
        LOG(INFO) << "Using default RGB device path: " << m_config.devicePath;
    } else {
        LOG(INFO) << "Using RGB device path from config: "
                  << m_config.devicePath;
    }

    // A camera with the GW5200 ISP offers YUV422; the bare AR0234 only Bayer.
    if (RGBV4L2YuvBackend::supportsYuv422(m_config.devicePath)) {
        m_backend = std::make_unique<RGBV4L2YuvBackend>();
    } else {
        m_backend = std::make_unique<RGBV4L2FrameGrabberBackend>();
    }
    LOG(INFO) << "RGBSensor backend: " << m_backend->getBackendName();

    // The V4L2 device is opened and configured by initialize().
    m_isOpen = true;
    m_frameCount = 0;

    LOG(INFO) << "RGBSensor opened successfully: " << m_config.width << "x"
              << m_config.height << "@" << m_config.fps << "fps"
              << " (device: " << m_config.devicePath << ")";

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
    m_backendInitialized = false;
    m_isOpen = false;

    LOG(INFO) << "RGBSensor closed";

    return Status::OK;
}

Status RGBSensor::initialize() {
    if (!m_isOpen) {
        return Status::UNAVAILABLE;
    }
    if (isCapturing()) {
        return Status::BUSY;
    }
    m_backendInitialized = m_backend->initialize(m_config);
    return m_backendInitialized ? Status::OK : Status::GENERIC_ERROR;
}

Status RGBSensor::start() {
    if (!m_isOpen) {
        LOG(ERROR) << "RGBSensor not open";
        return Status::UNAVAILABLE;
    }

    if (isCapturing()) {
        LOG(WARNING) << "RGBSensor already capturing";
        return Status::BUSY;
    }

    if (!m_backendInitialized) {
        LOG(ERROR) << "RGBSensor backend not initialized, call "
                      "initialize() before start()";
        return Status::UNAVAILABLE;
    }

    const auto startBegin = std::chrono::steady_clock::now();
    if (!m_backend->start()) {
        LOG(ERROR) << "Failed to start AR0234 sensor capture";
        m_backend->stop();
        return Status::GENERIC_ERROR;
    }
    LOG(INFO) << "RGB timing: V4L2 stream on until sensor armed took "
              << std::chrono::duration<double, std::milli>(
                     std::chrono::steady_clock::now() - startBegin)
                     .count()
              << " ms";
    LOG(INFO) << "RGBSensor started capturing with "
              << m_backend->getBackendName() << " backend";
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

    LOG(INFO) << "RGBSensor stopped capturing";

    return Status::OK;
}

Status RGBSensor::getFrame(RGBFrame &frame, uint32_t timeoutMs) {
    if (!m_isOpen) {
        LOG(ERROR) << "RGBSensor not open";
        return Status::UNAVAILABLE;
    }

    if (!isCapturing()) {
        LOG(ERROR) << "RGBSensor not capturing";
        return Status::UNAVAILABLE;
    }

    // Get frame directly from backend (already uses RGBFrame)
    if (!m_backend->getFrame(frame, timeoutMs)) {
        return Status::GENERIC_ERROR;
    }

    frame.hasMetadata = false;
    if (!frame.embeddedData.empty()) {
        const Gw5AncillaryStatus parsed = parseGw5AncillaryData(
            frame.embeddedData.data(), frame.embeddedData.size(),
            frame.metadata);
        frame.hasMetadata = parsed == Gw5AncillaryStatus::OK;
        if (!frame.hasMetadata &&
            (++m_badMetadata == 1 || m_badMetadata % 100 == 0)) {
            LOG(WARNING) << "RGB embedded data rejected (status "
                         << static_cast<int>(parsed) << ", " << m_badMetadata
                         << " frames so far)";
        }
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

RGBBackend RGBSensor::getBackendType() const {
    return m_backend ? m_backend->getBackendType() : RGBBackend::UNKNOWN;
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
    stats << "Device Path: " << m_config.devicePath << "\n";
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
