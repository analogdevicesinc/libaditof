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

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

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

// ============================================================================
// AR0234Sensor Implementation
// ============================================================================

RGBSensor::RGBSensor()
    : m_backend(nullptr), m_isOpen(false), m_argusProbeOk(false),
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
        LOG(WARNING) << "AR0234Sensor already open";
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

    m_backend = createBackend();
    if (!m_backend) {
        LOG(ERROR) << "Failed to create RGB camera backend";
        return Status::GENERIC_ERROR;
    }

    // Pipeline creation is deferred to start() so the Argus probe subprocess
    // runs before any CameraProvider is open in this process, avoiding a
    // conflict where the probe child and parent both access the same sensor.
    m_isOpen = true;
    m_frameCount = 0;

    LOG(INFO) << "AR0234Sensor opened successfully: " << m_config.width << "x"
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

    // -----------------------------------------------------------------------
    // Step 1: Argus availability probe (first start only).
    //
    // The probe runs in a subprocess so any Argus SIGSEGV only kills the child.
    // On subsequent start() calls (stop→start cycles) we skip the probe —
    // Argus is known healthy and the 3-second overhead is wasteful.
    if (!m_argusProbeOk) {
        // num-buffers=0: pipeline runs indefinitely — we only need Argus to
        // reach PLAYING (no frame delivery required).  timeout kills it after
        // 5 s → exit 124 = Argus OK.  Immediate error exit = Argus absent.
        std::string probeCmd = "timeout 5 gst-launch-1.0 --eos-on-shutdown "
                               "nvarguscamerasrc num-buffers=0 sensor-id=" +
                               std::to_string(m_config.sensorId) +
                               " ! fakesink silent=true >/dev/null 2>&1";

        pid_t probePid = fork();
        if (probePid == 0) {
            execl("/bin/sh", "sh", "-c", probeCmd.c_str(), nullptr);
            _exit(127);
        } else if (probePid > 0) {
            int wstatus = 0;
            waitpid(probePid, &wstatus, 0);
            int exitCode = WIFEXITED(wstatus) ? WEXITSTATUS(wstatus) : -1;
            int sigNum = WIFSIGNALED(wstatus) ? WTERMSIG(wstatus) : 0;
            bool probeOk = (exitCode == 0 || exitCode == 124);
            if (!probeOk) {
                LOG(WARNING) << "Argus probe failed (exit=" << exitCode
                             << " sig=" << sigNum
                             << ") \xe2\x80\x94 RGB capture unavailable; "
                                "continuing in depth-only mode.";
                return Status::GENERIC_ERROR;
            }
            LOG(INFO) << "Argus probe OK (exit=" << exitCode
                      << "), creating pipeline.";
            m_argusProbeOk = true;
        } else {
            LOG(WARNING) << "fork() failed for Argus probe, skipping RGB.";
            return Status::GENERIC_ERROR;
        }
    }

    // -----------------------------------------------------------------------
    // Step 2: Create the GStreamer pipeline and start it on a fresh thread.
    //
    // initialize() builds the nvarguscamerasrc pipeline (NULL state).
    // Running startPipeline() (NULL→PAUSED→PLAYING) on a fresh std::thread
    // avoids the EGL API binding conflict when the caller is the tof-viewer
    // main thread (eglBindAPI EGL_OPENGL_API vs nvarguscamerasrc's ES API).
    LOG(INFO) << "AR0234Sensor using backend: " << m_backend->getBackendName();
    if (!m_backend->initialize(m_config)) {
        LOG(ERROR) << "Failed to initialize AR0234 sensor backend";
        return Status::GENERIC_ERROR;
    }

    bool startResult = false;
    std::thread startThread(
        [this, &startResult]() { startResult = m_backend->start(); });
    startThread.join();

    if (!startResult) {
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

Status RGBSensor::getFrame(RGBFrame &frame, uint32_t timeoutMs) {
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
