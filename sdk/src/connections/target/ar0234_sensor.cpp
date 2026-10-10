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
#include "rgb_v4l2_frame_grabber_backend.h"
#endif

#ifdef HAS_NVARGUS_BACKEND
// #include "nvargus_frame_grabber.h"  // Future implementation
#endif

#include <chrono>
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
    // Selected at build time with RGB_CAMERA_BACKEND
#ifdef HAS_GSTREAMER_BACKEND
    return std::make_unique<GStreamerFrameGrabber>();
#elif defined(HAS_V4L2_BACKEND)
    return std::make_unique<RGBV4L2FrameGrabberBackend>();
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
      m_backendInitialized(false), m_frameCount(0) {
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

    // GStreamer pipeline creation is deferred to start() so the Argus probe
    // subprocess runs before any CameraProvider is open in this process,
    // avoiding a conflict where the probe child and parent both access the same
    // sensor. The V4L2 backend is prepared by initialize() instead.
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
    m_backendInitialized = false;
    m_isOpen = false;

    LOG(INFO) << "AR0234Sensor closed";

    return Status::OK;
}

Status RGBSensor::initialize() {
    if (!m_isOpen) {
        return Status::UNAVAILABLE;
    }
    if (isCapturing()) {
        return Status::BUSY;
    }
    // The GStreamer pipeline is destroyed by stop(), so start() builds it.
    if (m_backend->getBackendType() == RGBBackend::GSTREAMER) {
        return Status::OK;
    }
    m_backendInitialized = m_backend->initialize(m_config);
    return m_backendInitialized ? Status::OK : Status::GENERIC_ERROR;
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

    const auto startBegin = std::chrono::steady_clock::now();
    auto elapsedMs = [](std::chrono::steady_clock::time_point since) {
        return std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - since)
            .count();
    };

    // -----------------------------------------------------------------------
    // Step 1: Argus availability probe (first start only, GStreamer backend).
    //
    // The probe runs in a subprocess so any Argus SIGSEGV only kills the child.
    // On subsequent start() calls (stop→start cycles) we skip the probe —
    // Argus is known healthy and the 3-second overhead is wasteful.
    const bool useArgus = m_backend->getBackendType() == RGBBackend::GSTREAMER;
    if (!useArgus && !m_backendInitialized) {
        LOG(ERROR) << "AR0234Sensor backend not initialized, call "
                      "initialize() before start()";
        return Status::UNAVAILABLE;
    }
    if (useArgus && !m_argusProbeOk) {
        // timeout 3: healthy Argus initialises in ~3s (exit 124 = timeout = OK);
        // broken Argus exits with error before the timeout fires.
        const std::string probeCmd =
            "timeout 3 gst-launch-1.0 --eos-on-shutdown "
            "nvarguscamerasrc num-buffers=1 sensor-id=" +
            std::to_string(m_config.sensorId) +
            " ! fakesink silent=true >/dev/null 2>&1";

        // After a crash (SIGSEGV) the nvargus-daemon can still hold the dead
        // process's CaptureSession, so the first probe often fails. The daemon
        // reclaims the stale session once it notices the client socket closed,
        // which can take a few seconds. Retry the probe with escalating backoff
        // instead of giving up on the first failure (which would disable RGB
        // for the entire session).
        constexpr int kMaxProbeAttempts = 3;
        for (int attempt = 1; !m_argusProbeOk && attempt <= kMaxProbeAttempts;
             ++attempt) {
            const auto probeBegin = std::chrono::steady_clock::now();
            pid_t probePid = fork();
            if (probePid == 0) {
                execl("/bin/sh", "sh", "-c", probeCmd.c_str(), nullptr);
                _exit(127);
            } else if (probePid < 0) {
                LOG(WARNING) << "fork() failed for Argus probe, skipping RGB.";
                return Status::GENERIC_ERROR;
            }

            int wstatus = 0;
            waitpid(probePid, &wstatus, 0);
            int exitCode = WIFEXITED(wstatus) ? WEXITSTATUS(wstatus) : -1;
            int sigNum = WIFSIGNALED(wstatus) ? WTERMSIG(wstatus) : 0;
            bool probeOk = (exitCode == 0 || exitCode == 124);
            LOG(INFO) << "RGB timing: probe attempt " << attempt << " took "
                      << elapsedMs(probeBegin) << " ms (exit=" << exitCode
                      << ")";

            if (probeOk) {
                LOG(INFO) << "Argus probe OK (exit=" << exitCode << ", attempt "
                          << attempt << "), creating pipeline.";
                m_argusProbeOk = true;
                break;
            }

            LOG(WARNING) << "Argus probe failed (exit=" << exitCode
                         << " sig=" << sigNum << ", attempt " << attempt << "/"
                         << kMaxProbeAttempts << ")";

            if (attempt < kMaxProbeAttempts) {
                // Escalating settle time (1s, 2s, 3s) lets the daemon reclaim a
                // stale post-crash CaptureSession before the next probe.
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(1000 * attempt));
            }
        }

        if (!m_argusProbeOk) {
            LOG(WARNING) << "Argus probe failed after " << kMaxProbeAttempts
                         << " attempts \xe2\x80\x94 RGB capture unavailable; "
                            "continuing in depth-only mode.";
            return Status::GENERIC_ERROR;
        }
    }

    // -----------------------------------------------------------------------
    // Step 2: Create the GStreamer pipeline and start it on a fresh thread
    // (V4L2: start the backend that initialize() prepared).
    //
    // initialize() builds the nvarguscamerasrc pipeline (NULL state).
    // Running startPipeline() (NULL→PAUSED→PLAYING) on a fresh std::thread
    // avoids the EGL API binding conflict when the caller is the tof-viewer
    // main thread (eglBindAPI EGL_OPENGL_API vs nvarguscamerasrc's ES API).
    LOG(INFO) << "AR0234Sensor using backend: " << m_backend->getBackendName();
    if (useArgus) {
        LOG(INFO) << "RGB timing: probe step finished " << elapsedMs(startBegin)
                  << " ms after start() began";
    }

    // GStreamer only: retry the initialize()+start() sequence a few times.
    // When the viewer is closed and reopened, the nvargus-daemon occasionally
    // still holds a stale CaptureSession from the previous process, so the
    // first pipeline start can fail. A full teardown (m_backend->stop()
    // destroys the pipeline) followed by a short settle lets the daemon release
    // the session before we retry.
    constexpr int kMaxStartAttempts = 3;
    const int maxStartAttempts = useArgus ? kMaxStartAttempts : 1;
    for (int attempt = 1; attempt <= maxStartAttempts; ++attempt) {
        bool startResult = false;
        const auto pipelineBegin = std::chrono::steady_clock::now();

        if (useArgus) {
            // initialize() builds the nvarguscamerasrc pipeline (NULL state).
            // Running startPipeline() (NULL→PAUSED→PLAYING) on a fresh
            // std::thread avoids the EGL API binding conflict when the caller
            // is the tof-viewer main thread (eglBindAPI EGL_OPENGL_API vs
            // nvarguscamerasrc's ES API).
            if (m_backend->initialize(m_config)) {
                std::thread startThread([this, &startResult]() {
                    startResult = m_backend->start();
                });
                startThread.join();
            } else {
                LOG(ERROR)
                    << "Failed to initialize AR0234 sensor backend (attempt "
                    << attempt << "/" << maxStartAttempts << ")";
            }
        } else {
            // Already initialized by initialize(), checked at the top of start().
            startResult = m_backend->start();
        }

        if (startResult) {
            LOG(INFO) << "RGB timing: "
                      << (useArgus ? "pipeline initialize+start"
                                   : "V4L2 stream on until sensor armed")
                      << " took " << elapsedMs(pipelineBegin)
                      << " ms; total start() " << elapsedMs(startBegin)
                      << " ms";
            LOG(INFO) << "AR0234Sensor started capturing with "
                      << m_backend->getBackendName() << " backend"
                      << (attempt > 1 ? " (after retry)" : "");
            return Status::OK;
        }

        // Tear down completely so the Argus session is released, then let the
        // daemon settle before the next attempt.
        m_backend->stop();
        if (attempt < maxStartAttempts) {
            LOG(WARNING) << "RGB start failed (attempt " << attempt << "/"
                         << maxStartAttempts
                         << "), retrying after Argus settle...";
            std::this_thread::sleep_for(std::chrono::milliseconds(700));
        }
    }

    LOG(ERROR) << "Failed to start AR0234 sensor capture after "
               << maxStartAttempts << " attempts";
    return Status::GENERIC_ERROR;
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

Status RGBSensor::discardFrame(uint32_t timeoutMs) {
    if (!isCapturing()) {
        return Status::UNAVAILABLE;
    }
    return m_backend->discardFrame(timeoutMs) ? Status::OK
                                              : Status::GENERIC_ERROR;
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
