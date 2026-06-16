/**
 * AR0234 RGB Camera Sensor
 * 
 * High-level sensor class for AR0234 1920x1200 RGB camera.
 * This file contains everything needed for AR0234 camera:
 * - Sensor configuration and frame structures
 * - Internal backend interface (for GStreamer/V4L2/nvargus)
 * - Main AR0234Sensor class for applications
 * 
 * Copyright (c) 2025 Analog Devices, Inc.
 */

#ifndef AR0234_SENSOR_H
#define AR0234_SENSOR_H

#ifdef HAS_RGB_CAMERA

#include <aditof/status_definitions.h>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace aditof {

// ============================================================================
// SECTION 1: Frame and Configuration Structures
// ============================================================================

/**
 * @brief Frame captured from AR0234 sensor
 */
struct AR0234Frame {
    std::vector<uint8_t>
        data;           ///< Raw pixel data (BGRx format, 4 bytes per pixel)
    uint32_t width;     ///< Frame width in pixels
    uint32_t height;    ///< Frame height in pixels
    uint64_t timestamp; ///< Frame timestamp in microseconds

    AR0234Frame() : width(0), height(0), timestamp(0) {}

    /// Get frame size in bytes
    size_t size() const { return data.size(); }

    /// Check if frame contains valid data
    bool isValid() const { return !data.empty() && width > 0 && height > 0; }
};

/**
 * @brief Camera mode index (maps to DTS modes)
 */
enum class AR0234Mode {
    MODE_0_1920x1200, ///< Mode 0: 1920x1200 (DTS mode0)
    MODE_1_1920x1080, ///< Mode 1: 1920x1080 (DTS mode1)
    MODE_2_1280x720,  ///< Mode 2: 1280x720 (DTS mode2)
    MODE_CUSTOM       ///< Custom mode (specify resolution manually)
};

/**
 * @brief Sensor mode definition (matches DTS active_w/active_h)
 */
struct AR0234ModeConfig {
    int mode_index; ///< Camera mode index (0, 1, 2)
    int width;      ///< active_w from DTS
    int height;     ///< active_h from DTS
    int fps;        ///< Frame rate (sensor-dependent)
    const char *description;
};

/**
 * @brief Available sensor modes (matching DTS configuration)
 * 
 * These modes correspond to the device tree modes:
 * mode0: 1920x1200@60fps
 * mode1: 1920x1080@60fps  
 * mode2: 1280x720@120fps
 */
static const AR0234ModeConfig AR0234_MODES[] = {
    {0, 1920, 1200, 60, "Mode 0: 1920x1200@60fps"}, // DTS mode0
    {1, 1920, 1080, 60, "Mode 1: 1920x1080@60fps"}, // DTS mode1
    {2, 1280, 720, 120, "Mode 2: 1280x720@120fps"}, // DTS mode2
};

/**
 * @brief Configuration for AR0234 RGB camera sensor
 */
struct AR0234SensorConfig {
    int sensorId; ///< Camera sensor ID (0, 1, etc. for multi-camera)
    int mode;     ///< Camera mode index (0, 1, 2) - maps to DTS modes
    int width;    ///< Frame width in pixels (active_w from DTS)
    int height;   ///< Frame height in pixels (active_h from DTS)
    int fps;      ///< Target frames per second
    std::string
        devicePath; ///< V4L2 device path (e.g., "/dev/video0", "/dev/video1")

    AR0234SensorConfig()
        : sensorId(0), mode(0), width(1920), height(1200), fps(60),
          devicePath("/dev/video0") {}

    /**
     * @brief Create config from predefined mode enum
     */
    static AR0234SensorConfig fromMode(AR0234Mode modeEnum, int sensorId = 0) {
        AR0234SensorConfig config;
        config.sensorId = sensorId;

        if (modeEnum != AR0234Mode::MODE_CUSTOM) {
            int mode_idx = static_cast<int>(modeEnum);
            const auto &modeConfig = AR0234_MODES[mode_idx];
            config.mode = modeConfig.mode_index;
            config.width = modeConfig.width;
            config.height = modeConfig.height;
            config.fps = modeConfig.fps;
        }

        return config;
    }

    /**
     * @brief Get description of current mode
     */
    std::string getModeDescription() const {
        // Check if mode index is valid
        if (mode >= 0 && mode < 3) {
            return AR0234_MODES[mode].description;
        }
        return "Custom: " + std::to_string(width) + "x" +
               std::to_string(height) + "@" + std::to_string(fps) + "fps";
    }
};

// ============================================================================
// SECTION 2: Internal Backend Interface
// (Used internally by AR0234Sensor - applications don't use this directly)
// ============================================================================

/**
 * @brief Backend type - which capture method is being used
 */
enum class AR0234Backend {
    GSTREAMER, ///< GStreamer-based capture (current)
    V4L2,      ///< Direct V4L2 capture (future)
    NVARGUS,   ///< NVIDIA Argus direct API (future)
    UNKNOWN
};

/**
 * @brief Internal interface for AR0234 camera backends
 * 
 * This is used internally by AR0234Sensor to communicate with different
 * capture backends (GStreamer, V4L2, nvargus). Applications should use
 * AR0234Sensor class instead of using this interface directly.
 * 
 * Each backend (GStreamer, V4L2, nvargus) implements these methods.
 */
class AR0234Backend_Internal {
  public:
    virtual ~AR0234Backend_Internal() = default;

    /**
     * @brief Get backend type
     */
    virtual AR0234Backend getBackendType() const = 0;

    /**
     * @brief Get backend name as string
     */
    virtual std::string getBackendName() const = 0;

    /**
     * @brief Initialize the backend with configuration
     */
    virtual bool initialize(const AR0234SensorConfig &config) = 0;

    /**
     * @brief Start frame capture
     */
    virtual bool start() = 0;

    /**
     * @brief Stop frame capture
     */
    virtual bool stop() = 0;

    /**
     * @brief Check if currently capturing
     */
    virtual bool isRunning() const = 0;

    /**
     * @brief Get next frame (blocking with timeout)
     */
    virtual bool getFrame(AR0234Frame &frame, uint32_t timeoutMs = 1000) = 0;

    /**
     * @brief Get statistics string
     */
    virtual std::string getStatistics() const = 0;
};

// ============================================================================
// SECTION 3: Main AR0234 Sensor Class (Application Interface)
// ============================================================================

/**
 * @class AR0234Sensor
 * @brief Main class for AR0234 RGB camera sensor
 * 
 * This is the class applications should use. It provides a simple sensor-style
 * interface similar to other sensors in the SDK (like Adsd3500Sensor).
 * 
 * Internally, it uses one of the available backends (GStreamer, V4L2, or nvargus)
 * to actually capture frames, but you don't need to know which one.
 * 
 * Usage Example:
 * ```cpp
 * AR0234Sensor sensor;
 * 
 * AR0234SensorConfig config;
 * config.sensorId = 0;
 * config.width = 1920;
 * config.height = 1200;
 * config.fps = 60;
 * 
 * if (sensor.open(config) == Status::OK) {
 *     sensor.start();
 *     
 *     AR0234Frame frame;
 *     while (capturing) {
 *         if (sensor.getFrame(frame) == Status::OK) {
 *             // Process frame.data (1920x1200 BGRx pixels)
 *         }
 *     }
 *     
 *     sensor.stop();
 *     sensor.close();
 * }
 * ```
 */
class AR0234Sensor {
  public:
    AR0234Sensor();
    ~AR0234Sensor();

    /**
     * @brief Open and initialize the sensor
     * @param config Sensor configuration
     * @return Status::OK on success
     */
    Status open(const AR0234SensorConfig &config = AR0234SensorConfig());

    /**
     * @brief Close the sensor
     * @return Status::OK on success
     */
    Status close();

    /**
     * @brief Start frame capture
     * @return Status::OK on success
     */
    Status start();

    /**
     * @brief Stop frame capture
     * @return Status::OK on success
     */
    Status stop();

    /**
     * @brief Get next frame from sensor
     * @param frame Output frame structure
     * @param timeoutMs Timeout in milliseconds (default: 1000ms)
     * @return Status::OK on success, Status::GENERIC_ERROR on timeout/error
     */
    Status getFrame(AR0234Frame &frame, uint32_t timeoutMs = 1000);

    /**
     * @brief Check if sensor is currently capturing
     * @return true if capturing, false otherwise
     */
    bool isCapturing() const;

    /**
     * @brief Check if sensor is open
     * @return true if open, false otherwise
     */
    bool isOpen() const { return m_isOpen; }

    /**
     * @brief Get current sensor configuration
     * @return Sensor configuration
     */
    AR0234SensorConfig getConfig() const { return m_config; }

    /**
     * @brief Get backend name being used
     * @return Backend name (e.g., "GStreamer", "V4L2", "NVARGUS")
     */
    std::string getBackendName() const;

    /**
     * @brief Get sensor statistics
     * @return String with performance metrics and status
     */
    std::string getStatistics() const;

    /**
     * @brief Get total number of frames captured
     * @return Frame count
     */
    uint64_t getFrameCount() const { return m_frameCount; }

  private:
    std::unique_ptr<AR0234Backend_Internal>
        m_backend;               ///< Internal backend implementation
    AR0234SensorConfig m_config; ///< Current configuration
    bool m_isOpen;               ///< Open state flag
    uint64_t m_frameCount;       ///< Total frames captured
};

} // namespace aditof

#endif // HAS_RGB_CAMERA

#endif // AR0234_SENSOR_H
