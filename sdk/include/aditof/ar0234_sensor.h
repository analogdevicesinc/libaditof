/**
 * AR0234 RGB Camera Sensor
 * 
 * High-level sensor class for AR0234 1920x1200 RGB camera.
 * This file contains everything needed for AR0234 camera:
 * - Sensor configuration and frame structures
 * - Internal backend interface (V4L2)
 * - Main RGBSensor class for applications
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
 * @brief Pixel layout of RGBFrame::data
 */
enum class RGBPixelFormat {
    BGR,  ///< 3 bytes per pixel, B G R order (Bayer V4L2 backend)
    UYVY, ///< YUV 4:2:2 packed, 2 bytes per pixel, U Y0 V Y1 order
    YUYV  ///< YUV 4:2:2 packed, 2 bytes per pixel, Y0 U Y1 V order
};

/**
 * @brief Per-frame metadata reported by the RGB camera (GW5200 embedded data).
 *
 * Timestamps are on the camera ISP clock (ms), not the host clock.
 */
struct RgbFrameMetadata {
    uint32_t outputFrameCount = 0;
    uint32_t inputFrameCount = 0;
    uint32_t inputFrameErrorCount = 0;
    uint64_t outputTimestampMs = 0;
    uint64_t inputTimestampMs = 0;
    uint32_t ispState = 0;
    float ispTemperatureC = 0.0f;
    float frameRate = 0.0f;
    uint16_t vts = 0;
    uint16_t integrationLines[4] = {};
    float totalGain[4] = {};
    float sensorGain[4] = {};
    float ispGain[4] = {};
};

/**
 * @brief Frame captured from RGB sensor
 */
struct RGBFrame {
    std::vector<uint8_t> data; ///< Pixel data, laid out as described by format
    uint32_t width;            ///< Frame width in pixels
    uint32_t height;           ///< Frame height in pixels
    uint32_t stride;           ///< Bytes per row of data
    uint32_t sequence;         ///< V4L2 frame sequence number
    uint64_t monotonicUs;      ///< CLOCK_MONOTONIC time (us), 0 if unknown
    RGBPixelFormat format;     ///< Pixel layout of data
    std::vector<uint8_t> embeddedData; ///< GW5 ancillary data, empty if none
    RgbFrameMetadata metadata;         ///< Decoded embeddedData
    bool hasMetadata;                  ///< metadata is valid

    RGBFrame()
        : width(0), height(0), stride(0), sequence(0), monotonicUs(0),
          format(RGBPixelFormat::BGR), hasMetadata(false) {}

    /// Get frame size in bytes
    size_t size() const { return data.size(); }

    /// Check if frame contains valid data
    bool isValid() const { return !data.empty() && width > 0 && height > 0; }
};

/**
 * @brief Camera mode index (maps to DTS modes)
 */
enum class RGBMode {
    MODE_0_1920x1200, ///< Mode 0: 1920x1200 (DTS mode0)
    MODE_1_1920x1080, ///< Mode 1: 1920x1080 (DTS mode1)
    MODE_2_1280x720,  ///< Mode 2: 1280x720 (DTS mode2)
    MODE_CUSTOM       ///< Custom mode (specify resolution manually)
};

/**
 * @brief Sensor mode definition (matches DTS active_w/active_h)
 */
struct RGBModeConfig {
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
static const RGBModeConfig RGB_MODES[] = {
    {0, 1920, 1200, 60, "Mode 0: 1920x1200@60fps"}, // DTS mode0
    {1, 1920, 1080, 60, "Mode 1: 1920x1080@60fps"}, // DTS mode1
    {2, 1280, 720, 120, "Mode 2: 1280x720@120fps"}, // DTS mode2
};

/**
 * @brief Configuration for RGB camera sensor
 */
struct RGBSensorConfig {
    int sensorId; ///< Camera sensor ID (0, 1, etc. for multi-camera)
    int mode;     ///< Camera mode index (0, 1, 2) - maps to DTS modes
    int width;    ///< Frame width in pixels (active_w from DTS)
    int height;   ///< Frame height in pixels (active_h from DTS)
    int fps;      ///< Target frames per second
    std::string
        devicePath; ///< V4L2 device path (e.g., "/dev/video0", "/dev/video1")

    RGBSensorConfig()
        : sensorId(0), mode(0), width(1920), height(1200), fps(60),
          devicePath("/dev/video0") {}

    /**
     * @brief Create config from predefined mode enum
     */
    static RGBSensorConfig fromMode(RGBMode modeEnum, int sensorId = 0) {
        RGBSensorConfig config;
        config.sensorId = sensorId;

        if (modeEnum != RGBMode::MODE_CUSTOM) {
            int mode_idx = static_cast<int>(modeEnum);
            const auto &modeConfig = RGB_MODES[mode_idx];
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
            return RGB_MODES[mode].description;
        }
        return "Custom: " + std::to_string(width) + "x" +
               std::to_string(height) + "@" + std::to_string(fps) + "fps";
    }
};

// ============================================================================
// SECTION 2: Internal Backend Interface
// (Used internally by RGBSensor - applications don't use this directly)
// ============================================================================

/**
 * @brief Backend type - which capture method is being used
 */
enum class RGBBackend {
    V4L2, ///< Direct V4L2 capture of the raw Bayer stream
    UNKNOWN
};

/**
 * @brief Internal interface for RGB camera backends
 * 
 * This is used internally by RGBSensor to communicate with the capture
 * backend. Applications should use RGBSensor class instead of using this
 * interface directly.
 */
class RGBBackend_Internal {
  public:
    virtual ~RGBBackend_Internal() = default;

    /**
     * @brief Get backend type
     */
    virtual RGBBackend getBackendType() const = 0;

    /**
     * @brief Get backend name as string
     */
    virtual std::string getBackendName() const = 0;

    /**
     * @brief Initialize the backend with configuration
     */
    virtual bool initialize(const RGBSensorConfig &config) = 0;

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
    virtual bool getFrame(RGBFrame &frame, uint32_t timeoutMs = 1000) = 0;

    /**
     * @brief Get statistics string
     */
    virtual std::string getStatistics() const = 0;
};

// ============================================================================
// SECTION 3: Main RGB Sensor Class (Application Interface)
// ============================================================================

/**
 * @class RGBSensor
 * @brief Main class for RGB camera sensor
 * 
 * This is the class applications should use. It provides a simple sensor-style
 * interface similar to other sensors in the SDK (like Adsd3500Sensor).
 * 
 * Internally, it uses a V4L2 backend to capture frames.
 * 
 * Usage Example:
 * ```cpp
 * RGBSensor sensor;
 * 
 * RGBSensorConfig config;
 * config.sensorId = 0;
 * config.width = 1920;
 * config.height = 1200;
 * config.fps = 60;
 * 
 * if (sensor.open(config) == Status::OK) {
 *     sensor.initialize(); // before start(), e.g. in setMode()
 *     sensor.start();
 *     
 *     RGBFrame frame;
 *     while (capturing) {
 *         if (sensor.getFrame(frame) == Status::OK) {
 *             // Process frame.data, laid out as frame.format
 *         }
 *     }
 *     
 *     sensor.stop();
 *     sensor.close();
 * }
 * ```
 */
class RGBSensor {
  public:
    RGBSensor();
    ~RGBSensor();

    /**
     * @brief Open and initialize the sensor
     * @param config Sensor configuration
     * @return Status::OK on success
     */
    Status open(const RGBSensorConfig &config = RGBSensorConfig());

    /**
     * @brief Close the sensor
     * @return Status::OK on success
     */
    Status close();

    /**
     * @brief Prepare the backend before start(), e.g. in setMode()
     *
     * Opens and configures the V4L2 device.
     * @return Status::OK on success, Status::BUSY while capturing
     */
    Status initialize();

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
    Status getFrame(RGBFrame &frame, uint32_t timeoutMs = 1000);

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
    RGBSensorConfig getConfig() const { return m_config; }

    /**
     * @brief Get the type of the backend being used
     * @return Backend type, RGBBackend::UNKNOWN if the sensor is not open
     */
    RGBBackend getBackendType() const;

    /**
     * @brief Get backend name being used
     * @return Backend name (e.g., "V4L2")
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
    std::unique_ptr<RGBBackend_Internal>
        m_backend;             ///< Internal backend implementation
    RGBSensorConfig m_config;  ///< Current configuration
    bool m_isOpen;             ///< Open state flag
    bool m_backendInitialized; ///< True after initialize() prepared the backend
    uint64_t m_frameCount;     ///< Total frames captured
    uint64_t m_badMetadata = 0; ///< Frames whose embedded data failed to parse
};

} // namespace aditof

#endif // HAS_RGB_CAMERA

#endif // AR0234_SENSOR_H
