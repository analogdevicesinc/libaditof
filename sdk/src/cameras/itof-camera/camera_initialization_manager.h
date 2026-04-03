/*
 * MIT License
 *
 * Copyright (c) 2025 Analog Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef CAMERA_INITIALIZATION_MANAGER_H
#define CAMERA_INITIALIZATION_MANAGER_H

#include <aditof/camera_definitions.h>
#include <aditof/depth_sensor_interface.h>
#include <aditof/status_definitions.h>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// Include tofi headers for TofiXYZDealiasData type
#include "tofi/tofi_camera_intrinsics.h"

namespace aditof {

// Forward declarations
class Adsd3500Controller;
class CalibrationManager;
class CameraConfiguration;
struct CameraDetails;
struct DepthSensorModeDetails;

/**
 * @class CameraInitializationManager
 * @brief Manages camera initialization workflow and sensor configuration.
 *
 * Handles the complex initialization sequence for ToF cameras including:
 * - Sensor opening and connectivity validation
 * - Imager type detection (ADSD3100, ADSD3030, ADTF3080, ADTF3066)
 * - Mode discovery and capability enumeration
 * - Intrinsic parameters and dealias data acquisition
 * - CCB (Configuration Calibration Block) loading for non-ISP modes
 * - Hardware configuration (FSYNC, MIPI speed, deskew, temperature compensation)
 * - Serial number retrieval
 *
 * This manager orchestrates interactions between the depth sensor, calibration manager,
 * controller, and configuration subsystems to ensure proper camera setup before streaming.
 */
class CameraInitializationManager {
  public:
    /**
     * @brief Constructs initialization manager with required subsystems.
     *
     * @param depthSensor Interface to underlying depth sensor hardware
     * @param calibrationMgr Manager for calibration data (intrinsics, CCB, etc.)
     * @param controller ADSD3500 hardware controller for register access
     * @param config Camera configuration settings (JSON-based parameters)
     */
    CameraInitializationManager(
        std::shared_ptr<DepthSensorInterface> depthSensor,
        CalibrationManager *calibrationMgr, Adsd3500Controller *controller,
        CameraConfiguration *config);

    /**
     * @brief Initializes camera for online operation.
     *
     * Performs the complete initialization sequence:
     * 1. Opens depth sensor interface (V4L2 drivers)
     * 2. Detects imager type from hardware control
     * 3. Enumerates available modes and retrieves their details
     * 4. Reads intrinsic calibration and dealias parameters for each mode
     * 5. Loads CCB data if any mode requires non-ISP depth computation
     * 6. Configures FSYNC, MIPI output speed, and deskew settings
     * 7. Enables temperature compensation and edge confidence (if configured)
     * 8. Retrieves module serial number (if firmware supports)
     *
     * @param[out] cameraDetails Populated with firmware version, intrinsics, imager type
     * @param[out] availableModes List of supported mode numbers
     * @param[out] availableSensorModeDetails Detailed mode information (resolution, framerate, etc.)
     * @param[out] imagerType Detected imager type
     * @param[out] adsd3500FwVersion Firmware version string (e.g., "4.0.1.0")
     * @param[out] adsd3500FwHash Firmware git hash
     * @param[in] configFilepath Optional path to JSON configuration file
     *
     * @return Status::OK on successful initialization
     * @return Status::UNAVAILABLE if ADSD3500 is not enabled in target mode
     * @return Status::GENERIC_ERROR on sensor open failure, mode query failure,
     *         intrinsics read failure, or configuration errors
     *
     * @note This function must be called before start() or setMode()
     * @note Requires depth sensor to be in a valid state (not already streaming)
     */
    aditof::Status initializeOnlineMode(
        CameraDetails &cameraDetails, std::vector<uint8_t> &availableModes,
        std::vector<DepthSensorModeDetails> &availableSensorModeDetails,
        ImagerType &imagerType,
        std::pair<std::string, std::string> &adsd3500FwVersion,
        const std::string &configFilepath);

    /**
     * @brief Initializes camera for offline (playback) operation.
     *
     * Minimal initialization for playback from recorded files. Sets up
     * offline mode flag without hardware interaction.
     *
     * @return Status::OK (offline initialization always succeeds)
     */
    aditof::Status initializeOfflineMode();

  private:
    /**
     * @brief Opens the depth sensor hardware interface.
     *
     * Establishes connection to V4L2 driver and validates sensor availability.
     * Sets devStarted flag on success.
     *
     * @param[in,out] devStarted Flag indicating if device has been opened
     * @return Status::OK if sensor opens successfully
     * @return Status::GENERIC_ERROR on open failure
     */
    aditof::Status openSensor(bool &devStarted);

    /**
     * @brief Detects imager type from sensor control interface.
     *
     * Queries "imagerType" control and maps to ImagerType enum.
     * Supports ADSD3100, ADSD3030, ADTF3080, ADTF3066.
     *
     * @param[out] detectedType Populated with detected imager type
     * @return Status::OK if imager type detected successfully
     * @return Status::UNAVAILABLE if imager type unknown or unsupported
     */
    aditof::Status detectImagerType(ImagerType &detectedType);

    /**
     * @brief Discovers available sensor modes and retrieves their details.
     *
     * Queries sensor for mode list and fetches detailed information
     * (resolution, framerate, frame content) for each mode.
     *
     * @param[out] modes List of available mode numbers
     * @param[out] modeDetails Detailed information for each mode
     * @return Status::OK if modes enumerated successfully
     * @return Status::GENERIC_ERROR on query failure
     */
    aditof::Status
    discoverAvailableModes(std::vector<uint8_t> &modes,
                           std::vector<DepthSensorModeDetails> &modeDetails);

    /**
     * @brief Reads intrinsic calibration and dealias parameters for a mode.
     *
     * Uses ADSD3500 payload commands to retrieve:
     * - Camera intrinsics (56 bytes via command 0x01)
     * - Dealias parameters (32 bytes via command 0x02)
     *
     * @param[in] modeNumber Mode to read parameters for
     * @param[out] dealiasData Combined intrinsics + dealias data
     * @param[out] cameraDetails Updated with intrinsics for the mode
     * @return Status::OK if parameters read successfully
     * @return Status::GENERIC_ERROR on read failure
     */
    aditof::Status readModeCalibrationData(uint8_t modeNumber,
                                           TofiXYZDealiasData &dealiasData,
                                           CameraDetails &cameraDetails);

    /**
     * @brief Loads CCB data for non-ISP depth computation modes.
     *
     * Checks if any mode requires non-ISP processing (depthComputeIspEnable != "1"),
     * and if so, reads raw CCB data from ADSD3500 sensor memory.
     *
     * @param[in] modes List of available modes to check
     * @return Status::OK if CCB loaded successfully or not needed
     * @return Status::GENERIC_ERROR on CCB read failure (warning only, continues init)
     */
    aditof::Status loadCCBDataIfNeeded(const std::vector<uint8_t> &modes);

    /**
     * @brief Configures hardware settings (FSYNC, MIPI, deskew, temp compensation).
     *
     * Applies configuration values from JSON or platform defaults:
     * - FSYNC toggle mode
     * - MIPI output speed
     * - Deskew enable/disable
     * - Temperature compensation
     * - Edge confidence
     *
     * @return Status::OK if all configurations applied successfully
     * @return Status::GENERIC_ERROR on configuration failure
     */
    aditof::Status applyHardwareConfiguration();

    /**
     * @brief Reads firmware version from ADSD3500.
     *
     * Retrieves firmware version string and git hash.
     *
     * @param[out] fwVersion Firmware version (e.g., "4.0.1.0")
     * @param[out] fwHash Firmware git hash
     * @return Status::OK if version read successfully
     * @return Status::GENERIC_ERROR on read failure
     */
    aditof::Status readFirmwareVersion(std::string &fwVersion,
                                       std::string &fwHash);

    /**
     * @brief Reads module serial number.
     *
     * Attempts to read serial number if supported by firmware.
     *
     * @param[out] serialNumber Module serial number string
     * @return Status::OK if serial read successfully
     * @return Status::UNAVAILABLE if firmware doesn't support serial reading
     * @return Status::GENERIC_ERROR on read failure
     */
    aditof::Status readSerialNumber(std::string &serialNumber);

  private:
    std::shared_ptr<DepthSensorInterface> m_depthSensor;
    CalibrationManager *m_calibrationMgr;
    Adsd3500Controller *m_controller;
    CameraConfiguration *m_config;
};

} // namespace aditof

#endif // CAMERA_INITIALIZATION_MANAGER_H
