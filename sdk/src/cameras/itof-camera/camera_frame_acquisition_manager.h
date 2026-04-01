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

#ifndef CAMERA_FRAME_ACQUISITION_MANAGER_H
#define CAMERA_FRAME_ACQUISITION_MANAGER_H

#include <aditof/depth_sensor_interface.h>
#include <aditof/frame.h>
#include <aditof/status_definitions.h>
#include <memory>

namespace aditof {

// Forward declarations
class CalibrationManager;
class CameraConfiguration;
struct DepthSensorModeDetails;

/**
 * @class CameraFrameAcquisitionManager
 * @brief Manages frame acquisition, post-processing, and metadata handling.
 *
 * Handles the complete frame request workflow:
 * - Frame buffer validation and allocation
 * - First frame dropping logic (optional)
 * - Raw frame acquisition from depth sensor
 * - Raw bypass mode handling (Bayer data passthrough)
 * - XYZ point cloud computation from depth
 * - Disabled data type zeroing (depth, AB, confidence)
 * - Metadata extraction from AB frame header
 * - Metadata generation and population
 *
 * This manager coordinates interactions between the frame buffer, depth sensor,
 * calibration data, and configuration settings to deliver complete frames.
 */
class CameraFrameAcquisitionManager {
  public:
    /**
     * @brief Constructs a frame acquisition manager instance.
     *
     * @param[in] depthSensor Shared pointer to the depth sensor interface.
     * @param[in] calibrationMgr Pointer to calibration manager for XYZ tables.
     * @param[in] config Pointer to camera configuration for acquisition settings.
     */
    CameraFrameAcquisitionManager(
        std::shared_ptr<DepthSensorInterface> depthSensor,
        CalibrationManager *calibrationMgr, CameraConfiguration *config);

    /**
     * @brief Requests and processes a frame from the sensor.
     *
     * Complete frame acquisition and processing sequence:
     * 1. Validates frame buffer and updates frame details if needed
     * 2. Drops first frame if configured (reduces sensor initialization artifacts)
     * 3. Acquires raw frame from depth sensor
     * 4. For raw bypass mode: returns immediately (no post-processing)
     * 5. Computes XYZ point cloud if enabled
     * 6. Zeros disabled data types (depth/AB/confidence)
     * 7. Extracts or generates metadata
     * 8. Copies metadata to frame buffer
     *
     * @param[in,out] frame Pointer to frame object to populate with data.
     * @param[in] index Frame index for multi-frame acquisition (default 0).
     * @param[in] frameType Current frame type configuration.
     * @param[in] modeDetails Current mode configuration details.
     * @param[in] isOffline True if in offline/playback mode.
     * @param[in] isPcmFrame True if frame contains PCM (phase correlation) data.
     * @param[in] depthEnabled True if depth data should be populated.
     * @param[in] abEnabled True if AB (amplitude) data should be populated.
     * @param[in] confEnabled True if confidence data should be populated.
     * @param[in] xyzEnabled True if XYZ point cloud should be computed.
     * @param[in] confBitsPerPixel Confidence map bit depth (4, 8, or 16).
     * @param[in] abBitsPerPixel AB frame bit depth (8 or 16).
     * @param[in] depthBitsPerPixel Depth frame bit depth (typically 16).
     * @param[in,out] dropFrameOnce Reference to drop-frame state flag.
     *
     * @return Status::OK if frame acquired and processed successfully;
     *         Status::INVALID_ARGUMENT if frame pointer is null;
     *         error codes if sensor acquisition or processing fails.
     *
     * @note For raw bypass mode, only raw Bayer data is returned (no XYZ, metadata, etc.).
     * @note XYZ computation requires valid calibration data in CalibrationManager.
     */
    Status requestFrame(Frame *frame, uint32_t index,
                        const FrameDetails &frameType,
                        const DepthSensorModeDetails &modeDetails,
                        bool isOffline, bool isPcmFrame, bool depthEnabled,
                        bool abEnabled, bool confEnabled, bool xyzEnabled,
                        uint8_t confBitsPerPixel, uint8_t abBitsPerPixel,
                        uint8_t depthBitsPerPixel, bool &dropFrameOnce);

  private:
    /**
     * @brief Validates and updates frame buffer configuration if needed.
     *
     * @param[in,out] frame Frame object to validate/update.
     * @param[in] frameType Expected frame type configuration.
     * @param[in] confBitsPerPixel Confidence bit depth.
     * @param[in] abBitsPerPixel AB bit depth.
     *
     * @return Status::OK if frame is valid or updated successfully.
     */
    Status validateAndUpdateFrame(Frame *frame, const FrameDetails &frameType,
                                  uint8_t confBitsPerPixel,
                                  uint8_t abBitsPerPixel);

    /**
     * @brief Gets pointer to frame data buffer (depth or AB depending on mode).
     *
     * @param[in] frame Frame object to query.
     * @param[in] isPcmFrame True if PCM frame (use AB buffer).
     * @param[out] dataLocation Pointer set to frame buffer address.
     *
     * @return Status::OK if buffer retrieved successfully.
     */
    Status getFrameDataLocation(Frame *frame, bool isPcmFrame,
                                uint16_t **dataLocation);

    /**
     * @brief Drops first frame from sensor if configured.
     *
     * @param[in] dataLocation Frame buffer for dropped frame.
     * @param[in] index Frame index.
     * @param[in,out] dropFrameOnce State flag updated after drop.
     *
     * @return Status::OK if frame dropped successfully.
     */
    Status dropFirstFrameIfNeeded(uint16_t *dataLocation, uint32_t index,
                                  bool &dropFrameOnce, bool isOffline);

    /**
     * @brief Acquires raw frame data from depth sensor.
     *
     * @param[out] dataLocation Buffer to receive frame data.
     * @param[in] index Frame index.
     *
     * @return Status::OK if frame acquired successfully.
     */
    Status acquireRawFrame(uint16_t *dataLocation, uint32_t index);

    /**
     * @brief Computes XYZ point cloud from depth frame.
     *
     * @param[in] frame Frame object containing depth data.
     * @param[in] modeDetails Mode configuration for resolution info.
     *
     * @return Status::OK if XYZ computed successfully.
     */
    Status computeXYZIfEnabled(Frame *frame, bool xyzEnabled, bool depthEnabled,
                               const DepthSensorModeDetails &modeDetails);

    /**
     * @brief Zeros depth frame if depth is disabled.
     *
     * @param[in,out] frame Frame object to modify.
     * @param[in] modeDetails Mode configuration for resolution.
     *
     * @return Status::OK if operation successful.
     */
    Status zeroDepthIfDisabled(Frame *frame, bool depthEnabled,
                               const DepthSensorModeDetails &modeDetails);

    /**
     * @brief Zeros AB frame if AB is disabled.
     *
     * @param[in,out] frame Frame object to modify.
     * @param[in] modeDetails Mode configuration for resolution.
     *
     * @return Status::OK if operation successful.
     */
    Status zeroABIfDisabled(Frame *frame, bool abEnabled,
                            const DepthSensorModeDetails &modeDetails);

    /**
     * @brief Extracts or generates frame metadata.
     *
     * If metadata-in-AB is enabled and available, extracts from AB header.
     * Otherwise, generates metadata from current configuration.
     *
     * @param[in] frame Frame object to read AB data from.
     * @param[out] metadata Metadata structure to populate.
     * @param[in] modeDetails Mode configuration.
     * @param[in] abEnabled True if AB frame available.
     * @param[in] isPcmFrame True if PCM frame.
     * @param[in] xyzEnabled True if XYZ enabled.
     * @param[in] depthBitsPerPixel Depth bit depth.
     * @param[in] abBitsPerPixel AB bit depth.
     * @param[in] confBitsPerPixel Confidence bit depth.
     *
     * @return Status::OK if metadata extracted/generated successfully.
     */
    Status extractOrGenerateMetadata(Frame *frame, Metadata &metadata,
                                     const DepthSensorModeDetails &modeDetails,
                                     bool abEnabled, bool isPcmFrame,
                                     bool xyzEnabled, uint8_t depthBitsPerPixel,
                                     uint8_t abBitsPerPixel,
                                     uint8_t confBitsPerPixel);

    /**
     * @brief Writes metadata to frame buffer.
     *
     * @param[in,out] frame Frame object to write metadata to.
     * @param[in] metadata Metadata to write.
     *
     * @return Status::OK if metadata written successfully.
     */
    Status writeMetadataToFrame(Frame *frame, const Metadata &metadata);

  private:
    std::shared_ptr<DepthSensorInterface> m_depthSensor;
    CalibrationManager *m_calibrationMgr;
    CameraConfiguration *m_config;
};

} // namespace aditof

#endif // CAMERA_FRAME_ACQUISITION_MANAGER_H
