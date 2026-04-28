/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 */
#ifndef RECORDING_MANAGER_H
#define RECORDING_MANAGER_H

#include <aditof/camera_definitions.h>
#include <aditof/depth_sensor_interface.h>
#include <aditof/status_definitions.h>

#include "tofi/tofi_camera_intrinsics.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace aditof {

// Forward declarations
class CalibrationManager;
class CameraConfiguration;

/**
 * @brief Manages recording and playback of ToF camera frame data.
 *
 * Handles serialization of frame metadata, XYZ calibration data, and raw frames
 * to disk for offline playback. Supports both recording (online mode) and
 * playback (offline mode) operations.
 */
class RecordingManager {
  public:
    RecordingManager(std::shared_ptr<DepthSensorInterface> sensor,
                     CalibrationManager *calibMgr, CameraConfiguration *config);
    ~RecordingManager() = default;

    // ========== Recording Operations (Online Mode) ==========
    /**
     * @brief Starts recording frames to a file.
     * @param filePath Destination file path for recorded data
     * @param cameraFps Current camera frame rate
     * @param details Camera details (mode, width, height)
     * @param modeDetailsCache Cached mode configuration
     * @param availableModes List of available sensor modes
     * @param depthEnabled Depth frame recording enabled
     * @param abEnabled AB frame recording enabled
     * @param confEnabled Confidence frame recording enabled
     * @param xyzEnabled XYZ frame recording enabled
     * @return Status::OK on success
     */
    Status startRecording(
        std::string &filePath, uint16_t cameraFps, const CameraDetails &details,
        const DepthSensorModeDetails &modeDetailsCache,
        const std::vector<DepthSensorModeDetails> &availableModes,
        bool depthEnabled, bool abEnabled, bool confEnabled, bool xyzEnabled);

    /**
     * @brief Stops recording and closes the recording file.
     * @return Status::OK on success
     */
    Status stopRecording();

    // ========== Playback Operations (Offline Mode) ==========
    /**
     * @brief Sets the playback file for offline replay.
     * @param filePath Path to previously recorded file
     * @return Status::OK on success
     */
    Status setPlaybackFile(std::string &filePath);

    /**
     * @brief Loads recording header and initializes playback state.
     * @param[out] modeDetailsCache Mode details loaded from file
     * @param[out] details Camera details loaded from file
     * @return Status::OK on success
     */
    Status loadPlaybackHeader(DepthSensorModeDetails &modeDetailsCache,
                              CameraDetails &details);

    /**
     * @brief Gets the number of frames in the playback file.
     * @return Frame count
     */
    uint32_t getNumberOfFrames() const {
        return m_offline_parameters.numberOfFrames;
    }

    /**
     * @brief Gets the frame rate from playback file.
     * @return Frame rate in FPS
     */
    uint16_t getFrameRate() const { return m_offline_parameters.frameRate; }

  private:
    std::shared_ptr<DepthSensorInterface> m_depthSensor;
    CalibrationManager *m_calibrationMgr;
    CameraConfiguration *m_config;

    // Offline parameters structure (matches recording file format)
    struct offlineparameter_struct {
        static const uint32_t MAX_FRAME_DATA_DETAILS_SAVE = 8;
        static const uint32_t MAX_FRAME_CONTENT = 6;
        uint32_t numberOfFrames;
        const uint32_t formatVersion = 0x00000001;
        uint16_t frameRate;
        uint16_t enableMetaDatainAB;
        TofiXYZDealiasData dealias;
        struct {
            uint32_t mode;
            uint32_t width;
            uint32_t height;
            uint32_t totalCaptures;
        } details;
        struct {
            uint8_t modeNumber;
            char frameContent[MAX_FRAME_CONTENT][16];
            uint8_t numberOfFrequencies;
            uint8_t numberOfPhases;
            int pixelFormatIndex;
            int frameWidthInBytes;
            int frameHeightInBytes;
            int baseResolutionWidth;
            int baseResolutionHeight;
            int metadataSize;
            int isPCM;
        } modeDetailsCache;
        uint32_t fdatadetailsCount;
        struct {
            char type[32];
            uint32_t width;
            uint32_t height;
            uint32_t subelementSize;
            uint32_t subelementsPerElement;
            uint32_t bytesCount;
        } fDataDetails[MAX_FRAME_DATA_DETAILS_SAVE];
    } m_offline_parameters;
};

} // namespace aditof

#endif // RECORDING_MANAGER_H
