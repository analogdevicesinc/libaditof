/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 */
#include "recording_manager.h"
#include "calibration_manager.h"
#include "camera_configuration.h"

#include "tofi/algorithms.h"
#include <aditof/frame_definitions.h>
#include <aditof/log.h>
#include <aditof/playback_interface.h>
#include <aditof/recordable_interface.h>

#include <algorithm>
#include <cstring>

using namespace aditof;

RecordingManager::RecordingManager(std::shared_ptr<DepthSensorInterface> sensor,
                                   CalibrationManager *calibMgr,
                                   CameraConfiguration *config)
    : m_depthSensor(sensor), m_calibrationMgr(calibMgr),
      m_config(config), m_offline_parameters{} {
    // Value-initialization ensures all members are properly zero-initialized
}

Status RecordingManager::startRecording(
    std::string &filePath, uint16_t cameraFps, const CameraDetails &details,
    const DepthSensorModeDetails &modeDetailsCache,
    const std::vector<DepthSensorModeDetails> &availableModes,
    bool depthEnabled, bool abEnabled, bool confEnabled, bool xyzEnabled) {

    // Reset frame counter
    m_offline_parameters.numberOfFrames = 0;

    // Store XYZ dealias data
    TofiXYZDealiasData dealias;
    m_calibrationMgr->getXYZDealiasData(details.mode, dealias);
    memcpy(&m_offline_parameters.dealias, &dealias, sizeof(dealias));

    // Store frame rate
    m_offline_parameters.frameRate = cameraFps;

    // Store metadata embedding flag
    m_offline_parameters.enableMetaDatainAB = m_config->getMetadataInAB();

    // Copy mode details cache
    m_offline_parameters.modeDetailsCache.modeNumber =
        modeDetailsCache.modeNumber;
    m_offline_parameters.modeDetailsCache.numberOfFrequencies =
        modeDetailsCache.numberOfFrequencies;
    m_offline_parameters.modeDetailsCache.numberOfPhases =
        modeDetailsCache.numberOfPhases;
    m_offline_parameters.modeDetailsCache.pixelFormatIndex =
        modeDetailsCache.pixelFormatIndex;
    m_offline_parameters.modeDetailsCache.frameHeightInBytes =
        modeDetailsCache.frameHeightInBytes;
    m_offline_parameters.modeDetailsCache.frameWidthInBytes =
        modeDetailsCache.frameWidthInBytes;
    m_offline_parameters.modeDetailsCache.baseResolutionWidth =
        modeDetailsCache.baseResolutionWidth;
    m_offline_parameters.modeDetailsCache.baseResolutionHeight =
        modeDetailsCache.baseResolutionHeight;
    m_offline_parameters.modeDetailsCache.metadataSize =
        modeDetailsCache.metadataSize;
    m_offline_parameters.modeDetailsCache.isPCM = modeDetailsCache.isPCM;

    // Zero out frameContent array to avoid garbage data
    memset(m_offline_parameters.modeDetailsCache.frameContent, 0,
           sizeof(m_offline_parameters.modeDetailsCache.frameContent));

    uint32_t idx = 0;
    LOG(INFO) << "[RECORDING] Enable flags: depth=" << depthEnabled
              << " ab=" << abEnabled << " conf=" << confEnabled
              << " xyz=" << xyzEnabled;
    LOG(INFO) << "[RECORDING] Original frameContent has "
              << modeDetailsCache.frameContent.size() << " items";
    for (const std::string &val : modeDetailsCache.frameContent) {
        LOG(INFO) << "[RECORDING] frameContent item: \"" << val << "\"";
        // Skip raw from frameContent header - it's handled in fDataDetails
        if (val == "raw") {
            LOG(INFO) << "[RECORDING] Skipping raw in frameContent "
                         "(handled in fDataDetails)";
            continue;
        }

        LOG(INFO) << "[RECORDING] Adding to header: " << val;
        memcpy(
            (char *)m_offline_parameters.modeDetailsCache.frameContent[idx++],
            val.c_str(), val.length());
    }
    LOG(INFO) << "[RECORDING] Total frame types in header: " << idx;

    // Build frame details
    m_offline_parameters.fdatadetailsCount = 0;

    uint32_t mode = details.mode;
    auto modeIt = std::find_if(availableModes.begin(), availableModes.end(),
                               [&mode](const DepthSensorModeDetails &d) {
                                   return (d.modeNumber == mode);
                               });

    if (modeIt == availableModes.end()) {
        LOG(WARNING) << "Mode: " << (int)mode << " not supported by camera";
        return Status::INVALID_ARGUMENT;
    }

    m_offline_parameters.details.mode = mode;
    m_offline_parameters.details.width = (*modeIt).baseResolutionWidth;
    m_offline_parameters.details.height = (*modeIt).baseResolutionHeight;
    m_offline_parameters.details.totalCaptures = 1;

    LOG(INFO) << "[RECORDING] Building fDataDetails from frameContent...";
    for (const auto &item : modeDetailsCache.frameContent) {

        if (m_offline_parameters.fdatadetailsCount >
            m_offline_parameters.MAX_FRAME_DATA_DETAILS_SAVE) {
            LOG(ERROR) << "Frame data details count exceeded the limit";
            break;
        }

        LOG(INFO) << "[RECORDING] Adding to fDataDetails: " << item;
        uint32_t idx = m_offline_parameters.fdatadetailsCount;

        // Copy frame type name safely
        strncpy(m_offline_parameters.fDataDetails[idx].type, item.c_str(),
                sizeof(m_offline_parameters.fDataDetails[idx].type) - 1);
        m_offline_parameters.fDataDetails[idx]
            .type[sizeof(m_offline_parameters.fDataDetails[idx].type) - 1] =
            '\0';

        m_offline_parameters.fDataDetails[idx].width =
            modeDetailsCache.baseResolutionWidth;
        m_offline_parameters.fDataDetails[idx].height =
            modeDetailsCache.baseResolutionHeight;
        m_offline_parameters.fDataDetails[idx].subelementSize =
            sizeof(uint16_t);
        m_offline_parameters.fDataDetails[idx].subelementsPerElement = 1;

        if (item == "xyz") {
            m_offline_parameters.fDataDetails[idx].subelementsPerElement = 3;
        } else if (item == "raw") {
            // Raw bypass: V4L2 buffer with NVIDIA padding (width × height + width bytes)
            size_t rawBufferSize = modeDetailsCache.frameWidthInBytes *
                                       modeDetailsCache.frameHeightInBytes +
                                   modeDetailsCache.frameWidthInBytes;
            m_offline_parameters.fDataDetails[idx].width = rawBufferSize;
            m_offline_parameters.fDataDetails[idx].height = 1;
            m_offline_parameters.fDataDetails[idx].subelementSize = 1;
        } else if (item == "metadata") {
            m_offline_parameters.fDataDetails[idx].subelementSize = 1;
            m_offline_parameters.fDataDetails[idx].width = 128;
            m_offline_parameters.fDataDetails[idx].height = 1;
        } else if (item == "conf") {
            m_offline_parameters.fDataDetails[idx].subelementSize =
                sizeof(float);
        }
        m_offline_parameters.fDataDetails[idx].bytesCount =
            m_offline_parameters.fDataDetails[idx].width *
            m_offline_parameters.fDataDetails[idx].height *
            m_offline_parameters.fDataDetails[idx].subelementSize *
            m_offline_parameters.fDataDetails[idx].subelementsPerElement;

        m_offline_parameters.fdatadetailsCount++;
    }

    LOG(INFO) << "[RECORDING] Final fDataDetails count: "
              << m_offline_parameters.fdatadetailsCount;
    LOG(INFO) << "[RECORDING] Writing header to file: " << filePath;

    // Write header to recording file
    auto recordableInterface =
        std::dynamic_pointer_cast<RecordableInterface>(m_depthSensor);
    if (!recordableInterface) {
        LOG(ERROR) << "Recording interface not available (sensor does not "
                      "support recording)";
        return Status::UNAVAILABLE;
    }

    return recordableInterface->startRecording(filePath,
                                               (uint8_t *)&m_offline_parameters,
                                               sizeof(m_offline_parameters));
}

Status RecordingManager::stopRecording() {
    auto recordableInterface =
        std::dynamic_pointer_cast<RecordableInterface>(m_depthSensor);
    if (!recordableInterface) {
        return Status::OK; // Not an error if recording wasn't supported
    }
    return recordableInterface->stopRecording();
}

Status RecordingManager::setPlaybackFile(std::string &filePath) {
    auto playbackInterface =
        std::dynamic_pointer_cast<PlaybackInterface>(m_depthSensor);
    if (!playbackInterface) {
        LOG(ERROR) << "Playback interface not available (sensor does not "
                      "support playback)";
        return Status::UNAVAILABLE;
    }
    return playbackInterface->setPlaybackFile(filePath);
}

Status
RecordingManager::loadPlaybackHeader(DepthSensorModeDetails &modeDetailsCache,
                                     CameraDetails &details) {

    // Clear and read header from playback file
    memset((void *)&m_offline_parameters, 0, sizeof(m_offline_parameters));

    auto playbackInterface =
        std::dynamic_pointer_cast<PlaybackInterface>(m_depthSensor);
    if (!playbackInterface) {
        LOG(ERROR) << "Playback interface not available (sensor does not "
                      "support playback)";
        return Status::UNAVAILABLE;
    }

    Status status = playbackInterface->getHeader(
        (uint8_t *)&m_offline_parameters, sizeof(m_offline_parameters));
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to get header from device";
        return status;
    }

    LOG(INFO) << "Number of Frames: " << m_offline_parameters.numberOfFrames;
    LOG(INFO) << "Frame Rate: " << m_offline_parameters.frameRate;

    // Copy mode details from file header
    modeDetailsCache.modeNumber =
        m_offline_parameters.modeDetailsCache.modeNumber;
    modeDetailsCache.numberOfFrequencies =
        m_offline_parameters.modeDetailsCache.numberOfFrequencies;
    modeDetailsCache.numberOfPhases =
        m_offline_parameters.modeDetailsCache.numberOfPhases;
    modeDetailsCache.pixelFormatIndex =
        m_offline_parameters.modeDetailsCache.pixelFormatIndex;
    modeDetailsCache.frameHeightInBytes =
        m_offline_parameters.modeDetailsCache.frameHeightInBytes;
    modeDetailsCache.frameWidthInBytes =
        m_offline_parameters.modeDetailsCache.frameWidthInBytes;
    modeDetailsCache.baseResolutionWidth =
        m_offline_parameters.modeDetailsCache.baseResolutionWidth;
    modeDetailsCache.baseResolutionHeight =
        m_offline_parameters.modeDetailsCache.baseResolutionHeight;
    modeDetailsCache.metadataSize =
        m_offline_parameters.modeDetailsCache.metadataSize;
    modeDetailsCache.isPCM = m_offline_parameters.modeDetailsCache.isPCM;

    // Offline recordings always contain processed output, never raw bypass
    modeDetailsCache.isRawBypass = false;
    modeDetailsCache.frameContent.clear();

    LOG(INFO) << "[PLAYBACK] Reading frameContent from file header...";
    for (uint32_t idx = 0; idx < m_offline_parameters.MAX_FRAME_CONTENT;
         idx++) {
        std::string frameContent =
            m_offline_parameters.modeDetailsCache.frameContent[idx];
        if (!frameContent.empty()) {
            LOG(INFO) << "[PLAYBACK] Found in header: " << frameContent;
            modeDetailsCache.frameContent.emplace_back(frameContent);
        }
    }
    LOG(INFO) << "[PLAYBACK] Total frame types loaded: "
              << modeDetailsCache.frameContent.size();

    // Copy camera details
    details.mode = m_offline_parameters.details.mode;
    details.frameType.width = m_offline_parameters.details.width;
    details.frameType.height = m_offline_parameters.details.height;
    details.frameType.totalCaptures =
        m_offline_parameters.details.totalCaptures;
    details.frameType.dataDetails.clear();

    LOG(INFO) << "[PLAYBACK] Loading " << m_offline_parameters.fdatadetailsCount
              << " fDataDetails entries...";
    for (uint32_t idx = 0; idx < m_offline_parameters.fdatadetailsCount;
         idx++) {
        FrameDataDetails fDataDetails;
        fDataDetails.type = m_offline_parameters.fDataDetails[idx].type;
        LOG(INFO) << "[PLAYBACK] fDataDetails[" << idx
                  << "]: type=" << fDataDetails.type;
        fDataDetails.width = m_offline_parameters.fDataDetails[idx].width;
        fDataDetails.height = m_offline_parameters.fDataDetails[idx].height;
        fDataDetails.subelementSize =
            m_offline_parameters.fDataDetails[idx].subelementSize;
        fDataDetails.subelementsPerElement =
            m_offline_parameters.fDataDetails[idx].subelementsPerElement;
        fDataDetails.bytesCount =
            m_offline_parameters.fDataDetails[idx].bytesCount;
        details.frameType.dataDetails.emplace_back(fDataDetails);
    }

    // Generate XYZ tables from dealias data
    const int GEN_XYZ_ITERATIONS = 20;
    TofiXYZDealiasData *pDealias = &m_offline_parameters.dealias;
    m_calibrationMgr->cleanupXYZtables();
    XYZTable xyzTable = m_calibrationMgr->getXYZTable();
    int ret = Algorithms::GenerateXYZTables(
        &xyzTable.p_x_table, &xyzTable.p_y_table, &xyzTable.p_z_table,
        &(pDealias->camera_intrinsics), pDealias->n_sensor_rows,
        pDealias->n_sensor_cols, modeDetailsCache.baseResolutionHeight,
        modeDetailsCache.baseResolutionWidth, pDealias->n_offset_rows,
        pDealias->n_offset_cols, pDealias->row_bin_factor,
        pDealias->col_bin_factor, GEN_XYZ_ITERATIONS);
    m_calibrationMgr->setXYZTable(xyzTable);

    if (ret != 0 || !xyzTable.p_x_table || !xyzTable.p_y_table ||
        !xyzTable.p_z_table) {
        LOG(ERROR) << "Failed to generate the XYZ tables";
        return Status::GENERIC_ERROR;
    }

    return Status::OK;
}
