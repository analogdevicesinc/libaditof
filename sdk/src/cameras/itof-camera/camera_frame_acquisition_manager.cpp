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

#include "camera_frame_acquisition_manager.h"
#include "calibration_manager.h"
#include "camera_configuration.h"
#include "tofi/algorithms.h"
#include "tofi/tofi_util.h"
#include <aditof/camera_definitions.h>
#include <aditof/frame_operations.h>
#include <aditof/log.h>
#include <cstring>

namespace aditof {

// Metadata size embedded in AB frame header
static constexpr size_t skMetaDataBytesCount = 128;

CameraFrameAcquisitionManager::CameraFrameAcquisitionManager(
    std::shared_ptr<DepthSensorInterface> depthSensor,
    CalibrationManager *calibrationMgr, CameraConfiguration *config)
    : m_depthSensor(depthSensor), m_calibrationMgr(calibrationMgr),
      m_config(config) {}

Status CameraFrameAcquisitionManager::requestFrame(
    Frame *frame, uint32_t index, const FrameDetails &frameType,
    const DepthSensorModeDetails &modeDetails, bool isOffline, bool isPcmFrame,
    bool depthEnabled, bool abEnabled, bool confEnabled, bool xyzEnabled,
    uint8_t confBitsPerPixel, uint8_t abBitsPerPixel, uint8_t depthBitsPerPixel,
    bool &dropFrameOnce) {

    Status status = Status::OK;

    // Step 1: Validate frame pointer
    if (frame == nullptr) {
        return Status::INVALID_ARGUMENT;
    }

    // Step 2: Validate and update frame configuration if needed
    status = validateAndUpdateFrame(frame, frameType, confBitsPerPixel,
                                    abBitsPerPixel);
    if (status != Status::OK) {
        return status;
    }

    // Step 3: Get frame data buffer location
    uint16_t *frameDataLocation = nullptr;
    status = getFrameDataLocation(frame, isPcmFrame, &frameDataLocation);
    if (status != Status::OK) {
        return status;
    }

    // Step 4: Drop first frame if configured
    status = dropFirstFrameIfNeeded(frameDataLocation, index, dropFrameOnce,
                                    isOffline);
    if (status != Status::OK) {
        return status;
    }

    // Step 5: Acquire raw frame from sensor
    status = acquireRawFrame(frameDataLocation, index);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to get frame from device";
        return status;
    }

    // Step 6: For raw bypass mode, skip all post-processing
    if (modeDetails.isRawBypass) {
        return Status::OK;
    }

    // Step 7: Compute XYZ point cloud if enabled
    status = computeXYZIfEnabled(frame, xyzEnabled, depthEnabled, modeDetails);
    if (status != Status::OK) {
        return status;
    }

    // Step 8: Zero disabled data types
    status = zeroDepthIfDisabled(frame, depthEnabled, modeDetails);
    if (status != Status::OK) {
        return status;
    }

    status = zeroABIfDisabled(frame, abEnabled, modeDetails);
    if (status != Status::OK) {
        return status;
    }

    // Step 9: Extract or generate metadata
    Metadata metadata;
    status = extractOrGenerateMetadata(
        frame, metadata, modeDetails, abEnabled, isPcmFrame, xyzEnabled,
        depthBitsPerPixel, abBitsPerPixel, confBitsPerPixel);
    if (status != Status::OK) {
        return status;
    }

    // Step 10: Write metadata to frame
    status = writeMetadataToFrame(frame, metadata);
    if (status != Status::OK) {
        return status;
    }

    return Status::OK;
}

Status CameraFrameAcquisitionManager::validateAndUpdateFrame(
    Frame *frame, const FrameDetails &frameType, uint8_t confBitsPerPixel,
    uint8_t abBitsPerPixel) {

    FrameDetails currentDetails;
    frame->getDetails(currentDetails);

    // Update frame configuration if it differs from expected
    if (frameType != currentDetails) {
        frame->setDetails(frameType, confBitsPerPixel, abBitsPerPixel);
    }

    return Status::OK;
}

Status CameraFrameAcquisitionManager::getFrameDataLocation(
    Frame *frame, bool isPcmFrame, uint16_t **dataLocation) {

    Status status;

    // For PCM frames, use AB buffer; otherwise use frameData buffer
    if (!isPcmFrame) {
        status = frame->getData("frameData", dataLocation);
    } else {
        status = frame->getData("ab", dataLocation);
    }

    if (status != Status::OK) {
        LOG(ERROR) << "Failed to get frame data location, status: "
                   << static_cast<int>(status);
        return status;
    }

    if (*dataLocation == nullptr) {
        LOG(ERROR) << "Frame data location is null despite success status";
        return Status::GENERIC_ERROR;
    }

    return Status::OK;
}

Status CameraFrameAcquisitionManager::dropFirstFrameIfNeeded(
    uint16_t *dataLocation, uint32_t index, bool &dropFrameOnce,
    bool isOffline) {

    // Only drop first frame in online mode if configured
    if (!m_config->getDropFirstFrame() || !dropFrameOnce || isOffline) {
        return Status::OK;
    }

    Status status = m_depthSensor->getFrame(dataLocation, index);
    if (status != Status::OK) {
        dropFrameOnce = true; // Keep trying
        LOG(INFO) << "Failed to drop first frame!";
        return status;
    }

    dropFrameOnce = false;
    LOG(INFO) << "Dropped first frame";

    return Status::OK;
}

Status CameraFrameAcquisitionManager::acquireRawFrame(uint16_t *dataLocation,
                                                      uint32_t index) {
    return m_depthSensor->getFrame(dataLocation, index);
}

Status CameraFrameAcquisitionManager::computeXYZIfEnabled(
    Frame *frame, bool xyzEnabled, bool depthEnabled,
    const DepthSensorModeDetails &modeDetails) {

    // XYZ computation requires depth and XYZ both enabled
    if (!xyzEnabled || !depthEnabled || !frame->haveDataType("xyz")) {
        return Status::OK;
    }

    uint16_t *depthFrame = nullptr;
    uint16_t *xyzFrame = nullptr;

    Status getDepthStatus = frame->getData("depth", &depthFrame);
    Status getXYZStatus = frame->getData("xyz", &xyzFrame);

    if (getDepthStatus == Status::OK && getXYZStatus == Status::OK &&
        depthFrame != nullptr && xyzFrame != nullptr) {

        // Get XYZ calibration table
        XYZTable xyzTable = m_calibrationMgr->getXYZTable();

        // Compute point cloud
        Algorithms::ComputeXYZ(static_cast<const uint16_t *>(depthFrame),
                               &xyzTable, reinterpret_cast<int16_t *>(xyzFrame),
                               modeDetails.baseResolutionHeight,
                               modeDetails.baseResolutionWidth);

        return Status::OK;
    } else {
        LOG(WARNING) << "XYZ enabled but frame buffers not allocated. "
                     << "Depth status: " << static_cast<int>(getDepthStatus)
                     << ", XYZ status: " << static_cast<int>(getXYZStatus);
        return Status::OK; // Non-fatal
    }
}

Status CameraFrameAcquisitionManager::zeroDepthIfDisabled(
    Frame *frame, bool depthEnabled,
    const DepthSensorModeDetails &modeDetails) {

    // If depth is enabled or frame doesn't have depth buffer, skip
    if (depthEnabled || !frame->haveDataType("depth")) {
        return Status::OK;
    }

    uint16_t *depthFrame = nullptr;
    Status status = frame->getData("depth", &depthFrame);
    if (status != Status::OK || depthFrame == nullptr) {
        LOG(ERROR) << "Failed to get depth frame location";
        return status;
    }

    // Zero depth buffer
    size_t depthSize = modeDetails.baseResolutionHeight *
                       modeDetails.baseResolutionWidth * sizeof(uint16_t);
    memset(depthFrame, 0, depthSize);

    return Status::OK;
}

Status CameraFrameAcquisitionManager::zeroABIfDisabled(
    Frame *frame, bool abEnabled, const DepthSensorModeDetails &modeDetails) {

    // If AB is enabled or frame doesn't have AB buffer, skip
    if (abEnabled || !frame->haveDataType("ab")) {
        return Status::OK;
    }

    uint16_t *abFrame = nullptr;
    Status status = frame->getData("ab", &abFrame);
    if (status != Status::OK || abFrame == nullptr) {
        LOG(ERROR) << "Failed to get AB frame location";
        return status;
    }

    // Zero AB buffer
    size_t abSize = modeDetails.baseResolutionHeight *
                    modeDetails.baseResolutionWidth * sizeof(uint16_t);
    memset(abFrame, 0, abSize);

    return Status::OK;
}

Status CameraFrameAcquisitionManager::extractOrGenerateMetadata(
    Frame *frame, Metadata &metadata, const DepthSensorModeDetails &modeDetails,
    bool abEnabled, bool isPcmFrame, bool xyzEnabled, uint8_t depthBitsPerPixel,
    uint8_t abBitsPerPixel, uint8_t confBitsPerPixel) {

    // Try to extract metadata from AB frame header if enabled
    if (m_config->getMetadataInAB() && abEnabled) {
        uint16_t *abFrame = nullptr;
        Status status = frame->getData("ab", &abFrame);
        if (status != Status::OK || abFrame == nullptr) {
            LOG(ERROR) << "Failed to get AB frame location for metadata";
            return status;
        }

        // Validate metadata size
        static_assert(sizeof(Metadata) <= skMetaDataBytesCount,
                      "Metadata struct exceeds AB frame header size");

        // Extract metadata from AB header
        memcpy(reinterpret_cast<uint8_t *>(&metadata), abFrame,
               sizeof(metadata));

        // Clear metadata bytes from AB frame
        memset(abFrame, 0, sizeof(metadata));
    } else {
        // Generate metadata from current configuration
        memset(static_cast<void *>(&metadata), 0, sizeof(metadata));
        metadata.width = modeDetails.baseResolutionWidth;
        metadata.height = modeDetails.baseResolutionHeight;
        metadata.imagerMode = modeDetails.modeNumber;
        metadata.bitsInDepth = depthBitsPerPixel;
        metadata.bitsInAb = abBitsPerPixel;
        metadata.bitsInConfidence = confBitsPerPixel;

        // For PCM frames, AB is always 16-bit
        if (isPcmFrame) {
            metadata.bitsInAb = 16;
        }
    }

    // Update XYZ status in metadata
    metadata.xyzEnabled = xyzEnabled;

    return Status::OK;
}

Status
CameraFrameAcquisitionManager::writeMetadataToFrame(Frame *frame,
                                                    const Metadata &metadata) {

    uint16_t *metadataLocation = nullptr;
    Status status = frame->getData("metadata", &metadataLocation);
    if (status != Status::OK || metadataLocation == nullptr) {
        LOG(ERROR) << "Failed to get metadata location";
        return status;
    }

    // Copy metadata to frame buffer
    memcpy(reinterpret_cast<uint8_t *>(metadataLocation),
           reinterpret_cast<const uint8_t *>(&metadata), sizeof(metadata));

    return Status::OK;
}

} // namespace aditof
