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
#include "adsd3500_mode_manager.h"
#include "adsd3500_command_interface.h"
#include "buffer_processor_interface.h"
#include "frame_configuration.h"
#include "platform/platform_impl.h"
#include "sensor_control_registry.h"
#include "video_device_driver.h"
#include <aditof/log.h>
#include <aditof/utils.h>
#include <linux/videodev2.h>

// Platform-specific control IDs
#define CTRL_SET_MODE                                                          \
    (aditof::platform::Platform::getInstance().getV4L2ModeControlId())

namespace aditof {

struct Adsd3500ModeManager::Impl {
    VideoDeviceDriver *videoDriver;
    BufferProcessorInterface *bufferProcessor;
    SensorControlRegistry *controlRegistry;
    Adsd3500CommandInterface *commandInterface;
    DepthSensorModeDetails currentModeDetails;
    uint8_t currentMode;

    Impl()
        : videoDriver(nullptr), bufferProcessor(nullptr),
          controlRegistry(nullptr), commandInterface(nullptr),
          currentMode(0xFF) {
        currentModeDetails.modeNumber = 0xFF;
    }
};

Adsd3500ModeManager::Adsd3500ModeManager() : m_impl(new Impl()) {}

Adsd3500ModeManager::~Adsd3500ModeManager() = default;

void Adsd3500ModeManager::setDependencies(
    VideoDeviceDriver *videoDriver, BufferProcessorInterface *bufferProcessor,
    SensorControlRegistry *controlRegistry,
    Adsd3500CommandInterface *commandInterface) {
    m_impl->videoDriver = videoDriver;
    m_impl->bufferProcessor = bufferProcessor;
    m_impl->controlRegistry = controlRegistry;
    m_impl->commandInterface = commandInterface;
}

Status Adsd3500ModeManager::setMode(
    uint8_t mode, const std::vector<DepthSensorModeDetails> &availableModes) {

    if (!m_impl->videoDriver || !m_impl->bufferProcessor) {
        LOG(ERROR) << "Mode manager dependencies not set";
        return Status::GENERIC_ERROR;
    }

    LOG(INFO) << "Setting mode: " << static_cast<int>(mode);

    // Find mode details in available modes
    bool modeFound = false;
    DepthSensorModeDetails modeDetails;
    for (const auto &availableMode : availableModes) {
        if (availableMode.modeNumber == mode) {
            modeDetails = availableMode;
            modeFound = true;
            break;
        }
    }

    if (!modeFound) {
        LOG(ERROR) << "Mode " << static_cast<int>(mode) << " not found";
        return Status::INVALID_ARGUMENT;
    }

    // Set mode control on subdevice
    Status status = m_impl->videoDriver->setControl(CTRL_SET_MODE, mode);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to set mode control";
        return status;
    }

    // Determine pixel format
    __u32 pixelFormat = 0;
    if (modeDetails.pixelFormatIndex == 1) {
        pixelFormat = V4L2_PIX_FMT_SRGGB12;
    } else {
        pixelFormat =
            aditof::platform::Platform::getInstance().getV4L2PixelFormat8bit();
    }

    // Set video format
    VideoFormat videoFormat;
    videoFormat.pixelFormat = pixelFormat;

    // Raw bypass mode: frameWidthInBytes is in bytes (pixels × 2), convert to
    // pixels Standard modes: frameWidthInBytes is already in pixels
    if (modeDetails.isRawBypass) {
        videoFormat.width = modeDetails.frameWidthInBytes / 2;
    } else {
        videoFormat.width = modeDetails.frameWidthInBytes;
    }
    videoFormat.height = modeDetails.frameHeightInBytes;
    videoFormat.bytesPerLine = 0; // Let driver decide
    videoFormat.sizeImage = 0;    // Let driver decide

    status = m_impl->videoDriver->setFormat(videoFormat);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to set video format";
        return status;
    }

    // Get negotiated format back from driver
    status = m_impl->videoDriver->getFormat(videoFormat);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to get negotiated format";
        return status;
    }

    // For raw bypass, update mode details with driver-negotiated dimensions
    if (modeDetails.isRawBypass) {
        modeDetails.frameWidthInBytes = videoFormat.bytesPerLine;
        modeDetails.frameHeightInBytes = videoFormat.height;
    }

    // Request video buffers (simplified - actual buffer mapping done elsewhere)
    const unsigned int EXTRA_BUFFERS_COUNT = 3;
    unsigned int capturesPerFrame = 1; // Simplified
    unsigned int requestedBufferCount = capturesPerFrame + EXTRA_BUFFERS_COUNT;

    status = m_impl->videoDriver->requestBuffers(requestedBufferCount,
                                                 V4L2_MEMORY_MMAP);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to request video buffers";
        return status;
    }

    // Configure buffer processor for this mode
    if (!modeDetails.isPCM) {
        // Get AB and confidence bit depths from control registry
        unsigned int bitsInAB = 0;
        unsigned int bitsInConf = 0;

        if (m_impl->controlRegistry) {
            // Try to get from registry; ignore errors and use defaults
            std::string abBitsStr, confBitsStr;
            if (m_impl->controlRegistry->getControl("abBits", abBitsStr) ==
                Status::OK) {
                bitsInAB = std::stoi(abBitsStr);
            }
            if (m_impl->controlRegistry->getControl(
                    "confidenceBits", confBitsStr) == Status::OK) {
                bitsInConf = std::stoi(confBitsStr);
            }
        }

        // Construct FrameConfiguration value object
        FrameDimensions dimensions(modeDetails.baseResolutionWidth,
                                   modeDetails.baseResolutionHeight);
        BitDepthConfiguration bitDepths(16, bitsInAB, bitsInConf);
        FrameConfiguration frameConfig(dimensions, bitDepths, mode,
                                       modeDetails.isRawBypass);

        status = m_impl->bufferProcessor->setVideoProperties(
            frameConfig, modeDetails.frameWidthInBytes,
            modeDetails.frameHeightInBytes);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to set buffer processor properties";
            return status;
        }
    }

    // Update current mode
    m_impl->currentMode = mode;
    m_impl->currentModeDetails = modeDetails;

    LOG(INFO) << "Mode " << static_cast<int>(mode) << " set successfully";
    return Status::OK;
}

Status Adsd3500ModeManager::getCurrentModeDetails(
    DepthSensorModeDetails &details) const {
    if (m_impl->currentMode == 0xFF) {
        LOG(WARNING) << "No mode currently set";
        return Status::GENERIC_ERROR;
    }

    details = m_impl->currentModeDetails;
    return Status::OK;
}

uint8_t Adsd3500ModeManager::getCurrentMode() const {
    return m_impl->currentMode;
}

} // namespace aditof
