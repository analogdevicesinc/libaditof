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
#ifndef BUFFER_PROCESSOR_INTERFACE_H
#define BUFFER_PROCESSOR_INTERFACE_H

#include <aditof/status_definitions.h>
#include <cstdint>

// Forward declarations
struct VideoDev;
class TofiConfig;

namespace aditof {

/**
 * @brief Interface for frame buffer processing operations.
 *
 * Abstracts buffer processing logic to enable:
 * - Dependency injection for testing
 * - Alternative processing implementations
 * - Hardware-specific optimizations
 *
 * Implementations handle:
 * - Multi-threaded frame capture from V4L2 devices
 * - Depth computation (via ToFi library or hardware ISP)
 * - Buffer pool management
 * - Frame format conversion
 *
 * Following Dependency Inversion Principle: high-level sensor code
 * depends on this interface, not concrete BufferProcessor implementation.
 */
class BufferProcessorInterface {
  public:
    virtual ~BufferProcessorInterface() = default;

    /**
     * @brief Opens and initializes the buffer processor.
     *
     * @return Status::OK on success
     */
    virtual Status open() = 0;

    /**
     * @brief Sets the V4L2 input device for frame capture.
     *
     * @param inputVideoDev Pointer to VideoDev structure
     * @return Status::OK on success
     */
    virtual Status setInputDevice(VideoDev *inputVideoDev) = 0;

    /**
     * @brief Configures video properties and allocates buffers.
     *
     * @param frameWidth Output frame width in pixels
     * @param frameHeight Output frame height in pixels
     * @param WidthInBytes Raw frame width in bytes (stride)
     * @param HeightInBytes Raw frame height in bytes
     * @param modeNumber Frame mode number
     * @param bitsInAB Bit depth for AB (amplitude/brightness) data
     * @param bitsInConf Bit depth for confidence data
     * @param isRawBypass True if ISP is bypassing depth computation
     * @return Status::OK on success, Status::GENERIC_ERROR on allocation failure
     */
    virtual Status setVideoProperties(int frameWidth, int frameHeight,
                                      int WidthInBytes, int HeightInBytes,
                                      int modeNumber, uint8_t bitsInAB,
                                      uint8_t bitsInConf, uint8_t bitsInDepth,
                                      bool isRawBypass = false) = 0;

    /**
     * @brief Configures ToFi depth computation properties.
     *
     * @param iniFile Pointer to INI configuration file data
     * @param iniFileLength Length of INI file in bytes
     * @param calData Pointer to calibration data
     * @param calDataLength Length of calibration data in bytes
     * @param mode Frame mode number
     * @param ispEnabled True if hardware ISP is enabled
     * @return Status::OK on success
     */
    virtual Status setProcessorProperties(uint8_t *iniFile,
                                          uint16_t iniFileLength,
                                          uint8_t *calData,
                                          uint32_t calDataLength, uint16_t mode,
                                          bool ispEnabled) = 0;

    /**
     * @brief Processes a frame buffer (blocking until frame available).
     *
     * @param buffer Pointer to output buffer for processed frame
     * @return Status::OK on success
     */
    virtual Status processBuffer(uint16_t *buffer) = 0;

    /**
     * @brief Gets the ToFi configuration object.
     *
     * @return Pointer to TofiConfig, or nullptr if not initialized
     */
    virtual TofiConfig *getTofiConfig() const = 0;

    /**
     * @brief Gets the depth compute version status.
     *
     * @param enabled Output parameter: 1 if open-source enabled, 0 otherwise
     * @return Status::OK on success
     */
    virtual Status getDepthComputeVersion(uint8_t &enabled) const = 0;

    /**
     * @brief Enables/disables lens scatter compensation.
     *
     * @param enabled True to enable lens scatter compensation
     */
    virtual void setLensScatterCompensationEnabled(bool enabled) = 0;

    /**
     * @brief Gets lens scatter compensation status.
     *
     * @return True if enabled
     */
    virtual bool getLensScatterCompensationEnabled() const = 0;

    /**
     * @brief Enables/disables frame rotation.
     *
     * @param needsRotation True to enable 180-degree rotation
     */
    virtual void setNeedsRotation(bool needsRotation) = 0;

    /**
     * @brief Gets frame rotation status.
     *
     * @return True if rotation enabled
     */
    virtual bool getNeedsRotation() const = 0;

    /**
     * @brief Starts the capture and processing worker threads.
     */
    virtual void startThreads() = 0;

    /**
     * @brief Stops the capture and processing worker threads.
     */
    virtual void stopThreads() = 0;

    /**
     * @brief Registers a second ("alternate") mode configuration and
     * activates per-frame dispatch between it and the currently active
     * single-mode configuration. Used to correctly process frames produced
     * by hardware Dynamic Mode Switching (DMS) when the two modes in the
     * DMS pair have different resolutions/bit layouts.
     *
     * @param modeNumber Alternate mode number (the second mode in the DMS pair)
     * @param frameWidth Output frame width in pixels for the alternate mode
     * @param frameHeight Output frame height in pixels for the alternate mode
     * @param widthInBytes Raw frame width in bytes (stride) for the alternate mode
     * @param heightInBytes Raw frame height in bytes for the alternate mode
     * @param bitsInAB Bit depth for AB data for the alternate mode
     * @param bitsInConf Bit depth for confidence data for the alternate mode
     * @param bitsInDepth Bit depth for depth data for the alternate mode
     * @param isRawBypass True if the alternate mode bypasses ToFi compute
     * @param ispEnabled True if hardware ISP pre-computes depth for the alternate mode
     * @param iniFile Pointer to INI configuration data for the alternate mode
     * @param iniFileLength Length of INI data in bytes
     * @param calData Pointer to calibration data for the alternate mode
     * @param calDataLength Length of calibration data in bytes
     * @param repeatPrimary Number of consecutive frames using the primary mode
     * @param repeatAlternate Number of consecutive frames using the alternate mode
     * @return Status::OK on success
     */
    virtual Status setAlternateModeConfiguration(
        uint8_t modeNumber, int frameWidth, int frameHeight,
        int widthInBytes, int heightInBytes, uint8_t bitsInAB,
        uint8_t bitsInConf, uint8_t bitsInDepth, bool isRawBypass,
        bool ispEnabled, uint8_t *iniFile, uint16_t iniFileLength,
        uint8_t *calData, uint32_t calDataLength, uint8_t repeatPrimary,
        uint8_t repeatAlternate) = 0;

    /**
     * @brief Deactivates per-frame mode dispatch and frees the alternate
     * mode's compute context, reverting to the single active configuration.
     *
     * @return Status::OK on success
     */
    virtual Status clearAlternateModeConfiguration() = 0;

    /**
     * @brief Returns the mode number of the most recently delivered frame
     * (via processBuffer()). Ground truth for callers, since the chip's
     * embedded per-frame metadata can land at the wrong buffer offset when
     * the primary and alternate DMS modes differ in resolution.
     *
     * @return The primary or alternate mode number, whichever was delivered
     */
    virtual uint8_t getLastDeliveredModeNumber() const = 0;
};

} // namespace aditof

#endif // BUFFER_PROCESSOR_INTERFACE_H
