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
#ifndef FRAME_CONFIGURATION_H
#define FRAME_CONFIGURATION_H

#include <cstdint>

namespace aditof {

/**
 * @brief Represents frame dimensions with computed properties.
 *
 * Value object that encapsulates frame size and provides computed
 * properties like stride, total pixels, and buffer sizes.
 * Replaces primitive int parameters passed around the codebase.
 */
struct FrameDimensions {
    unsigned int width;  /**< Frame width in pixels */
    unsigned int height; /**< Frame height in pixels */

    /**
     * @brief Constructs frame dimensions with validation.
     *
     * @param w Width in pixels (must be > 0)
     * @param h Height in pixels (must be > 0)
     */
    FrameDimensions(unsigned int w = 0, unsigned int h = 0)
        : width(w), height(h) {}

    /**
     * @brief Gets the row stride in bytes (width * 2 for uint16_t pixels).
     *
     * @return Stride in bytes
     */
    unsigned int strideBytes() const { return width * 2; }

    /**
     * @brief Gets the total number of pixels.
     *
     * @return Total pixels (width * height)
     */
    unsigned int totalPixels() const { return width * height; }

    /**
     * @brief Gets buffer size for a single plane (depth or AB).
     *
     * @return Buffer size in bytes
     */
    unsigned int planeBufferSize() const {
        return totalPixels() * sizeof(uint16_t);
    }

    /**
     * @brief Checks if dimensions are valid (non-zero).
     *
     * @return true if valid
     */
    bool isValid() const { return width > 0 && height > 0; }

    /**
     * @brief Equality comparison.
     */
    bool operator==(const FrameDimensions &other) const {
        return width == other.width && height == other.height;
    }

    bool operator!=(const FrameDimensions &other) const {
        return !(*this == other);
    }
};

/**
 * @brief Represents bit depth configuration for frame data.
 *
 * Value object that encapsulates bit depths for different data types
 * (depth, AB, confidence) with validation and computed properties.
 */
struct BitDepthConfiguration {
    unsigned int
        depthBits;       /**< Bit depth for depth data (0, 8, 10, 12, 14, 16) */
    unsigned int abBits; /**< Bit depth for AB (amplitude/brightness) data */
    unsigned int confidenceBits; /**< Bit depth for confidence data */

    /**
     * @brief Constructs bit depth configuration.
     *
     * @param depth Depth bit depth
     * @param ab AB bit depth
     * @param conf Confidence bit depth
     */
    BitDepthConfiguration(unsigned int depth = 16, unsigned int ab = 16,
                          unsigned int conf = 0)
        : depthBits(depth), abBits(ab), confidenceBits(conf) {}

    /**
     * @brief Checks if configuration is valid.
     *
     * Valid bit depths are: 0 (disabled), 4, 8, 10, 12, 14, 16
     *
     * @return true if all bit depths are valid
     */
    bool isValid() const {
        return isValidBitDepth(depthBits) && isValidBitDepth(abBits) &&
               isValidBitDepth(confidenceBits);
    }

    /**
     * @brief Gets bytes per pixel for depth data.
     *
     * @return Bytes per pixel (depth is always stored as uint16_t)
     */
    unsigned int depthBytesPerPixel() const { return 2; }

    /**
     * @brief Gets bytes per pixel for AB data.
     *
     * @return Bytes per pixel
     */
    unsigned int abBytesPerPixel() const { return (abBits <= 8) ? 1 : 2; }

    /**
     * @brief Gets bytes per pixel for confidence data.
     *
     * @return Bytes per pixel (confidence is stored as float)
     */
    unsigned int confidenceBytesPerPixel() const {
        return (confidenceBits > 0) ? 4 : 0;
    }

    /**
     * @brief Calculates total bytes per pixel for complete frame data.
     *
     * @return Total bytes per pixel (depth + AB + confidence)
     */
    unsigned int totalBytesPerPixel() const {
        return depthBytesPerPixel() + abBytesPerPixel() +
               confidenceBytesPerPixel();
    }

    /**
     * @brief Equality comparison.
     */
    bool operator==(const BitDepthConfiguration &other) const {
        return depthBits == other.depthBits && abBits == other.abBits &&
               confidenceBits == other.confidenceBits;
    }

    bool operator!=(const BitDepthConfiguration &other) const {
        return !(*this == other);
    }

  private:
    static bool isValidBitDepth(unsigned int bits) {
        return bits == 0 || bits == 4 || bits == 8 || bits == 10 ||
               bits == 12 || bits == 14 || bits == 16;
    }
};

/**
 * @brief Complete frame configuration combining dimensions and bit depths.
 *
 * Aggregates frame dimensions and bit depth configuration into a single
 * value object. Provides computed properties for buffer management.
 */
struct FrameConfiguration {
    FrameDimensions dimensions;      /**< Frame size */
    BitDepthConfiguration bitDepths; /**< Bit depth settings */
    unsigned int modeNumber;         /**< Frame mode identifier */
    bool isRawBypass; /**< True if ISP bypasses depth computation */

    /**
     * @brief Constructs frame configuration.
     *
     * @param dims Frame dimensions
     * @param bits Bit depth configuration
     * @param mode Mode number
     * @param rawBypass Raw bypass flag
     */
    FrameConfiguration(
        const FrameDimensions &dims = FrameDimensions(),
        const BitDepthConfiguration &bits = BitDepthConfiguration(),
        unsigned int mode = 0, bool rawBypass = false)
        : dimensions(dims), bitDepths(bits), modeNumber(mode),
          isRawBypass(rawBypass) {}

    /**
     * @brief Checks if configuration is valid.
     *
     * @return true if both dimensions and bit depths are valid
     */
    bool isValid() const { return dimensions.isValid() && bitDepths.isValid(); }

    /**
     * @brief Calculates total frame buffer size.
     *
     * @return Buffer size in bytes for complete frame (depth + AB + confidence)
     */
    unsigned int totalBufferSize() const {
        return dimensions.totalPixels() * bitDepths.totalBytesPerPixel();
    }

    /**
     * @brief Gets depth plane buffer size.
     *
     * @return Depth buffer size in bytes
     */
    unsigned int depthBufferSize() const {
        return dimensions.totalPixels() * bitDepths.depthBytesPerPixel();
    }

    /**
     * @brief Gets AB plane buffer size.
     *
     * @return AB buffer size in bytes
     */
    unsigned int abBufferSize() const {
        return dimensions.totalPixels() * bitDepths.abBytesPerPixel();
    }

    /**
     * @brief Gets confidence plane buffer size.
     *
     * @return Confidence buffer size in bytes
     */
    unsigned int confidenceBufferSize() const {
        return dimensions.totalPixels() * bitDepths.confidenceBytesPerPixel();
    }

    /**
     * @brief Equality comparison.
     */
    bool operator==(const FrameConfiguration &other) const {
        return dimensions == other.dimensions && bitDepths == other.bitDepths &&
               modeNumber == other.modeNumber &&
               isRawBypass == other.isRawBypass;
    }

    bool operator!=(const FrameConfiguration &other) const {
        return !(*this == other);
    }
};

} // namespace aditof

#endif // FRAME_CONFIGURATION_H
