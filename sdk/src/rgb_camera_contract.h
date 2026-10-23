/*
 * MIT License
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef RGB_CAMERA_CONTRACT_H
#define RGB_CAMERA_CONTRACT_H

#include <cstddef>
#include <cstdint>

namespace aditof {

/**
 * @brief Per-frame metadata reported by the RGB camera.
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

/// Size of the GW5 ancillary-data packet (MIPI DT 0x12) sent before each image.
constexpr size_t GW5_ANCILLARY_DATA_SIZE = 181;

enum class Gw5AncillaryStatus {
    OK,
    TOO_SHORT,
    BAD_HEADER,
    UNSUPPORTED_VERSION,
    BAD_PAYLOAD_SIZE,
    BAD_CHECKSUM,
};

/**
 * @brief Validates and decodes one GW5 ancillary-data packet.
 * @param data Start of the packet; needs no alignment.
 * @param size Bytes available at @p data; trailing bytes are ignored.
 * @param[out] metadata Written only when the result is OK.
 */
Gw5AncillaryStatus parseGw5AncillaryData(const uint8_t *data, size_t size,
                                         RgbFrameMetadata &metadata);

} // namespace aditof

#endif // RGB_CAMERA_CONTRACT_H
