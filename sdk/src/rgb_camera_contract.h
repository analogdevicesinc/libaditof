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

#ifdef HAS_RGB_CAMERA

#include <aditof/ar0234_sensor.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace aditof {

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

/**
 * @brief Encodes @p metadata as a GW5 ancillary-data packet.
 *
 * Stands in for the camera on hardware without embedded data; fields not in
 * RgbFrameMetadata are zero.
 */
void writeGw5AncillaryData(const RgbFrameMetadata &metadata,
                           std::vector<uint8_t> &packet);

} // namespace aditof

#endif // HAS_RGB_CAMERA

#endif // RGB_CAMERA_CONTRACT_H
