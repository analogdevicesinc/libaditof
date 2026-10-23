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

#include "rgb_camera_contract.h"

#include <cstring>

#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "GW5 ancillary data is little-endian; add byte swapping for this target"
#endif

namespace aditof {

namespace {

constexpr uint8_t GW5_MAGIC[4] = {'G', 'W', '5', 'A'};
constexpr uint8_t GW5_PAYLOAD_MAJOR_VERSION = 0;

#pragma pack(push, 1)
struct Gw5AncillaryWire {
    uint8_t header[4];
    uint8_t payloadMajorVersion;
    uint8_t payloadMinorVersion;
    uint16_t payloadNumBytes;

    uint32_t gw5State;
    float gw5Temperature;
    uint32_t inputFrameErrorCount;
    uint32_t inputFrameCount;
    uint32_t inputFrameCrc;
    uint64_t inputFrameTimestampMs;
    uint32_t outputFrameCount;
    uint32_t outputFrameCrc;
    uint64_t outputFrameTimestampMs;

    float totalGain[4];
    float sensorGain[4];
    float ispGain[4];
    float unclippedEv[4];
    float unclippedEvAvg;
    uint16_t integrationLines[4];
    uint16_t vts;
    uint8_t padAe[2];
    float frameRate;
    uint8_t numberOfSensors;
    uint8_t padSensor[3];
    uint16_t longMediumRatio;
    uint16_t mediumShortRatio;
    uint16_t shortVeryShortRatio;
    uint8_t padCombiner[2];
    int16_t colorMatrix[9];
    uint8_t padCcm[2];
    uint16_t ltmGain;
    uint8_t padLtm[2];
    uint16_t blackOffsetR;
    uint16_t blackOffsetGR;
    uint16_t blackOffsetGB;
    uint16_t blackOffsetB;

    uint8_t payloadChecksum;
};
#pragma pack(pop)

constexpr size_t PAYLOAD_BEGIN = offsetof(Gw5AncillaryWire, gw5State);
constexpr size_t PAYLOAD_END = offsetof(Gw5AncillaryWire, payloadChecksum);
constexpr size_t PAYLOAD_SIZE = PAYLOAD_END - PAYLOAD_BEGIN;

static_assert(sizeof(Gw5AncillaryWire) == GW5_ANCILLARY_DATA_SIZE,
              "GW5 ancillary data layout changed");
static_assert(PAYLOAD_BEGIN == 8 && PAYLOAD_SIZE == 172,
              "GW5 ancillary payload layout changed");
static_assert(offsetof(Gw5AncillaryWire, totalGain) == 52 &&
                  offsetof(Gw5AncillaryWire, integrationLines) == 120 &&
                  offsetof(Gw5AncillaryWire, frameRate) == 132 &&
                  offsetof(Gw5AncillaryWire, colorMatrix) == 148,
              "GW5 ancillary field offsets changed");

} // namespace

Gw5AncillaryStatus parseGw5AncillaryData(const uint8_t *data, size_t size,
                                         RgbFrameMetadata &metadata) {
    if (!data || size < GW5_ANCILLARY_DATA_SIZE) {
        return Gw5AncillaryStatus::TOO_SHORT;
    }

    Gw5AncillaryWire wire;
    std::memcpy(&wire, data, sizeof(wire));

    if (std::memcmp(wire.header, GW5_MAGIC, sizeof(GW5_MAGIC)) != 0) {
        return Gw5AncillaryStatus::BAD_HEADER;
    }
    if (wire.payloadMajorVersion != GW5_PAYLOAD_MAJOR_VERSION) {
        return Gw5AncillaryStatus::UNSUPPORTED_VERSION;
    }
    if (wire.payloadNumBytes != PAYLOAD_SIZE) {
        return Gw5AncillaryStatus::BAD_PAYLOAD_SIZE;
    }

    uint8_t sum = 0;
    for (size_t i = PAYLOAD_BEGIN; i < PAYLOAD_END; ++i) {
        sum = static_cast<uint8_t>(sum + data[i]);
    }
    if (sum != wire.payloadChecksum) {
        return Gw5AncillaryStatus::BAD_CHECKSUM;
    }

    metadata.outputFrameCount = wire.outputFrameCount;
    metadata.inputFrameCount = wire.inputFrameCount;
    metadata.inputFrameErrorCount = wire.inputFrameErrorCount;
    metadata.outputTimestampMs = wire.outputFrameTimestampMs;
    metadata.inputTimestampMs = wire.inputFrameTimestampMs;
    metadata.ispState = wire.gw5State;
    metadata.ispTemperatureC = wire.gw5Temperature;
    metadata.frameRate = wire.frameRate;
    metadata.vts = wire.vts;
    std::memcpy(metadata.integrationLines, wire.integrationLines,
                sizeof(metadata.integrationLines));
    std::memcpy(metadata.totalGain, wire.totalGain, sizeof(metadata.totalGain));
    std::memcpy(metadata.sensorGain, wire.sensorGain,
                sizeof(metadata.sensorGain));
    std::memcpy(metadata.ispGain, wire.ispGain, sizeof(metadata.ispGain));

    return Gw5AncillaryStatus::OK;
}

} // namespace aditof
