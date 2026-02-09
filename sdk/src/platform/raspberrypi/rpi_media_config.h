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
#ifndef RPI_MEDIA_CONFIG_H
#define RPI_MEDIA_CONFIG_H

#include <string>

namespace aditof {
namespace platform {
namespace rpi {

/**
 * @brief Configure RP1 CFE media pipeline to fix format validation errors
 * 
 * Raspberry Pi 5 requires specific media-ctl configuration before streaming
 * to ensure format consistency between sensor, CSI2, and video device.
 * 
 * @param mediaDevice Media device path (e.g., "/dev/media0")
 * @param videoDevice Video device path (e.g., "/dev/video0")
 * @param sensorEntity Sensor entity name (e.g., "adsd3500 10-0038")
 * @param width Frame width
 * @param height Frame height
 * @param bitDepth Bit depth (8 or 12)
 * @return true if configuration succeeded, false otherwise
 */
/**
 * @brief Detect the RP1 CFE media device
 * 
 * Scans available media devices to find the one with rp1-cfe driver.
 * 
 * @return Media device path (e.g., "/dev/media3"), or empty string if not found
 */
std::string detectRpiMediaDevice();

/**
 * @brief Detect the ADSD3500 sensor entity name
 * 
 * Queries the media device to find the adsd3500 sensor entity.
 * The I2C address may vary (e.g., "adsd3500 10-0038" or "adsd3500 10-0043").
 * 
 * @param mediaDevice Media device path (e.g., "/dev/media3")
 * @return Sensor entity name (e.g., "adsd3500 10-0038"), or empty string if not found
 */
std::string detectSensorEntity(const std::string &mediaDevice);

bool configureMediaPipeline(const std::string &mediaDevice,
                            const std::string &videoDevice,
                            const std::string &sensorEntity, int width,
                            int height, int bitDepth);

} // namespace rpi
} // namespace platform
} // namespace aditof

#endif // RPI_MEDIA_CONFIG_H
