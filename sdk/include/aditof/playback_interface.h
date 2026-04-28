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
#ifndef PLAYBACK_INTERFACE_H
#define PLAYBACK_INTERFACE_H

#include <aditof/status_definitions.h>
#include <cstdint>
#include <string>

namespace aditof {

/**
 * @class PlaybackInterface
 * @brief Interface for sensors that support frame playback from files (ISP segregation).
 * 
 * Enables reading previously recorded frames from disk for offline analysis.
 * Only offline/simulation sensors implement this interface.
 * 
 * Hardware capture sensors should NOT implement this interface.
 */
class PlaybackInterface {
  public:
    /**
     * @brief Destructor
     */
    virtual ~PlaybackInterface() = default;

    /**
     * @brief Set the file to use for frame playback
     * @param[in] filePath - Path to the playback file
     * @return Status
     */
    virtual aditof::Status setPlaybackFile(const std::string filePath) = 0;

    /**
     * @brief Stop the current playback session
     * @return Status
     */
    virtual aditof::Status stopPlayback() = 0;

    /**
     * @brief Get header information from recorded file
     * @param[out] buffer - Buffer to store header data
     * @param[in] bufferSize - Size of the buffer in bytes
     * @return Status
     */
    virtual aditof::Status getHeader(uint8_t *buffer, uint32_t bufferSize) = 0;

    /**
     * @brief Get the total number of frames in playback file
     * @param[out] frameCount - Variable to store the frame count
     * @return Status
     */
    virtual aditof::Status getFrameCount(uint32_t &frameCount) = 0;
};

} // namespace aditof

#endif // PLAYBACK_INTERFACE_H
