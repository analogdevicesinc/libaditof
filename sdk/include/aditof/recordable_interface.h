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
#ifndef RECORDABLE_INTERFACE_H
#define RECORDABLE_INTERFACE_H

#include <aditof/status_definitions.h>
#include <cstdint>
#include <string>

namespace aditof {

/**
 * @class RecordableInterface
 * @brief Interface for sensors that support frame recording (ISP segregation).
 * 
 * Enables live frame capture to be written to disk for later analysis or playback.
 * Only sensors with active hardware capture implement this interface.
 * 
 * Offline/playback-only sensors should NOT implement this interface.
 */
class RecordableInterface {
  public:
    /**
     * @brief Destructor
     */
    virtual ~RecordableInterface() = default;

    /**
     * @brief Start recording frames to a file
     * @param[in] fileName - Name of the file to record to
     * @param[in] parameters - Recording parameters buffer
     * @param[in] paramSize - Size of parameters buffer in bytes
     * @return Status
     */
    virtual aditof::Status startRecording(std::string &fileName,
                                          uint8_t *parameters,
                                          uint32_t paramSize) = 0;

    /**
     * @brief Stop the current recording session
     * @return Status
     */
    virtual aditof::Status stopRecording() = 0;
};

} // namespace aditof

#endif // RECORDABLE_INTERFACE_H
