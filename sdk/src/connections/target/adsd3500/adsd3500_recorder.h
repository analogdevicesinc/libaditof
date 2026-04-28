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
#ifndef ADSD3500_RECORDER_H
#define ADSD3500_RECORDER_H

#include <aditof/status_definitions.h>
#include <cstdint>
#include <string>

namespace aditof {

class BufferProcessorInterface;

/**
 * @brief Manages stream recording and playback for ADSD3500 frames.
 *
 * Encapsulates file-based recording of depth/AB/confidence frames and playback
 * of previously recorded data. Separates recording concerns from sensor operations
 * (SRP compliance).
 */
class Adsd3500Recorder {
  public:
    explicit Adsd3500Recorder(BufferProcessorInterface *bufferProcessor);
    ~Adsd3500Recorder();

    /**
     * @brief Start recording frames to file.
     * @param[in] fileName Path to output recording file
     * @param[in] parameters Recording parameters (mode info, resolution, etc.)
     * @param[in] paramSize Size of parameters buffer in bytes
     * @return Status::OK on success, error code otherwise
     */
    Status startRecording(const std::string &fileName, uint8_t *parameters,
                          uint32_t paramSize);

    /**
     * @brief Stop active recording and close file.
     * @return Status::OK on success, error code otherwise
     */
    Status stopRecording();

    /**
     * @brief Set file for playback mode.
     * @param[in] filePath Path to recording file for playback
     * @return Status::OK on success, error code otherwise
     */
    Status setPlaybackFile(const std::string &filePath);

    /**
     * @brief Stop playback and close file.
     * @return Status::OK on success, error code otherwise
     */
    Status stopPlayback();

    /**
     * @brief Get recording header information.
     * @param[out] buffer Buffer to receive header data
     * @param[in] bufferSize Size of buffer in bytes
     * @return Status::OK on success, error code otherwise
     */
    Status getHeader(uint8_t *buffer, uint32_t bufferSize);

    /**
     * @brief Get total frame count in recording.
     * @param[out] frameCount Total number of frames recorded
     * @return Status::OK on success, error code otherwise
     */
    Status getFrameCount(uint32_t &frameCount);

    /**
     * @brief Check if currently recording.
     * @return true if recording active, false otherwise
     */
    bool isRecording() const;

    /**
     * @brief Check if currently in playback mode.
     * @return true if playback active, false otherwise
     */
    bool isPlayback() const;

  private:
    BufferProcessorInterface *m_bufferProcessor;
    bool m_isRecordingActive;
    bool m_isPlaybackActive;
};

} // namespace aditof

#endif // ADSD3500_RECORDER_H
