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
#include "adsd3500_recorder.h"
#include "buffer_processor.h"
#include "buffer_processor_interface.h"
#include <aditof/log.h>

namespace aditof {

Adsd3500Recorder::Adsd3500Recorder(BufferProcessorInterface *bufferProcessor)
    : m_bufferProcessor(bufferProcessor), m_isRecordingActive(false),
      m_isPlaybackActive(false) {}

Adsd3500Recorder::~Adsd3500Recorder() = default;

Status Adsd3500Recorder::startRecording(const std::string &fileName,
                                        uint8_t *parameters,
                                        uint32_t paramSize) {
    if (!m_bufferProcessor) {
        LOG(ERROR) << "Buffer processor not initialized";
        return Status::GENERIC_ERROR;
    }

    // Cast to concrete BufferProcessor for recording functionality
    auto *processor = dynamic_cast<BufferProcessor *>(m_bufferProcessor);
    if (!processor) {
        LOG(ERROR) << "Buffer processor does not support recording";
        return Status::UNAVAILABLE;
    }

    std::string fileNameCopy = fileName;
    processor->startRecording(fileNameCopy, parameters, paramSize);
    m_isRecordingActive = true;

    LOG(INFO) << "Started recording to: " << fileName;
    return Status::OK;
}

Status Adsd3500Recorder::stopRecording() {
    if (!m_isRecordingActive) {
        LOG(WARNING) << "Recording not active";
        return Status::OK;
    }

    if (!m_bufferProcessor) {
        LOG(ERROR) << "Buffer processor not initialized";
        return Status::GENERIC_ERROR;
    }

    // Cast to concrete BufferProcessor for recording functionality
    auto *processor = dynamic_cast<BufferProcessor *>(m_bufferProcessor);
    if (!processor) {
        LOG(ERROR) << "Buffer processor does not support recording";
        return Status::UNAVAILABLE;
    }

    Status status = processor->stopRecording();
    if (status == Status::OK) {
        m_isRecordingActive = false;
        LOG(INFO) << "Stopped recording";
    }

    return status;
}

Status Adsd3500Recorder::setPlaybackFile(const std::string &filePath) {
    // Playback not supported on target sensors
    LOG(WARNING) << "Playback not supported on target sensor";
    return Status::UNAVAILABLE;
}

Status Adsd3500Recorder::stopPlayback() {
    // Playback not supported on target sensors
    LOG(WARNING) << "Playback not supported on target sensor";
    return Status::UNAVAILABLE;
}

Status Adsd3500Recorder::getHeader(uint8_t *buffer, uint32_t bufferSize) {
    // Playback not supported on target sensors
    LOG(WARNING) << "Playback not supported on target sensor";
    return Status::UNAVAILABLE;
}

Status Adsd3500Recorder::getFrameCount(uint32_t &frameCount) {
    // Not implemented for target sensor
    frameCount = 0;
    return Status::UNAVAILABLE;
}

bool Adsd3500Recorder::isRecording() const { return m_isRecordingActive; }

bool Adsd3500Recorder::isPlayback() const { return m_isPlaybackActive; }

} // namespace aditof
