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
#include "frame_handler_impl.h"
#include <aditof/frame_handler.h>
#include <aditof/log.h>
#include <memory>
using namespace aditof;

FrameHandler::FrameHandler() : m_impl(std::make_unique<FrameHandlerImpl>()) {}

FrameHandler::~FrameHandler() = default;

FrameHandler::FrameHandler(FrameHandler &&) noexcept = default;

FrameHandler &FrameHandler::operator=(FrameHandler &&) noexcept = default;

Status FrameHandler::setOutputFilePath(const std::string &filePath) {
    return m_impl->setOutputFilePath(filePath);
}

Status FrameHandler::setInputFileName(const std::string &fullFileName) {
    return m_impl->setInputFileName(fullFileName);
}

Status FrameHandler::saveFrameToFile(aditof::Frame &frame,
                                     const std::string &fileName) {
    return m_impl->saveFrameToFile(frame, fileName);
}

Status FrameHandler::saveFrameToFileMultithread(aditof::Frame &frame,
                                                const std::string &fileName) {

    return m_impl->saveFrameToFileMultithread(frame, fileName);
}

Status FrameHandler::readNextFrame(aditof::Frame &frame,
                                   const std::string &fullFileName) {
    return m_impl->readNextFrame(frame, fullFileName);
}

Status FrameHandler::setCustomFormat(const std::string &format) {
    return m_impl->setCustomFormat(format);
}

Status FrameHandler::storeFramesToSingleFile(bool enable) {
    return m_impl->storeFramesToSingleFile(enable);
}

Status FrameHandler::setFrameContent(const std::string &frameContent) {
    return m_impl->setFrameContent(frameContent);
}

aditof::Status FrameHandler::SnapShotFrames(const char *baseFileName,
                                            aditof::Frame *frame,
                                            const uint8_t *ab,
                                            const uint8_t *depth) {
    return m_impl->SnapShotFrames(baseFileName, frame, ab, depth);
}