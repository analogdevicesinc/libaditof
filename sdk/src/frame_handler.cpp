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

/**
 * @brief Default constructor for FrameHandler.
 *
 * Creates a new FrameHandler instance and initializes the internal
 * FrameHandlerImpl implementation object. This class follows the PIMPL
 * (Pointer to IMPLementation) idiom to hide implementation details.
 */
FrameHandler::FrameHandler() : m_impl(std::make_unique<FrameHandlerImpl>()) {}

/**
 * @brief Destructor for FrameHandler.
 *
 * Cleans up the FrameHandler and releases the internal implementation object.
 */
FrameHandler::~FrameHandler() = default;

/**
 * @brief Move constructor for FrameHandler.
 *
 * Transfers ownership of the internal implementation from another FrameHandler.
 * The source FrameHandler is left in a valid but unspecified state.
 */
FrameHandler::FrameHandler(FrameHandler &&) noexcept = default;

/**
 * @brief Move assignment operator for FrameHandler.
 *
 * Transfers ownership of the internal implementation from another FrameHandler.
 * The source FrameHandler is left in a valid but unspecified state.
 *
 * @return A reference to this FrameHandler object.
 */
FrameHandler &FrameHandler::operator=(FrameHandler &&) noexcept = default;

/**
 * @brief Sets the output file path for saving frames.
 *
 * Configures the directory path where frame data will be saved. The path
 * should exist or be writable for frame storage operations to succeed.
 *
 * @param filePath The directory path where frames will be saved.
 * @return Status::OK on success, or an error status if the path is invalid.
 */
Status FrameHandler::setOutputFilePath(const std::string &filePath) {
    return m_impl->setOutputFilePath(filePath);
}

/**
 * @brief Sets the input file name for reading frames.
 *
 * Configures the file path from which frame data will be read. The file
 * must exist and contain valid frame data in the expected format.
 *
 * @param fullFileName The full path to the input frame file.
 * @return Status::OK on success, or an error status if the file cannot be
 *         opened or is invalid.
 */
Status FrameHandler::setInputFileName(const std::string &fullFileName) {
    return m_impl->setInputFileName(fullFileName);
}

/**
 * @brief Saves a single frame to a file (single-threaded).
 *
 * Writes the frame data to disk using a single-threaded approach. This is
 * suitable for occasional saves or when thread safety is a concern.
 *
 * @param frame The Frame object containing the data to save.
 * @param fileName The name of the file to create (relative to the output path
 *                 if set, or absolute path).
 * @return Status::OK on success, or an error status if the write fails.
 */
Status FrameHandler::saveFrameToFile(aditof::Frame &frame,
                                     const std::string &fileName) {
    return m_impl->saveFrameToFile(frame, fileName);
}

/**
 * @brief Saves a frame to a file using multithreading.
 *
 * Writes the frame data to disk using a background worker thread. This allows
 * the caller to continue processing while the frame is being saved. Useful for
 * high-throughput frame capture scenarios where disk I/O should not block
 * frame acquisition.
 *
 * @param frame The Frame object containing the data to save.
 * @param fileName The name of the file to create (relative to the output path
 *                 if set, or absolute path).
 * @return Status::OK on success, or an error status if the write operation
 *         cannot be queued.
 */
Status FrameHandler::saveFrameToFileMultithread(aditof::Frame &frame,
                                                const std::string &fileName) {

    return m_impl->saveFrameToFileMultithread(frame, fileName);
}

/**
 * @brief Reads the next frame from an input file.
 *
 * Loads frame data from disk into the provided Frame object. If reading from
 * a multi-frame file, this advances to the next frame on each call.
 *
 * @param frame Output Frame object where the loaded data will be stored.
 * @param fullFileName The full path to the input file to read from.
 * @return Status::OK on success, Status::UNAVAILABLE if no more frames are
 *         available, or an error status if the read fails.
 */
Status FrameHandler::readNextFrame(aditof::Frame &frame,
                                   const std::string &fullFileName) {
    return m_impl->readNextFrame(frame, fullFileName);
}

/**
 * @brief Sets a custom file format for frame storage.
 *
 * Configures the format string used when saving frames. This allows
 * customization of the file naming scheme or format.
 *
 * @param format The custom format string to use.
 * @return Status::OK on success, or an error status if the format is invalid.
 */
Status FrameHandler::setCustomFormat(const std::string &format) {
    return m_impl->setCustomFormat(format);
}

/**
 * @brief Configures whether to store multiple frames in a single file.
 *
 * When enabled, successive frames are appended to a single file rather than
 * creating separate files for each frame. Useful for recording frame sequences.
 *
 * @param enable true to store frames in a single file, false to create separate
 *               files for each frame.
 * @return Status::OK on success.
 */
Status FrameHandler::storeFramesToSingleFile(bool enable) {
    return m_impl->storeFramesToSingleFile(enable);
}

/**
 * @brief Sets the content types to include when saving frames.
 *
 * Specifies which frame data types (e.g., "depth", "ab", "confidence", "xyz")
 * should be included when saving frames to disk. This allows selective storage
 * of only the required data types.
 *
 * @param frameContent A string specifying the frame content to save (e.g.,
 *                     "depth", "depth,ab", "all").
 * @return Status::OK on success, or an error status if the content specification
 *         is invalid.
 */
Status FrameHandler::setFrameContent(const std::string &frameContent) {
    return m_impl->setFrameContent(frameContent);
}

/**
 * @brief Saves a snapshot of frame data as image files.
 *
 * Exports frame data to disk as human-readable image formats (JPEG, PNG, or PLY).
 * This is useful for quick visualization or debugging. Saves depth and AB data
 * as separate image files with the specified base filename.
 *
 * @param baseFileName The base filename for the snapshot files (extension will
 *                     be added automatically).
 * @param frame Pointer to the Frame object containing metadata and overall frame
 *              information.
 * @param ab Pointer to the AB (amplitude/brightness) data buffer to save as an
 *           image (8-bit format expected).
 * @param depth Pointer to the depth data buffer to save as an image (8-bit format
 *              expected).
 * @return Status::OK on success, or an error status if the snapshot cannot be saved.
 */
aditof::Status FrameHandler::SnapShotFrames(const char *baseFileName,
                                            aditof::Frame *frame,
                                            const uint8_t *ab,
                                            const uint8_t *depth) {
    return m_impl->SnapShotFrames(baseFileName, frame, ab, depth);
}