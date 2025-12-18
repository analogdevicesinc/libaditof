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
#ifndef FRAME_HANDLER_H
#define FRAME_HANDLER_H

#include <aditof/frame.h>
#include <aditof/status_definitions.h>

#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#define METADATA_SIZE 128

class FrameHandlerImpl;

namespace aditof {

/**
 * @class FrameHandler
 * @brief Handles operations on a frame such as saving it to file and creating a new frame with content read from file.
 */
class FrameHandler {
  public:
    /**
     * @brief Constructor for FrameHandler
     */
    SDK_API FrameHandler();

    /**
     * @brief Destructor for FrameHandler
     */
    SDK_API ~FrameHandler();

    /**
     * @brief Move constructor
     */
    SDK_API FrameHandler(FrameHandler &&) noexcept;
    /**
     * @brief Move assignment
     */
    SDK_API FrameHandler &operator=(FrameHandler &&) noexcept;

    /**
     * @brief Set the output path/output folder for the frames
     * @param[in] filePath - File path
     * @return Status
     */
    SDK_API aditof::Status setOutputFilePath(const std::string &filePath);

    /**
     * @brief Set the input file for the frames
     * @param[in] fullFileName - Input file name
     * @return Status
     */
    SDK_API aditof::Status setInputFileName(const std::string &fullFileName);

    /**
     * @brief Store frame to file
     * @param[in] frame - frame object to be stored in file
     * @param[in] fileName - the name of the file. If none provided
     * it will automatically generate one depending on time and frame number.
     * @return Status
     */
    SDK_API aditof::Status saveFrameToFile(aditof::Frame &frame,
                                           const std::string &fileName = "");

    /**
     * @brief Store frame to file using a separate thread.
     * @param[in] frame - frame object to be stored in file
     * @param[in] fileName - the name of the file. If none provided
     * it will automatically generate one depending on time and frame number.
     * @return Status
     */
    SDK_API aditof::Status
    saveFrameToFileMultithread(aditof::Frame &frame,
                               const std::string &fileName = "");

    /**
     * @brief Reads frame from a file. If same file is provided it will 
     * continue to read from the last position
     * @param[out] frame - Frame object in which the frame will be stored
     * @param[in] fullFileName - Full file name
     * @return Status
     */
    SDK_API aditof::Status readNextFrame(aditof::Frame &frame,
                                         const std::string &fullFileName = "");

    /**
     * @brief A custom format in which the frames will be stored/read
     * @param[in] format - Format name
     * @return Status
     */
    SDK_API aditof::Status setCustomFormat(const std::string &format);

    /**
     * @brief Enable/disable if single file is intended to use for 
     * storing data
     * @param[in] enable - true for single file use/ false for multiple file use
     * @return Status
     */
    SDK_API aditof::Status storeFramesToSingleFile(bool enable);
    /**
     * @brief Set which frame element you want to store/read (depth/ab/conf)
     * @param[in] frameContent - a string providing the frame subelements that will be stored
     * @return Status
     */
    SDK_API aditof::Status setFrameContent(const std::string &frameContent);

    /**
     * @brief Take a snapshot of frame data and save to files
     * @param[in] baseFileName - Base name for the output files (without extension)
     * @param[in] frame - Pointer to the frame object containing frame data
     * @param[in] ab - Pointer to the AB (active brightness) data buffer
     * @param[in] depth - Pointer to the depth data buffer
     * @return Status
     */
    SDK_API aditof::Status SnapShotFrames(const char *baseFileName,
                                          aditof::Frame *frame,
                                          const uint8_t *ab,
                                          const uint8_t *depth);

  private:
    std::unique_ptr<FrameHandlerImpl> m_impl;
};
} // namespace aditof
#endif // FRAME_HANDLER_H