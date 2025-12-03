/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef FRAME_HANDLER
#define FRAME_HANDLER

#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/frame_handler.h>
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

class FrameHandlerImpl {
  public:
    FrameHandlerImpl();
    ~FrameHandlerImpl();

    //Open existing file/create new file to store data
    aditof::Status setOutputFilePath(const std::string &filePath);
    aditof::Status setInputFileName(const std::string &fullFileName);

    //the api should only use frame objects when saving/reading data from file
    //the conversion between formats should happen inside the functions

    //store frame to file
    aditof::Status saveFrameToFile(aditof::Frame &frame,
                                   const std::string &fileName = "");

    //aditof::Status splitFrames(bool enable);
    aditof::Status storeFramesToSingleFile(bool enable);
    //aditof::Status storeSingleFrameToFile(bool enable);
    //aditof::Status storeToSingleFile(bool enable);
    
    aditof::Status SnapShotFrames(const char *baseFileName,
                                  aditof::Frame *frame, const uint8_t *ab,
                                  const uint8_t *depth);

  private:
    aditof::Status SaveRGBAsJPEG(const char *filename, const uint8_t *data,
                                 uint32_t width, uint32_t height);
    aditof::Status SaveFloatAsJPEG(const char *filename, const float *data,
                                   uint32_t width, uint32_t height);
    aditof::Status SaveUint16AsJPEG(const char *filename, const uint16_t *data,
                                    uint32_t width, uint32_t height);
    aditof::Status SavePointCloudPLYBinary(const char *filename,
                                           const uint16_t *data, uint32_t width,
                                           uint32_t height);
    aditof::Status SaveMetaAsTxt(const char *filename,
                                 const aditof::Metadata *data);
    std::string getOutputFileFullPath(const std::string &fileName);

    aditof::Status createFile(const std::string &fileName);
    void threadWritter();
    //aditof::Status writtingThread(std::string fileName = "");
    //We should be able do decide if we want to store frames in the same file
    //or store them in different files
    bool m_concatFrames;

    //Let the users decide if they want to use multithreading or not
    bool m_enableMultithreading;

    //we should offer a standart format that would be compatible with our examples
    //(viewer/data-collect/python bindings/etc)
    //it would be nice if we could give the users the posibility to store data in other
    //formats (mp4, avi, etc)
    bool m_customFormat;
    std::string m_customFormatType;

    //relevant data extracted from metadata that can help us compute frame size
    int m_bitsInDepth;
    int m_bitsInAB;
    int m_bitsInConf;
    int m_frameWidth;
    int m_frameHeight;
    int m_frameIndex;

    aditof::Frame m_swapFrame;
    aditof::FrameDetails m_frDetails;
    aditof::Metadata m_metadataStruct;

    //variables used for file handling
    std::string m_dir;
    std::string m_outputFileName;
    std::string m_fullInputFileName;
    std::string m_inputFileName;
    bool m_fileCreated;
    bool m_endOfFile;
    std::fstream m_file;
    size_t m_pos;
    int m_frameCount = 0;

    //multithread variables
    std::mutex m_mutex;
    std::thread m_threadWorker;
    std::queue<aditof::Frame> m_frameQueue;
    std::queue<std::string> m_frameNameQueue;
    volatile bool m_threadRunning;
};

#endif // FRAME_HANDLER