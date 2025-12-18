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

    //TO DO: write function to save frames on different thread
    //aditof::Status enqueueFrameToSaveToFile(aditof::Frame frame);
    aditof::Status saveFrameToFileMultithread(aditof::Frame &frame,
                                              const std::string &filename = "");

    //read new frame from file and process metadata to get new frame
    //charateristics if we have different frame types in the same file
    aditof::Status readNextFrame(aditof::Frame &frame,
                                 const std::string &fullFileName = "");

    //We could offer support for a couple of standart formats (avi/mp4/..)
    //and let the users decide between them
    aditof::Status setCustomFormat(const std::string &format);

    //aditof::Status splitFrames(bool enable);
    aditof::Status storeFramesToSingleFile(bool enable);
    //aditof::Status storeSingleFrameToFile(bool enable);
    //aditof::Status storeToSingleFile(bool enable);

    //we should be able to give the users the ability to choose which data
    //type they want to store (depth/ab/conf/metadata/full-data) or any combinations
    //between this 2
    //NOTE: metadata should be always enabled for a better data processing
    aditof::Status setFrameContent(const std::string &frameContent);
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