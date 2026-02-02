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
#include <aditof/log.h>
#include <algorithm>
#include <cstring>
#include <memory>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <aditof/stb_image_write.h>

using namespace aditof;

FrameHandlerImpl::FrameHandlerImpl()
    : m_concatFrames(true), m_enableMultithreading(false),
      m_customFormat(false), m_bitsInDepth(0), m_bitsInAB(0), m_bitsInConf(0),
      m_frameWidth(0), m_frameHeight(0), m_frameIndex(0), m_fileCreated(false),
      m_endOfFile(false), m_pos(0), m_threadRunning(false) {}

FrameHandlerImpl::~FrameHandlerImpl() {
    m_dir = ".";
    if (m_threadWorker.joinable()) {
        m_threadWorker.join();
    }
}

Status FrameHandlerImpl::setOutputFilePath(const std::string &filePath) {
    Status status = Status::OK;
    m_dir = filePath;
    m_fileCreated = false;
    return status;
}

Status FrameHandlerImpl::setInputFileName(const std::string &fullFileName) {
    Status status = Status::OK;
    m_fullInputFileName = fullFileName;
    m_pos = 0;
    return status;
}

Status FrameHandlerImpl::saveFrameToFile(aditof::Frame &frame,
                                         const std::string &fileName) {
    Status status = Status::OK;

    if (m_concatFrames) {
        if (!m_fileCreated) {
            status = createFile(fileName);
        } else {
            m_file = std::fstream(getOutputFileFullPath(m_outputFileName),
                                  std::ios::app | std::ios::binary);
            m_file.seekg(std::ios::end);
        }
    } else {
        status = createFile(fileName);
    }

    m_inputFileName = fileName;

    if (status != Status::OK) {
        LOG(ERROR) << "Failed to create file!";
        return status;
    }

    //Store frames in file in followind order: metadata depth ab conf
    uint16_t *metaData;
    uint16_t *depthData;
    uint16_t *abData;
    uint16_t *confData;
    uint16_t *xyzData;

    frame.getData("metadata", &metaData);

    Metadata metadataStruct;
    frame.getMetadataStruct(metadataStruct);

    //at first we assume that we have metadata enabled by default
    //TO DO: implement use-case where we don't have metadata
    m_file.write(reinterpret_cast<char *>(metaData), METADATA_SIZE);

    if (metadataStruct.bitsInDepth) {
        frame.getData("depth", &depthData);
        m_file.write(reinterpret_cast<char *>(depthData),
                     metadataStruct.width * metadataStruct.height * 2);
    }

    if (metadataStruct.bitsInAb) {
        frame.getData("ab", &abData);
        m_file.write(reinterpret_cast<char *>(abData),
                     metadataStruct.width * metadataStruct.height * 2);
    }

    if (metadataStruct.bitsInConfidence) {
        frame.getData("conf", &confData);
        m_file.write(reinterpret_cast<char *>(confData),
                     metadataStruct.width * metadataStruct.height * 4);
    }

    if (metadataStruct.xyzEnabled) {
        frame.getData("xyz", &xyzData);
        m_file.write(reinterpret_cast<char *>(xyzData),
                     metadataStruct.width * metadataStruct.height * 6);
    }

    m_file.close();

    if (!m_frameQueue.empty()) {
        m_mutex.lock();
        m_frameQueue.pop();
        m_frameNameQueue.pop();
        m_mutex.unlock();
    }

    return status;
}

Status
FrameHandlerImpl::saveFrameToFileMultithread(aditof::Frame &frame,
                                             const std::string &fileName) {

    using namespace aditof;
    Status status = Status::OK;

    m_mutex.lock();
    m_frameQueue.push(std::move(frame));
    m_frameNameQueue.push(fileName);
    m_mutex.unlock();

    frame = Frame();

    if (!m_threadRunning) {
        if (m_threadWorker.joinable()) {
            m_threadWorker.join();
        }

        m_threadWorker =
            std::thread(std::bind(&FrameHandlerImpl::threadWritter, this));
    }

    return status;
}

void FrameHandlerImpl::threadWritter() {
    m_threadRunning = true;
    aditof::Status status;

    while (!m_frameQueue.empty()) {
        status =
            saveFrameToFile(m_frameQueue.front(), m_frameNameQueue.front());
        if (status != aditof::Status::OK)
            return; // status;
    }

    m_threadRunning = false;
    return; // status;
}

Status FrameHandlerImpl::readNextFrame(aditof::Frame &frame,
                                       const std::string &fullFileName) {
    Status status = Status::OK;
    if (m_fullInputFileName.empty() && fullFileName.empty()) {
        LOG(ERROR) << "No input file provided!";
        return Status::GENERIC_ERROR;
    }

    if (fullFileName != m_fullInputFileName) {
        m_fullInputFileName = fullFileName;
        m_pos = 0;
    }

    m_file = std::fstream(m_fullInputFileName, std::ios::in | std::ios::binary);

    if (!m_file) {
        LOG(ERROR) << "Failed open file!";
        return Status::GENERIC_ERROR;
    }

    m_file.seekg(m_pos, std::ios::beg);
    if (m_file.eof()) {
        LOG(WARNING) << "End of file reached! No more frames left to read.";
        m_file.close();
        return Status::UNAVAILABLE;
    }

    m_file.read(reinterpret_cast<char *>(&m_metadataStruct), METADATA_SIZE);

    m_frDetails.width = m_metadataStruct.width;
    m_frDetails.height = m_metadataStruct.height;
    m_frDetails.cameraMode = std::to_string(m_metadataStruct.imagerMode);
    m_frDetails.totalCaptures = 1;
    m_frDetails.cameraMode = m_metadataStruct.imagerMode;

    FrameDataDetails frDataDetails;
    frDataDetails.type = "metadata";
    frDataDetails.width = METADATA_SIZE;
    frDataDetails.height = 1;
    frDataDetails.subelementSize = sizeof(uint8_t);
    frDataDetails.subelementsPerElement = 1;
    m_frDetails.dataDetails.emplace_back(frDataDetails);

    if (m_metadataStruct.bitsInDepth) {
        frDataDetails.type = "depth";
        frDataDetails.width = m_metadataStruct.width;
        frDataDetails.height = m_metadataStruct.height;
        frDataDetails.subelementSize = sizeof(uint16_t);
        frDataDetails.subelementsPerElement = 1;
        m_frDetails.dataDetails.emplace_back(frDataDetails);
    }

    if (m_metadataStruct.bitsInAb) {
        frDataDetails.type = "ab";
        frDataDetails.width = m_metadataStruct.width;
        frDataDetails.height = m_metadataStruct.height;
        frDataDetails.subelementSize = sizeof(uint16_t);
        frDataDetails.subelementsPerElement = 1;
        m_frDetails.dataDetails.emplace_back(frDataDetails);
    }

    if (m_metadataStruct.bitsInConfidence) {
        frDataDetails.type = "conf";
        frDataDetails.width = m_metadataStruct.width;
        frDataDetails.height = m_metadataStruct.height;
        frDataDetails.subelementSize = sizeof(float);
        frDataDetails.subelementsPerElement = 1;
        m_frDetails.dataDetails.emplace_back(frDataDetails);
    }

    if (m_metadataStruct.xyzEnabled) {
        frDataDetails.type = "xyz";
        frDataDetails.width = m_metadataStruct.width;
        frDataDetails.height = m_metadataStruct.height;
        frDataDetails.subelementSize = sizeof(uint16_t);
        frDataDetails.subelementsPerElement = 3;
        m_frDetails.dataDetails.emplace_back(frDataDetails);
    }

    status = frame.setDetails(m_frDetails, m_metadataStruct.bitsInConfidence,
                              m_metadataStruct.bitsInAb);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to set frame details.";
        return status;
    }

    //Read frames from the file in followind order: metadata depth ab conf xyz
    uint16_t *metaData;
    uint16_t *depthData;
    uint16_t *abData;
    uint16_t *confData;
    uint16_t *xyzData;

    frame.getData("metadata", &metaData);
    memcpy(metaData, reinterpret_cast<uint8_t *>(&m_metadataStruct),
           METADATA_SIZE);

    if (m_metadataStruct.bitsInDepth) {
        frame.getData("depth", &depthData);
        m_file.read(reinterpret_cast<char *>(depthData),
                    m_metadataStruct.width * m_metadataStruct.height * 2);
    }

    if (m_metadataStruct.bitsInAb) {
        frame.getData("ab", &abData);
        m_file.read(reinterpret_cast<char *>(abData),
                    m_metadataStruct.width * m_metadataStruct.height * 2);
    }

    if (m_metadataStruct.bitsInConfidence) {
        frame.getData("conf", &confData);
        m_file.read(reinterpret_cast<char *>(confData),
                    m_metadataStruct.width * m_metadataStruct.height * 4);
    }

    if (m_metadataStruct.xyzEnabled) {
        frame.getData("xyz", &xyzData);
        m_file.read(reinterpret_cast<char *>(xyzData),
                    m_metadataStruct.width * m_metadataStruct.height * 6);
    }

    m_pos = m_file.tellg();
    m_file.close();

    return Status::OK;
}

Status FrameHandlerImpl::setCustomFormat(const std::string &format) {
    return Status::UNAVAILABLE;
}

Status FrameHandlerImpl::storeFramesToSingleFile(bool enable) {
    Status status = Status::OK;
    m_concatFrames = enable;

    return status;
}

Status FrameHandlerImpl::setFrameContent(const std::string &frameContent) {
    return Status::UNAVAILABLE;
}

Status FrameHandlerImpl::createFile(const std::string &fileName) {
    if (fileName.empty()) {
        char time_buffer[128];
        time_t rawtime;
        time(&rawtime);
        struct tm timeinfo;
#ifdef _WIN32
        localtime_s(&timeinfo, &rawtime);
#else
        localtime_r(&rawtime, &timeinfo);
#endif
        strftime(time_buffer, sizeof(time_buffer), "%Y_%m_%d_%H_%M_%S",
                 &timeinfo);
        m_outputFileName = "frame" + std::string(time_buffer) + "_" +
                           std::to_string(m_frameCount) + ".bin";
        m_frameCount++;
    } else {
        m_outputFileName = fileName;
    }

    m_file = std::fstream(getOutputFileFullPath(m_outputFileName),
                          std::ios::app | std::ios::out | std::ios::binary);

    if (!m_file) {
        LOG(ERROR) << "Failed to create output file!";
        return Status::GENERIC_ERROR;
    }
    m_fileCreated = true;

    return Status::OK;
}

std::string
FrameHandlerImpl::getOutputFileFullPath(const std::string &fileName) {
    std::string fullPath;
#ifdef _WIN32
    const std::string pathSeparator = "\\";
#else
    const std::string pathSeparator = "//";
#endif

    if (m_dir.empty()) {
        fullPath = fileName;
    } else {
        fullPath = m_dir + pathSeparator + fileName;
    }

    return fullPath;
}

aditof::Status FrameHandlerImpl::SaveFloatAsJPEG(const char *filename,
                                                 const float *data,
                                                 uint32_t width,
                                                 uint32_t height) {

    if (data == nullptr) {
        return aditof::Status::GENERIC_ERROR;
    }

    std::vector<uint8_t> img_8bit(width * height);

    auto minmax = std::minmax_element(data, data + width * height);
    float min_val = *minmax.first;
    float max_val = *minmax.second;

    // Avoid divide by zero
    float range = (max_val == min_val) ? 1.0f : (max_val - min_val);

    for (uint32_t i = 0; i < width * height; ++i) {
        float norm = (data[i] - min_val) / range; // [0,1]
        img_8bit[i] = static_cast<uint8_t>(norm * 255.0f + 0.5f);
    }

    auto result =
        stbi_write_jpg(filename, width, height, 1, img_8bit.data(), 100);

    LOG(INFO) << __func__ << ":  " << result << " " << filename;

    return result ? aditof::Status::OK : aditof::Status::GENERIC_ERROR;
}

aditof::Status FrameHandlerImpl::SaveUint16AsJPEG(const char *filename,
                                                  const uint16_t *data,
                                                  uint32_t width,
                                                  uint32_t height) {

    if (data == nullptr) {
        return aditof::Status::GENERIC_ERROR;
    }

    std::vector<uint8_t> img_8bit(width * height);

    auto minmax = std::minmax_element(data, data + width * height);
    uint16_t min_val = *minmax.first;
    uint16_t max_val = *minmax.second;
    float range =
        (max_val == min_val) ? 1.0f : static_cast<float>(max_val - min_val);

    for (uint32_t i = 0; i < width * height; ++i) {
        float norm = static_cast<float>(data[i] - min_val) / range; // [0,1]
        img_8bit[i] = static_cast<uint8_t>(norm * 255.0f + 0.5f);
    }

    auto result =
        stbi_write_jpg(filename, width, height, 1, img_8bit.data(), 100);

    LOG(INFO) << __func__ << ":  " << result << " " << filename;

    return result ? aditof::Status::OK : aditof::Status::GENERIC_ERROR;
}

aditof::Status FrameHandlerImpl::SaveRGBAsJPEG(const char *filename,
                                               const uint8_t *data,
                                               uint32_t width,
                                               uint32_t height) {

    if (data == nullptr) {
        return aditof::Status::GENERIC_ERROR;
    }

    LOG(INFO) << __func__ << ":  " << filename;

    auto result = stbi_write_jpg(filename, width, height, 3, data, 100);

    LOG(INFO) << __func__ << ":  " << result << " " << filename;

    return result ? aditof::Status::OK : aditof::Status::GENERIC_ERROR;
}

aditof::Status FrameHandlerImpl::SavePointCloudPLYBinary(const char *filename,
                                                         const uint16_t *data,
                                                         uint32_t width,
                                                         uint32_t height) {

    if (data == nullptr) {
        return aditof::Status::GENERIC_ERROR;
    }

    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        LOG(ERROR) << "Failed to open file: " << filename;
        return aditof::Status::GENERIC_ERROR;
    }

    uint32_t num_points = width * height;

    // Write the PLY header
    std::string header = "ply\n"
                         "format binary_little_endian 1.0\n"
                         "element vertex " +
                         std::to_string(num_points) +
                         "\n"
                         "property float x\n"
                         "property float y\n"
                         "property float z\n"
                         "end_header\n";

    file.write(header.c_str(), header.size());

    // Write binary float data: 3 floats per point
    file.write(reinterpret_cast<const char *>(data),
               num_points * 3 * sizeof(uint16_t));

    file.close();

    LOG(INFO) << __func__ << ":  " << filename;

    return aditof::Status::OK;
}

aditof::Status FrameHandlerImpl::SaveMetaAsTxt(const char *filename,
                                               const aditof::Metadata *data) {

    if (data == nullptr) {
        return aditof::Status::GENERIC_ERROR;
    }

    std::ofstream out(filename);
    if (!out.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << "\n";
        return aditof::Status::GENERIC_ERROR;
    }

    out << "width: " << data->width << " pixels" << '\n';
    out << "height: " << data->height << " pixels" << '\n';
    out << "outputConfiguration: "
        << static_cast<int>(data->outputConfiguration) << '\n';
    out << "bitsInDepth: " << static_cast<int>(data->bitsInDepth) << '\n';
    out << "bitsInAb: " << static_cast<int>(data->bitsInAb) << '\n';
    out << "bitsInConfidence: " << static_cast<int>(data->bitsInConfidence)
        << '\n';
    out << "invalidPhaseValue: " << data->invalidPhaseValue << '\n';
    out << "frequencyIndex: " << static_cast<int>(data->frequencyIndex) << '\n';
    out << "abFrequencyIndex: " << static_cast<int>(data->abFrequencyIndex)
        << '\n';
    out << "frameNumber: " << data->frameNumber << '\n';
    out << "imagerMode: " << static_cast<int>(data->imagerMode) << '\n';
    out << "numberOfPhases: " << static_cast<int>(data->numberOfPhases) << '\n';
    out << "numberOfFrequencies: "
        << static_cast<int>(data->numberOfFrequencies) << '\n';
    out << "xyzEnabled: " << static_cast<int>(data->xyzEnabled) << '\n';
    out << "elapsedTimeFractionalValue: " << data->elapsedTimeFractionalValue
        << '\n';
    out << "elapsedTimeSecondsValue: " << data->elapsedTimeSecondsValue << '\n';
    out << "sensorTemperature: " << data->sensorTemperature << " C" << '\n';
    out << "laserTemperature: " << data->laserTemperature << " C" << '\n';

    out.close();

    LOG(INFO) << __func__ << ":  " << filename;

    return aditof::Status::OK;
}

aditof::Status FrameHandlerImpl::SnapShotFrames(const char *baseFileName,
                                                aditof::Frame *frame,
                                                const uint8_t *ab,
                                                const uint8_t *depth) {

    if (baseFileName == nullptr || frame == nullptr) {
        return aditof::Status::GENERIC_ERROR;
    }

    std::string stringBaseFileName(baseFileName);

    if (stringBaseFileName.empty()) {
        LOG(ERROR) << "Base file name is empty!";
        return aditof::Status::GENERIC_ERROR;
    }

    aditof::Status status;

    Metadata metadata;
    uint16_t *abFrame;
    uint16_t *depthFrame;
    float *confFrame;
    uint16_t *xyzFrame;
    uint8_t *rgbFrame;
    FrameDetails frameDetails;

    status = frame->getMetadataStruct(metadata);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to get metadata location";
        return status;
    }

    // Try to get AB frame - optional
    if (frame->haveDataType("ab")) {
        status = frame->getData("ab", &abFrame);
        if (status != Status::OK) {
            LOG(WARNING) << "AB frame exists but failed to get location";
            abFrame = nullptr;
        }
    } else {
        LOG(INFO) << "AB frame not available, skipping AB snapshot";
        abFrame = nullptr;
    }

    // Try to get depth frame - optional
    if (frame->haveDataType("depth")) {
        status = frame->getData("depth", &depthFrame);
        if (status != Status::OK) {
            LOG(WARNING) << "Depth frame exists but failed to get location";
            depthFrame = nullptr;
        }
    } else {
        LOG(INFO) << "Depth frame not available, skipping depth snapshot";
        depthFrame = nullptr;
    }

    // Only get confidence frame if it exists
    if (frame->haveDataType("conf")) {
        status = frame->getData("conf", (uint16_t **)&confFrame);
        if (status != Status::OK) {
            LOG(WARNING)
                << "Confidence frame exists but failed to get location";
        }
    } else {
        confFrame = nullptr;
    }

    // Only get XYZ frame if it exists
    if (frame->haveDataType("xyz")) {
        status = frame->getData("xyz", &xyzFrame);
        if (status != Status::OK) {
            LOG(WARNING) << "XYZ frame exists but failed to get location";
            xyzFrame = nullptr;
        }
    } else {
        xyzFrame = nullptr;
    }

    // Try to get RGB frame if it exists
    if (frame->haveDataType("rgb")) {
        status = frame->getData("rgb", (uint16_t **)&rgbFrame);
        if (status != Status::OK) {
            LOG(WARNING) << "RGB frame exists but failed to get location";
            rgbFrame = nullptr;
        }
    } else {
        rgbFrame = nullptr;
    }

    status = frame->getDetails(frameDetails);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to get frame details";
        return status;
    }

    std::string metadataFileName = stringBaseFileName + "_" +
                                   std::to_string(metadata.frameNumber) +
                                   "_metadata.txt";
    std::string xyzFileName = stringBaseFileName + "_" +
                              std::to_string(metadata.frameNumber) +
                              "_pointcloud.ply";
    std::string depthFileName = stringBaseFileName + "_" +
                                std::to_string(metadata.frameNumber) +
                                "_depth.jpg";
    std::string depthProcessedFileName = stringBaseFileName + "_" +
                                         std::to_string(metadata.frameNumber) +
                                         "_depth_processed.jpg";
    std::string abFileName = stringBaseFileName + "_" +
                             std::to_string(metadata.frameNumber) + "_ab.jpg";
    std::string abProcessedFileName = stringBaseFileName + "_" +
                                      std::to_string(metadata.frameNumber) +
                                      "_ab_processed.jpg";
    std::string confFileName = stringBaseFileName + "_" +
                               std::to_string(metadata.frameNumber) +
                               "_conf.jpg";
    std::string rgbFileName = stringBaseFileName + "_" +
                              std::to_string(metadata.frameNumber) + "_rgb.jpg";

    status = SaveMetaAsTxt(metadataFileName.c_str(), &metadata);

    if (xyzFrame != nullptr) {
        status =
            SavePointCloudPLYBinary(xyzFileName.c_str(), xyzFrame,
                                    frameDetails.width, frameDetails.height);
    }

    if (depthFrame != nullptr) {
        status = SaveUint16AsJPEG(depthFileName.c_str(), depthFrame,
                                  frameDetails.width, frameDetails.height);
        if (depth != nullptr) {
            status = SaveRGBAsJPEG(depthProcessedFileName.c_str(), depth,
                                   frameDetails.width, frameDetails.height);
        }
    }

    if (abFrame != nullptr) {
        status = SaveUint16AsJPEG(abFileName.c_str(), abFrame,
                                  frameDetails.width, frameDetails.height);
        if (ab != nullptr) {
            status = SaveRGBAsJPEG(abProcessedFileName.c_str(), ab,
                                   frameDetails.width, frameDetails.height);
        }
    }

    if (confFrame != nullptr) {
        if (frameDetails.width == 1024 && frameDetails.height == 1024) {
            status = SaveFloatAsJPEG(confFileName.c_str(), confFrame,
                                     frameDetails.width, frameDetails.height);
        } else {
            status =
                SaveUint16AsJPEG(confFileName.c_str(), (uint16_t *)confFrame,
                                 frameDetails.width, frameDetails.height);
        }
    }

    // Save RGB frame if available (convert from BGR to RGB)
    if (rgbFrame != nullptr) {
        // Get RGB frame dimensions
        FrameDataDetails rgbDetails;
        status = frame->getDataDetails("rgb", rgbDetails);
        if (status == Status::OK) {
            // Convert BGR to RGB (swap R and B channels)
            size_t totalPixels = rgbDetails.width * rgbDetails.height;
            std::vector<uint8_t> rgbConverted(totalPixels * 3);

            for (size_t i = 0; i < totalPixels; ++i) {
                // Input is BGR, output is RGB
                rgbConverted[i * 3 + 0] = rgbFrame[i * 3 + 2]; // R
                rgbConverted[i * 3 + 1] = rgbFrame[i * 3 + 1]; // G
                rgbConverted[i * 3 + 2] = rgbFrame[i * 3 + 0]; // B
            }

            status = SaveRGBAsJPEG(rgbFileName.c_str(), rgbConverted.data(),
                                   rgbDetails.width, rgbDetails.height);
        } else {
            LOG(WARNING) << "Failed to get RGB frame details for snapshot";
        }
    }

    return aditof::Status::OK;
}