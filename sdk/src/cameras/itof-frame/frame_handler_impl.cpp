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

/**
 * @brief Constructor for FrameHandlerImpl.
 *
 * Initializes a frame handler with default settings for concatenating frames,
 * frame dimensions, and file management. Sets up ready state for frame I/O operations.
 *
 * @note Multithreading is disabled by default.
 * @note Frame concatenation is enabled by default.
 */
FrameHandlerImpl::FrameHandlerImpl()
    : m_concatFrames(true), m_enableMultithreading(false),
      m_customFormat(false), m_bitsInDepth(0), m_bitsInAB(0), m_bitsInConf(0),
      m_frameWidth(0), m_frameHeight(0), m_frameIndex(0), m_fileCreated(false),
      m_endOfFile(false), m_pos(0), m_threadRunning(false) {}

/**
 * @brief Destructor for FrameHandlerImpl.
 *
 * Cleans up resources by joining the worker thread if active and closing
 * any open file streams. Resets output directory to current directory.
 */
FrameHandlerImpl::~FrameHandlerImpl() {
    m_dir = ".";
    if (m_threadWorker.joinable()) {
        m_threadWorker.join();
    }
}

/**
 * @brief Sets the output directory path for saved frame files.
 *
 * Configures the destination directory where frame binary files will be saved.
 * Resets the file creation flag to allow creation of new files in the new directory.
 *
 * @param[in] filePath Directory path where output files should be saved.
 *
 * @return aditof::Status::OK.
 */
Status FrameHandlerImpl::setOutputFilePath(const std::string &filePath) {
    Status status = Status::OK;
    m_dir = filePath;
    m_fileCreated = false;
    return status;
}

/**
 * @brief Sets the input file path for frame reading operations.
 *
 * Configures the source file from which frames will be read. Resets the
 * file position pointer to the beginning for sequential frame reading.
 *
 * @param[in] fullFileName Complete path to the input frame file.
 *
 * @return aditof::Status::OK.
 */
Status FrameHandlerImpl::setInputFileName(const std::string &fullFileName) {
    Status status = Status::OK;
    m_fullInputFileName = fullFileName;
    m_pos = 0;
    return status;
}

/**
 * @brief Saves a frame to a binary file in sequential order.
 *
 * Writes frame data to disk in the following order: metadata, depth, AB,
 * confidence, and XYZ (if enabled). Creates a new file or appends to an
 * existing file based on the concatenation flag. File format is binary with
 * no compression.
 *
 * @details
 * - Metadata: Fixed 128-byte header with frame information
 * - Depth: width × height × 2 bytes (uint16_t)
 * - AB: width × height × 2 bytes (uint16_t), if enabled
 * - Confidence: width × height × 4 bytes (float), if enabled
 * - XYZ: width × height × 6 bytes (3×int16_t per pixel), if enabled
 *
 * @param[in,out] frame Frame object containing the data to save.
 * @param[in] fileName Name for the output file (auto-generated with timestamp if empty).
 *
 * @return aditof::Status::OK if frame saved successfully;
 *         aditof::Status::GENERIC_ERROR if file creation fails.
 *
 * @note The frame queue is cleared after successful write if multithreading is enabled.
 */
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

/**
 * @brief Saves a frame to file asynchronously using a background worker thread.
 *
 * Queues a frame for asynchronous saving via a dedicated thread. Allows the
 * caller to continue without blocking on file I/O. A worker thread is spawned
 * on first call and processes frames from the queue sequentially.
 *
 * @details
 * - Locks a mutex to safely enqueue the frame and filename
 * - Spawns a new worker thread if one isn't already running
 * - Clears the frame object after queuing (moves ownership to queue)
 * - Processes queue items in FIFO order via threadWritter()
 *
 * @param[in,out] frame Frame object to save (ownership transferred, cleared after call).
 * @param[in] fileName Name for the output file.
 *
 * @return aditof::Status::OK if frame successfully queued.
 *
 * @note Thread-safe: uses mutex to protect frame and filename queues.
 * @note The frame object is invalidated after this call.
 */
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

/**
 * @brief Worker thread function for asynchronous frame saving.
 *
 * Processes frames from the queue via saveFrameToFile() until the queue is empty.
 * Runs in a background thread spawned by saveFrameToFileMultithread().
 * Sets the thread running flag and clears it when done.
 *
 * @note This function is called by std::thread and should not be called directly.
 * @note Returns silently on first error encountered (no error recovery).
 */
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

/**
 * @brief Reads the next frame from a binary file.
 *
 * Sequentially reads frame data from the specified file in the format written
 * by saveFrameToFile(). Manages file position internally for continuous reading.
 * Resets position counter if a different file is specified.
 *
 * @details
 * Reads in order: metadata (128 bytes), depth, AB, confidence, and XYZ data
 * based on the metadata flags indicating which components are present.
 *
 * @param[out] frame Frame object to be populated with read data.
 * @param[in] fullFileName Path to the binary frame file to read from.
 *                         If different from previous call, resets position.
 *
 * @return aditof::Status::OK if frame read successfully;
 *         aditof::Status::GENERIC_ERROR if file cannot be opened or read fails.
 *
 * @note Both frame object parameter and stored filename must be non-empty.
 * @note Logs warning when end-of-file is reached.
 * @note File position is maintained internally across calls for sequential reading.
 */
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

/**
 * @brief Sets a custom frame format (not currently implemented).
 *
 * Placeholder for future frame format customization functionality.
 *
 * @param[in] format Frame format specification string (currently ignored).
 *
 * @return aditof::Status::UNAVAILABLE (feature not yet implemented).
 */
Status FrameHandlerImpl::setCustomFormat(const std::string &format) {
    return Status::UNAVAILABLE;
}

/**
 * @brief Enables or disables concatenation of multiple frames to a single file.
 *
 * Controls whether frames are appended to an existing file (true) or each frame
 * is saved to a new file (false).
 *
 * @param[in] enable True to concatenate frames to one file; false for separate files.
 *
 * @return aditof::Status::OK.
 */
Status FrameHandlerImpl::storeFramesToSingleFile(bool enable) {
    Status status = Status::OK;
    m_concatFrames = enable;

    return status;
}

/**
 * @brief Configures which frame components to save (not currently implemented).
 *
 * Placeholder for future frame component selection functionality.
 *
 * @param[in] frameContent Frame content specification string (currently ignored).
 *
 * @return aditof::Status::UNAVAILABLE (feature not yet implemented).
 */
Status FrameHandlerImpl::setFrameContent(const std::string &frameContent) {
    return Status::UNAVAILABLE;
}

/**
 * @brief Creates a new output binary file for frame storage.
 *
 * Opens a file stream for writing frame data. If no filename is provided,
 * generates a timestamped filename automatically (format: frame_YYYY_MM_DD_HH_MM_SS_N.bin).
 * Updates internal filename tracking for subsequent append operations.
 *
 * @param[in] fileName Desired output filename. If empty, auto-generates with timestamp.
 *
 * @return aditof::Status::OK if file created successfully;
 *         aditof::Status::GENERIC_ERROR if file creation fails.
 *
 * @note Sets m_fileCreated flag to true on success for concatenation tracking.
 * @note Increments internal frame counter for auto-generated filenames.
 */
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

/**
 * @brief Constructs the full file path from directory and filename.
 *
 * Combines the configured output directory (m_dir) with the provided filename
 * using the platform-specific path separator (backslash on Windows, forward
 * slash on Unix-like systems).
 *
 * @param[in] fileName Filename to append to the output directory path.
 *
 * @return Full file path string. If directory is empty, returns filename alone.
 *
 * @note Path separator is OS-specific (backslash on Windows, forward slash on Unix).
 */
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

/**
 * @brief Saves floating-point image data as a JPEG file.
 *
 * Converts normalized float data to 8-bit grayscale and writes as JPEG with
 * quality 100. Automatically scales data range [min, max] to [0, 255].
 * Handles edge case where all pixels have identical values.
 *
 * @param[in] filename Output JPEG file path.
 * @param[in] data Pointer to float image data (width * height elements).
 * @param[in] width Image width in pixels.
 * @param[in] height Image height in pixels.
 *
 * @return aditof::Status::OK if JPEG saved successfully;
 *         aditof::Status::GENERIC_ERROR if data is null or encoding fails.
 *
 * @note Uses stbi_write_jpg() for JPEG encoding.
 */
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

    if (result) {
        LOG(INFO) << __func__ << ":  " << filename;
    } else {
        LOG(ERROR) << __func__ << ":  Failed to save JPEG " << filename;
    }

    return result ? aditof::Status::OK : aditof::Status::GENERIC_ERROR;
}

/**
 * @brief Saves 16-bit unsigned integer image data as a JPEG file.
 *
 * Converts uint16_t depth/amplitude data to 8-bit grayscale and writes as JPEG
 * with quality 100. Automatically scales data range [min, max] to [0, 255].
 * Handles edge case where all pixels have identical values.
 *
 * @param[in] filename Output JPEG file path.
 * @param[in] data Pointer to uint16_t image data (width * height elements).
 * @param[in] width Image width in pixels.
 * @param[in] height Image height in pixels.
 *
 * @return aditof::Status::OK if JPEG saved successfully;
 *         aditof::Status::GENERIC_ERROR if data is null or encoding fails.
 *
 * @note Uses stbi_write_jpg() for JPEG encoding.
 * @note Typical use: saving depth or AB frames as viewable images.
 */
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

    if (result) {
        LOG(INFO) << __func__ << ":  " << filename;
    } else {
        LOG(ERROR) << __func__ << ":  Failed to save JPEG " << filename;
    }

    return result ? aditof::Status::OK : aditof::Status::GENERIC_ERROR;
}

/**
 * @brief Saves RGB color image data as a JPEG file.
 *
 * Writes 8-bit RGB image (3 channels per pixel) directly to JPEG without conversion.
 * Assumes data is already in uint8_t format ready for encoding.
 *
 * @param[in] filename Output JPEG file path.
 * @param[in] data Pointer to RGB image data (width * height * 3 bytes).
 * @param[in] width Image width in pixels.
 * @param[in] height Image height in pixels.
 *
 * @return aditof::Status::OK if JPEG saved successfully;
 *         aditof::Status::GENERIC_ERROR if data is null or encoding fails.
 *
 * @note Uses stbi_write_jpg() with 3 color channels.
 * @note Typical use: saving processed/annotated depth or confidence visualizations.
 */
aditof::Status FrameHandlerImpl::SaveRGBAsJPEG(const char *filename,
                                               const uint8_t *data,
                                               uint32_t width,
                                               uint32_t height) {

    if (data == nullptr) {
        return aditof::Status::GENERIC_ERROR;
    }

    LOG(INFO) << __func__ << ":  " << filename;

    auto result = stbi_write_jpg(filename, width, height, 3, data, 100);

    if (result) {
        LOG(INFO) << __func__ << ":  " << filename;
    } else {
        LOG(ERROR) << __func__ << ":  Failed to save JPEG " << filename;
    }

    return result ? aditof::Status::OK : aditof::Status::GENERIC_ERROR;
}

/**
 * @brief Saves XYZ point cloud data as a binary PLY (Polygon File) file.
 *
 * Converts 3D point data (stored as int16_t XYZ triplets) to a standard PLY format
 * with floating-point coordinates. Scales values from millimeters to meters using
 * a 0.001 scaling factor. Creates a binary little-endian PLY file compatible
 * with 3D visualization tools.
 *
 * @details
 * - Input data format: 3 * width * height int16_t values (X, Y, Z per pixel)
 * - Output format: Binary PLY with 3 float properties (x, y, z) per vertex
 * - Scaling: millimeters to meters (divide by 1000)
 *
 * @param[in] filename Output PLY file path.
 * @param[in] data Pointer to XYZ point cloud data (3 int16_t values per pixel).
 * @param[in] width Image width in pixels.
 * @param[in] height Image height in pixels.
 *
 * @return aditof::Status::OK if PLY saved successfully;
 *         aditof::Status::GENERIC_ERROR if data is null or file I/O fails.
 *
 * @note PLY format used: binary_little_endian with float x,y,z properties.
 * @note Typical use: saving 3D point clouds for visualization in Meshlab, CloudCompare, etc.
 */
aditof::Status FrameHandlerImpl::SavePointCloudPLYBinary(const char *filename,
                                                         const uint16_t *data,
                                                         uint32_t width,
                                                         uint32_t height) {

    if (!data) {
        return aditof::Status::GENERIC_ERROR;
    }

    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        LOG(ERROR) << "Failed to create file: " << filename;
        return aditof::Status::GENERIC_ERROR;
    }

    const uint32_t num_points = width * height;

    // PLY header: 3 floats per vertex
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

    // Cast data to int16_t* since XYZ data is stored as 3 int16_t values per pixel
    const int16_t *xyz_data = reinterpret_cast<const int16_t *>(data);
    const float scale = 0.001f; // Convert from millimeters to meters

    // Write XYZ coordinates as floats
    // XYZ data layout: each pixel has [X, Y, Z] as int16_t values
    for (uint32_t i = 0; i < num_points; ++i) {
        float x = static_cast<float>(xyz_data[3 * i + 0]) * scale;
        float y = static_cast<float>(xyz_data[3 * i + 1]) * scale;
        float z = static_cast<float>(xyz_data[3 * i + 2]) * scale;

        file.write(reinterpret_cast<const char *>(&x), sizeof(float));
        file.write(reinterpret_cast<const char *>(&y), sizeof(float));
        file.write(reinterpret_cast<const char *>(&z), sizeof(float));
    }

    file.close();

    LOG(INFO) << __func__ << ":  " << filename;
    return aditof::Status::OK;
}

/**
 * @brief Saves frame metadata to a human-readable text file.
 *
 * Exports all fields from the Metadata structure to a plaintext file with
 * key-value pairs (one per line). Useful for debugging, documentation, and
 * reviewing frame acquisition settings.
 *
 * @details Output includes:
 * - Image dimensions (width, height in pixels)
 * - Bit depths (depth, AB, confidence bits)
 * - Processing parameters (phases, frequencies, imager mode)
 * - Timing information (frame number, elapsed time)
 * - Temperature readings (sensor, laser in Celsius)
 * - Feature flags (XYZ enabled, output configuration)
 * - Phase invalidation threshold
 *
 * @param[in] filename Output text file path.
 * @param[in] data Pointer to Metadata structure to write.
 *
 * @return aditof::Status::OK if file written successfully;
 *         aditof::Status::GENERIC_ERROR if data is null or file I/O fails.
 *
 * @note Creates a new file, overwriting any existing file with the same name.
 */
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

/**
 * @brief Saves all frame components and metadata as separate files (snapshot export).
 *
 * Exports a complete frame snapshot including metadata, depth, AB, confidence,
 * and XYZ data in multiple formats for visualization and analysis. Creates:
 * - Metadata text file (human-readable frame info)
 * - JPEG grayscale images (depth, AB, confidence raw values)
 * - JPEG RGB images (depth, AB processed/annotated versions)
 * - Binary PLY file (3D point cloud if XYZ enabled)
 *
 * @details
 * File naming convention uses frame number from metadata:
 * - {baseFileName}_{frameNumber}_metadata.txt
 * - {baseFileName}_{frameNumber}_depth.jpg
 * - {baseFileName}_{frameNumber}_depth_processed.jpg (if ab/depth RGB provided)
 * - {baseFileName}_{frameNumber}_ab.jpg
 * - {baseFileName}_{frameNumber}_ab_processed.jpg (if provided)
 * - {baseFileName}_{frameNumber}_conf.jpg (if confidence data available)
 * - {baseFileName}_{frameNumber}_pointcloud.ply (if XYZ enabled)
 *
 * @param[in] baseFileName Base filename for output files (frame number appended).
 * @param[in] frame Frame object containing depth, AB, confidence, XYZ, and metadata.
 * @param[in] ab Optional pre-processed 8-bit RGB AB visualization (can be nullptr).
 * @param[in] depth Optional pre-processed 8-bit RGB depth visualization (can be nullptr).
 *
 * @return aditof::Status::OK if snapshot saved successfully;
 *         aditof::Status::GENERIC_ERROR if input is invalid or file I/O fails.
 *
 * @note Extracts metadata, dimensions, and data pointers from frame object.
 * @note Confidence data format depends on resolution: float for 1024x1024, uint16 otherwise.
 * @note Gracefully handles missing optional data (RGB visualizations, confidence, XYZ).
 * @note All errors retrieving frame data are logged but do not halt export.
 */
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
    FrameDetails frameDetails;

    status = frame->getMetadataStruct(metadata);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to get metadata location";
        return status;
    }

    status = frame->getData("ab", &abFrame);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to get AB location";
        return status;
    }

    status = frame->getData("depth", &depthFrame);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to get depth location";
        return status;
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
        }
    } else {
        xyzFrame = nullptr;
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

    SaveMetaAsTxt(metadataFileName.c_str(), &metadata);

    if (xyzFrame != nullptr) {
        SavePointCloudPLYBinary(xyzFileName.c_str(), xyzFrame,
                                frameDetails.width, frameDetails.height);
    }

    SaveUint16AsJPEG(depthFileName.c_str(), depthFrame, frameDetails.width,
                     frameDetails.height);
    if (depth != nullptr) {
        SaveRGBAsJPEG(depthProcessedFileName.c_str(), depth, frameDetails.width,
                      frameDetails.height);
    }

    SaveUint16AsJPEG(abFileName.c_str(), abFrame, frameDetails.width,
                     frameDetails.height);
    if (ab != nullptr) {
        SaveRGBAsJPEG(abProcessedFileName.c_str(), ab, frameDetails.width,
                      frameDetails.height);
    }

    if (confFrame != nullptr) {
        if (frameDetails.width == 1024 && frameDetails.height == 1024) {
            SaveFloatAsJPEG(confFileName.c_str(), confFrame, frameDetails.width,
                            frameDetails.height);
        } else {
            SaveUint16AsJPEG(confFileName.c_str(), (uint16_t *)confFrame,
                             frameDetails.width, frameDetails.height);
        }
    }

    return aditof::Status::OK;
}
