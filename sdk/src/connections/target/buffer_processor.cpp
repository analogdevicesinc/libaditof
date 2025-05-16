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

// TO DO: This exists in linux_utils.h which is not included on Dragoboard.
// Should not have duplicated code if possible.

#include <algorithm>
#include <arm_neon.h>
#include <cmath>
#include <fcntl.h>
#include <fstream>
#include <unistd.h>
#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
#include <linux/videodev2.h>
#include <memory>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unordered_map>

#include "buffer_processor.h"

uint8_t depthComputeOpenSourceEnabled = 0;

#define CLEAR(x) memset(&(x), 0, sizeof(x))

static int xioctl(int fh, unsigned int request, void *arg) {
    int r;

    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno && errno != 0);

    return r;
}

BufferProcessor::BufferProcessor()
    : m_vidPropSet(false), m_processorPropSet(false), m_outputFrameWidth(0),
      m_outputFrameHeight(0), m_processedBuffer(nullptr), m_tofiConfig(nullptr),
      m_tofiComputeContext(nullptr), m_inputVideoDev(nullptr) {
    m_outputVideoDev = new VideoDev();
}

BufferProcessor::~BufferProcessor() {
    if (NULL != m_tofiComputeContext) {
        LOG(INFO) << "freeComputeLibrary";
        FreeTofiCompute(m_tofiComputeContext);
        m_tofiComputeContext = NULL;
    }

    if (m_tofiConfig != NULL) {
        FreeTofiConfig(m_tofiConfig);
        m_tofiConfig = NULL;
    }

    if (m_outputVideoDev->fd != -1) {
        if (::close(m_outputVideoDev->fd) == -1) {
            LOG(ERROR) << "Failed to close " << m_videoDeviceName
                       << " error: " << strerror(errno);
        }
    }
}

aditof::Status BufferProcessor::open() {
    using namespace aditof;
    Status status = Status::OK;

    //TO DO: remove when we re-enable uvc
    return aditof::Status::OK;

    m_outputVideoDev->fd = ::open(m_videoDeviceName, O_RDWR);
    if (m_outputVideoDev->fd == -1) {
        LOG(ERROR) << "Cannot open " << OUTPUT_DEVICE << "errno: " << errno
                   << "error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    if (xioctl(m_outputVideoDev->fd, VIDIOC_QUERYCAP, &m_videoCap) == -1) {
        LOG(ERROR) << m_videoDeviceName << " VIDIOC_QUERYCAP error";
        return Status::GENERIC_ERROR;
    }

    memset(&m_videoFormat, 0, sizeof(m_videoFormat));
    if (xioctl(m_outputVideoDev->fd, VIDIOC_G_FMT, &m_videoFormat) == -1) {
        // LOG(ERROR) << m_videoDeviceName << " VIDIOC_G_FMT error";
        // return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status BufferProcessor::setInputDevice(VideoDev *inputVideoDev) {
    m_inputVideoDev = inputVideoDev;

    return aditof::Status::OK;
}

aditof::Status BufferProcessor::setVideoProperties(int frameWidth,
                                                   int frameHeight) {
    using namespace aditof;
    Status status = Status::OK;

    m_outputFrameWidth = frameWidth;
    m_outputFrameHeight = frameHeight;

    LOG(INFO) << __func__ << ": Width : " << m_outputFrameWidth << " Height: " << m_outputFrameHeight;

    size_t rawFrameSize = static_cast<size_t>(m_outputFrameWidth) * m_outputFrameHeight * 6.5;

    rawFrameBufferSize = rawFrameSize;
    preallocatedFrameBuffers.reserve(10);
    {
        std::lock_guard<std::mutex> lock(preallocMutex);
        for (int i = 0; i < 10; ++i) {
            uint8_t* buffer = static_cast<uint8_t*>(aligned_alloc(64, rawFrameSize));
            if (buffer) {
                preallocatedFrameBuffers.push_back(buffer);
                freeFrameBuffers.push(buffer);
            } else {
                LOG(ERROR) << __func__ << ": Failed to allocate raw buffer!";
                status = Status::GENERIC_ERROR;
            }
        }
    }

    uint32_t depthSize = m_outputFrameWidth * m_outputFrameHeight * 2;		/* | Depth Frame ( W * H * 2 (type: uint16_t)) |   */
    uint32_t abSize = depthSize;						/* | AB Frame ( W * H * 2 (type: uint16_t)) |      */
    uint32_t confSize = m_outputFrameWidth * m_outputFrameHeight * 4;		/* | Confidance Frame ( W * H * 4 (type: float)) | */
    tofiBufferSize = depthSize + abSize + confSize;

    for (int i = 0; i < TOFI_BUFFER_COUNT; ++i) {
        uint16_t* buffer = static_cast<uint16_t*>(aligned_alloc(64, tofiBufferSize));
        if (buffer) {
            std::lock_guard<std::mutex> lock(tofiBufferMutex);
            tofiBufferfifo.push(buffer);
        } else {
            LOG(ERROR) << __func__ << ": Failed to allocate ToFi buffer!";
            status = Status::GENERIC_ERROR;
        }
    }

    LOG(INFO) << __func__ << ": RawBufferSize: " << rawFrameBufferSize << "  tofiBufferSize: " << tofiBufferSize;
    return status;
}

aditof::Status BufferProcessor::setProcessorProperties(
    uint8_t *iniFile, uint16_t iniFileLength, uint8_t *calData,
    uint16_t calDataLength, uint16_t mode, bool ispEnabled) {

    if (ispEnabled) {
        uint32_t status = ADI_TOFI_SUCCESS;
        ConfigFileData calDataStruct = {calData, calDataLength};
        if (iniFile != nullptr) {
            ConfigFileData depth_ini = {iniFile, iniFileLength};
            if (ispEnabled) {
                memcpy(m_xyzDealiasData, calData, calDataLength);
                m_tofiConfig =
                    InitTofiConfig_isp((ConfigFileData *)&depth_ini, mode,
                                       &status, m_xyzDealiasData);
            } else {
                if (calDataStruct.p_data != NULL) {
                    m_tofiConfig = InitTofiConfig(&calDataStruct, NULL,
                                                  &depth_ini, mode, &status);
                } else {
                    LOG(ERROR) << "Failed to get calibration data";
                }
            }

        } else {
            m_tofiConfig =
                InitTofiConfig(&calDataStruct, NULL, NULL, mode, &status);
        }

        if ((m_tofiConfig == NULL) ||
            (m_tofiConfig->p_tofi_cal_config == NULL) ||
            (status != ADI_TOFI_SUCCESS)) {
            LOG(ERROR) << "InitTofiConfig failed";
            return aditof::Status::GENERIC_ERROR;

        } else {
            m_tofiComputeContext =
                InitTofiCompute(m_tofiConfig->p_tofi_cal_config, &status);
            if (m_tofiComputeContext == NULL || status != ADI_TOFI_SUCCESS) {
                LOG(ERROR) << "InitTofiCompute failed";
                return aditof::Status::GENERIC_ERROR;
            }
        }
    } else {
        LOG(ERROR) << "Could not initialize compute library because config "
                      "data hasn't been loaded";
        return aditof::Status::GENERIC_ERROR;
    }

    return aditof::Status::OK;
}

void BufferProcessor::captureFrameThread() {
    LOG(INFO) << __func__ << ": Capture thread started";

    long long totalCaptureTime = 0;
    int totalV4L2Captured = 0;

    size_t bufferIndex = 0;
    size_t preallocSize = preallocatedFrameBuffers.size();

    while (!stopThreadsFlag) {
        aditof::Status status;
        struct v4l2_buffer buf;
        struct VideoDev *dev = m_inputVideoDev;
        uint8_t *pdata = nullptr;
        unsigned int buf_data_len = 0;

        {
            std::unique_lock<std::mutex> lock(poolMutex);
            bufferNotFull.wait(lock, [this, preallocSize]() {
                return bufferPool.size() < preallocSize || stopThreadsFlag;
            });

            if (stopThreadsFlag) {
                break;
            }
        }

        auto captureStart = std::chrono::high_resolution_clock::now();

        status = waitForBufferPrivate(dev);
        if (status != aditof::Status::OK){
            LOG(ERROR) << __func__ << ": waitForBufferPrivate() Failed, retrying...";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        status = dequeueInternalBufferPrivate(buf, dev);
        if (status != aditof::Status::OK){
            LOG(ERROR) << __func__ << ": dequeueInternalBufferPrivate() Failed, retrying...";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        status = getInternalBufferPrivate(&pdata, buf_data_len, buf, dev);
        if (status != aditof::Status::OK || !pdata || buf_data_len == 0) {
            LOG(ERROR) << __func__ << ": dequeueInternalBufferPrivate() Failed. Buffer index: " << buf.index;
            // Always requeue the buffer to avoid memory leak
            enqueueInternalBufferPrivate(buf, dev);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        auto captureEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> captureTime = captureEnd - captureStart;
        totalCaptureTime += static_cast<long long>(captureTime.count());

	if(totalV4L2Captured == 0)
		LOG(INFO) << __func__ << ": V4l2 frame size: " << buf_data_len;
        totalV4L2Captured++;

        Frame frame;
        {
            std::lock_guard<std::mutex> lock(poolMutex);
            uint8_t* targetBuffer = preallocatedFrameBuffers[bufferIndex];
            memcpy(targetBuffer, pdata, buf_data_len);

            frame.data = targetBuffer;

            bufferPool.push(std::move(frame));
            bufferNotEmpty.notify_one();

            bufferIndex = (bufferIndex + 1) % preallocSize;

        }

        status = enqueueInternalBufferPrivate(buf, dev);
        if (status != aditof::Status::OK) {
            LOG(ERROR) << __func__ << ": enqueueInternalBufferPrivate() Failed";
        }
        // (Optional) A short sleep to avoid hammering the device, if needed.
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (totalV4L2Captured > 0) {
        double averageCaptureTime = static_cast<double>(totalCaptureTime) / totalV4L2Captured;
        //LOG(INFO) << __func__ << ": Average capture time: " << averageCaptureTime << " ms";
    }
}

void BufferProcessor::processThread() {
    LOG(INFO) << __func__ << ": Processing thread started";

    long long totalProcessTime = 0;
    int totalProcessedFrame = 0;
    const size_t maxProcessedQueueSize = 50; //preallocatedFrameBuffers.size(); // Match bufferPool size

    while (!stopThreadsFlag) {
        Frame frame;
        {
            std::unique_lock<std::mutex> lock(poolMutex);
            if (!bufferNotEmpty.wait_for(lock, std::chrono::seconds(2), [this] { return !bufferPool.empty() || stopThreadsFlag; })) {
                LOG(WARNING) << __func__ << ": Timeout! No new frames.";
                continue;
            }

            if (stopThreadsFlag) break;

            frame = std::move(bufferPool.front());
            bufferPool.pop();
            bufferNotFull.notify_one();
        }

        // Fetch a buffer from tofiBufferPool
        uint16_t* tofiBuffer = nullptr;
        {
            std::unique_lock<std::mutex> tofiLock(tofiBufferMutex);
            if (!tofiBufferNotEmpty.wait_for(tofiLock, std::chrono::seconds(2), [this] { return !tofiBufferfifo.empty(); })) {
                LOG(WARNING) << __func__ << ": No available ToFi buffers, skipping frame!";
                continue;
            }

            tofiBuffer = tofiBufferfifo.front();
            tofiBufferfifo.pop();
        }

        uint16_t *tempDepthFrame = m_tofiComputeContext->p_depth_frame;
        uint16_t *tempAbFrame = m_tofiComputeContext->p_ab_frame;
        float *tempConfFrame = m_tofiComputeContext->p_conf_frame;

        if (tofiBuffer) {
            m_tofiComputeContext->p_depth_frame = tofiBuffer;
            m_tofiComputeContext->p_ab_frame = tofiBuffer + (m_outputFrameWidth * m_outputFrameHeight);
            m_tofiComputeContext->p_conf_frame = (float *)(tofiBuffer + (m_outputFrameWidth * m_outputFrameHeight * 2));

            auto processStart = std::chrono::high_resolution_clock::now();

            uint32_t ret = TofiCompute(reinterpret_cast<uint16_t *>(frame.data), m_tofiComputeContext, NULL);
            if (ret != ADI_TOFI_SUCCESS) {
                LOG(ERROR) << __func__ << ": TofiCompute failed Skipping frame!";
                 {
                    std::lock_guard<std::mutex> lock(tofiBufferMutex);
                    tofiBufferfifo.push(tofiBuffer);
                    tofiBufferNotEmpty.notify_one();
                }
                continue;
            }

            auto processEnd = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> processTime = processEnd - processStart;
            totalProcessTime += static_cast<long long>(processTime.count());
            totalProcessedFrame++;

            m_tofiComputeContext->p_depth_frame = tempDepthFrame;
            m_tofiComputeContext->p_ab_frame = tempAbFrame;
            m_tofiComputeContext->p_conf_frame = tempConfFrame;
        }

        frame.tofiBuffer = std::shared_ptr<uint16_t>(
            tofiBuffer,
            [this](uint16_t* ptr) {
                {
                    std::lock_guard<std::mutex> lock(tofiBufferMutex);
                    tofiBufferfifo.push(ptr);
                }
                tofiBufferNotEmpty.notify_one();
            }
        );
        {
            std::unique_lock<std::mutex> processedLock(processedMutex);
            if (processedBufferQueue.size() >= maxProcessedQueueSize) {
                LOG(WARNING) << __func__ << ": processedBufferQueue full, dropping oldest frame!";
                processedBufferQueue.pop(); // Drop oldest frame
                processedNotFull.notify_one();
            }

	    frame.size = tofiBufferSize;
            processedBufferQueue.push(std::move(frame));
            processedNotEmpty.notify_one();

        }
    }
    if (totalProcessedFrame > 0) {
        double averageProcessTime = static_cast<double>(totalProcessTime) / totalProcessedFrame;
        //LOG(INFO) << __func__ << ": Average tofi_comupte process time: " << averageProcessTime << " ms";
    }
}

aditof::Status BufferProcessor::processBuffer(uint16_t *buffer) {
    Frame frame;
    {
        std::unique_lock<std::mutex> processedLock(processedMutex);
        if (!processedNotEmpty.wait_for(processedLock, std::chrono::seconds(2), [this] { return !processedBufferQueue.empty() || stopThreadsFlag; })) {
            LOG(WARNING) << __func__ << ": Timeout! No processed frames.";
            return aditof::Status::BUSY;
        }

        if (processedBufferQueue.empty()) {
            LOG(ERROR) << __func__ << ": No processed frame available!";
            return aditof::Status::BUSY;
        }

        frame = std::move(processedBufferQueue.front());
        processedBufferQueue.pop();
        processedNotFull.notify_one();

    }

    if (buffer && frame.tofiBuffer && frame.size > 0) {
        memcpy(buffer, frame.tofiBuffer.get(), frame.size);
    } else {
        LOG(ERROR) << "Invalid buffer or size!";
        return aditof::Status::GENERIC_ERROR;
    }
    frame.size = 0;
    return aditof::Status::OK;
}

aditof::Status BufferProcessor::waitForBufferPrivate(struct VideoDev *dev) {
    fd_set fds;
    struct timeval tv;
    int r;

    if (dev == nullptr)
        dev = m_inputVideoDev;

    FD_ZERO(&fds);
    FD_SET(dev->fd, &fds);

    tv.tv_sec = 20;
    tv.tv_usec = 0;

    r = select(dev->fd + 1, &fds, NULL, NULL, &tv);

    if (r == -1) {
        LOG(WARNING) << "select error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return aditof::Status::GENERIC_ERROR;
    } else if (r == 0) {
        LOG(WARNING) << "select timeout";
        return aditof::Status::GENERIC_ERROR;
    }
    return aditof ::Status::OK;
}

aditof::Status
BufferProcessor::dequeueInternalBufferPrivate(struct v4l2_buffer &buf,
                                              struct VideoDev *dev) {
    using namespace aditof;
    Status status = Status::OK;

    if (dev == nullptr)
        dev = m_inputVideoDev;

    CLEAR(buf);
    buf.type = dev->videoBuffersType;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.length = 1;
    buf.m.planes = dev->planes;

    if (xioctl(dev->fd, VIDIOC_DQBUF, &buf) == -1) {
        LOG(WARNING) << "VIDIOC_DQBUF error "
                     << "errno: " << errno << " error: " << strerror(errno);
        switch (errno) {
        case EAGAIN:
        case EIO:
            break;
        default:
            return Status::GENERIC_ERROR;
        }
    }

    if (buf.index >= dev->nVideoBuffers) {
        LOG(WARNING) << "Not enough buffers avaialable";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status BufferProcessor::getInternalBufferPrivate(
    uint8_t **buffer, uint32_t &buf_data_len, const struct v4l2_buffer &buf,
    struct VideoDev *dev) {
    if (dev == nullptr)
        dev = m_inputVideoDev;

    *buffer = static_cast<uint8_t *>(dev->videoBuffers[buf.index].start);
    buf_data_len = buf.bytesused;

    return aditof::Status::OK;
}

aditof::Status
BufferProcessor::enqueueInternalBufferPrivate(struct v4l2_buffer &buf,
                                              struct VideoDev *dev) {
    if (dev == nullptr)
        dev = m_inputVideoDev;

    if (xioctl(dev->fd, VIDIOC_QBUF, &buf) == -1) {
        LOG(WARNING) << "VIDIOC_QBUF error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return aditof::Status::GENERIC_ERROR;
    }

    return aditof::Status::OK;
}

aditof::Status BufferProcessor::getDeviceFileDescriptor(int &fileDescriptor) {
    fileDescriptor = m_outputVideoDev->fd;
    return aditof::Status::OK;
}

aditof::Status BufferProcessor::waitForBuffer() {

    return waitForBufferPrivate();
}

aditof::Status BufferProcessor::dequeueInternalBuffer(struct v4l2_buffer &buf) {

    return dequeueInternalBufferPrivate(buf);
}

aditof::Status
BufferProcessor::getInternalBuffer(uint8_t **buffer, uint32_t &buf_data_len,
                                   const struct v4l2_buffer &buf) {

    return getInternalBufferPrivate(buffer, buf_data_len, buf);
}

aditof::Status BufferProcessor::enqueueInternalBuffer(struct v4l2_buffer &buf) {

    return enqueueInternalBufferPrivate(buf);
}

TofiConfig *BufferProcessor::getTofiCongfig() { return m_tofiConfig; }

aditof::Status BufferProcessor::getDepthComputeVersion(uint8_t &enabled) {
    enabled = depthComputeOpenSourceEnabled;
    return aditof::Status::OK;
}

void BufferProcessor::startThreads() {
    stopThreadsFlag = false;
    streamRunning = true;

    LOG(INFO) << __func__ << ": Starting Threads..";
    captureThread = std::thread(&BufferProcessor::captureFrameThread, this);
    processingThread = std::thread(&BufferProcessor::processThread, this);
}

void BufferProcessor::stopThreads() {

    {
        std::lock_guard<std::mutex> lock(poolMutex);
        stopThreadsFlag = true;
        bufferNotEmpty.notify_all();
        bufferNotFull.notify_all();
        while (!bufferPool.empty()) {
            bufferPool.pop();
        }
    }
    {
        std::lock_guard<std::mutex> lock(processedMutex);
        processedNotEmpty.notify_all();
        processedNotFull.notify_all();
        // Clear processedBufferQueue to release Frame objects and tofiBuffer
        while (!processedBufferQueue.empty()) {
            processedBufferQueue.pop();
        }
    }
    {
        std::lock_guard<std::mutex> lock(tofiBufferMutex);
        tofiBufferNotEmpty.notify_all();  // Wake up processThread from tofiBuffer wait
    }

    if (captureThread.joinable())
        captureThread.join();
    if (processingThread.joinable())
        processingThread.join();

    {
        std::lock_guard<std::mutex> lock(tofiBufferMutex);
        while (!tofiBufferfifo.empty()) {
            free(tofiBufferfifo.front());
            tofiBufferfifo.pop();
        }
    }

    // Free preallocatedFrameBuffers
    {
        std::lock_guard<std::mutex> lock(preallocMutex);
        for (auto buffer : preallocatedFrameBuffers) {
            free(buffer);
        }
        preallocatedFrameBuffers.clear();
        while (!freeFrameBuffers.empty()) {
            freeFrameBuffers.pop();
        }
    }

    LOG(INFO) << __func__ << ": Threads Stopped..";
    streamRunning = false;
}
