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
    : bufferPool(MAX_QUEUE_SIZE),
      processedBufferQueue(MAX_QUEUE_SIZE),
      freeFrameBufferQueue(TOFI_BUFFER_COUNT),
      tofiBufferQueue(TOFI_BUFFER_COUNT),
      m_vidPropSet(false), m_processorPropSet(false), m_outputFrameWidth(0),
      m_outputFrameHeight(0), m_processedBuffer(nullptr), m_tofiConfig(nullptr),
      m_tofiComputeContext(nullptr), m_inputVideoDev(nullptr) {
    m_outputVideoDev = new VideoDev();

    preallocatedFrameBuffers.clear();
    tofiBuffers.clear();
    LOG(INFO) << "BufferProcessor initialized";
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
    delete m_outputVideoDev;
    m_outputVideoDev = nullptr;
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
    m_vidPropSet = true;

    m_outputFrameWidth = frameWidth;
    m_outputFrameHeight = frameHeight;

    LOG(INFO) << __func__ << ": Width : " << m_outputFrameWidth << " Height: " << m_outputFrameHeight;

    size_t rawFrameSize = static_cast<size_t>(m_outputFrameWidth) * m_outputFrameHeight * 6.5;
    rawFrameBufferSize = rawFrameSize;
    {
        preallocatedFrameBuffers.clear();
        preallocatedFrameBuffers.reserve(TOFI_BUFFER_COUNT);
        std::lock_guard<std::mutex> lock(preallocMutex);
        for (int i = 0; i < TOFI_BUFFER_COUNT; ++i) {
            uint8_t* buffer = static_cast<uint8_t*>(aligned_alloc(64, rawFrameSize));
            if (!buffer) {
                LOG(ERROR) << __func__ << ": Failed to allocate raw buffer!";
                status = Status::GENERIC_ERROR;
            }
            preallocatedFrameBuffers.push_back(buffer);
            freeFrameBufferQueue.push(buffer);
        }
    }

    uint32_t depthSize = m_outputFrameWidth * m_outputFrameHeight * 2;		/* | Depth Frame ( W * H * 2 (type: uint16_t)) |   */
    uint32_t abSize = m_outputFrameWidth * m_outputFrameHeight * 2;		/* | AB Frame ( W * H * 2 (type: uint16_t)) |      */
    uint32_t confSize = m_outputFrameWidth * m_outputFrameHeight * 4;		/* | Confidance Frame ( W * H * 4 (type: float)) | */
    tofiBufferSize = depthSize + abSize + confSize;

    tofiBuffers.clear();
    tofiBuffers.reserve(TOFI_BUFFER_COUNT);
    for (int i = 0; i < TOFI_BUFFER_COUNT; ++i) {
        uint16_t* buffer = static_cast<uint16_t*>(aligned_alloc(64, tofiBufferSize));
        if (!buffer) {
            LOG(ERROR) << "setVideoProperties: Failed to allocate ToFi buffer!";
            return aditof::Status::GENERIC_ERROR;
        }
        tofiBuffers.push_back(buffer);
        tofiBufferQueue.push(buffer);
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

    while (!stopThreadsFlag) {
        aditof::Status status;
        struct v4l2_buffer buf;
        struct VideoDev *dev = m_inputVideoDev;
        uint8_t *pdata = nullptr;
        unsigned int buf_data_len = 0;
        uint8_t* targetBuffer;

        if (!freeFrameBufferQueue.pop(targetBuffer)) {
            LOG(WARNING) << "captureFrameThread: No free buffers FreeQueueSize: " << freeFrameBufferQueue.size();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        auto captureStart = std::chrono::high_resolution_clock::now();

        status = waitForBufferPrivate(dev);
        if (status != aditof::Status::OK){
            LOG(ERROR) << __func__ << ": waitForBufferPrivate() Failed, retrying...";
            freeFrameBufferQueue.push(targetBuffer);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        status = dequeueInternalBufferPrivate(buf, dev);
        if (status != aditof::Status::OK){
            LOG(ERROR) << __func__ << ": dequeueInternalBufferPrivate() Failed, retrying...";
            freeFrameBufferQueue.push(targetBuffer);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        status = getInternalBufferPrivate(&pdata, buf_data_len, buf, dev);
        if (status != aditof::Status::OK || !pdata || buf_data_len == 0) {
            LOG(ERROR) << __func__ << ": dequeueInternalBufferPrivate() Failed. Buffer index: " << buf.index << ", pdata: " << (void*)pdata << ", len: " << buf_data_len;
            // Always requeue the buffer to avoid memory leak
            enqueueInternalBufferPrivate(buf, dev);
            freeFrameBufferQueue.push(targetBuffer);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

	    memcpy(targetBuffer, pdata, buf_data_len);

        auto captureEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> captureTime = captureEnd - captureStart;
        totalCaptureTime += static_cast<long long>(captureTime.count());

	    if(totalV4L2Captured == 0)
		    LOG(INFO) << __func__ << ": V4l2 frame size: " << buf_data_len;
        totalV4L2Captured++;

        Tofi_v4l2_buffer v4l2_frame;
        v4l2_frame.data = targetBuffer;
        v4l2_frame.size = buf_data_len;

        if (!bufferPool.push(std::move(v4l2_frame))) {
            LOG(WARNING) << "captureFrameThread: Push timeout to bufferPool, BufferPoolSize: " << bufferPool.size();
            freeFrameBufferQueue.push(targetBuffer);
            enqueueInternalBufferPrivate(buf);
            continue;
        }

        if(enqueueInternalBufferPrivate(buf, dev) != aditof::Status::OK) {
            LOG(ERROR) << __func__ << ": enqueueInternalBufferPrivate() Failed";
        }

        // (Optional) A short sleep to avoid hammering the device, if needed.
        //std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (totalV4L2Captured > 0) {
        double averageCaptureTime = static_cast<double>(totalCaptureTime) / totalV4L2Captured;
        LOG(INFO) << __func__ << ": Total frames captured from v4l2: " << totalV4L2Captured;
        //LOG(INFO) << __func__ << ": Average capture time: " << averageCaptureTime << " ms";
    }
}

void BufferProcessor::processThread() {
    LOG(INFO) << __func__ << ": Processing thread started";

    long long totalProcessTime = 0;
    int totalProcessedFrame = 0;

    while (!stopThreadsFlag) {
        Tofi_v4l2_buffer process_frame;
        if (!bufferPool.pop(process_frame)) {
            LOG(WARNING) << "processThread: No new frames, BufferPoolSize: " << bufferPool.size();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        uint16_t* tofiBuffer;
        if (!tofiBufferQueue.pop(tofiBuffer)) {
            LOG(WARNING) << "processThread: No ToFi buffers, TofiQueueSize: " << tofiBufferQueue.size();
            freeFrameBufferQueue.push(process_frame.data);
            continue;
        }

        uint16_t *tempDepthFrame = m_tofiComputeContext->p_depth_frame;
        uint16_t *tempAbFrame = m_tofiComputeContext->p_ab_frame;
        float *tempConfFrame = m_tofiComputeContext->p_conf_frame;

        if (tofiBuffer) {
            m_tofiComputeContext->p_depth_frame = tofiBuffer;
            m_tofiComputeContext->p_ab_frame = tofiBuffer + (m_outputFrameWidth * m_outputFrameHeight);
            m_tofiComputeContext->p_conf_frame = (float *)(tofiBuffer + (m_outputFrameWidth * m_outputFrameHeight * 2));

            auto processStart = std::chrono::high_resolution_clock::now();

            uint32_t ret = TofiCompute(reinterpret_cast<uint16_t *>(process_frame.data), m_tofiComputeContext, NULL);
            if (ret != ADI_TOFI_SUCCESS) {
                LOG(ERROR) << "processThread: TofiCompute failed";
                tofiBufferQueue.push(tofiBuffer);
                freeFrameBufferQueue.push(process_frame.data);
                m_tofiComputeContext->p_depth_frame = tempDepthFrame;
                m_tofiComputeContext->p_ab_frame = tempAbFrame;
                m_tofiComputeContext->p_conf_frame = tempConfFrame;
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

        process_frame.tofiBuffer = tofiBuffer;
        process_frame.size = tofiBufferSize;

        if (!processedBufferQueue.push(std::move(process_frame))) {
            LOG(WARNING) << "processThread: Push timeout to processedBufferQueue, ProcessedQueueSize: " << processedBufferQueue.size();
            tofiBufferQueue.push(tofiBuffer);
            freeFrameBufferQueue.push(process_frame.data);
            continue;
        }
    }
    if (totalProcessedFrame > 0) {
        double averageProcessTime = static_cast<double>(totalProcessTime) / totalProcessedFrame;
        //LOG(INFO) << __func__ << ": Average tofi_comupte process time: " << averageProcessTime << " ms";
    }
}

aditof::Status BufferProcessor::processBuffer(uint16_t *buffer) {
    Tofi_v4l2_buffer tof_processed_frame;
    if (!processedBufferQueue.pop(tof_processed_frame)) {
        LOG(WARNING) << "processBuffer: No processed frames, ProcessedQueueSize: " << processedBufferQueue.size();
        return aditof::Status::BUSY;
    }

    if (buffer && tof_processed_frame.tofiBuffer && tof_processed_frame.size > 0) {
        memcpy(buffer, tof_processed_frame.tofiBuffer, tof_processed_frame.size);
        tofiBufferQueue.push(tof_processed_frame.tofiBuffer);
        freeFrameBufferQueue.push(tof_processed_frame.data);
        return aditof::Status::OK;
    }

    LOG(ERROR) << "processBuffer: Invalid buffer";
    if (tof_processed_frame.tofiBuffer) tofiBufferQueue.push(tof_processed_frame.tofiBuffer);
    freeFrameBufferQueue.push(tof_processed_frame.data);
    return aditof::Status::GENERIC_ERROR;
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
    sched_param param;
    param.sched_priority = 20;
    pthread_setschedparam(processingThread.native_handle(), SCHED_FIFO, &param);
}

void BufferProcessor::stopThreads() {
    stopThreadsFlag = true;
    streamRunning = false;

    Tofi_v4l2_buffer frame;
    while (bufferPool.pop(frame)) {
        if (frame.data) freeFrameBufferQueue.push(frame.data);
        if (frame.tofiBuffer) tofiBufferQueue.push(frame.tofiBuffer);
    }
    while (processedBufferQueue.pop(frame)) {
        if (frame.data) freeFrameBufferQueue.push(frame.data);
        if (frame.tofiBuffer) tofiBufferQueue.push(frame.tofiBuffer);
    }
    uint16_t* tofiBuffer;
    while (tofiBufferQueue.pop(tofiBuffer)) {}


    if (captureThread.joinable())
        captureThread.join();
    if (processingThread.joinable())
        processingThread.join();

    {
        std::lock_guard<std::mutex> lock(preallocMutex);
        for (auto buffer : preallocatedFrameBuffers) free(buffer);
        preallocatedFrameBuffers.clear();
        for (auto buffer : tofiBuffers) free(buffer);
        tofiBuffers.clear();
        uint8_t* rawBuffer;
        while (freeFrameBufferQueue.pop(rawBuffer)) {}
    }

    LOG(INFO) << __func__ << ": Threads Stopped..";

}
