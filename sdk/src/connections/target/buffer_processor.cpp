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
      m_outputFrameHeight(0), m_tofiConfig(nullptr),
      m_tofiComputeContext(nullptr), m_inputVideoDev(nullptr),
      m_bufferAllocator(BufferAllocator::getInstance()) {

    m_outputVideoDev = new VideoDev();

    LOG(INFO) << "BufferProcessor initialized with BufferAllocator at: "
              << static_cast<void*>(m_bufferAllocator.get());
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

/**
 * @function BufferProcessor::setVideoProperties
 *
 * Initializes and configures internal buffer properties based on the given
 * frame resolution and memory layout. Allocates raw and ToFi processing buffers
 * aligned to 64 bytes for optimal performance.
 *
 * @param frameWidth         The width of the output frame in pixels.
 * @param frameHeight        The height of the output frame in pixels.
 * @param WidthInBytes       The width of the raw frame in bytes (stride).
 * @param HeightInBytes      The height of the raw frame in bytes.
 *
 * @return aditof::Status    Returns OK on success, GENERIC_ERROR on allocation failure.
 */
aditof::Status BufferProcessor::setVideoProperties(int frameWidth,
                                                   int frameHeight,
                                                   int WidthInBytes,
                                                   int HeightInBytes,
                                                   int modeNumber) {
    using namespace aditof;
    Status status = Status::OK;
    m_vidPropSet = true;

    m_currentModeNumber = modeNumber;

    m_outputFrameWidth = frameWidth;
    m_outputFrameHeight = frameHeight;

    m_rawFrameBufferSize = static_cast<size_t>(WidthInBytes) * HeightInBytes;

    uint32_t depthSize = m_outputFrameWidth * m_outputFrameHeight * 2;
    uint32_t abSize = depthSize;
    uint32_t confSize = m_outputFrameWidth * m_outputFrameHeight * 4;
    m_tofiBufferSize = depthSize + abSize + confSize;

    LOG(INFO) << "setVideoProperties: Mode " << modeNumber
              << ", RawBufferSize: " << m_rawFrameBufferSize
              << ", ToFiBufferSize: " << m_tofiBufferSize;
    LOG(INFO) << "BufferAllocator instance: " << static_cast<void*>(m_bufferAllocator.get());
    LOG(INFO) << "Verifying allocated buffer sizes: m_v4l2_input_buffer_Q size: "
              << m_bufferAllocator->m_v4l2_input_buffer_Q.size()
              << ", m_tofi_io_Buffer_Q size: "
              << m_bufferAllocator->m_tofi_io_Buffer_Q.size();
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
/**
 * @function BufferProcessor::captureFrameThread
 *
 * Thread function that captures raw frames from a V4L2 video device.
 * It manages buffer queuing/dequeuing, performs sanity checks, copies
 * the captured data into a target buffer, and pushes it to a shared buffer pool.
 */
void BufferProcessor::captureFrameThread() {
    long long totalCaptureTime = 0;
    int totalV4L2Captured = 0;

    while (!stopThreadsFlag) {
        aditof::Status status;
        struct v4l2_buffer buf;
        struct VideoDev *dev = m_inputVideoDev;
        uint8_t *pdata = nullptr;
        unsigned int buf_data_len = 0;
        std::shared_ptr<uint8_t> v4l2_frame_holder;

        LOG(INFO) << "captureFrameThread: Attempting to pop from m_v4l2_input_buffer_Q, size: "
                  << m_bufferAllocator->m_v4l2_input_buffer_Q.size();
        if (!m_bufferAllocator->m_v4l2_input_buffer_Q.pop(v4l2_frame_holder) ||
            !v4l2_frame_holder) {
            if (stopThreadsFlag)
                break;
            LOG(WARNING) << "captureFrameThread: No free buffers "
                            "m_v4l2_input_buffer_Q size: "
                         << m_bufferAllocator->m_v4l2_input_buffer_Q.size();
            std::this_thread::sleep_for(
                std::chrono::milliseconds(TIME_OUT_DELAY));
            continue;
        }
        LOG(INFO) << "captureFrameThread: Popped raw buffer at address: "
                  << static_cast<void*>(v4l2_frame_holder.get());
        auto captureStart = std::chrono::high_resolution_clock::now();

        status = waitForBufferPrivate(dev);
        if (status != aditof::Status::OK) {
            LOG(ERROR) << __func__
                       << ": waitForBufferPrivate() Failed, retrying...";
            if (!m_bufferAllocator->m_v4l2_input_buffer_Q.push(v4l2_frame_holder)) {
                LOG(ERROR) << "captureFrameThread: Failed to push back raw buffer";
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(TIME_OUT_DELAY));
            continue;
        }

        status = dequeueInternalBufferPrivate(buf, dev);
        if (status != aditof::Status::OK) {
            LOG(ERROR)
                << __func__
                << ": dequeueInternalBufferPrivate() Failed, retrying...";
            if (!m_bufferAllocator->m_v4l2_input_buffer_Q.push(v4l2_frame_holder)) {
                LOG(ERROR) << "captureFrameThread: Failed to push back raw buffer";
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(TIME_OUT_DELAY));
            continue;
        }

        status = getInternalBufferPrivate(&pdata, buf_data_len, buf, dev);
        if (status != aditof::Status::OK || !pdata || buf_data_len == 0) {
            LOG(ERROR)
                << __func__
                << ": dequeueInternalBufferPrivate() Failed. Buffer index: "
                << buf.index << ", pdata: " << (void *)pdata
                << ", len: " << buf_data_len;
            // Always requeue the buffer to avoid memory leak
            enqueueInternalBufferPrivate(buf, dev);
            if (!m_bufferAllocator->m_v4l2_input_buffer_Q.push(v4l2_frame_holder)) {
                LOG(ERROR) << "captureFrameThread: Failed to push back raw buffer";
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(TIME_OUT_DELAY));
            continue;
        }

        if (v4l2_frame_holder != nullptr) {
            memcpy(v4l2_frame_holder.get(), pdata, buf_data_len);
        } else {
            LOG(WARNING)
                << __func__
                << ": v4l2_frame_holder is nullptr skipping frame copy";
            enqueueInternalBufferPrivate(buf, dev);
            if (!m_bufferAllocator->m_v4l2_input_buffer_Q.push(v4l2_frame_holder)) {
                LOG(ERROR) << "captureFrameThread: Failed to push back raw buffer";
            }
            continue;
        }

        auto captureEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> captureTime =
            captureEnd - captureStart;
        totalCaptureTime += static_cast<long long>(captureTime.count());

        totalV4L2Captured++;

        BufferAllocator::Tofi_v4l2_buffer v4l2_frame;
        v4l2_frame.data = v4l2_frame_holder;
        v4l2_frame.size = buf_data_len;

        if (!m_bufferAllocator->m_capture_to_process_Q.push(std::move(v4l2_frame))) {
            LOG(WARNING) << "captureFrameThread: Push timeout to bufferPool, "
                            "m_capture_to_process_Q Size: "
                         << m_bufferAllocator->m_capture_to_process_Q.size();
            if (!m_bufferAllocator->m_v4l2_input_buffer_Q.push(v4l2_frame_holder)) {
                LOG(ERROR) << "captureFrameThread: Failed to push back raw buffer";
            }
            enqueueInternalBufferPrivate(buf);
            continue;
        }

        if (enqueueInternalBufferPrivate(buf, dev) != aditof::Status::OK) {
            LOG(ERROR) << __func__ << ": enqueueInternalBufferPrivate() Failed";
        }

        LOG(INFO) << "captureFrameThread: Frame captured, m_v4l2_input_buffer_Q size: "
                  << m_bufferAllocator->m_v4l2_input_buffer_Q.size()
                  << ", m_capture_to_process_Q size: "
                  << m_bufferAllocator->m_capture_to_process_Q.size();

        // (Optional) A short sleep to avoid hammering the device, if needed.
        //std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (totalV4L2Captured > 0) {
        double averageCaptureTime =
            static_cast<double>(totalCaptureTime) / totalV4L2Captured;
        //LOG(INFO) << __func__ << ": Average capture time: " << averageCaptureTime << " ms";
    }
}

/**
 * @brief Thread to process raw frames using the ToFi compute engine.
 *
 * This thread:
 *   - Waits for raw frames in `m_v4l2_capture_queue`.
 *   - Pops a preallocated processing buffer from `tofiBufferQueue`.
 *   - Splits that buffer into depth, AB, and confidence sections.
 *   - Runs the `TofiCompute()` pipeline.
 *   - Pushes the processed frame to `processedBufferQueue`.
 *
 * After processing, it restores compute context pointers and returns used buffers.
 */

void BufferProcessor::processThread() {
    long long totalProcessTime = 0;
    int totalProcessedFrame = 0;

    while (!stopThreadsFlag) {
        BufferAllocator::Tofi_v4l2_buffer process_frame;
        LOG(INFO) << "processThread: Attempting to pop from m_capture_to_process_Q, size: "
                  << m_bufferAllocator->m_capture_to_process_Q.size();
        if (!m_bufferAllocator->m_capture_to_process_Q.pop(process_frame)) {
            if (stopThreadsFlag)
                break;
            LOG(WARNING) << "processThread: No new frames, "
                            "m_capture_to_process_Q Size: "
                         << m_bufferAllocator->m_capture_to_process_Q.size();
            std::this_thread::sleep_for(
                std::chrono::milliseconds(TIME_OUT_DELAY));
            continue;
        }
        LOG(INFO) << "processThread: Popped frame with raw buffer at address: "
                  << static_cast<void*>(process_frame.data.get());

        std::shared_ptr<uint16_t> tofi_compute_io_buff;
        LOG(INFO) << "processThread: Attempting to pop from m_tofi_io_Buffer_Q, size: "
                  << m_bufferAllocator->m_tofi_io_Buffer_Q.size();
        if (!m_bufferAllocator->m_tofi_io_Buffer_Q.pop(tofi_compute_io_buff)) {
            if (stopThreadsFlag)
                break;
            LOG(WARNING)
                << "processThread: No ToFi buffers, m_tofi_io_Buffer_Q Size: "
                << m_bufferAllocator->m_tofi_io_Buffer_Q.size();

           if (!m_bufferAllocator->m_v4l2_input_buffer_Q.push(process_frame.data)) {
                LOG(ERROR) << "processThread: Failed to push back raw buffer";
            }
            
            std::this_thread::sleep_for(
                std::chrono::milliseconds(TIME_OUT_DELAY));
            continue;
        }
        LOG(INFO) << "processThread: Popped ToFi buffer at address: "
                  << static_cast<void*>(tofi_compute_io_buff.get());

        if (process_frame.size > m_rawFrameBufferSize) {
            LOG(ERROR)
                << "processThread: Frame size " << process_frame.size
                << " exceeds allocated raw buffer size " << m_rawFrameBufferSize;
            if (!m_bufferAllocator->m_tofi_io_Buffer_Q.push(tofi_compute_io_buff)) {
                LOG(ERROR) << "processThread: Failed to push back ToFi buffer";
            }
            if (!m_bufferAllocator->m_v4l2_input_buffer_Q.push(process_frame.data)) {
                LOG(ERROR) << "processThread: Failed to push back raw buffer";
            }
            continue;
        }

        uint16_t *tempDepthFrame = m_tofiComputeContext->p_depth_frame;
        uint16_t *tempAbFrame = m_tofiComputeContext->p_ab_frame;
        float *tempConfFrame = m_tofiComputeContext->p_conf_frame;

        if (tofi_compute_io_buff) {

            // Each ToFi buffer holds three sections: depth (uint16), AB (uint16), and confidence (float)
            // Layout: [depth: W * H * 2 | AB: W * H * 2 | confidence: W * H * 4]
            // All sections are packed into a single buffer to optimize allocation and cache locality.

            const int numPixels = m_outputFrameWidth * m_outputFrameHeight;

            // Map ToFi processing outputs to sections of the shared buffer
            m_tofiComputeContext->p_depth_frame =
                tofi_compute_io_buff.get(); // Depth starts at offset 0
            m_tofiComputeContext->p_ab_frame =
                tofi_compute_io_buff.get() + numPixels; // AB follows depth
            m_tofiComputeContext->p_conf_frame = reinterpret_cast<float *>(
                tofi_compute_io_buff.get() +
                numPixels * 2); // Confidence follows AB

#ifdef DUAL
            if (m_currentModeNumber == 0 ||
                m_currentModeNumber ==
                    1) { // For dual pulsatrix mode 1 and 0 confidance frame is not enabled
                memcpy(m_tofiComputeContext->p_depth_frame,
                       process_frame.data.get(), numPixels);

                memcpy(m_tofiComputeContext->p_ab_frame,
                       process_frame.data.get() + numPixels, numPixels);
            }
#else
            auto processStart = std::chrono::high_resolution_clock::now();

            uint32_t ret = TofiCompute(
                reinterpret_cast<uint16_t *>(process_frame.data.get()),
                m_tofiComputeContext, NULL);
            if (ret != ADI_TOFI_SUCCESS) {
                LOG(ERROR) << "processThread: TofiCompute failed";
                m_bufferAllocator->m_tofi_io_Buffer_Q.push(tofi_compute_io_buff);
                m_bufferAllocator->m_v4l2_input_buffer_Q.push(process_frame.data);
                m_tofiComputeContext->p_depth_frame = tempDepthFrame;
                m_tofiComputeContext->p_ab_frame = tempAbFrame;
                m_tofiComputeContext->p_conf_frame = tempConfFrame;
                continue;
            }
#endif
            auto processEnd = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> processTime =
                processEnd - processStart;
            totalProcessTime += static_cast<long long>(processTime.count());
            totalProcessedFrame++;

            m_tofiComputeContext->p_depth_frame = tempDepthFrame;
            m_tofiComputeContext->p_ab_frame = tempAbFrame;
            m_tofiComputeContext->p_conf_frame = tempConfFrame;
        } else {
            LOG(WARNING)
                << "processThread: Invalid ToFi buffer";
            if (!m_bufferAllocator->m_tofi_io_Buffer_Q.push(tofi_compute_io_buff)) {
                LOG(ERROR) << "processThread: Failed to push back ToFi buffer";
            }
            if (!m_bufferAllocator->m_v4l2_input_buffer_Q.push(process_frame.data)) {
                LOG(ERROR) << "processThread: Failed to push back raw buffer";
            }
            continue;
        }

        process_frame.tofiBuffer = tofi_compute_io_buff;
        process_frame.size = m_tofiBufferSize;

        if (!m_bufferAllocator->m_process_done_Q.push(std::move(process_frame))) {
            LOG(WARNING) << "processThread: Push timeout to "
                            "m_process_done_Q, ProcessedQueueSize: "
                         << m_bufferAllocator->m_process_done_Q.size();
            m_bufferAllocator->m_tofi_io_Buffer_Q.push(tofi_compute_io_buff);
            m_bufferAllocator->m_v4l2_input_buffer_Q.push(process_frame.data);
            continue;
        }
    }
    if (totalProcessedFrame > 0) {
        double averageProcessTime =
            static_cast<double>(totalProcessTime) / totalProcessedFrame;
        //LOG(INFO) << __func__ << ": Average tofi_comupte process time: " << averageProcessTime << " ms";
    }
}

/**
 * @function BufferProcessor::processBuffer
 *
 * Function to retrieve the next available processed buffer.
 * It copies the computed output into the user-provided buffer and manages buffer reuse.
 * Returns a status indicating success, busy (no frames), or error.
 */
aditof::Status BufferProcessor::processBuffer(uint16_t *buffer) {
    BufferAllocator::Tofi_v4l2_buffer tof_processed_frame;
    const std::chrono::milliseconds m_retryDelay(10);

    // Loop for maxTries attempts. 'attempt' counts from 0 to m_maxTries - 1.
    for (int attempt = 0; attempt < m_maxTries; ++attempt) {
        if (m_bufferAllocator->m_process_done_Q.pop(tof_processed_frame)) {
            if (buffer && tof_processed_frame.tofiBuffer &&
                tof_processed_frame.size > 0) {
                // Ensure 'buffer' has enough allocated memory for 'tof_processed_frame.size'
                // For this example, we assume `buffer` is large enough.
                memcpy(buffer, tof_processed_frame.tofiBuffer.get(),
                       tof_processed_frame.size);

                // Return buffers to their respective pools
                    if (!m_bufferAllocator->m_tofi_io_Buffer_Q.push(tof_processed_frame.tofiBuffer)) {
                        LOG(ERROR) << "processBuffer: Failed to push back ToFi buffer";
                    }
                    if (!m_bufferAllocator->m_v4l2_input_buffer_Q.push(tof_processed_frame.data)) {
                        LOG(ERROR) << "processBuffer: Failed to push back raw buffer";
                    }

                return aditof::Status::OK; // Success, exit function
            } else {
                // Pop succeeded, but the frame data itself was invalid.
                LOG(ERROR) << "processBuffer: Pop succeeded but frame data is "
                              "invalid (buffer/tofiBuffer/size). "
                           << "Returning error immediately.\n";
                return aditof::Status::GENERIC_ERROR;
            }
        } else {
            if (attempt < m_maxTries - 1) {
                // If it's not the last attempt, wait and then the loop will try again.
                LOG(INFO) << "processBuffer: Pop failed on attempt #"
                          << (attempt + 1) << ". Retrying in "
                          << m_retryDelay.count() << "ms...\n";
                std::this_thread::sleep_for(m_retryDelay);
            } else {
                // This was the last attempt (m_maxTries - 1 index) and it failed.
                LOG(WARNING)
                    << "processBuffer: Failed to pop frame after " << m_maxTries
                    << " attempts. "
                    << "m_process_done_Q size: " << m_bufferAllocator->m_process_done_Q.size()
                    << "\n";
                return aditof::Status::
                    GENERIC_ERROR; // Indicate final failure to the caller
            }
        }
    }

    // This line should technically not be reached if m_maxTries > 0,
    // as the loop will either return OK or GENERIC_ERROR.
    // Included as a safeguard.
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

    tv.tv_sec = stopThreadsFlag ? 0 : 20;
    //tv.tv_sec = 20;
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
    m_captureThread = std::thread(&BufferProcessor::captureFrameThread, this);
    m_processingThread = std::thread(&BufferProcessor::processThread, this);

    // Schedule capture thread
    sched_param cap_param;
    cap_param.sched_priority = 20; // Lower priority than processing thread
    if (pthread_setschedparam(m_captureThread.native_handle(), SCHED_FIFO, &cap_param) != 0) {
        LOG(WARNING) << "Failed to set SCHED_FIFO for capture thread: " << strerror(errno);
    } else {
        LOG(INFO) << "Capture thread set to SCHED_FIFO with priority 15";
    }

    sched_param param;
    param.sched_priority = 15;
    if (pthread_setschedparam(m_processingThread.native_handle(), SCHED_FIFO, &param) != 0) {
        LOG(WARNING) << "Failed to set SCHED_FIFO for processing thread: " << strerror(errno);
    } else {
        LOG(INFO) << "Processing thread set to SCHED_FIFO with priority 20";
    }
}

void BufferProcessor::stopThreads() {
    stopThreadsFlag = true;
    streamRunning = false;

    if (m_captureThread.joinable())
        m_captureThread.join();

    if (m_processingThread.joinable())
        m_processingThread.join();

    LOG(INFO) << __func__ << ": Threads Stopped..";
}