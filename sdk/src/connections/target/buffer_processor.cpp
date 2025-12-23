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
// TO DO: This exists in linux_utils.h which is not included on Dragoboard.
// Should not have duplicated code if possible.

#include <aditof/log.h>
#include <algorithm>
#include <arm_neon.h>
#include <cmath>
#include <exception>
#include <fcntl.h>
#include <fstream>
#include <linux/videodev2.h>
#include <memory>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <unordered_map>

#include "buffer_processor.h"

uint8_t depthComputeOpenSourceEnabled = 0;

// Named constants for configuration
namespace {
constexpr int MAX_RETRIES = 3;
constexpr std::chrono::milliseconds RETRY_DELAY{10};
constexpr std::chrono::milliseconds QUEUE_TIMEOUT{1100};
constexpr int SELECT_TIMEOUT_SEC = 20;
constexpr int THREAD_PRIORITY = 20;
} // namespace

#define CLEAR(x) memset(&(x), 0, sizeof(x))

static int xioctl(int fh, unsigned int request, void *arg) {
    int r;

    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno && errno != 0);

    return r;
}

BufferProcessor::BufferProcessor()
    :
#ifdef HAS_RGB_CAMERA
      m_rgbSensor(nullptr), m_rgbCaptureEnabled(false), m_totalRGBCaptured(0),
      m_totalRGBFailures(0), m_rgb_frame_Q(BufferProcessor::MAX_QUEUE_SIZE),
#endif
      m_v4l2_input_buffer_Q(BufferProcessor::MAX_QUEUE_SIZE),
      m_capture_to_process_Q(BufferProcessor::MAX_QUEUE_SIZE),
      m_tofi_io_Buffer_Q(BufferProcessor::MAX_QUEUE_SIZE),
      m_process_done_Q(BufferProcessor::MAX_QUEUE_SIZE) {
    m_outputVideoDev = new VideoDev();
    m_outputFrameWidth = 0;
    m_outputFrameHeight = 0;
    m_processorPropSet = false;
    m_vidPropSet = false;
    stopThreadsFlag = true;
    stopThreadsFlag.store(true, std::memory_order_release);
    streamRunning = false;
    m_tofiConfig = nullptr;
    m_tofiComputeContext = nullptr;
    m_inputVideoDev = nullptr;
    m_v4l2_input_buffer_Q.set_max_size(BufferProcessor::MAX_QUEUE_SIZE);
    m_capture_to_process_Q.set_max_size(BufferProcessor::MAX_QUEUE_SIZE);
    m_tofi_io_Buffer_Q.set_max_size(BufferProcessor::MAX_QUEUE_SIZE);
    m_process_done_Q.set_max_size(BufferProcessor::MAX_QUEUE_SIZE);

    LOG(INFO) << "BufferProcessor initialized"
#ifdef HAS_RGB_CAMERA
              << " with RGB support"
#endif
        ;
}

BufferProcessor::~BufferProcessor() {
    // STEP 1: Stop threads first (this also drains all queues)
    if (!stopThreadsFlag.load(std::memory_order_acquire)) {
        stopThreads();
    }

    // STEP 2: Free ToFi resources
    // After stopThreads() completes, all worker threads have exited and queues are drained.
    // The processThread() properly restores ToFi context pointers after each frame,
    // so the context pointers should point to the original buffers allocated by InitTofiCompute().
    // The context pointers should still point to the original buffers
    // allocated by InitTofiCompute(), allowing proper cleanup
    if (NULL != m_tofiComputeContext) {
        LOG(INFO) << "freeComputeLibrary";
        FreeTofiCompute(m_tofiComputeContext);
        m_tofiComputeContext = NULL;
    }

    if (m_tofiConfig != NULL) {
        FreeTofiConfig(m_tofiConfig);
        m_tofiConfig = NULL;
    }

    // STEP 3: Close video device
    if (m_outputVideoDev != nullptr) {
        if (m_outputVideoDev->fd != -1) {
            if (::close(m_outputVideoDev->fd) == -1) {
                LOG(ERROR) << "Failed to close " << m_videoDeviceName
                           << " error: " << strerror(errno);
            }
        }
        delete m_outputVideoDev;
        m_outputVideoDev = nullptr;
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
aditof::Status BufferProcessor::setVideoProperties(
    int frameWidth, int frameHeight, int WidthInBytes, int HeightInBytes,
    int modeNumber, uint8_t bitsInAB, uint8_t bitsInConf) {

    // Clear all queues to prevent memory leaks if setVideoProperties is called multiple times
    if (!stopThreadsFlag.load(std::memory_order_acquire)) {
        stopThreads();
    }

    using namespace aditof;
    Status status = Status::OK;
    m_vidPropSet = true;

    m_currentModeNumber = modeNumber;

    m_outputFrameWidth = frameWidth;
    m_outputFrameHeight = frameHeight;

#ifdef NVIDIA
    m_rawFrameBufferSize =
        static_cast<size_t>(WidthInBytes) * HeightInBytes + WidthInBytes;
#else
    m_rawFrameBufferSize = static_cast<size_t>(WidthInBytes) * HeightInBytes;
#endif
    {
        LOG(INFO) << __func__ << ": Allocating "
                  << BufferProcessor::MAX_QUEUE_SIZE
                  << " raw frame buffers, each of size " << m_rawFrameBufferSize
                  << " bytes (total: "
                  << (BufferProcessor::MAX_QUEUE_SIZE * m_rawFrameBufferSize) /
                         (1024.0 * 1024.0)
                  << " MB)";
        for (int i = 0; i < (int)BufferProcessor::MAX_QUEUE_SIZE; ++i) {
            auto buffer =
                std::shared_ptr<uint8_t>(new uint8_t[m_rawFrameBufferSize],
                                         std::default_delete<uint8_t[]>());
            if (!buffer) {
                LOG(ERROR) << __func__ << ": Failed to allocate raw buffer!";
                status = Status::GENERIC_ERROR;
            }
            m_v4l2_input_buffer_Q.push(buffer);
        }
    }

    calculateFrameSize(bitsInAB, bitsInConf);

    LOG(INFO) << __func__ << ": Allocating " << BufferProcessor::MAX_QUEUE_SIZE
              << " ToFi buffers, each of size "
              << m_tofiBufferSize * sizeof(uint16_t) << " bytes (total: "
              << (BufferProcessor::MAX_QUEUE_SIZE * m_tofiBufferSize *
                  sizeof(uint16_t)) /
                     (1024.0 * 1024.0)
              << " MB)";
    for (int i = 0; i < (int)BufferProcessor::MAX_QUEUE_SIZE; ++i) {
        auto buffer = std::shared_ptr<uint16_t>(
            new uint16_t[m_tofiBufferSize], std::default_delete<uint16_t[]>());
        if (!buffer) {
            LOG(ERROR) << "setVideoProperties: Failed to allocate ToFi buffer!";
            return aditof::Status::GENERIC_ERROR;
        }
        m_tofi_io_Buffer_Q.push(buffer);
    }

    return status;
}

/**
 * @function BufferProcessor::calculateFrameSize
 *
 * Calculate the frame size for given bit combination of
 * AB and confidence 
 *
 * @param bitsInAB           Bits per pixel for AB frame.
 * @param bitsInConf         Bits per pixel for Confidence frame
 */

void BufferProcessor::calculateFrameSize(uint8_t &bitsInAB,
                                         uint8_t &bitsInConf) {

    /* | Depth Frame ( W * H (type: uint16_t)) |   */
    /* | AB Frame ( W * H (type: uint16_t)) |    */
    /* | Confidance Frame ( W * H * 2 (type: float)) | */

    uint32_t depthSize = m_outputFrameWidth * m_outputFrameHeight;
    uint32_t abSize = 0;
    uint32_t confSize = 0;

    // check whether the bit are not configured 0
    // for 0 bit configuration, it'll not contribute in framesize
    if ((bitsInAB != 0) && (bitsInConf == 0)) {
        // Conf bit is set to 0
        abSize = m_outputFrameWidth * m_outputFrameHeight;

    } else if ((bitsInAB == 0) && (bitsInConf != 0)) {
        // AB bit is set to 0
        confSize = m_outputFrameWidth * m_outputFrameHeight * 2;
    } else if ((bitsInAB == 0) && (bitsInConf == 0)) {
        // No need to add size
    } else {

        abSize = m_outputFrameWidth * m_outputFrameHeight;
        confSize = m_outputFrameWidth * m_outputFrameHeight * 2;
    }

    m_tofiBufferSize = depthSize + abSize + confSize;
}

aditof::Status BufferProcessor::setProcessorProperties(
    uint8_t *iniFile, uint16_t iniFileLength, uint8_t *calData,
    uint16_t calDataLength, uint16_t mode, bool ispEnabled) {

    // Free previous compute context and config to avoid memory leaks on repeated mode changes
    if (m_tofiComputeContext != nullptr) {
        LOG(INFO) << __func__ << ": Freeing previous compute context.";
        FreeTofiCompute(m_tofiComputeContext);
        m_tofiComputeContext = nullptr;
    }
    if (m_tofiConfig != nullptr) {
        LOG(INFO) << __func__ << ": Freeing previous config.";
        FreeTofiConfig(m_tofiConfig);
        m_tofiConfig = nullptr;
    }

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

#ifdef HAS_RGB_CAMERA
/**
 * @function BufferProcessor::captureRGBFrameThread
 *
 * Thread function that captures RGB frames from AR0234 sensor using GStreamer backend.
 * It continuously captures RGB frames and pushes them to m_rgb_frame_Q for synchronization
 * with depth frames. This thread runs independently of the depth processing pipeline.
 */
void BufferProcessor::captureRGBFrameThread() {
    long long totalRGBCaptureTime = 0;
    int totalRGBFramesCaptured = 0;

    bool loggedWaiting = false;
    while (!stopThreadsFlag) {
        // Check if RGB capture is enabled and sensor is available
        if (!m_rgbCaptureEnabled || m_rgbSensor == nullptr) {
            if (!loggedWaiting) {
                LOG(INFO) << "RGB capture thread waiting for sensor...";
                loggedWaiting = true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        auto captureStart = std::chrono::high_resolution_clock::now();

        // Capture RGB frame from sensor (GStreamer backend with optimized pull model)
        aditof::RGBFrame rgbFrame;
        aditof::Status status = m_rgbSensor->getFrame(rgbFrame, 200);

        auto captureEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> captureTime =
            captureEnd - captureStart;

        if (status == aditof::Status::OK && rgbFrame.isValid()) {
            // Push RGB frame to queue
            if (m_rgb_frame_Q.push(std::move(rgbFrame))) {
                totalRGBFramesCaptured++;
                totalRGBCaptureTime += captureTime.count();
                m_totalRGBCaptured++;
            } else {
                LOG(WARNING) << "RGB frame queue full, dropping frame";
                m_totalRGBFailures++;
            }
        } else {
            m_totalRGBFailures++;

            // Auto-disable after too many consecutive failures
            if (m_totalRGBFailures > 10 && m_totalRGBCaptured == 0) {
                LOG(ERROR) << "Too many RGB failures (" << m_totalRGBFailures
                           << "), disabling RGB capture";
                m_rgbCaptureEnabled = false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
#ifdef DBG_MEASURE_TIME
    if (totalRGBFramesCaptured > 0) {
        double avgCaptureTime =
            static_cast<double>(totalRGBCaptureTime) / totalRGBFramesCaptured;
        LOG(INFO) << "RGB capture complete: " << totalRGBFramesCaptured
                  << " frames, avg " << avgCaptureTime << " ms";
    }
#endif //DBG_MEASURE_TIME
}
#endif // HAS_RGB_CAMERA

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

    while (!stopThreadsFlag.load(std::memory_order_acquire)) {
        aditof::Status status;
        struct v4l2_buffer buf;
        struct VideoDev *dev = m_inputVideoDev;
        uint8_t *pdata = nullptr;
        unsigned int buf_data_len = 0;
        std::shared_ptr<uint8_t> v4l2_frame_holder;

        if (!m_v4l2_input_buffer_Q.pop(v4l2_frame_holder) ||
            !v4l2_frame_holder) {
            if (stopThreadsFlag.load(std::memory_order_acquire))
                break;
            LOG(WARNING) << "captureFrameThread: No free buffers "
                            "m_v4l2_input_buffer_Q size: "
                         << m_v4l2_input_buffer_Q.size();
            std::this_thread::sleep_for(
                std::chrono::milliseconds(BufferProcessor::getTimeoutDelay()));
            continue;
        }

        auto captureStart = std::chrono::high_resolution_clock::now();

        status = waitForBufferPrivate(dev);
        if (status != aditof::Status::OK) {
            LOG(ERROR) << __func__
                       << ": waitForBufferPrivate() Failed, retrying...";
            m_v4l2_input_buffer_Q.push(v4l2_frame_holder);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(BufferProcessor::getTimeoutDelay()));
            continue;
        }

        status = dequeueInternalBufferPrivate(buf, dev);
        if (status != aditof::Status::OK) {
            LOG(ERROR)
                << __func__
                << ": dequeueInternalBufferPrivate() Failed, retrying...";
            m_v4l2_input_buffer_Q.push(v4l2_frame_holder);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(BufferProcessor::getTimeoutDelay()));
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
            m_v4l2_input_buffer_Q.push(v4l2_frame_holder);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(BufferProcessor::getTimeoutDelay()));
            continue;
        }

        if (v4l2_frame_holder != nullptr) {
            memcpy(v4l2_frame_holder.get(), pdata, buf_data_len);
        } else {
            LOG(WARNING)
                << __func__
                << ": v4l2_frame_holder is nullptr skipping frame copy";
            continue;
        }

        auto captureEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> captureTime =
            captureEnd - captureStart;
        totalCaptureTime += static_cast<long long>(captureTime.count());

        totalV4L2Captured++;

        Tofi_v4l2_buffer v4l2_frame;
        v4l2_frame.data = v4l2_frame_holder;
        v4l2_frame.size = buf_data_len;

        if (!m_capture_to_process_Q.push(std::move(v4l2_frame))) {
            LOG(WARNING) << "captureFrameThread: Push timeout to bufferPool, "
                            "m_captureToProcessQueue Size: "
                         << m_capture_to_process_Q.size();
            m_v4l2_input_buffer_Q.push(v4l2_frame_holder);
            enqueueInternalBufferPrivate(buf);
            continue;
        }

        if (enqueueInternalBufferPrivate(buf, dev) != aditof::Status::OK) {
            LOG(ERROR) << __func__ << ": enqueueInternalBufferPrivate() Failed";
        }

        // (Optional) A short sleep to avoid hammering the device, if needed.
        //std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
#ifdef DBG_MEASURE_TIME
    if (totalV4L2Captured > 0) {
        double averageCaptureTime =
            static_cast<double>(totalCaptureTime) / totalV4L2Captured;
        LOG(INFO) << __func__
                  << ": Average capture time: " << averageCaptureTime << " ms";
    }
#endif //DBG_MEASURE_TIME
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
#ifdef DBG_MEASURE_TIME
    long long totalProcessTime = 0;
    int totalProcessedFrame = 0;
#endif //DBG_MEASURE_TIME

    while (!stopThreadsFlag.load(std::memory_order_acquire)) {
        Tofi_v4l2_buffer process_frame;
        if (!m_capture_to_process_Q.pop(process_frame)) {
            if (stopThreadsFlag.load(std::memory_order_acquire))
                break;
            LOG(WARNING) << "processThread: No new frames, "
                            "m_captureToProcessQueue Size: "
                         << m_capture_to_process_Q.size();
            std::this_thread::sleep_for(
                std::chrono::milliseconds(BufferProcessor::getTimeoutDelay()));
            continue;
        }
        std::shared_ptr<uint16_t> tofi_compute_io_buff;
        if (!m_tofi_io_Buffer_Q.pop(tofi_compute_io_buff)) {
            if (stopThreadsFlag.load(std::memory_order_acquire))
                break;
            LOG(WARNING)
                << "processThread: No ToFi buffers, m_tofi_io_Buffer_Q Size: "
                << m_tofi_io_Buffer_Q.size();
            std::this_thread::sleep_for(
                std::chrono::milliseconds(BufferProcessor::getTimeoutDelay()));
            m_v4l2_input_buffer_Q.push(process_frame.data);
            continue;
        }

        uint16_t *tempDepthFrame = m_tofiComputeContext->p_depth_frame;
        uint16_t *tempAbFrame = m_tofiComputeContext->p_ab_frame;
        float *tempConfFrame = m_tofiComputeContext->p_conf_frame;

        if (tofi_compute_io_buff) {

            // Buffer layout depends on bit configuration via m_tofiBufferSize
            // m_tofiBufferSize = depthSize + abSize + confSize (in uint16_t units)
            // Only allocate/point to components that are actually configured

            const int numPixels = m_outputFrameWidth * m_outputFrameHeight;

            // Always have depth frame (always allocated)
            m_tofiComputeContext->p_depth_frame = tofi_compute_io_buff.get();

            // Calculate what's actually allocated after depth
            uint32_t allocatedAfterDepth =
                m_tofiBufferSize - static_cast<uint32_t>(numPixels);

            // Set AB pointer only if AB is allocated
            if (allocatedAfterDepth > 0) {
                m_tofiComputeContext->p_ab_frame =
                    tofi_compute_io_buff.get() + numPixels;

                // Calculate remaining space after AB (for confidence)
                // AB can be either numPixels or numPixels/2 depending on 8-bit vs 16-bit
                uint32_t abSize = std::min(allocatedAfterDepth,
                                           static_cast<uint32_t>(numPixels));
                uint32_t allocatedAfterAB = allocatedAfterDepth - abSize;

                // Set confidence pointer only if confidence is allocated
                if (allocatedAfterAB > 0) {
                    m_tofiComputeContext->p_conf_frame =
                        reinterpret_cast<float *>(tofi_compute_io_buff.get() +
                                                  numPixels + abSize);
                } else {
                    // No confidence allocated - point to a safe dummy location or keep original
                    m_tofiComputeContext->p_conf_frame = tempConfFrame;
                }
            } else {
                // No AB or confidence allocated - keep original pointers
                m_tofiComputeContext->p_ab_frame = tempAbFrame;
                m_tofiComputeContext->p_conf_frame = tempConfFrame;
            }
#ifdef DUAL
            // For dual pulsatrix mode 1 and 0, data comes deinterleaved from ISP
            // Copy only the frames that are actually allocated based on m_tofiBufferSize
            // Buffer layout: [depth: numPixels uint16_t | AB: varies | conf: varies]
            if (m_currentModeNumber == 0 || m_currentModeNumber == 1) {

                // Always copy depth frame
                memcpy(m_tofiComputeContext->p_depth_frame,
                       process_frame.data.get(), numPixels * 2);

                // Calculate actual sizes based on m_tofiBufferSize
                // m_tofiBufferSize is in uint16_t units: depth + AB + conf
                uint32_t remainingSize =
                    m_tofiBufferSize -
                    static_cast<uint32_t>(numPixels); // After depth

                // Copy AB frame if allocated (remainingSize > 0)
                if (remainingSize > 0) {
                    uint32_t abCopySize =
                        std::min(remainingSize,
                                 static_cast<uint32_t>(
                                     numPixels)); // AB is at most numPixels
                    memcpy(m_tofiComputeContext->p_ab_frame,
                           process_frame.data.get() + numPixels * 2,
                           abCopySize * 2);
                    remainingSize -= abCopySize;
                }

                // Copy confidence frame if allocated (remainingSize > 0 after AB)
                // Confidence is numPixels*2 uint16_t (same as numPixels float)
                if (remainingSize > 0) {
                    uint32_t confCopySize =
                        std::min(remainingSize,
                                 static_cast<uint32_t>(numPixels * 2)) *
                        2; // In bytes
                    memcpy(m_tofiComputeContext->p_conf_frame,
                           process_frame.data.get() + numPixels * 4,
                           confCopySize);
                }
            } else {

                // If only depth (no deinterleaving needed), just copy
                // m_tofiBufferSize tells us what's allocated: depth + AB + conf sizes
                bool needsTofiCompute = true;
                uint32_t allocatedAfterDepth =
                    m_tofiBufferSize - static_cast<uint32_t>(numPixels);

                // Case 1: Only depth (0 AB, 0 Conf) - direct copy
                if (allocatedAfterDepth == 0) {
                    memcpy(m_tofiComputeContext->p_depth_frame,
                           process_frame.data.get(), numPixels * 2);
                    needsTofiCompute = false;
                }

                // For other combinations, use TofiCompute
                if (needsTofiCompute) {

                    uint32_t ret = TofiCompute(
                        reinterpret_cast<uint16_t *>(process_frame.data.get()),
                        m_tofiComputeContext, NULL);
                    if (ret != ADI_TOFI_SUCCESS) {
                        LOG(ERROR)
                            << "processThread: TofiCompute failed with code: "
                            << ret;
                        m_tofi_io_Buffer_Q.push(tofi_compute_io_buff);
                        m_v4l2_input_buffer_Q.push(process_frame.data);
                        m_tofiComputeContext->p_depth_frame = tempDepthFrame;
                        m_tofiComputeContext->p_ab_frame = tempAbFrame;
                        m_tofiComputeContext->p_conf_frame = tempConfFrame;
                        continue;
                    }
                }
            }
#else
            uint32_t ret = TofiCompute(
                reinterpret_cast<uint16_t *>(process_frame.data.get()),
                m_tofiComputeContext, NULL);
            if (ret != ADI_TOFI_SUCCESS) {
                LOG(ERROR) << "processThread: TofiCompute failed with code: "
                           << ret;
                m_tofi_io_Buffer_Q.push(tofi_compute_io_buff);
                m_v4l2_input_buffer_Q.push(process_frame.data);
                m_tofiComputeContext->p_depth_frame = tempDepthFrame;
                m_tofiComputeContext->p_ab_frame = tempAbFrame;
                m_tofiComputeContext->p_conf_frame = tempConfFrame;
                continue;
            }
#endif
            m_tofiComputeContext->p_depth_frame = tempDepthFrame;
            m_tofiComputeContext->p_ab_frame = tempAbFrame;
            m_tofiComputeContext->p_conf_frame = tempConfFrame;
        }

        // Only attempt to write if recording is still active and stream is open
        if (m_state == ST_RECORD && m_stream_file_out.is_open()) {
#ifdef HAS_RGB_CAMERA
            // Concatenate RGB data into ToF buffer if enabled
            if (m_rgbCaptureEnabled) {
                aditof::RGBFrame rgbFrame;
                if (m_rgb_frame_Q.pop(rgbFrame,
                                      std::chrono::milliseconds(50))) {
                    if (rgbFrame.isValid()) {
                        // Calculate buffer layout
                        const uint32_t rgbSize = rgbFrame.data.size();
                        const uint32_t totalSize = m_tofiBufferSize * sizeof(uint16_t) + rgbSize;

                        // Allocate combined buffer
                        std::vector<uint8_t> combinedBuffer(totalSize);

                        // Copy Depth + AB
                        memcpy(combinedBuffer.data(),
                               tofi_compute_io_buff.get(), m_tofiBufferSize * sizeof(uint16_t));

                        // Insert RGB after AB
                        memcpy(combinedBuffer.data() + m_tofiBufferSize * sizeof(uint16_t),
                               rgbFrame.data.data(), rgbSize);

                        // Write combined buffer once
                        aditof::Status writeStatus =
                            writeFrame(combinedBuffer.data(), totalSize, true);
                        if (writeStatus != aditof::Status::OK) {
                            LOG(WARNING) << "Failed to write combined frame "
                                            "during recording";
                        }
                    } else {
                        // RGB frame invalid, write ToF only
                        aditof::Status writeStatus = writeFrame(
                            (uint8_t *)tofi_compute_io_buff.get(),
                            m_tofiBufferSize * sizeof(uint16_t), true);
                        if (writeStatus != aditof::Status::OK) {
                            LOG(WARNING)
                                << "Failed to write ToF frame during recording";
                        }
                    }
                } else {
                    // No RGB frame available, write ToF only
                    aditof::Status writeStatus =
                        writeFrame((uint8_t *)tofi_compute_io_buff.get(),
                                   m_tofiBufferSize * sizeof(uint16_t), true);
                    if (writeStatus != aditof::Status::OK) {
                        LOG(WARNING)
                            << "Failed to write ToF frame during recording";
                    }
                }
            } else {
                // RGB not enabled, write ToF only
                aditof::Status writeStatus =
                    writeFrame((uint8_t *)tofi_compute_io_buff.get(),
                               m_tofiBufferSize * sizeof(uint16_t), true);
                if (writeStatus != aditof::Status::OK) {
                    LOG(WARNING)
                        << "Failed to write ToF frame during recording";
                }
            }
#else
            // No RGB support, write ToF only
            aditof::Status writeStatus =
                writeFrame((uint8_t *)tofi_compute_io_buff.get(),
                           m_tofiBufferSize * sizeof(uint16_t), true);
            if (writeStatus != aditof::Status::OK) {
                LOG(WARNING)
                    << "Failed to write processed frame during recording";
            }
#endif // HAS_RGB_CAMERA
            m_frames_written++;
        }

        process_frame.tofiBuffer = tofi_compute_io_buff;
        process_frame.size = m_tofiBufferSize;

        if (!m_process_done_Q.push(std::move(process_frame))) {
            LOG(WARNING) << "processThread: Push timeout to "
                            "m_process_done_Q, ProcessedQueueSize: "
                         << m_process_done_Q.size();
            m_tofi_io_Buffer_Q.push(tofi_compute_io_buff);
            m_v4l2_input_buffer_Q.push(process_frame.data);
            continue;
        }
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
    Tofi_v4l2_buffer tof_processed_frame;

    // Loop for MAX_RETRIES attempts. 'attempt' counts from 0 to MAX_RETRIES - 1.
    for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
        if (m_process_done_Q.pop(tof_processed_frame)) {
            if (buffer && tof_processed_frame.tofiBuffer &&
                tof_processed_frame.size > 0) {

                memcpy(buffer, tof_processed_frame.tofiBuffer.get(),
                       tof_processed_frame.size * sizeof(uint16_t));

                // Return buffers to their respective pools
                m_tofi_io_Buffer_Q.push(tof_processed_frame.tofiBuffer);
                m_v4l2_input_buffer_Q.push(tof_processed_frame.data);

                return aditof::Status::OK; // Success, exit function
            } else {
                // Pop succeeded, but the frame data itself was invalid.
                LOG(ERROR) << "processBuffer: Pop succeeded but frame data is "
                              "invalid (buffer/tofiBuffer/size). "
                           << "Returning error immediately.\n";
                return aditof::Status::GENERIC_ERROR;
            }
        } else {
            if (attempt < MAX_RETRIES - 1) {
                // If it's not the last attempt, wait and then the loop will try again.
                LOG(WARNING)
                    << "processBuffer: Pop failed on attempt #" << (attempt + 1)
                    << ". Retrying in " << RETRY_DELAY.count() << "ms...\n";
                std::this_thread::sleep_for(RETRY_DELAY);
            } else {
                // This was the last attempt (MAX_RETRIES - 1 index) and it failed.
                LOG(ERROR) << "processBuffer: Failed to pop frame after "
                           << MAX_RETRIES << " attempts. "
                           << "m_process_done_Q size: "
                           << m_process_done_Q.size() << "\n";
                return aditof::Status::
                    GENERIC_ERROR; // Indicate final failure to the caller
            }
        }
    }

    // This line should technically not be reached if MAX_RETRIES > 0,
    // as the loop will either return OK or GENERIC_ERROR.
    // Included as a safeguard.
    return aditof::Status::GENERIC_ERROR;
}

#ifdef HAS_RGB_CAMERA
/**
 * @brief Extended processBuffer with RGB frame retrieval
 *
 * This version allows retrieving both depth and RGB frames in a synchronized manner.
 * It first gets the depth frame from the processing queue, then attempts to get
 * the latest RGB frame from the RGB capture thread's queue.
 *
 * @param depthBuffer Buffer to receive depth + AB + confidence data
 * @param rgbFrame Pointer to RGBFrame to receive RGB data (can be nullptr for depth-only)
 * @return Status::OK on success, error otherwise
 */
aditof::Status BufferProcessor::processBuffer(uint16_t *depthBuffer,
                                              aditof::RGBFrame *rgbFrame) {
    Tofi_v4l2_buffer tof_processed_frame;
    const std::chrono::milliseconds m_retryDelay(10);

    // First, get the depth frame (same as original processBuffer)
    for (int attempt = 0; attempt < m_maxTries; ++attempt) {
        if (m_process_done_Q.pop(tof_processed_frame)) {
            // Validate and copy depth data
            if (depthBuffer && tof_processed_frame.tofiBuffer &&
                tof_processed_frame.size > 0) {
                memcpy(depthBuffer, tof_processed_frame.tofiBuffer.get(),
                       tof_processed_frame.size);
            } else {
                LOG(ERROR) << "processBuffer: Invalid depth buffer data";
                return aditof::Status::GENERIC_ERROR;
            }

            // If RGB is requested and available in the buffer, use it
            if (rgbFrame != nullptr) {
                if (tof_processed_frame.hasRGB &&
                    tof_processed_frame.rgbFrame.isValid()) {
                    *rgbFrame = std::move(tof_processed_frame.rgbFrame);
                } else {
                    // Try to get latest RGB frame from the RGB queue
                    aditof::RGBFrame latestRGBFrame;
                    if (m_rgb_frame_Q.pop(latestRGBFrame,
                                          std::chrono::milliseconds(50))) {
                        *rgbFrame = std::move(latestRGBFrame);
                    } else {
                        LOG(WARNING)
                            << "processBuffer: RGB requested but not available";
                        // Return empty frame rather than failing completely
                        *rgbFrame = aditof::RGBFrame();
                    }
                }
            }

            // Return buffers to pools
            m_tofi_io_Buffer_Q.push(tof_processed_frame.tofiBuffer);
            m_v4l2_input_buffer_Q.push(tof_processed_frame.data);

            return aditof::Status::OK;
        } else {
            if (attempt < m_maxTries - 1) {
                LOG(INFO) << "processBuffer: Pop failed on attempt #"
                          << (attempt + 1) << ". Retrying...";
                std::this_thread::sleep_for(m_retryDelay);
            } else {
                LOG(WARNING) << "processBuffer: Failed after " << m_maxTries
                             << " attempts";
                return aditof::Status::GENERIC_ERROR;
            }
        }
    }

    return aditof::Status::GENERIC_ERROR;
}
#endif // HAS_RGB_CAMERA

aditof::Status BufferProcessor::waitForBufferPrivate(struct VideoDev *dev) {
    fd_set fds;
    struct timeval tv;
    int r;

    if (dev == nullptr)
        dev = m_inputVideoDev;

    FD_ZERO(&fds);
    FD_SET(dev->fd, &fds);

    tv.tv_sec = stopThreadsFlag.load(std::memory_order_acquire)
                    ? 0
                    : SELECT_TIMEOUT_SEC;
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

TofiConfig *BufferProcessor::getTofiCongfig() const { return m_tofiConfig; }

aditof::Status BufferProcessor::getDepthComputeVersion(uint8_t &enabled) const {
    enabled = depthComputeOpenSourceEnabled;
    return aditof::Status::OK;
}

#ifdef HAS_RGB_CAMERA
aditof::Status BufferProcessor::setRGBSensor(aditof::RGBSensor *sensor) {
    if (sensor == nullptr) {
        LOG(ERROR) << "setRGBSensor: RGB sensor is nullptr";
        return aditof::Status::INVALID_ARGUMENT;
    }

    m_rgbSensor = sensor;
    LOG(INFO) << "setRGBSensor: RGB sensor registered with BufferProcessor";
    return aditof::Status::OK;
}

aditof::Status BufferProcessor::enableRGBCapture(bool enable) {
    if (enable && m_rgbSensor == nullptr) {
        LOG(ERROR) << "enableRGBCapture: Cannot enable - no RGB sensor set";
        return aditof::Status::UNAVAILABLE;
    }

    m_rgbCaptureEnabled = enable;
    m_totalRGBCaptured = 0;
    m_totalRGBFailures = 0;

    LOG(INFO) << "enableRGBCapture: RGB capture "
              << (enable ? "enabled" : "disabled");
    return aditof::Status::OK;
}

aditof::Status BufferProcessor::getLatestRGBFrame(aditof::RGBFrame &frame) {
    if (!m_rgbCaptureEnabled) {
        return aditof::Status::UNAVAILABLE;
    }

    // Try to get frame from queue (non-blocking with short timeout)
    if (m_rgb_frame_Q.pop(frame, std::chrono::milliseconds(100))) {
        return aditof::Status::OK;
    }

    return aditof::Status::BUSY;
}
#endif // HAS_RGB_CAMERA

void BufferProcessor::startThreads() {
    stopThreadsFlag.store(false, std::memory_order_release);
    streamRunning = true;

    LOG(INFO) << __func__ << ": Starting Threads..";
    m_captureThread = std::thread(&BufferProcessor::captureFrameThread, this);
    m_processingThread = std::thread(&BufferProcessor::processThread, this);
    sched_param param;
    param.sched_priority = THREAD_PRIORITY;
    pthread_setschedparam(m_processingThread.native_handle(), SCHED_FIFO,
                          &param);

#ifdef HAS_RGB_CAMERA
    m_captureRGBThread =
        std::thread(&BufferProcessor::captureRGBFrameThread, this);
#endif
}

void BufferProcessor::stopThreads() {
    // Signal threads to stop
    stopThreadsFlag.store(true, std::memory_order_release);
    streamRunning = false;

    stopRecording();

    // Wait for queue drainage with timeout to prevent indefinite blocking
    auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (((m_capture_to_process_Q.size() > 0) ||
            (m_process_done_Q.size() > 0)) &&
           std::chrono::steady_clock::now() < timeout) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (std::chrono::steady_clock::now() >= timeout) {
        LOG(WARNING) << "stopThreads: Queue drainage timed out. Forcing thread "
                        "shutdown.";
    }

    // Join threads FIRST - wait for them to fully exit before touching any buffers
    // This prevents race conditions where threads access buffers during cleanup
    if (m_captureThread.joinable()) {
        m_captureThread.join();
    }
    if (m_processingThread.joinable()) {
        m_processingThread.join();
    }
#ifdef HAS_RGB_CAMERA
    if (m_captureRGBThread.joinable()) {
        m_captureRGBThread.join();
    }
#endif

    // Reset thread objects
    m_captureThread = std::thread();
    m_processingThread = std::thread();
#ifdef HAS_RGB_CAMERA
    m_captureRGBThread = std::thread();
#endif

    // Now that threads are stopped, flush remaining frames from intermediate queues
    // Return buffers to their pools for potential reuse
    {
        Tofi_v4l2_buffer frame;
        while (m_capture_to_process_Q.pop(frame)) {
            if (frame.data)
                m_v4l2_input_buffer_Q.push(frame.data);
            if (frame.tofiBuffer)
                m_tofi_io_Buffer_Q.push(frame.tofiBuffer);
        }
    }
    size_t rawFreed = 0, tofiFreed = 0;
#ifdef HAS_RGB_CAMERA
    size_t rgbFreed = 0;
#endif
    std::shared_ptr<uint8_t> inputBuf;
    while (m_v4l2_input_buffer_Q.pop(inputBuf)) {
        ++rawFreed;
    }

    {
        Tofi_v4l2_buffer frame;
        while (m_process_done_Q.pop(frame)) {
            if (frame.data)
                m_v4l2_input_buffer_Q.push(frame.data);
            if (frame.tofiBuffer)
                m_tofi_io_Buffer_Q.push(frame.tofiBuffer);
        }
    }

    LOG(INFO) << __func__ << ": Threads stopped successfully. Queue sizes - "
              << "v4l2_input: " << m_v4l2_input_buffer_Q.size()
              << ", capture_to_process: " << m_capture_to_process_Q.size()
              << ", tofi_io: " << m_tofi_io_Buffer_Q.size()
              << ", process_done: " << m_process_done_Q.size();
#ifdef HAS_RGB_CAMERA
    aditof::RGBFrame rgbFrame;
    while (m_rgb_frame_Q.pop(rgbFrame)) {
        ++rgbFreed;
    }
#endif

    LOG(INFO) << __func__
              << ": Threads Stopped. Raw buffers freed: " << rawFreed
              << ", ToFi buffers freed: " << tofiFreed
#ifdef HAS_RGB_CAMERA
              << ", RGB frames freed: " << rgbFreed
#endif
        ;
}

#pragma region Stream_Recording_and_Playback

#include "aditof/utils.h"

aditof::Status BufferProcessor::startRecording(std::string &fileName,
                                               uint8_t *parameters,
                                               uint32_t paramSize) {

    using namespace aditof;

    m_state = ST_STOP;
    m_folder_path = m_folder_path_folder;
    if (!aditof::Utils::folderExists(m_folder_path)) {
        if (!aditof::Utils::createFolder(m_folder_path)) {
            LOG(ERROR) << "Failed to create folder for recordings: "
                       << m_folder_path;
            return aditof::Status::GENERIC_ERROR;
        }
    }

    fileName = m_folder_path + "/" + aditof::Utils::generateFileName();

    if (m_stream_file_out.is_open()) {
        m_stream_file_out.close();
    }

    m_frame_count = 0;
    m_frames_written = 0;

    m_stream_file_out = std::ofstream(fileName, std::ios::binary);

    m_state = ST_RECORD;

    writeFrame(parameters, paramSize, true);

    return aditof::Status::OK;
}

aditof::Status BufferProcessor::stopRecording() {

    using namespace aditof;

    Status status = Status::GENERIC_ERROR;

    if (m_stream_file_out.is_open()) {
        // Write the number of frames recorded at the end of the file

        // Seek back to the beginning
        m_stream_file_out.seekp(
            8,
            std::ios::
                beg); // Skip over the number of bytes to read and the tag of 0xFFFF_FFFF
        // Overwrite the placeholder
        LOG(INFO) << "count : " << m_frame_count;
        m_frame_count--; // Take into account the header frame
        m_stream_file_out.write(reinterpret_cast<const char *>(&m_frame_count),
                                sizeof(m_frame_count));

        m_stream_file_out.close();
        LOG(INFO) << "Recording stopped. Total frames saved: " << m_frame_count;

        status = aditof::Status::OK;
    }

    return status;
}

aditof::Status BufferProcessor::writeFrame(uint8_t *buffer, uint32_t bufferSize,
                                           bool incrementCount) {
    if (m_state != ST_RECORD) {
        return aditof::Status::GENERIC_ERROR;
    }

    try {
        if (m_stream_file_out.is_open()) {
            // Write size of buffer
            uint32_t x = 0xFFFFFFFF;
            m_stream_file_out.write((char *)&x, sizeof(x));

            m_stream_file_out.write((char *)&bufferSize, sizeof(bufferSize));
            // Write buffer data
            m_stream_file_out.write((char *)(buffer), bufferSize);

            if (incrementCount) {
                m_frame_count++;
            }

            return aditof::Status::OK;
        }
    } catch (const std::ofstream::failure &e) {
        LOG(ERROR) << "File I/O exception caught: " << e.what();
        m_stream_file_out.close();
        m_state = ST_STOP;
    }
    return aditof::Status::GENERIC_ERROR;
}

aditof::Status BufferProcessor::automaticStop() {
    if (m_state == ST_PLAYBACK) {
        return aditof::Status::GENERIC_ERROR;
    } else if (m_state == ST_RECORD) {
        stopRecording();
    }

    return aditof::Status::OK;
}
