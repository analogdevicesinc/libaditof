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

/**
 * @file buffer_processor.cpp
 * @brief Multi-threaded buffer processor for Time-of-Flight frame capture and processing.
 *
 * Implements a two-thread pipeline architecture:
 * - Capture thread: Dequeues frames from V4L2 device (DQBUF), copies to processing queue
 * - Process thread: Runs ToFi depth computation, handles ISP-computed and raw bypass modes
 *
 * Frame Modes:
 * - MP (modes 0-1, 1024×1024): ISP hardware-computed depth, just copy data
 * - QMP (modes 2-6, 512×512): Various ISP configurations, calls TofiCompute() for deinterleaving
 *
 * Thread-safe queue management ensures proper buffer lifecycle and prevents memory leaks.
 */

#include "platform/platform_impl.h"
#include <aditof/log.h>
#include <aditof/utils.h>
#include <algorithm>
#include <arm_neon.h>
#include <cmath>
#include <cstdio>
#include <dlfcn.h>
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
#include <vector>

#include "buffer_processor.h"
#include <aditof/frame_handler.h>

// Increased from 3 to 8 to handle slow first-frame TofiCompute in lens scatter mode
// (can take 3+ seconds for initialization; 8 buffers provides ~800ms cushion at 10 FPS)
const size_t MAX_QUEUE_SIZE = 8;

// Named constants for configuration
namespace {
constexpr int MAX_RETRIES = 3;
constexpr std::chrono::milliseconds RETRY_DELAY{10};
constexpr int SELECT_TIMEOUT_SEC = 20;
constexpr int THREAD_PRIORITY = 20;
} // namespace

#define CLEAR(x) memset(&(x), 0, sizeof(x))

/**
 * @brief Wrapper for ioctl that retries on EINTR.
 *
 * Repeatedly calls ioctl until it succeeds or fails with an error other than EINTR.
 * This handles the case where ioctl is interrupted by a signal.
 *
 * @param[in] fh File descriptor
 * @param[in] request ioctl request code
 * @param[in,out] arg Pointer to ioctl argument structure
 *
 * @return ioctl result: 0 on success, -1 on error (with errno set)
 */
static int xioctl(int fh, unsigned int request, void *arg) {
    int r;

    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno && errno != 0);

    return r;
}

/**
 * @brief Constructs a BufferProcessor object.
 *
 * Initializes all queues with MAX_QUEUE_SIZE capacity, creates video device structures,
 * sets initial state flags, and prepares for multi-threaded frame capture and processing.
 * All buffer pools are empty at construction; they are allocated later via setVideoProperties.
 */
BufferProcessor::BufferProcessor()
    : m_v4l2_input_buffer_Q(MAX_QUEUE_SIZE),
      m_capture_to_process_Q(MAX_QUEUE_SIZE),
      m_tofi_io_Buffer_Q(MAX_QUEUE_SIZE), m_process_done_Q(MAX_QUEUE_SIZE) {

    m_outputFrameWidth = 0;
    m_outputFrameHeight = 0;
    m_abFrameSize = 0;
    m_processorPropSet = false;
    m_vidPropSet = false;
    m_bitsInAB = 0;
    m_bitsInConf = 0;
    m_bitsInDepth = 16;
    m_isRawBypassMode = false;
    m_ispEnabled = false;
    m_needsRotation = false;
    stopThreadsFlag = true;
    stopThreadsFlag.store(true, std::memory_order_release);
    streamRunning = false;
    m_tofiConfig = nullptr;
    m_tofiComputeContext = nullptr;
    m_inputVideoDev = nullptr;
    m_v4l2_input_buffer_Q.set_max_size(MAX_QUEUE_SIZE);
    m_capture_to_process_Q.set_max_size(MAX_QUEUE_SIZE);
    m_tofi_io_Buffer_Q.set_max_size(MAX_QUEUE_SIZE);
    m_process_done_Q.set_max_size(MAX_QUEUE_SIZE);
    LOG(INFO) << "BufferProcessor initialized";
}

/**
 * @brief Destructor for BufferProcessor.
 *
 * Stops worker threads and frees ToFi compute resources (context and config).
 * Thread shutdown drains all queues and ensures ToFi context pointers are restored
 * before cleanup, preventing use-after-free errors.
 */
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

    // STEP 2b: Free the alternate DMS mode's compute context, if registered
    if (m_altConfig.tofiComputeContext != nullptr) {
        FreeTofiCompute(m_altConfig.tofiComputeContext);
    }
    if (m_altConfig.tofiConfig != nullptr) {
        FreeTofiConfig(m_altConfig.tofiConfig);
    }

    // STEP 3: Input device cleanup handled by caller (adsd3500_sensor)
}

/**
 * @brief Initializes the buffer processor (no-op).
 *
 * Reserved for future initialization requirements.
 *
 * @return Status::OK
 */
aditof::Status BufferProcessor::open() { return aditof::Status::OK; }

/**
 * @brief Sets the input video device for frame capture.
 *
 * Assigns the V4L2 input device that will be used by captureFrameThread for
 * acquiring raw frames from the sensor.
 *
 * @param[in] inputVideoDev Pointer to the input VideoDev structure
 *
 * @return Status::OK on success
 */
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
    int modeNumber, uint8_t bitsInAB, uint8_t bitsInConf, uint8_t bitsInDepth,
    bool isRawBypass) {

    // Clear all queues to prevent memory leaks if setVideoProperties is called multiple times
    if (!stopThreadsFlag.load(std::memory_order_acquire)) {
        stopThreads();
    }

    // Clear the buffer pool before reallocating the buffer
    {
        std::shared_ptr<uint8_t> clr_buffer;
        while (m_v4l2_input_buffer_Q.pop(clr_buffer)) {
            // Buffer deallocation via shared_ptr
        }
    }

    {
        std::shared_ptr<uint16_t> clr_buffer;
        while (m_tofi_io_Buffer_Q.pop(clr_buffer)) {
            // Buffer deallocation via shared_ptr
        }
    }

    using namespace aditof;
    Status status = Status::OK;
    m_vidPropSet = true;

    m_currentModeNumber = modeNumber;
    m_bitsInAB = bitsInAB;
    m_bitsInConf = bitsInConf;
    m_bitsInDepth = (bitsInDepth > 0) ? bitsInDepth : 16u;
    m_isRawBypassMode = isRawBypass;

    m_outputFrameWidth = frameWidth;
    m_outputFrameHeight = frameHeight;

    // Store driver dimensions for raw bypass buffer calculation
    m_driverFrameWidth = WidthInBytes;
    m_driverFrameHeight = HeightInBytes;

    m_rawFrameBufferSize =
        aditof::platform::Platform::getInstance().calculateBufferSize(
            WidthInBytes, HeightInBytes);

    {
        LOG(INFO) << __func__ << ": Allocating " << MAX_QUEUE_SIZE
                  << " raw frame buffers, each of size " << m_rawFrameBufferSize
                  << " bytes (total: "
                  << (MAX_QUEUE_SIZE * m_rawFrameBufferSize) / (1024.0 * 1024.0)
                  << " MB)";
        for (int i = 0; i < (int)MAX_QUEUE_SIZE; ++i) {
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

    LOG(INFO) << __func__ << ": Allocating " << MAX_QUEUE_SIZE
              << " ToFi buffers, each of size "
              << m_tofiBufferSize * sizeof(uint16_t) << " bytes (total: "
              << (MAX_QUEUE_SIZE * m_tofiBufferSize * sizeof(uint16_t)) /
                     (1024.0 * 1024.0)
              << " MB)";
    for (int i = 0; i < (int)MAX_QUEUE_SIZE; ++i) {
        auto buffer = std::shared_ptr<uint16_t>(
            new uint16_t[m_tofiBufferSize], std::default_delete<uint16_t[]>());
        if (!buffer) {
            LOG(ERROR) << "setVideoProperties: Failed to allocate ToFi buffer!";
            return aditof::Status::GENERIC_ERROR;
        }
        m_tofi_io_Buffer_Q.push(buffer);
    }

    // Allocate the rotation ping-pong output buffer (avoids per-frame memcpy)
    m_rotationOutputBuffer = std::shared_ptr<uint16_t>(
        new uint16_t[m_tofiBufferSize], std::default_delete<uint16_t[]>());
    if (!m_rotationOutputBuffer) {
        LOG(ERROR)
            << "setVideoProperties: Failed to allocate rotation output buffer!";
        return aditof::Status::GENERIC_ERROR;
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

    // For raw bypass mode, use driver dimensions (full raw buffer size)
    // For normal ToF mode, use output dimensions (processed frame size)
    uint32_t width =
        m_isRawBypassMode ? m_driverFrameWidth : m_outputFrameWidth;
    uint32_t height =
        m_isRawBypassMode ? m_driverFrameHeight : m_outputFrameHeight;

    uint32_t depthSize = width * height;
    uint32_t abSize = 0;
    uint32_t confSize = 0;

    // check whether the bit are not configured 0
    // for 0 bit configuration, it'll not contribute in framesize
    if ((bitsInAB != 0) && (bitsInConf == 0)) {
        // Conf bit is set to 0
        abSize = width * height;

    } else if ((bitsInAB == 0) && (bitsInConf != 0)) {
        // AB bit is set to 0
        confSize = width * height * 2;
    } else if ((bitsInAB == 0) && (bitsInConf == 0)) {
        // No need to add size
    } else {

        abSize = width * height;
        confSize = width * height * 2;
    }

    m_tofiBufferSize = depthSize + abSize + confSize;
    m_abFrameSize = abSize;

    // For raw bypass, ensure ToFi buffer can hold the complete V4L2 buffer including NVIDIA alignment
    if (m_isRawBypassMode) {
        size_t rawBufferSizeInUint16 =
            (m_rawFrameBufferSize + sizeof(uint16_t) - 1) / sizeof(uint16_t);
        if (rawBufferSizeInUint16 > m_tofiBufferSize) {
            m_tofiBufferSize = rawBufferSizeInUint16;
        }
    }
}

void BufferProcessor::rotateEntireToFiBuffer(const uint16_t *src, uint16_t *dst,
                                             uint32_t width, uint32_t height,
                                             uint32_t bufferSize) {
    // L1-block staging: T=64 tile sweep, all planes fused in one pass.
    // blkD 8KB + blkA 8KB + blkC 16KB = 32KB < L1D 64KB → scatter reads stay in L1.

    const uint32_t W = width;
    const uint32_t H = height;
    const uint32_t numPixels = W * H;
    const uint32_t confOffset =
        numPixels + m_abFrameSize; // actual layout: depth | [AB] | conf
    const bool hasAB = m_abFrameSize > 0;
    const bool hasConf = bufferSize > numPixels + m_abFrameSize;
    constexpr uint32_t T = 64;

    // Three explicit branch-free code paths so the hot inner loop contains no conditionals.
    // Each path fuses all active planes into one tile sweep (single pass over src/dst).
    // blkD 8KB + blkA 8KB + blkC 16KB = 32KB < L1D 64KB → scatter reads stay in L1.
    if (!hasAB && !hasConf) {
        // --- depth only ---
        uint16_t blkD[T][T];
        for (uint32_t tc = 0; tc < W; tc += T) {
            const uint32_t tW = std::min(T, W - tc);
            for (uint32_t tr = 0; tr < H; tr += T) {
                const uint32_t tH = std::min(T, H - tr);
                for (uint32_t r = 0; r < tH; r++)
                    memcpy(blkD[r], &src[(tr + r) * W + tc],
                           tW * sizeof(uint16_t));
                for (uint32_t c = 0; c < tW; c++) {
                    uint16_t *__restrict__ d =
                        &dst[(tc + c) * H + (H - tr - tH)];
                    for (int32_t r = (int32_t)tH - 1; r >= 0; r--)
                        *d++ = blkD[r][c];
                }
            }
        }
    } else if (hasAB && !hasConf) {
        // --- depth + AB (fused) ---
        const uint16_t *__restrict__ sa = src + numPixels;
        uint16_t *__restrict__ da = dst + numPixels;
        uint16_t blkD[T][T], blkA[T][T];
        for (uint32_t tc = 0; tc < W; tc += T) {
            const uint32_t tW = std::min(T, W - tc);
            for (uint32_t tr = 0; tr < H; tr += T) {
                const uint32_t tH = std::min(T, H - tr);
                for (uint32_t r = 0; r < tH; r++) {
                    memcpy(blkD[r], &src[(tr + r) * W + tc],
                           tW * sizeof(uint16_t));
                    memcpy(blkA[r], &sa[(tr + r) * W + tc],
                           tW * sizeof(uint16_t));
                }
                for (uint32_t c = 0; c < tW; c++) {
                    uint16_t *__restrict__ d =
                        &dst[(tc + c) * H + (H - tr - tH)];
                    uint16_t *__restrict__ d2 =
                        &da[(tc + c) * H + (H - tr - tH)];
                    for (int32_t r = (int32_t)tH - 1; r >= 0; r--) {
                        *d++ = blkD[r][c];
                        *d2++ = blkA[r][c];
                    }
                }
            }
        }
    } else {
        // --- depth + AB + conf (fused, uint32 conf) ---
        const uint16_t *__restrict__ sa = src + numPixels;
        uint16_t *__restrict__ da = dst + numPixels;
        const uint32_t *__restrict__ sc =
            reinterpret_cast<const uint32_t *>(src + confOffset);
        uint32_t *__restrict__ dc =
            reinterpret_cast<uint32_t *>(dst + confOffset);
        uint16_t blkD[T][T], blkA[T][T];
        uint32_t blkC[T][T];
        for (uint32_t tc = 0; tc < W; tc += T) {
            const uint32_t tW = std::min(T, W - tc);
            for (uint32_t tr = 0; tr < H; tr += T) {
                const uint32_t tH = std::min(T, H - tr);
                for (uint32_t r = 0; r < tH; r++) {
                    memcpy(blkD[r], &src[(tr + r) * W + tc],
                           tW * sizeof(uint16_t));
                    memcpy(blkA[r], &sa[(tr + r) * W + tc],
                           tW * sizeof(uint16_t));
                    memcpy(blkC[r], &sc[(tr + r) * W + tc],
                           tW * sizeof(uint32_t));
                }
                for (uint32_t c = 0; c < tW; c++) {
                    uint16_t *__restrict__ d =
                        &dst[(tc + c) * H + (H - tr - tH)];
                    uint16_t *__restrict__ d2 =
                        &da[(tc + c) * H + (H - tr - tH)];
                    uint32_t *__restrict__ d3 =
                        &dc[(tc + c) * H + (H - tr - tH)];
                    for (int32_t r = (int32_t)tH - 1; r >= 0; r--) {
                        *d++ = blkD[r][c];
                        *d2++ = blkA[r][c];
                        *d3++ = blkC[r][c];
                    }
                }
            }
        }
    }
}

/**
 * @brief Initializes ToFi compute library with INI and calibration data.
 *
 * Frees any existing ToFi config/context, then calls InitTofiConfig_isp and InitTofiCompute
 * to set up depth computation for the specified mode. For ISP-enabled operation, uses
 * XYZ dealias data from calibration. Returns error if initialization fails.
 *
 * @param[in] iniFile Pointer to INI file data buffer
 * @param[in] iniFileLength Length of INI file in bytes
 * @param[in] calData Pointer to calibration data buffer
 * @param[in] calDataLength Length of calibration data in bytes
 * @param[in] mode Frame mode number
 * @param[in] ispEnabled Whether ISP depth computation is enabled
 *
 * @return Status::OK on success, Status::GENERIC_ERROR if initialization fails
 */
aditof::Status BufferProcessor::setProcessorProperties(
    uint8_t *iniFile, uint16_t iniFileLength, uint8_t *calData,
    uint32_t calDataLength, uint16_t mode, bool ispEnabled) {

    m_ispEnabled = ispEnabled;
    m_modeFusionEnabled =
        parseIniBoolFlag(iniFile, iniFileLength, "modeFusionEnabled", false);
    m_isSRFrameFirst =
        parseIniBoolFlag(iniFile, iniFileLength, "IsSRFrameFirst", false);

    // Log libtofi_compute version once per mode change.
    {
        char ver[64] = {};
        GetVersion(ver);
        // Strip "VERSIONINFO :" prefix if present
        const char *prefix = "VERSIONINFO :";
        const char *verStr = ver;
        if (strncmp(ver, prefix, strlen(prefix)) == 0)
            verStr = ver + strlen(prefix);
        LOG(INFO) << __func__ << ": libtofi_compute version: " << verStr;
    }

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

    return createModeTofiContext(iniFile, iniFileLength, calData,
                                 calDataLength, mode, ispEnabled, m_tofiConfig,
                                 m_tofiComputeContext);
}

/**
 * @function BufferProcessor::parseIniBoolFlag
 *
 * Scans a raw "key=value\n"-style ini blob for a boolean flag. Used to pull
 * modeFusionEnabled/IsSRFrameFirst out of the same ini text already handed
 * to InitTofiConfig_isp(), instead of adding new parameters just for these.
 */
bool BufferProcessor::parseIniBoolFlag(uint8_t *iniFile, uint16_t iniFileLength,
                                      const char *key, bool defaultValue) {
    if (iniFile == nullptr || iniFileLength == 0) {
        return defaultValue;
    }

    const std::string blob(reinterpret_cast<char *>(iniFile), iniFileLength);
    const std::string needle = std::string(key) + "=";
    size_t pos = blob.find(needle);
    if (pos == std::string::npos) {
        return defaultValue;
    }

    pos += needle.size();
    if (pos >= blob.size()) {
        return defaultValue;
    }

    return blob[pos] != '0';
}

/**
 * @function BufferProcessor::createModeTofiContext
 *
 * Builds a standalone TofiConfig/TofiComputeContext pair for the given mode,
 * without touching any single-mode member state. Used by both
 * setProcessorProperties() (the primary active mode) and
 * setAlternateModeConfiguration() (the DMS alternate mode), so the same
 * proven initialization logic isn't duplicated.
 */
aditof::Status BufferProcessor::createModeTofiContext(
    uint8_t *iniFile, uint16_t iniFileLength, uint8_t *calData,
    uint32_t calDataLength, uint16_t mode, bool ispEnabled,
    TofiConfig *&outConfig, TofiComputeContext *&outContext) {

    outConfig = nullptr;
    outContext = nullptr;

    using GetIntrinsicsDataBuffer_t = int (*)(uint16_t);
    auto fnGetIntrinsics = reinterpret_cast<GetIntrinsicsDataBuffer_t>(
        dlsym(RTLD_DEFAULT, "GetIntrinsicsDataBuffer"));

    if (ispEnabled) {
        uint32_t status = ADI_TOFI_SUCCESS;

        // For ISP mode, calData is already parsed TofiXYZDealiasData (not raw CCB)
        if (calData == nullptr ||
            calDataLength < sizeof(TofiXYZDealiasData) * 10) {
            LOG(ERROR) << "Invalid XYZ dealias data size for ISP mode "
                       << mode << ": " << calDataLength << " (expected "
                       << sizeof(TofiXYZDealiasData) * 10 << ")";
            return aditof::Status::GENERIC_ERROR;
        }
        // Local scratch copy so the primary and alternate mode contexts
        // never clobber each other's dealias data.
        TofiXYZDealiasData localDealiasData[10];
        memcpy(localDealiasData, calData, sizeof(TofiXYZDealiasData) * 10);

        if (iniFile != nullptr) {
            ConfigFileData depth_ini = {iniFile, iniFileLength};
            int p0_mode = fnGetIntrinsics ? fnGetIntrinsics(mode)
                                          : static_cast<int>(mode);
            if (p0_mode == -1) {
                LOG(ERROR) << "Failed to get the camera Intrinsics for mode "
                           << mode;
                return aditof::Status::GENERIC_ERROR;
            }
            try {
                outConfig = InitTofiConfig_isp(
                    &depth_ini, p0_mode, &status, localDealiasData);
            } catch (...) {
                LOG(ERROR)
                    << "Failed to initialize the Config for mode " << mode
                    << ": Please make sure calibration file corresponds to "
                       "input data file";
                return aditof::Status::GENERIC_ERROR;
            }
        } else {
            ConfigFileData calDataStruct = {(uint8_t *)localDealiasData,
                                            sizeof(TofiXYZDealiasData) * 10};
            outConfig =
                InitTofiConfig(&calDataStruct, NULL, NULL, mode, &status);
        }

        if ((outConfig == NULL) || (outConfig->p_tofi_cal_config == NULL) ||
            (status != ADI_TOFI_SUCCESS)) {
            LOG(ERROR) << "InitTofiConfig failed for mode " << mode;
            if (outConfig != NULL) {
                FreeTofiConfig(outConfig);
                outConfig = nullptr;
            }
            return aditof::Status::GENERIC_ERROR;
        }

        outContext = InitTofiCompute(outConfig->p_tofi_cal_config, &status);
        if (outContext == NULL || status != ADI_TOFI_SUCCESS) {
            LOG(ERROR) << "InitTofiCompute failed for mode " << mode;
            FreeTofiConfig(outConfig);
            outConfig = nullptr;
            return aditof::Status::GENERIC_ERROR;
        }
        LOG(INFO) << "createModeTofiContext: mode " << mode
                  << " TofiConfig n_rows=" << outConfig->n_rows
                  << " n_cols=" << outConfig->n_cols
                  << " hdr_size=" << outConfig->hdr_size
                  << " phases=" << outConfig->phases
                  << " freqs=" << outConfig->freqs;
    } else {
        // ISP disabled - use standard depth compute initialization with full calibration
        uint32_t status = ADI_TOFI_SUCCESS;
        ConfigFileData calDataStruct = {calData, calDataLength};

        int ccb_mode =
            fnGetIntrinsics ? fnGetIntrinsics(mode) : static_cast<int>(mode);
        if (ccb_mode == -1) {
            LOG(ERROR) << "Mode " << mode
                       << " not found in CCB calibration data";
            return aditof::Status::GENERIC_ERROR;
        }

        if (iniFile != nullptr) {
            ConfigFileData depth_ini = {iniFile, iniFileLength};
            try {
                // Use CCB mode instead of requested mode to avoid crash
                outConfig = InitTofiConfig(&calDataStruct, NULL, &depth_ini,
                                          ccb_mode, &status);
            } catch (...) {
                LOG(ERROR)
                    << "Failed to initialize the Config for mode " << mode
                    << ": Please make sure calibration file corresponds to "
                       "input data file";
                return aditof::Status::GENERIC_ERROR;
            }
        } else {
            outConfig =
                InitTofiConfig(&calDataStruct, NULL, NULL, ccb_mode, &status);
        }

        if ((outConfig == NULL) || (outConfig->p_tofi_cal_config == NULL) ||
            (status != ADI_TOFI_SUCCESS)) {
            LOG(ERROR) << "InitTofiConfig failed for mode " << mode
                       << ", status=" << status;
            if (outConfig != NULL) {
                FreeTofiConfig(outConfig);
                outConfig = nullptr;
            }
            return aditof::Status::GENERIC_ERROR;
        }

        outContext = InitTofiCompute(outConfig->p_tofi_cal_config, &status);
        if (outContext == NULL || status != ADI_TOFI_SUCCESS) {
            LOG(ERROR) << "InitTofiCompute failed for mode " << mode
                       << ", status=" << status;
            FreeTofiConfig(outConfig);
            outConfig = nullptr;
            return aditof::Status::GENERIC_ERROR;
        }
    }

    return aditof::Status::OK;
}

/**
 * @function BufferProcessor::growSharedBufferPools
 *
 * Reallocates the shared raw/ToFi buffer pools so they can hold the larger
 * of the primary and alternate mode buffers. Pauses the capture/process
 * threads (if running) for the duration of the resize.
 */
aditof::Status BufferProcessor::growSharedBufferPools(uint32_t newRawSize,
                                                       uint32_t newTofiSize) {
    bool wasRunning = !stopThreadsFlag.load(std::memory_order_acquire);
    if (wasRunning) {
        stopThreads();
    }

    {
        std::shared_ptr<uint8_t> clr;
        while (m_v4l2_input_buffer_Q.pop(clr, std::chrono::milliseconds(0))) {
        }
    }
    {
        std::shared_ptr<uint16_t> clr;
        while (m_tofi_io_Buffer_Q.pop(clr, std::chrono::milliseconds(0))) {
        }
    }

    m_rawFrameBufferSize = std::max(m_rawFrameBufferSize, newRawSize);
    m_tofiBufferSize = std::max(m_tofiBufferSize, newTofiSize);

    for (int i = 0; i < (int)MAX_QUEUE_SIZE; ++i) {
        auto buffer =
            std::shared_ptr<uint8_t>(new uint8_t[m_rawFrameBufferSize],
                                     std::default_delete<uint8_t[]>());
        m_v4l2_input_buffer_Q.push(buffer);
    }
    for (int i = 0; i < (int)MAX_QUEUE_SIZE; ++i) {
        auto buffer = std::shared_ptr<uint16_t>(
            new uint16_t[m_tofiBufferSize], std::default_delete<uint16_t[]>());
        m_tofi_io_Buffer_Q.push(buffer);
    }
    m_rotationOutputBuffer = std::shared_ptr<uint16_t>(
        new uint16_t[m_tofiBufferSize], std::default_delete<uint16_t[]>());

    if (wasRunning) {
        startThreads();
    }

    LOG(INFO) << "growSharedBufferPools: raw=" << m_rawFrameBufferSize
              << " bytes, tofi=" << (m_tofiBufferSize * sizeof(uint16_t))
              << " bytes";

    return aditof::Status::OK;
}

/**
 * @function BufferProcessor::setAlternateModeConfiguration
 *
 * Registers the buffer layout and compute context for a second mode and
 * activates per-frame dispatch between it and the primary mode, following
 * the given repeat pattern (matching the Dynamic Mode Switching sequence
 * programmed on the ADSD3500).
 */
aditof::Status BufferProcessor::setAlternateModeConfiguration(
    uint8_t modeNumber, int frameWidth, int frameHeight, int widthInBytes,
    int heightInBytes, uint8_t bitsInAB, uint8_t bitsInConf,
    uint8_t bitsInDepth, bool isRawBypass, bool ispEnabled, uint8_t *iniFile,
    uint16_t iniFileLength, uint8_t *calData, uint32_t calDataLength,
    uint8_t repeatPrimary, uint8_t repeatAlternate) {

    if (repeatPrimary == 0 || repeatAlternate == 0) {
        LOG(ERROR) << "setAlternateModeConfiguration: repeat counts must be "
                      "greater than zero";
        return aditof::Status::INVALID_ARGUMENT;
    }

    AltModeConfig cfg;
    cfg.modeNumber = modeNumber;
    cfg.outputFrameWidth = static_cast<uint16_t>(frameWidth);
    cfg.outputFrameHeight = static_cast<uint16_t>(frameHeight);
    cfg.driverFrameWidth = static_cast<uint16_t>(widthInBytes);
    cfg.driverFrameHeight = static_cast<uint16_t>(heightInBytes);
    cfg.bitsInAB = bitsInAB;
    cfg.bitsInConf = bitsInConf;
    cfg.bitsInDepth = (bitsInDepth > 0) ? bitsInDepth : 16u;
    cfg.isRawBypass = isRawBypass;
    cfg.ispEnabled = ispEnabled;
    cfg.modeFusionEnabled =
        parseIniBoolFlag(iniFile, iniFileLength, "modeFusionEnabled", false);
    cfg.isSRFrameFirst =
        parseIniBoolFlag(iniFile, iniFileLength, "IsSRFrameFirst", false);

    cfg.rawFrameBufferSize =
        aditof::platform::Platform::getInstance().calculateBufferSize(
            widthInBytes, heightInBytes);

    // Mirrors calculateFrameSize(), applied to the alternate mode's own
    // dimensions/bit config instead of the primary member state.
    {
        uint32_t width =
            cfg.isRawBypass ? cfg.driverFrameWidth : cfg.outputFrameWidth;
        uint32_t height =
            cfg.isRawBypass ? cfg.driverFrameHeight : cfg.outputFrameHeight;

        uint32_t depthSize = width * height;
        uint32_t abSize = 0;
        uint32_t confSize = 0;

        if ((cfg.bitsInAB != 0) && (cfg.bitsInConf == 0)) {
            abSize = width * height;
        } else if ((cfg.bitsInAB == 0) && (cfg.bitsInConf != 0)) {
            confSize = width * height * 2;
        } else if ((cfg.bitsInAB == 0) && (cfg.bitsInConf == 0)) {
            // No AB/conf contribution
        } else {
            abSize = width * height;
            confSize = width * height * 2;
        }

        cfg.tofiBufferSize = depthSize + abSize + confSize;
        cfg.abFrameSize = abSize;

        if (cfg.isRawBypass) {
            size_t rawBufferSizeInUint16 =
                (cfg.rawFrameBufferSize + sizeof(uint16_t) - 1) /
                sizeof(uint16_t);
            if (rawBufferSizeInUint16 > cfg.tofiBufferSize) {
                cfg.tofiBufferSize =
                    static_cast<uint32_t>(rawBufferSizeInUint16);
            }
        }
    }

    if (!isRawBypass) {
        aditof::Status status = createModeTofiContext(
            iniFile, iniFileLength, calData, calDataLength, modeNumber,
            ispEnabled, cfg.tofiConfig, cfg.tofiComputeContext);
        if (status != aditof::Status::OK) {
            LOG(ERROR) << "setAlternateModeConfiguration: Failed to build "
                          "compute context for mode "
                       << (int)modeNumber;
            return status;
        }
    }

    // Grow shared buffer pools before swapping in the new config, so the
    // capture/process threads never see a config bigger than the buffers.
    uint32_t maxRawSize = std::max(m_rawFrameBufferSize, cfg.rawFrameBufferSize);
    uint32_t maxTofiSize = std::max(m_tofiBufferSize, cfg.tofiBufferSize);
    if (maxRawSize > m_rawFrameBufferSize || maxTofiSize > m_tofiBufferSize) {
        aditof::Status status = growSharedBufferPools(maxRawSize, maxTofiSize);
        if (status != aditof::Status::OK) {
            if (cfg.tofiComputeContext != nullptr) {
                FreeTofiCompute(cfg.tofiComputeContext);
            }
            if (cfg.tofiConfig != nullptr) {
                FreeTofiConfig(cfg.tofiConfig);
            }
            return status;
        }
    }

    // Free any previously-registered alternate context before replacing it.
    if (m_altConfig.tofiComputeContext != nullptr) {
        FreeTofiCompute(m_altConfig.tofiComputeContext);
    }
    if (m_altConfig.tofiConfig != nullptr) {
        FreeTofiConfig(m_altConfig.tofiConfig);
    }
    m_altConfig = cfg;
    m_altConfigValid = true;

    std::vector<bool> pattern;
    pattern.insert(pattern.end(), repeatPrimary, false);
    pattern.insert(pattern.end(), repeatAlternate, true);
    m_dmsPattern = std::move(pattern);
    m_dmsPos.store(0, std::memory_order_release);
    m_dmsActive.store(true, std::memory_order_release);

    LOG(INFO) << "setAlternateModeConfiguration: mode " << (int)modeNumber
              << " active (" << cfg.outputFrameWidth << "x"
              << cfg.outputFrameHeight << ", rawBypass=" << cfg.isRawBypass
              << "), pattern=" << (int)repeatPrimary << "/"
              << (int)repeatAlternate;

    return aditof::Status::OK;
}

/**
 * @function BufferProcessor::clearAlternateModeConfiguration
 *
 * Deactivates per-frame mode dispatch and frees the alternate mode's
 * compute context, reverting to the single active configuration.
 */
aditof::Status BufferProcessor::clearAlternateModeConfiguration() {
    m_dmsActive.store(false, std::memory_order_release);
    m_dmsPattern.clear();
    m_dmsPos.store(0, std::memory_order_release);

    if (m_altConfig.tofiComputeContext != nullptr) {
        FreeTofiCompute(m_altConfig.tofiComputeContext);
    }
    if (m_altConfig.tofiConfig != nullptr) {
        FreeTofiConfig(m_altConfig.tofiConfig);
    }
    m_altConfig = AltModeConfig();
    m_altConfigValid = false;

    return aditof::Status::OK;
}

/**
 * @function BufferProcessor::getLastDeliveredModeNumber
 *
 * Returns the mode number of the most recently delivered frame, tracked
 * internally rather than parsed from the chip's embedded per-frame
 * metadata (which lands at the wrong offset when the primary and alternate
 * DMS modes differ in resolution).
 */
uint8_t BufferProcessor::getLastDeliveredModeNumber() const {
    return m_lastFrameWasAlternate.load(std::memory_order_acquire)
              ? m_altConfig.modeNumber
              : m_currentModeNumber;
}

/**
 * @function BufferProcessor::buildSRLRFusedBuffer
 *
 * Concatenates a Short-Range and Long-Range raw frame into one input buffer
 * for SR/LR mode fusion (modeFusionEnabled / IsSRFrameFirst ini flags).
 * Each frame occupies width*height*bytesPerPixel bytes; bytesPerPixel is
 * the sum of the depth/AB/confidence bit depths, rounded up to whole bytes.
 */
aditof::Status BufferProcessor::buildSRLRFusedBuffer(
    const uint8_t *srData, uint32_t srWidth, uint32_t srHeight,
    const uint8_t *lrData, uint32_t lrWidth, uint32_t lrHeight,
    uint8_t bitsInDepth, uint8_t bitsInAB, uint8_t bitsInConf,
    bool isSRFrameFirst, std::vector<uint8_t> &pInputBuffer) {

    if (srData == nullptr || lrData == nullptr || srWidth == 0 ||
        srHeight == 0 || lrWidth == 0 || lrHeight == 0) {
        LOG(ERROR) << "buildSRLRFusedBuffer: invalid frame pointer/dimensions";
        return aditof::Status::INVALID_ARGUMENT;
    }

    const uint32_t bytesPerPixel =
        (static_cast<uint32_t>(bitsInDepth) + bitsInAB + bitsInConf + 7) / 8;
    if (bytesPerPixel == 0) {
        LOG(ERROR) << "buildSRLRFusedBuffer: bitsInDepth/AB/Conf sum to 0";
        return aditof::Status::INVALID_ARGUMENT;
    }

    const size_t srFrameBytes =
        static_cast<size_t>(srWidth) * srHeight * bytesPerPixel;
    const size_t lrFrameBytes =
        static_cast<size_t>(lrWidth) * lrHeight * bytesPerPixel;

    pInputBuffer.resize(srFrameBytes + lrFrameBytes);

    if (isSRFrameFirst) {
        memcpy(pInputBuffer.data(), srData, srFrameBytes);
        memcpy(pInputBuffer.data() + srFrameBytes, lrData, lrFrameBytes);
    } else {
        memcpy(pInputBuffer.data(), lrData, lrFrameBytes);
        memcpy(pInputBuffer.data() + lrFrameBytes, srData, srFrameBytes);
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
#ifdef DBG_MEASURE_TIME
    long long totalCaptureTime = 0;
    int totalV4L2Captured = 0;
#endif //DBG_MEASURE_TIME

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

#ifdef DBG_MEASURE_TIME
        auto captureStart = std::chrono::high_resolution_clock::now();
#endif //DBG_MEASURE_TIME

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

        // SECURITY: Validate buffer size before memcpy to prevent overflow
        if (buf_data_len > m_rawFrameBufferSize) {
            LOG(ERROR) << __func__
                       << ": Buffer overflow risk detected! buf_data_len="
                       << buf_data_len
                       << " exceeds allocated size=" << m_rawFrameBufferSize;
            enqueueInternalBufferPrivate(buf, dev);
            m_v4l2_input_buffer_Q.push(v4l2_frame_holder);
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

#ifdef DBG_MEASURE_TIME
        auto captureEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> captureTime =
            captureEnd - captureStart;
        totalCaptureTime += static_cast<long long>(captureTime.count());
        totalV4L2Captured++;
#endif //DBG_MEASURE_TIME
        Tofi_v4l2_buffer v4l2_frame;
        v4l2_frame.data = v4l2_frame_holder;
        v4l2_frame.size = buf_data_len;

        // Tag the frame with which slot of the known DMS repeat pattern it
        // falls in, so processThread() can pick the matching mode config.
        // The chip cycles through the programmed sequence in lock-step with
        // frames actually dequeued here, so advancing once per successfully
        // captured frame keeps this in sync.
        if (m_dmsActive.load(std::memory_order_acquire) &&
            !m_dmsPattern.empty()) {
            size_t idx =
                m_dmsPos.fetch_add(1, std::memory_order_acq_rel) %
                m_dmsPattern.size();
            v4l2_frame.isAlternate = m_dmsPattern[idx];
        }

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

    long long totalProcessTime = 0;
    (void)totalProcessTime; // reserved for future per-frame compute timing

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

        // SR/LR mode fusion: concatenate one LR + one SR raw frame into a
        // single buffer (LR first per IsSRFrameFirst=0) and hand it to the
        // fusion-enabled context as ONE frame, so the library emits a single
        // fused output. LR is identified by mode number (7,8 = LR).
        const bool fusionActive =
            m_dmsActive.load(std::memory_order_acquire) && m_altConfigValid &&
            m_modeFusionEnabled && m_altConfig.modeFusionEnabled;
        if (fusionActive) {
            if (process_frame.isAlternate) {
                m_pendingAltRaw = process_frame.data;
                m_pendingAltSize = process_frame.size;
            } else {
                m_pendingPrimaryRaw = process_frame.data;
                m_pendingPrimarySize = process_frame.size;
            }
            if (!m_pendingPrimaryRaw || !m_pendingAltRaw) {
                continue; // wait for the other half of the LR/SR pair
            }

            auto isLRmode = [](uint8_t m) { return m == 7 || m == 8; };
            const bool primaryIsLR = isLRmode(m_currentModeNumber);
            const uint8_t *lrRaw =
                primaryIsLR ? m_pendingPrimaryRaw.get() : m_pendingAltRaw.get();
            const size_t lrSize =
                primaryIsLR ? m_pendingPrimarySize : m_pendingAltSize;
            const uint8_t *srRaw =
                primaryIsLR ? m_pendingAltRaw.get() : m_pendingPrimaryRaw.get();
            const size_t srSize =
                primaryIsLR ? m_pendingAltSize : m_pendingPrimarySize;

            std::vector<uint8_t> fused(lrSize + srSize);
            if (m_isSRFrameFirst) {
                memcpy(fused.data(), srRaw, srSize);
                memcpy(fused.data() + srSize, lrRaw, lrSize);
            } else {
                memcpy(fused.data(), lrRaw, lrSize);
                memcpy(fused.data() + lrSize, srRaw, srSize);
            }

            m_v4l2_input_buffer_Q.push(m_pendingPrimaryRaw);
            m_v4l2_input_buffer_Q.push(m_pendingAltRaw);
            m_pendingPrimaryRaw.reset();
            m_pendingAltRaw.reset();

            auto fusedHolder = std::shared_ptr<uint8_t>(
                new uint8_t[fused.size()], std::default_delete<uint8_t[]>());
            memcpy(fusedHolder.get(), fused.data(), fused.size());
            process_frame.data = fusedHolder;
            process_frame.size = fused.size();
            process_frame.skipRawBufferRecycle = true;
            // Process the fused frame through the LR (primary) context path.
            process_frame.isAlternate = !primaryIsLR;
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
            if (!process_frame.skipRawBufferRecycle) {
                m_v4l2_input_buffer_Q.push(process_frame.data);
            }
            continue;
        }

        // Resolve which mode's config applies to this specific frame: the
        // alternate DMS mode's config if it was tagged as such at capture
        // time, otherwise the primary setMode()-active config (unchanged
        // behavior when no alternate mode is registered).
        const bool isAlt = process_frame.isAlternate && m_altConfigValid;
        const bool rawBypass = isAlt ? m_altConfig.isRawBypass : m_isRawBypassMode;
        const uint32_t rawMaxBufSize =
            isAlt ? m_altConfig.rawFrameBufferSize : m_rawFrameBufferSize;
        const uint32_t tofiBufSize =
            isAlt ? m_altConfig.tofiBufferSize : m_tofiBufferSize;
        const uint32_t outW =
            isAlt ? m_altConfig.outputFrameWidth : m_outputFrameWidth;
        const uint32_t outH =
            isAlt ? m_altConfig.outputFrameHeight : m_outputFrameHeight;
        const uint8_t bitsAB = isAlt ? m_altConfig.bitsInAB : m_bitsInAB;
        const uint8_t bitsConf = isAlt ? m_altConfig.bitsInConf : m_bitsInConf;
        const uint8_t bitsDepth =
            isAlt ? m_altConfig.bitsInDepth : m_bitsInDepth;
        TofiComputeContext *computeCtx =
            isAlt ? m_altConfig.tofiComputeContext : m_tofiComputeContext;

        // Raw bypass mode: Simple copy path, no ToFi processing
        // Must handle before accessing computeCtx (nullptr for raw bypass)
        if (rawBypass) {
            const size_t bufferSizeBytes = process_frame.size;
            // Use raw frame buffer size for comparison (includes NVIDIA alignment)
            const size_t maxBufferSize = rawMaxBufSize;

            if (bufferSizeBytes <= maxBufferSize) {
                // Raw bypass: Copy entire V4L2 buffer including NVIDIA padding
                // Keep buffer exactly as received from V4L2 driver
                uint8_t *src = process_frame.data.get();
                uint8_t *dst =
                    reinterpret_cast<uint8_t *>(tofi_compute_io_buff.get());

                // SECURITY: Validate destination buffer capacity
                size_t dst_capacity = tofiBufSize * sizeof(uint16_t);
                if (bufferSizeBytes > dst_capacity) {
                    LOG(ERROR)
                        << "Raw bypass buffer overflow risk: source="
                        << bufferSizeBytes << " exceeds dest=" << dst_capacity;
                    // Restore buffers and skip frame
                    m_tofi_io_Buffer_Q.push(tofi_compute_io_buff);
                    continue;
                }

                // Copy full V4L2 buffer (data + padding)
                memcpy(dst, src, bufferSizeBytes);

                // Record frame if recording is active
                if (m_state == ST_RECORD && m_stream_file_out.is_open()) {
                    aditof::Status writeStatus = writeFrame(
                        (uint8_t *)tofi_compute_io_buff.get(), bufferSizeBytes);
                    if (writeStatus != aditof::Status::OK) {
                        LOG(WARNING) << "Failed to write raw bypass frame "
                                        "during recording";
                    }
                }

                // Package processed frame for output
                process_frame.tofiBuffer = tofi_compute_io_buff;
                process_frame.size = bufferSizeBytes / sizeof(uint16_t);

                // Push to done queue
                if (!m_process_done_Q.push(std::move(process_frame))) {
                    LOG(WARNING) << "processThread: Failed to push raw "
                                    "bypass frame to done queue";
                    // Restore buffers on error
                    m_tofi_io_Buffer_Q.push(tofi_compute_io_buff);
                    if (!process_frame.skipRawBufferRecycle) {
                        m_v4l2_input_buffer_Q.push(process_frame.data);
                    }
                }

                continue; // Skip ToFi computation
            } else {
                LOG(ERROR) << "processThread: Raw bypass buffer size mismatch: "
                           << bufferSizeBytes << " > " << maxBufferSize;

                // Restore buffers and continue
                m_tofi_io_Buffer_Q.push(tofi_compute_io_buff);
                if (!process_frame.skipRawBufferRecycle) {
                    m_v4l2_input_buffer_Q.push(process_frame.data);
                }

                continue;
            }
        }

        if (computeCtx == nullptr) {
            LOG(ERROR) << "processThread: No compute context available for "
                       << (isAlt ? "alternate" : "primary") << " mode frame";
            m_tofi_io_Buffer_Q.push(tofi_compute_io_buff);
            if (!process_frame.skipRawBufferRecycle) {
                m_v4l2_input_buffer_Q.push(process_frame.data);
            }
            continue;
        }

        // Standard ToF mode: Save context pointers before modifying
        uint16_t *tempDepthFrame = computeCtx->p_depth_frame;
        uint16_t *tempAbFrame = computeCtx->p_ab_frame;
        float *tempConfFrame = computeCtx->p_conf_frame;

        if (tofi_compute_io_buff) {

            // Buffer layout depends on bit configuration via tofiBufSize
            // tofiBufSize = depthSize + abSize + confSize (in uint16_t units)
            // Only allocate/point to components that are actually configured

            const int numPixels = outW * outH;

            // Always have depth frame (always allocated)
            computeCtx->p_depth_frame = tofi_compute_io_buff.get();

            // Calculate what's actually allocated after depth
            uint32_t allocatedAfterDepth =
                tofiBufSize - static_cast<uint32_t>(numPixels);

            // Set AB pointer only if AB is allocated
            if (allocatedAfterDepth > 0) {
                computeCtx->p_ab_frame =
                    tofi_compute_io_buff.get() + numPixels;

                // Calculate remaining space after AB (for confidence)
                // AB can be either numPixels or numPixels/2 depending on 8-bit vs 16-bit
                uint32_t abSize = std::min(allocatedAfterDepth,
                                           static_cast<uint32_t>(numPixels));
                uint32_t allocatedAfterAB = allocatedAfterDepth - abSize;

                // Set confidence pointer only if confidence is allocated
                if (allocatedAfterAB > 0) {
                    computeCtx->p_conf_frame =
                        reinterpret_cast<float *>(tofi_compute_io_buff.get() +
                                                  numPixels + abSize);
                } else {
                    // No confidence allocated - point to a safe dummy location or keep original
                    computeCtx->p_conf_frame = tempConfFrame;
                }
            } else {
                // No AB or confidence allocated - keep original pointers
                computeCtx->p_ab_frame = tempAbFrame;
                computeCtx->p_conf_frame = tempConfFrame;
            }

            // Strip NVIDIA Tegra VI alignment padding AND Pulsatrix extra bytes.
            // Exact ToFi payload = outW × outH × (bitsInDepth + bitsInAB + bitsInConf) / 8
            // bitsDepth comes from bitsInPhaseOrDepth INI param (default 16).
            // This matches the "Total Bytes" column in the driver config table, e.g.:
            //   D=16, AB=16, conf=8 → 1024×1024×5   = 5,242,880  (strips 3072+1024)
            //   D=16, AB=12, conf=8 → 1024×1024×4.5 = 4,718,592  (no Pulsatrix padding)
            //   D=16, AB=8,  conf=8 → 1024×1024×4   = 4,194,304  (strips 3072+2048)
            const size_t tofiPayloadBytes =
                static_cast<size_t>(outW) * outH *
                (bitsDepth + bitsAB + bitsConf) / 8u;

            // During DMS the V4L2 buffer keeps the PRIMARY mode's line stride,
            // so an alternate (smaller) mode's lines sit at the start of each
            // primary-stride line with zero padding after. Reconstruct the
            // contiguous alternate payload by gathering each line before
            // handing it to TofiCompute; otherwise the padding is interleaved
            // into the data and depth comes out garbage.
            std::vector<uint8_t> destridedRaw;
            uint8_t *tofiInput = process_frame.data.get();
            if (isAlt && !process_frame.skipRawBufferRecycle) {
                const size_t dstStride =
                    static_cast<size_t>(outW) *
                    (bitsDepth + bitsAB + bitsConf) / 8u;
                const size_t srcStride =
                    static_cast<size_t>(m_outputFrameWidth) *
                    (m_bitsInDepth + m_bitsInAB + m_bitsInConf) / 8u;
                if (srcStride > dstStride && dstStride > 0 &&
                    process_frame.size >= srcStride * outH) {
                    destridedRaw.resize(dstStride * outH);
                    const uint8_t *src = process_frame.data.get();
                    for (uint32_t line = 0; line < outH; ++line) {
                        memcpy(destridedRaw.data() + line * dstStride,
                               src + line * srcStride, dstStride);
                    }
                    tofiInput = destridedRaw.data();
                    process_frame.size = destridedRaw.size();
                }
            }

            // A fused SR+LR buffer is intentionally larger than a single
            // mode's payload; only trim non-fused frames (skipRawBufferRecycle
            // is set only for the one-off fused buffer).
            if (!process_frame.skipRawBufferRecycle &&
                process_frame.size > tofiPayloadBytes) {
                process_frame.size = tofiPayloadBytes;
            }

            uint32_t ret = TofiCompute(
                reinterpret_cast<uint16_t *>(tofiInput), computeCtx, NULL);
            if (ret != ADI_TOFI_SUCCESS) {
                LOG(ERROR) << "processThread: TofiCompute failed with code: "
                           << ret;
                m_tofi_io_Buffer_Q.push(tofi_compute_io_buff);
                if (!process_frame.skipRawBufferRecycle) {
                    m_v4l2_input_buffer_Q.push(process_frame.data);
                }
                computeCtx->p_depth_frame = tempDepthFrame;
                computeCtx->p_ab_frame = tempAbFrame;
                computeCtx->p_conf_frame = tempConfFrame;
                continue;
            }
            computeCtx->p_depth_frame = tempDepthFrame;
            computeCtx->p_ab_frame = tempAbFrame;
            computeCtx->p_conf_frame = tempConfFrame;

        } // end if (tofi_compute_io_buff)

        // Apply 90-degree clockwise rotation if needed
        if (m_needsRotation && !rawBypass) {
            // Rotate from tofi_compute_io_buff into m_rotationOutputBuffer (no memcpy).
            // Then swap the two shared_ptrs: tofi_compute_io_buff gets the rotated result,
            // m_rotationOutputBuffer holds the old input and becomes the dst for next frame.

            // The ISP embeds 128-byte metadata at the start of the AB section.
            // Rotation scrambles those bytes; save them before and restore after.
            const size_t numPixels = static_cast<size_t>(outW) * outH;
            uint8_t metadataSave[METADATA_SIZE];
            memcpy(metadataSave,
                   reinterpret_cast<uint8_t *>(tofi_compute_io_buff.get()) +
                       numPixels * sizeof(uint16_t),
                   METADATA_SIZE);

            rotateEntireToFiBuffer(
                tofi_compute_io_buff.get(), m_rotationOutputBuffer.get(),
                outW, outH, tofiBufSize);
            std::swap(tofi_compute_io_buff, m_rotationOutputBuffer);

            // Restore metadata to the first 128 bytes of the AB section
            memcpy(reinterpret_cast<uint8_t *>(tofi_compute_io_buff.get()) +
                       numPixels * sizeof(uint16_t),
                   metadataSave, METADATA_SIZE);
        }

        // Only attempt to write if recording is still active and stream is open
        if (m_state == ST_RECORD && m_stream_file_out.is_open()) {
            aditof::Status writeStatus =
                writeFrame((uint8_t *)tofi_compute_io_buff.get(),
                           tofiBufSize * sizeof(uint16_t));
            if (writeStatus != aditof::Status::OK) {
                LOG(WARNING)
                    << "Failed to write processed frame during recording";
            }
        }

        process_frame.tofiBuffer = tofi_compute_io_buff;
        process_frame.size = tofiBufSize;

        if (!m_process_done_Q.push(std::move(process_frame))) {
            LOG(WARNING) << "processThread: Push timeout to "
                            "m_process_done_Q, ProcessedQueueSize: "
                         << m_process_done_Q.size();
            m_tofi_io_Buffer_Q.push(tofi_compute_io_buff);
            if (!process_frame.skipRawBufferRecycle) {
                m_v4l2_input_buffer_Q.push(process_frame.data);
            }
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

                m_lastFrameWasAlternate.store(
                    tof_processed_frame.isAlternate && m_altConfigValid,
                    std::memory_order_release);

                memcpy(buffer, tof_processed_frame.tofiBuffer.get(),
                       tof_processed_frame.size * sizeof(uint16_t));

                // Return buffers to their respective pools
                m_tofi_io_Buffer_Q.push(tof_processed_frame.tofiBuffer);
                if (!tof_processed_frame.skipRawBufferRecycle) {
                    m_v4l2_input_buffer_Q.push(tof_processed_frame.data);
                }

                return aditof::Status::OK; // Success, exit function
            } else {                       // NOLINT(llvm-else-after-return)
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

/**
 * @brief Waits for a buffer to be available on the V4L2 device using select.
 *
 * Blocks on select() with a timeout (SELECT_TIMEOUT_SEC) until the device is ready for reading.
 * If stopThreadsFlag is set, uses zero timeout to avoid blocking during shutdown.
 *
 * @param[in] dev Pointer to VideoDev structure (uses m_inputVideoDev if nullptr)
 *
 * @return Status::OK if buffer is ready, Status::GENERIC_ERROR on timeout or select error
 */
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

    aditof::Status status = aditof::Status::OK;
    if (r == -1) {
        LOG(WARNING) << "select error "
                     << "errno: " << errno << " error: " << strerror(errno);
        status = aditof::Status::GENERIC_ERROR;
    } else if (r == 0) {
        LOG(WARNING) << "select timeout";
        status = aditof::Status::GENERIC_ERROR;
    }
    return status;
}

/**
 * @brief Dequeues a buffer from the V4L2 device.
 *
 * Calls VIDIOC_DQBUF ioctl to dequeue a filled buffer from the driver. The buffer
 * contains captured frame data ready for processing.
 *
 * @param[out] buf v4l2_buffer structure to receive dequeued buffer info
 * @param[in] dev Pointer to VideoDev structure (uses m_inputVideoDev if nullptr)
 *
 * @return Status::OK on success, Status::GENERIC_ERROR on ioctl failure or invalid index
 */
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

/**
 * @brief Retrieves pointer and size of a dequeued V4L2 buffer.
 *
 * Returns a pointer to the mmap'd memory region for the given buffer index and
 * the actual number of bytes used (from v4l2_buffer.bytesused).
 *
 * @param[out] buffer Pointer to receive buffer memory address
 * @param[out] buf_data_len Variable to receive buffer data length in bytes
 * @param[in] buf v4l2_buffer structure from dequeue operation
 * @param[in] dev Pointer to VideoDev structure (uses m_inputVideoDev if nullptr)
 *
 * @return Status::OK on success
 */
aditof::Status BufferProcessor::getInternalBufferPrivate(
    uint8_t **buffer, uint32_t &buf_data_len, const struct v4l2_buffer &buf,
    struct VideoDev *dev) {
    if (dev == nullptr)
        dev = m_inputVideoDev;

    *buffer = static_cast<uint8_t *>(dev->videoBuffers[buf.index].start);
    buf_data_len = buf.bytesused;

    return aditof::Status::OK;
}

/**
 * @brief Re-queues a buffer to the V4L2 device for reuse.
 *
 * Calls VIDIOC_QBUF ioctl to return an empty buffer to the driver queue for
 * future frame capture. Must be called after processing buffer data.
 *
 * @param[in] buf v4l2_buffer structure to re-queue
 * @param[in] dev Pointer to VideoDev structure (uses m_inputVideoDev if nullptr)
 *
 * @return Status::OK on success, Status::GENERIC_ERROR on ioctl failure
 */
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

/**
 * @brief Retrieves the device file descriptor (not applicable for this processor).
 *
 * Returns -1 as this processor does not expose a direct device file descriptor.
 *
 * @param[out] fileDescriptor Variable to receive the file descriptor (-1)
 *
 * @return Status::OK
 */
aditof::Status BufferProcessor::getDeviceFileDescriptor(int &fileDescriptor) {
    fileDescriptor = -1;
    return aditof::Status::OK;
}

/**
 * @brief Public wrapper for waitForBufferPrivate.
 *
 * Waits for a buffer to be available on the default input video device.
 *
 * @return Status::OK if buffer is ready, Status::GENERIC_ERROR on error
 */
aditof::Status BufferProcessor::waitForBuffer() {

    return waitForBufferPrivate();
}

/**
 * @brief Public wrapper for dequeueInternalBufferPrivate.
 *
 * Dequeues a buffer from the default input video device.
 *
 * @param[out] buf v4l2_buffer structure to receive dequeued buffer info
 *
 * @return Status::OK on success, Status::GENERIC_ERROR on error
 */
aditof::Status BufferProcessor::dequeueInternalBuffer(struct v4l2_buffer &buf) {

    return dequeueInternalBufferPrivate(buf);
}

/**
 * @brief Public wrapper for getInternalBufferPrivate.
 *
 * Retrieves pointer and size of a dequeued buffer from the default input device.
 *
 * @param[out] buffer Pointer to receive buffer memory address
 * @param[out] buf_data_len Variable to receive buffer data length
 * @param[in] buf v4l2_buffer structure from dequeue operation
 *
 * @return Status::OK on success
 */
aditof::Status
BufferProcessor::getInternalBuffer(uint8_t **buffer, uint32_t &buf_data_len,
                                   const struct v4l2_buffer &buf) {

    return getInternalBufferPrivate(buffer, buf_data_len, buf);
}

/**
 * @brief Public wrapper for enqueueInternalBufferPrivate.
 *
 * Re-queues a buffer to the default input video device for reuse.
 *
 * @param[in] buf v4l2_buffer structure to re-queue
 *
 * @return Status::OK on success, Status::GENERIC_ERROR on error
 */
aditof::Status BufferProcessor::enqueueInternalBuffer(struct v4l2_buffer &buf) {

    return enqueueInternalBufferPrivate(buf);
}

/**
 * @brief Retrieves the ToFi configuration object.
 *
 * Returns a pointer to the current ToFi configuration structure, which contains
 * calibration and mode-specific settings.
 *
 * @return Pointer to TofiConfig structure, or nullptr if not initialized
 */
TofiConfig *BufferProcessor::getTofiConfig() const { return m_tofiConfig; }

/**
 * @brief Retrieves the depth compute library version/type.
 *
 * Returns whether the open-source depth compute library is enabled.
 *
 * @param[out] enabled Variable to receive enabled status (1 for open-source, 0 for proprietary)
 *
 * @return Status::OK on success
 */
aditof::Status BufferProcessor::getDepthComputeVersion(uint8_t &enabled) const {
    enabled = m_depthComputeConfig.getStatus();
    return aditof::Status::OK;
}

/**
 * @brief Starts the capture and processing worker threads.
 *
 * Clears stop flag, sets stream running, and launches captureFrameThread and processThread.
 * The processing thread priority is set to SCHED_FIFO with priority THREAD_PRIORITY for
 * real-time performance.
 */
void BufferProcessor::startThreads() {
    stopThreadsFlag.store(false, std::memory_order_release);
    streamRunning = true;

    m_captureThread = std::thread(&BufferProcessor::captureFrameThread, this);
    m_processingThread = std::thread(&BufferProcessor::processThread, this);
    sched_param param;
    param.sched_priority = THREAD_PRIORITY;
    pthread_setschedparam(m_processingThread.native_handle(), SCHED_FIFO,
                          &param);
}

/**
 * @brief Stops the capture and processing worker threads.
 *
 * Sets stop flag, stops recording if active, waits for queue drainage with 5-second timeout,
 * joins threads, and flushes remaining frames back to buffer pools. Ensures threads fully
 * exit before touching any buffers to prevent race conditions.
 */
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

    // Reset thread objects
    m_captureThread = std::thread();
    m_processingThread = std::thread();

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
}

#pragma region Stream_Recording_and_Playback

#include "aditof/utils.h"

/**
 * @brief Starts recording processed frames to a binary file.
 *
 * Creates output folder if needed, generates a timestamped filename, opens the output
 * file stream, writes header parameters, and transitions to recording state. Frames
 * are written during processThread execution.
 *
 * @param[out] fileName String to receive the generated recording file path
 * @param[in] parameters Pointer to header parameters buffer (written as first frame)
 * @param[in] paramSize Size of header parameters in bytes
 *
 * @return Status::OK on success, Status::GENERIC_ERROR if folder creation or file open fails
 */
aditof::Status BufferProcessor::startRecording(std::string &filePath,
                                               uint8_t *parameters,
                                               uint32_t paramSize) {

    using namespace aditof;

    m_state = ST_STOP;

    if (Utils::generateRecordingPath(filePath) == false) {
        LOG(ERROR) << "Failed to create output folder for recordings";
        return aditof::Status::GENERIC_ERROR;
    }

    if (m_stream_file_out.is_open()) {
        m_stream_file_out.close();
    }

    m_frame_count = 0;

    m_stream_file_out = std::ofstream(filePath, std::ios::binary);

    m_state = ST_RECORD;

    writeFrame(parameters, paramSize);

    return aditof::Status::OK;
}

/**
 * @brief Stops recording and finalizes the output file.
 *
 * Seeks back to the header to write the final frame count (excluding header frame),
 * then closes the file stream. Logs total frames saved.
 *
 * @return Status::OK if file was open and closed successfully,
 *         Status::GENERIC_ERROR if file stream was not open
 */
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
        m_frame_count--; // Take into account the header frame
        m_stream_file_out.write(reinterpret_cast<const char *>(&m_frame_count),
                                sizeof(m_frame_count));

        m_stream_file_out.close();
        LOG(INFO) << "Recording stopped. Total frames saved: " << m_frame_count;

        status = aditof::Status::OK;
    }

    return status;
}

/**
 * @brief Writes a frame to the recording file.
 *
 * Writes frame marker (0xFFFFFFFF), buffer size, and buffer data to the output stream.
 * Increments frame count on successful write. Only operates in recording state.
 *
 * @param[in] buffer Pointer to frame data to write
 * @param[in] bufferSize Size of frame data in bytes
 *
 * @return Status::OK on success, Status::GENERIC_ERROR if not recording or I/O error
 */
aditof::Status BufferProcessor::writeFrame(uint8_t *buffer,
                                           uint32_t bufferSize) {
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

            m_frame_count++;

            return aditof::Status::OK;
        }
    } catch (const std::ofstream::failure &e) {
        LOG(ERROR) << "File I/O exception caught: " << e.what();
        m_stream_file_out.close();
        m_state = ST_STOP;
    }
    return aditof::Status::GENERIC_ERROR;
}

/**
 * @brief Automatically stops recording or playback based on current state.
 *
 * Called during shutdown to clean up active recording or playback operations.
 * Stops recording if in recording state; returns error for playback state.
 *
 * @return Status::OK if recording stopped successfully,
 *         Status::GENERIC_ERROR for playback state or if not recording
 */
aditof::Status BufferProcessor::automaticStop() {
    aditof::Status status = aditof::Status::OK;
    if (m_state == ST_PLAYBACK) {
        status = aditof::Status::GENERIC_ERROR;
    }
    if (m_state == ST_RECORD) {
        status = stopRecording();
    }

    return status;
}
