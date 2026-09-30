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
#include <atomic>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <queue>
#include <random>
#include <thread>
#include <vector>

#include "aditof/depth_sensor_interface.h"
#include "buffer_processor_interface.h"
#include "depth_compute_config.h"
#include "v4l_buffer_access_interface.h"

#include "tofi/tofi_compute.h"
#include "tofi/tofi_config.h"
#include "tofi/tofi_util.h"

#define OUTPUT_DEVICE "/dev/video1"
#define CHIP_ID_SINGLE 0x5931
#define DEFAULT_MODE 0

struct buffer {
    void *start;
    size_t length;
};

struct VideoDev {
    int fd;
    int sfd;
    struct buffer *videoBuffers;
    unsigned int nVideoBuffers;
    struct v4l2_plane planes[8];
    enum v4l2_buf_type videoBuffersType;
    bool started;

    VideoDev()
        : fd(-1), sfd(-1), videoBuffers(nullptr), nVideoBuffers(0),
          started(false) {}
};

template <typename T>
class ThreadSafeQueue {
  private:
    std::queue<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable not_empty_;
    std::condition_variable not_full_;
    size_t max_size_;

  public:
    explicit ThreadSafeQueue(size_t max_size) { max_size_ = max_size; }

    bool
    push(T item,
         std::chrono::milliseconds timeout = std::chrono::milliseconds(1100)) {
        std::unique_lock<std::mutex> lock(mutex_);
        auto deadline = std::chrono::steady_clock::now() + timeout;
        if (!not_full_.wait_until(
                lock, deadline, [this] { return queue_.size() < max_size_; })) {
            return false;
        }
        queue_.push(std::move(item));
        lock.unlock();
        not_empty_.notify_all();
        return true;
    }

    bool
    pop(T &item,
        std::chrono::milliseconds timeout = std::chrono::milliseconds(1100)) {
        std::unique_lock<std::mutex> lock(mutex_);
        auto deadline = std::chrono::steady_clock::now() + timeout;
        if (!not_empty_.wait_until(lock, deadline,
                                   [this] { return !queue_.empty(); })) {
            return false;
        }
        item = std::move(queue_.front());
        queue_.pop();
        lock.unlock();
        not_full_.notify_all();
        return true;
    }

    size_t max_size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return max_size_;
    }

    void set_max_size(size_t new_max_size) {
        std::lock_guard<std::mutex> lock(mutex_);
        max_size_ = new_max_size;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
};

class BufferProcessor : public aditof::V4lBufferAccessInterface,
                        public aditof::BufferProcessorInterface {
  public:
    BufferProcessor();
    ~BufferProcessor();

  public:
    // BufferProcessorInterface implementation
    aditof::Status open() override;
    aditof::Status setInputDevice(VideoDev *inputVideoDev) override;
    aditof::Status setVideoProperties(int frameWidth, int frameHeight,
                                      int WidthInBytes, int HeightInBytes,
                                      int modeNumber, uint8_t bitsInAB,
                                      uint8_t bitsInConf, uint8_t bitsInDepth,
                                      bool isRawBypass = false) override;
    aditof::Status setProcessorProperties(uint8_t *iniFile,
                                          uint16_t iniFileLength,
                                          uint8_t *calData,
                                          uint32_t calDataLength, uint16_t mode,
                                          bool ispEnabled) override;
    aditof::Status processBuffer(uint16_t *buffer) override;
    TofiConfig *getTofiConfig() const override;
    aditof::Status getDepthComputeVersion(uint8_t &enabled) const override;
    void setLensScatterCompensationEnabled(bool enabled) override {
        m_lensScatterCompensationEnabled = enabled;
    }
    bool getLensScatterCompensationEnabled() const override {
        return m_lensScatterCompensationEnabled;
    }
    void setNeedsRotation(bool needsRotation) override {
        m_needsRotation = needsRotation;
    }
    bool getNeedsRotation() const override { return m_needsRotation; }

    void startThreads() override;
    void stopThreads() override;

    aditof::Status setAlternateModeConfiguration(
        uint8_t modeNumber, int frameWidth, int frameHeight,
        int widthInBytes, int heightInBytes, uint8_t bitsInAB,
        uint8_t bitsInConf, uint8_t bitsInDepth, bool isRawBypass,
        bool ispEnabled, uint8_t *iniFile, uint16_t iniFileLength,
        uint8_t *calData, uint32_t calDataLength, uint8_t repeatPrimary,
        uint8_t repeatAlternate) override;
    aditof::Status clearAlternateModeConfiguration() override;
    uint8_t getLastDeliveredModeNumber() const override;

    /**
     * @brief Concatenates a Short-Range and a Long-Range raw frame into a
     * single input buffer for SR/LR mode fusion, per the given IsSRFrameFirst
     * ordering. Each frame's byte size is width*height*bytesPerPixel, where
     * bytesPerPixel is derived from bitsInDepth+bitsInAB+bitsInConf.
     *
     * @param srData Raw SR frame bytes (srWidth x srHeight)
     * @param lrData Raw LR frame bytes (lrWidth x lrHeight)
     * @param bitsInDepth Depth bit depth (shared by SR and LR)
     * @param bitsInAB AB bit depth (shared by SR and LR)
     * @param bitsInConf Confidence bit depth (shared by SR and LR)
     * @param isSRFrameFirst True to place the SR frame first in the buffer
     * @param pInputBuffer Receives the concatenated SR+LR buffer
     * @return Status::OK on success, Status::INVALID_ARGUMENT on bad input
     */
    static aditof::Status
    buildSRLRFusedBuffer(const uint8_t *srData, uint32_t srWidth,
                        uint32_t srHeight, const uint8_t *lrData,
                        uint32_t lrWidth, uint32_t lrHeight,
                        uint8_t bitsInDepth, uint8_t bitsInAB,
                        uint8_t bitsInConf, bool isSRFrameFirst,
                        std::vector<uint8_t> &pInputBuffer);

    // Legacy method (keeping for backward compatibility)
    TofiConfig *getTofiCongfig() const { return getTofiConfig(); }
    static int getTimeoutDelay() { return TIME_OUT_DELAY; }

  public:
    virtual aditof::Status waitForBuffer() override;
    virtual aditof::Status
    dequeueInternalBuffer(struct v4l2_buffer &buf) override;
    virtual aditof::Status
    getInternalBuffer(uint8_t **buffer, uint32_t &buf_data_len,
                      const struct v4l2_buffer &buf) override;
    virtual aditof::Status
    enqueueInternalBuffer(struct v4l2_buffer &buf) override;
    virtual aditof::Status
    getDeviceFileDescriptor(int &fileDescriptor) override;

  private:
    aditof::Status waitForBufferPrivate(struct VideoDev *dev = nullptr);
    aditof::Status dequeueInternalBufferPrivate(struct v4l2_buffer &buf,
                                                struct VideoDev *dev = nullptr);
    aditof::Status getInternalBufferPrivate(uint8_t **buffer,
                                            uint32_t &buf_data_len,
                                            const struct v4l2_buffer &buf,
                                            struct VideoDev *dev = nullptr);
    aditof::Status enqueueInternalBufferPrivate(struct v4l2_buffer &buf,
                                                struct VideoDev *dev = nullptr);

    void captureFrameThread();
    void processThread();
    // Reads the ISP-embedded imagerMode from a raw DMS frame (metadata header
    // sits at the start of the AB block). Returns 0xFF if unreadable, so the
    // caller can fall back to the DMS counter tag.
    uint8_t readRawFrameImagerMode(const uint8_t *raw, size_t size) const;
    // Reads the ISP-embedded frameNumber from a raw DMS frame. Returns
    // 0xFFFFFFFF if unreadable.
    uint32_t readRawFrameNumber(const uint8_t *raw, size_t size) const;
    void calculateFrameSize(uint8_t &bitsInAB, uint8_t &bitsInConf);
    void rotateEntireToFiBuffer(const uint16_t *src, uint16_t *dst,
                                uint32_t width, uint32_t height,
                                uint32_t bufferSize);

    // Alternate-mode (Dynamic Mode Switching) support
    struct AltModeConfig;
    aditof::Status createModeTofiContext(uint8_t *iniFile,
                                         uint16_t iniFileLength,
                                         uint8_t *calData,
                                         uint32_t calDataLength, uint16_t mode,
                                         bool ispEnabled, TofiConfig *&outConfig,
                                         TofiComputeContext *&outContext);
    aditof::Status growSharedBufferPools(uint32_t newRawSize,
                                        uint32_t newTofiSize);

  private:
    bool m_vidPropSet;
    bool m_processorPropSet;

    uint16_t m_outputFrameWidth;
    uint16_t m_outputFrameHeight;
    uint16_t m_driverFrameWidth;
    uint16_t m_driverFrameHeight;

    TofiConfig *m_tofiConfig;
    TofiComputeContext *m_tofiComputeContext;
    TofiXYZDealiasData m_xyzDealiasData[11];

    struct VideoDev *m_inputVideoDev;

    struct Tofi_v4l2_buffer {
        std::shared_ptr<uint8_t> data;
        size_t size = 0;
        std::shared_ptr<uint16_t> tofiBuffer;
        // True if this frame was captured while the alternate DMS mode's
        // slot was active; ignored when no alternate mode is configured.
        bool isAlternate = false;
        // True when .data is a one-off SR/LR fused buffer, not a buffer
        // from m_v4l2_input_buffer_Q; must not be recycled into that pool.
        bool skipRawBufferRecycle = false;
    };

    // Buffer layout + compute context for a second mode, used to correctly
    // process frames produced by hardware Dynamic Mode Switching when that
    // mode differs in resolution/bit layout from the primary setMode()-active
    // configuration.
    struct AltModeConfig {
        uint8_t modeNumber = 0;
        uint16_t outputFrameWidth = 0;
        uint16_t outputFrameHeight = 0;
        uint16_t driverFrameWidth = 0;
        uint16_t driverFrameHeight = 0;
        uint8_t bitsInAB = 0;
        uint8_t bitsInConf = 0;
        uint8_t bitsInDepth = 16;
        bool isRawBypass = false;
        bool ispEnabled = true;
        uint32_t rawFrameBufferSize = 0;
        uint32_t tofiBufferSize = 0;
        uint32_t abFrameSize = 0;
        bool modeFusionEnabled = false;
        bool isSRFrameFirst = false;
        TofiConfig *tofiConfig = nullptr;
        TofiComputeContext *tofiComputeContext = nullptr;
    };

    // Thread-safe pool of empty raw frame buffers for use by capture thread
    ThreadSafeQueue<std::shared_ptr<uint8_t>> m_v4l2_input_buffer_Q;

    // Thread-safe queue to transfer captured raw frames to the process thread
    ThreadSafeQueue<Tofi_v4l2_buffer> m_capture_to_process_Q;

    // Thread-safe pool of ToFi compute output buffers (depth + AB + confidence)
    ThreadSafeQueue<std::shared_ptr<uint16_t>> m_tofi_io_Buffer_Q;

    // Thread-safe queue for frames that have been fully processed (compute done)
    ThreadSafeQueue<Tofi_v4l2_buffer> m_process_done_Q;

    uint32_t m_rawFrameBufferSize;
    uint32_t m_tofiBufferSize;
    uint32_t
        m_abFrameSize; ///< AB plane size in uint16_t units (0 when bitsInAB=0)
    // Ping-pong output buffer for rotation: avoids final memcpy by swapping with
    // tofi_compute_io_buff after each frame so the rotated result is used directly.
    std::shared_ptr<uint16_t> m_rotationOutputBuffer;

    std::thread m_captureThread;
    std::thread m_processingThread;

    std::atomic<bool> stopThreadsFlag;
    bool streamRunning = false;

    const static constexpr int TIME_OUT_DELAY = 5;

    int m_maxTries = 3;

    uint8_t m_currentModeNumber;
    uint8_t
        m_bitsInAB; ///< AB bits per pixel (0/8/12/16) — used to compute exact ToFi payload
    uint8_t
        m_bitsInConf; ///< Conf bits per pixel (0/4/8)  — used to compute exact ToFi payload
    uint8_t
        m_bitsInDepth; ///< Depth bits per pixel (from bitsInPhaseOrDepth INI param, default 16)
    bool m_isRawBypassMode;
    bool
        m_ispEnabled; // Whether ISP depth computation is enabled (pre-computed depth)
    bool
        m_lensScatterCompensationEnabled; // When true, raw bypass uses TofiCompute
    bool
        m_needsRotation; // When true, rotate frames 90 degrees clockwise (for ADTF3080)
    bool m_modeFusionEnabled = false; ///< parsed from the primary mode's ini blob
    bool m_isSRFrameFirst = false;    ///< parsed from the primary mode's ini blob

    aditof::DepthComputeConfig m_depthComputeConfig;

    // Alternate-mode (Dynamic Mode Switching) state
    AltModeConfig m_altConfig;
    bool m_altConfigValid = false;
    std::vector<bool> m_dmsPattern; ///< false=primary mode, true=alternate
    std::atomic<size_t> m_dmsPos{0};
    std::atomic<bool> m_dmsActive{false};
    // Which mode the most recently delivered (processBuffer()) frame used;
    // ground truth for the caller, since the embedded per-frame chip
    // metadata can land at the wrong offset when primary/alternate modes
    // differ in resolution.
    std::atomic<bool> m_lastFrameWasAlternate{false};

    // SR/LR mode-fusion pairing: holds whichever of the pair (primary/
    // alternate) arrives first, until its counterpart shows up.
    std::shared_ptr<uint8_t> m_pendingPrimaryRaw;
    size_t m_pendingPrimarySize = 0;
    std::shared_ptr<uint8_t> m_pendingAltRaw;
    size_t m_pendingAltSize = 0;

    static bool parseIniBoolFlag(uint8_t *iniFile, uint16_t iniFileLength,
                                const char *key, bool defaultValue);

  public:
    // Stream record and playback support
    aditof::Status startRecording(std::string &fileName, uint8_t *parameters,
                                  uint32_t paramSize);
    aditof::Status stopRecording();

  private:
    aditof::Status automaticStop();
    aditof::Status writeFrame(uint8_t *buffer, uint32_t bufferSize);
    enum StreamType { ST_STOP, ST_RECORD, ST_PLAYBACK } m_state;
    std::ofstream m_stream_file_out;
    std::ifstream m_stream_file_in;
    std::string m_stream_file_name;
    uint32_t m_frame_count;
};