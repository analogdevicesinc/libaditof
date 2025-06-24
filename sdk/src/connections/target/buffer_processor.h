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
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "v4l_buffer_access_interface.h"

#include "tofi/tofi_compute.h"
#include "tofi/tofi_config.h"
#include "tofi/tofi_util.h"
#include "buffer_allocator.h"

#define OUTPUT_DEVICE "/dev/video1"

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

#if 0
template <typename T>
class ThreadSafeQueue {
  private:
    std::queue<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable not_empty_;
    std::condition_variable not_full_;
    size_t max_size_;

  public:
    explicit ThreadSafeQueue(size_t max_size) : max_size_(max_size) {}

    bool
    push(T item,
         std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
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
        std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
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

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
};
#endif
class BufferProcessor : public aditof::V4lBufferAccessInterface {
  public:
    BufferProcessor();
    ~BufferProcessor();

  public:
    aditof::Status open();
    aditof::Status setInputDevice(VideoDev *inputVideoDev);
    aditof::Status setVideoProperties(int frameWidth, int frameHeight,
                                      int WidthInBytes, int HeightInBytes,
                                      int modeNumber);
    aditof::Status setProcessorProperties(uint8_t *iniFile,
                                          uint16_t iniFileLength,
                                          uint8_t *calData,
                                          uint16_t calDataLength, uint16_t mode,
                                          bool ispEnabled);
    aditof::Status processBuffer(uint16_t *buffer);
    TofiConfig *getTofiCongfig();
    aditof::Status getDepthComputeVersion(uint8_t &enabled);

    void startThreads();
    void stopThreads();

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

  private:
    bool m_vidPropSet;
    bool m_processorPropSet;

    uint16_t m_outputFrameWidth;
    uint16_t m_outputFrameHeight;

    TofiConfig *m_tofiConfig;
    TofiComputeContext *m_tofiComputeContext;
    TofiXYZDealiasData m_xyzDealiasData[11];

    struct v4l2_capability m_videoCap;
    struct v4l2_format m_videoFormat;
    const char *m_videoDeviceName = OUTPUT_DEVICE;

    struct VideoDev *m_inputVideoDev;
    struct VideoDev *m_outputVideoDev;

    BufferAllocator *m_bufferAllocator;

#if 0
    struct Tofi_v4l2_buffer {
        std::shared_ptr<uint8_t> data;
        size_t size = 0;
        std::shared_ptr<uint16_t> tofiBuffer;
    };


    // Thread-safe pool of empty raw frame buffers for use by capture thread
    ThreadSafeQueue<std::shared_ptr<uint8_t>> m_v4l2_input_buffer_Q;

    // Thread-safe queue to transfer captured raw frames to the process thread
    ThreadSafeQueue<Tofi_v4l2_buffer> m_capture_to_process_Q;

    // Thread-safe pool of ToFi compute output buffers (depth + AB + confidence)
    ThreadSafeQueue<std::shared_ptr<uint16_t>> m_tofi_io_Buffer_Q;

    // Thread-safe queue for frames that have been fully processed (compute done)
    ThreadSafeQueue<Tofi_v4l2_buffer> m_process_done_Q;
#endif
    uint32_t m_rawFrameBufferSize;
    uint32_t m_tofiBufferSize;

    std::thread m_captureThread;
    std::thread m_processingThread;

    std::atomic<bool> stopThreadsFlag;
    bool streamRunning = false;

    static constexpr size_t MAX_QUEUE_SIZE = 3;
    static constexpr int TIME_OUT_DELAY = 5;

    int m_maxTries = 3;

    uint8_t m_currentModeNumber;
};