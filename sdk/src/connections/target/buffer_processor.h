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

class BufferProcessor : public aditof::V4lBufferAccessInterface {
  public:
    BufferProcessor();
    ~BufferProcessor();

  public:
    aditof::Status open();
    aditof::Status setInputDevice(VideoDev *inputVideoDev);
    aditof::Status setVideoProperties(int frameWidth, int frameHeight,
                                      int WidthInBytes, int HeightInBytes,
                                      int modeNumber, uint8_t bitsInAB,
                                      uint8_t bitsInConf);
    aditof::Status setProcessorProperties(uint8_t *iniFile,
                                          uint16_t iniFileLength,
                                          uint8_t *calData,
                                          uint16_t calDataLength, uint16_t mode,
                                          bool ispEnabled);
    aditof::Status processBuffer(uint16_t *buffer);
    TofiConfig *getTofiCongfig() const;
    aditof::Status getDepthComputeVersion(uint8_t &enabled) const;

    void startThreads();
    void stopThreads();
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
    void calculateFrameSize(uint8_t &bitsInAB, uint8_t &bitsInConf);

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

    uint32_t m_rawFrameBufferSize;
    uint32_t m_tofiBufferSize;

    std::thread m_captureThread;
    std::thread m_processingThread;

    std::atomic<bool> stopThreadsFlag;
    bool streamRunning = false;

    const size_t MAX_QUEUE_SIZE = 3;
    const static constexpr int TIME_OUT_DELAY = 5;

    int m_maxTries = 3;

    uint8_t m_currentModeNumber;

  public:
    // Stream record and playback support
    aditof::Status startRecording(std::string &fileName, uint8_t *parameters,
                                  uint32_t paramSize);
    aditof::Status stopRecording();

  private:
    aditof::Status automaticStop();
    aditof::Status writeFrame(uint8_t *buffer, uint32_t bufferSize);
    enum StreamType { ST_STOP, ST_RECORD, ST_PLAYBACK } m_state;
    const std::string m_folder_path_folder = "./media";
    std::string m_folder_path;
    std::ofstream m_stream_file_out;
    std::ifstream m_stream_file_in;
    std::string m_stream_file_name;
    uint32_t m_frame_count;
};