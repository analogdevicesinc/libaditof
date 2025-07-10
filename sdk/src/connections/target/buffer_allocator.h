/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, Analog Devices, Inc.
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

template <typename T>
class m_ThreadSafeQueue {
  private:
    std::queue<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable not_empty_;
    std::condition_variable not_full_;
    size_t max_size_;

  public:
    explicit m_ThreadSafeQueue(size_t max_size = 8) : max_size_(max_size) {}

    bool
    push(T item,
         std::chrono::milliseconds timeout = std::chrono::milliseconds(200)) {
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
        std::chrono::milliseconds timeout = std::chrono::milliseconds(200)) {
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

class BufferAllocator {
  private:
    BufferAllocator();
    ~BufferAllocator();

    // Allow std::shared_ptr to access the private destructor
    template <typename T, typename _Lp>
    friend class std::_Sp_counted_ptr;

  public:
    static std::shared_ptr<BufferAllocator> getInstance();
    aditof::Status allocate_queues_memory();
    void freeQueues();

    template <typename T>
    static void clearQueue(m_ThreadSafeQueue<std::shared_ptr<T>> &queue) {
        std::shared_ptr<T> tmp;
        while (queue.pop(tmp)) {
        }
    }

  public:
    static constexpr size_t RAW_WIDTH_MAX = 2048;
    static constexpr size_t RAW_HEIGHT_MAX = 3328;
    static constexpr size_t TOFI_WIDTH_MAX = 1024;
    static constexpr size_t TOFI_HEIGHT_MAX = 1024;

    struct Tofi_v4l2_buffer {
        std::shared_ptr<uint8_t> data;
        size_t size = 0;
        std::shared_ptr<uint16_t> tofiBuffer;
    };

    // Thread-safe pool of empty raw frame buffers for use by capture thread
    m_ThreadSafeQueue<std::shared_ptr<uint8_t>> m_v4l2_input_buffer_Q;

    // Thread-safe queue to transfer captured raw frames to the process thread
    m_ThreadSafeQueue<Tofi_v4l2_buffer> m_capture_to_process_Q;

    // Thread-safe pool of ToFi compute output buffers (depth + AB + confidence)
    m_ThreadSafeQueue<std::shared_ptr<uint16_t>> m_tofi_io_Buffer_Q;

    // Thread-safe queue for frames that have been fully processed (compute done)
    m_ThreadSafeQueue<Tofi_v4l2_buffer> m_process_done_Q;

    uint32_t m_rawFrameBufferSize;
    uint32_t m_tofiBufferSize;

    static constexpr size_t MAX_QUEUE_SIZE = 4;

  private:
    static std::shared_ptr<BufferAllocator> s_instance;
};