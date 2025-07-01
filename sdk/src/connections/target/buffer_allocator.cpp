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

#include "buffer_allocator.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

std::shared_ptr<BufferAllocator> BufferAllocator::s_instance = nullptr;

std::shared_ptr<BufferAllocator> BufferAllocator::getInstance() {
    if (!s_instance) {
        // Custom deleter to call private destructor
        s_instance = std::shared_ptr<BufferAllocator>(
            new BufferAllocator(), [](BufferAllocator *ptr) {
                delete ptr; // Calls private destructor
            });
        //LOG(INFO) << "Created singleton BufferAllocator at: " << static_cast<void*>(s_instance.get());
    }
    return s_instance;
}

BufferAllocator::BufferAllocator()
    : m_v4l2_input_buffer_Q(MAX_QUEUE_SIZE), m_tofi_io_Buffer_Q(MAX_QUEUE_SIZE),
      m_capture_to_process_Q(MAX_QUEUE_SIZE), m_process_done_Q(MAX_QUEUE_SIZE) {
    LOG(INFO) << "BufferAllocator initialized with MAX_QUEUE_SIZE: "
              << MAX_QUEUE_SIZE;
}

BufferAllocator::~BufferAllocator() {
    clearQueue(m_v4l2_input_buffer_Q);
    clearQueue(m_tofi_io_Buffer_Q);
    LOG(INFO) << "All queues cleared.";
}

/**
 * @function BufferAllocator::allocate_queues_memory
 *
 * Allocate buffers for the maximum mode sizes:
 * - Raw buffer: 2048x3328 = 6,821,888 bytes per buffer
 * - ToFi buffer: [depth: 1024x1024x2 + AB: 1024x1024x2 + conf: 1024x1024x4] = 8,388,608 bytes per buffer
 * Total: 3 x (6,821,888 + 8,388,608) = 45,630,288 bytes (~43.5 MB)
 *
 * @return aditof::Status    Returns OK on success, GENERIC_ERROR on allocation failure.
 */
aditof::Status BufferAllocator::allocate_queues_memory() {
    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << __func__ << " called!!";

    // Allocate raw frame buffers for the maximum size (Mode 1: 2048x3328)
    m_rawFrameBufferSize = static_cast<size_t>(RAW_WIDTH_MAX) * RAW_HEIGHT_MAX;
    for (size_t i = 0; i < MAX_QUEUE_SIZE; ++i) {
        std::shared_ptr<uint8_t> buffer;
        try {
            buffer = std::shared_ptr<uint8_t>(new uint8_t[m_rawFrameBufferSize],
                                              std::default_delete<uint8_t[]>());
        } catch (const std::bad_alloc &e) {
            LOG(ERROR) << __func__ << ": Failed to allocate raw buffer #" << i
                       << ": " << e.what();
            status = Status::GENERIC_ERROR;
            break;
        }
        if (!buffer) {
            LOG(ERROR) << __func__ << ": Raw buffer #" << i << " is null!";
            status = Status::GENERIC_ERROR;
            break;
        }
        if (!m_v4l2_input_buffer_Q.push(buffer)) {
            LOG(ERROR) << __func__ << ": Failed to push raw buffer #" << i
                       << " to m_v4l2_input_buffer_Q";
            status = Status::GENERIC_ERROR;
            break;
        }
    }

    // Allocate ToFi compute buffers for the maximum size (MP mode: 1024x1024)
    uint32_t depthSize =
        TOFI_WIDTH_MAX * TOFI_HEIGHT_MAX *
        2; /* | Depth Frame ( W * H * 2 (type: uint16_t)) |   */
    uint32_t abSize =
        depthSize; /* | AB Frame ( W * H * 2 (type: uint16_t)) |      */
    uint32_t confSize = TOFI_WIDTH_MAX * TOFI_HEIGHT_MAX *
                        4; /* | Confidance Frame ( W * H * 4 (type: float)) | */
    m_tofiBufferSize = depthSize + abSize + confSize;

    for (size_t i = 0; i < MAX_QUEUE_SIZE; ++i) {
        std::shared_ptr<uint16_t> buffer;
        try {
            buffer = std::shared_ptr<uint16_t>(
                new uint16_t[m_tofiBufferSize / sizeof(uint16_t)],
                std::default_delete<uint16_t[]>());
        } catch (const std::bad_alloc &e) {
            LOG(ERROR) << __func__ << ": Failed to allocate ToFi buffer #" << i
                       << ": " << e.what();
            status = Status::GENERIC_ERROR;
            break;
        }
        if (!buffer) {
            LOG(ERROR) << __func__ << ": ToFi buffer #" << i << " is null!";
            status = Status::GENERIC_ERROR;
            break;
        }
        if (!m_tofi_io_Buffer_Q.push(buffer)) {
            LOG(ERROR) << __func__ << ": Failed to push ToFi buffer #" << i
                       << " to m_tofi_io_Buffer_Q";
            status = Status::GENERIC_ERROR;
            break;
        }
    }

    if (status != Status::OK) {
        LOG(ERROR) << __func__ << ": Allocation failed, clearing queues";
        clearQueue(m_v4l2_input_buffer_Q);
        clearQueue(m_tofi_io_Buffer_Q);
    }
    return status;
}