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

BufferAllocator::BufferAllocator()
    : m_v4l2_input_buffer_m0_Q(MAX_QUEUE_SIZE),
      m_v4l2_input_buffer_m1_Q(MAX_QUEUE_SIZE),
      m_v4l2_input_buffer_m2_m6_Q(MAX_QUEUE_SIZE),
      m_tofi_io_Buffer_MP_Q(MAX_QUEUE_SIZE),
      m_tofi_io_Buffer_QMP_Q(MAX_QUEUE_SIZE) {

    LOG(INFO) << "BufferAllocator initialized";
}

BufferAllocator::~BufferAllocator() {

    clearQueue(m_v4l2_input_buffer_m0_Q);
    clearQueue(m_v4l2_input_buffer_m1_Q);
    clearQueue(m_v4l2_input_buffer_m2_m6_Q);
    clearQueue(m_tofi_io_Buffer_MP_Q);
    clearQueue(m_tofi_io_Buffer_QMP_Q);

    LOG(INFO) << "All queues cleared.";

#if 0
    // Flush all remaining frames from capture-to-process queue
    Tofi_v4l2_buffer frame;
    while (m_capture_to_process_Q.pop(frame)) {
        if (frame.data)
            m_v4l2_input_buffer_Q.push(frame.data);
        if (frame.tofiBuffer)
            m_tofi_io_Buffer_Q.push(frame.tofiBuffer);
    }

    // Flush all remaining frames from process-done queue
    while (m_process_done_Q.pop(frame)) {
        if (frame.data)
            m_v4l2_input_buffer_Q.push(frame.data);
        if (frame.tofiBuffer)
            m_tofi_io_Buffer_Q.push(frame.tofiBuffer);
    }

    std::shared_ptr<uint8_t> inputBuf;
    while (m_v4l2_input_buffer_Q.pop(inputBuf)) {
    }

    std::shared_ptr<uint16_t> tofiBuf;
    while (m_tofi_io_Buffer_Q.pop(tofiBuf)) {
    }
    LOG(INFO) << "freeQueues";
#endif
}

aditof::Status BufferAllocator::getBuffer(int index, std::shared_ptr<uint8_t> &buffer) {
    // Implementation here
    return aditof::Status::OK;
}


/**
 * @function BufferAllocator::allocate_queue_memory
 *
 * Allocate Total 71,583,744 bytes (~68.3 MB)
 *
 *
 * @return aditof::Status    Returns OK on success, GENERIC_ERROR on allocation failure.
 */

aditof::Status BufferAllocator::allocate_queues_memory()
{
    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << __func__ << " called!!";
    //[mode 0] - Pre-allocate raw frame buffers for the V4L2 input queue
    // Resolution: 2048 x 2560 = 5,242,880 bytes per buffer
    // Total: 3 × 5,242,880 = 15,728,640 bytes (~15.0 MB)
    m_rawFrameBufferSize_m0 = static_cast<size_t>(RAW_WIDTH_M0) * RAW_HEIGHT_M0;
    for(int i = 0; i < MAX_QUEUE_SIZE; ++i) {
        auto buffer =
            std::shared_ptr<uint8_t>(new uint8_t[m_rawFrameBufferSize_m0],
                                     std::default_delete<uint8_t[]>());
        if (!buffer) {
            LOG(ERROR) << __func__ << ": Failed to allocate raw buffer!";
            status = Status::GENERIC_ERROR;
        } else {
            LOG(INFO) << "m_v4l2_input_buffer_Q[" << i << "] address: " << static_cast<void*>(buffer.get());
        }
        m_v4l2_input_buffer_m0_Q.push(buffer);
    }

    //[mode 1] - Pre-allocate raw frame buffers for the V4L2 input queue
    // Resolution: 2048 x 3328 = 6,821,888 bytes per buffer
    // Total: 3 × 6,821,888 = 20,465,664 bytes (~19.5 MB)
    m_rawFrameBufferSize_m1 = static_cast<size_t>(RAW_WIDTH_M1) * RAW_HEIGHT_M1;
    for(int i = 0; i < MAX_QUEUE_SIZE; ++i) {
        auto buffer =
            std::shared_ptr<uint8_t>(new uint8_t[m_rawFrameBufferSize_m1],
                                     std::default_delete<uint8_t[]>());
        if (!buffer) {
            LOG(ERROR) << __func__ << ": Failed to allocate raw buffer!";
            status = Status::GENERIC_ERROR;
        } else {
            LOG(INFO) << "m_v4l2_input_buffer_Q[" << i << "] address: " << static_cast<void*>(buffer.get());
        }
        m_v4l2_input_buffer_m1_Q.push(buffer);
    }

    //[mode 2 to mode 6] - Pre-allocate raw frame buffers for the V4L2 input queue (mode 4 is not supported)
    // Resolution: 2560 x 512 = 1,310,720 bytes per buffer
    // Total: 3 × 1,310,720 = 3,932,160 bytes (~3.75 MB)
    m_rawFrameBufferSize_m2_m6 = static_cast<size_t>(RAW_WIDTH_M2_M6) * RAW_HEIGHT_M2_M6;
    {
        for(int i = 0; i < MAX_QUEUE_SIZE; ++i) {
            auto buffer =
                std::shared_ptr<uint8_t>(new uint8_t[m_rawFrameBufferSize_m2_m6],
                                         std::default_delete<uint8_t[]>());
            if (!buffer) {
                LOG(ERROR) << __func__ << ": Failed to allocate raw buffer!";
                status = Status::GENERIC_ERROR;
            } else {
                LOG(INFO) << "m_v4l2_input_buffer_Q[" << i << "] address: " << static_cast<void*>(buffer.get());
            }
            m_v4l2_input_buffer_m2_m6_Q.push(buffer);
        }
    }

    //[MP Mode] - Pre-allocate intput/output buffers for the tofi compute queue.
    // [ depth = 1024×1024×2 = 2,097,152 | ab = 2,097,152 | conf = 1024×1024×4 = 4,194,304 ]
    // Total = 8,388,608 bytes per buffer
    // Allocated: 3 × 8,388,608 = 25,165,824 bytes (~24 MB)
    uint32_t depthSize =
        TOFI_WIDTH_MP * TOFI_HEIGHT_MP * 2;     /* | Depth Frame ( W * H * 2 (type: uint16_t)) |   */
    uint32_t abSize = depthSize;                /* | AB Frame ( W * H * 2 (type: uint16_t)) |      */
    uint32_t confSize =
        TOFI_WIDTH_MP * TOFI_HEIGHT_MP * 4;   /* | Confidance Frame ( W * H * 4 (type: float)) | */
    m_tofiBufferSize_mp = depthSize + abSize + confSize;

    for(int i = 0; i < MAX_QUEUE_SIZE; ++i) {
        auto buffer = std::shared_ptr<uint16_t>(
            new uint16_t[m_tofiBufferSize_mp], std::default_delete<uint16_t[]>());
        if (!buffer) {
            LOG(ERROR) << "setVideoProperties: Failed to allocate ToFi buffer!";
            return aditof::Status::GENERIC_ERROR;
        } else {
            LOG(INFO) << "m_tofi_io_Buffer_Q[" << i << "] address: " << static_cast<void*>(buffer.get());
        }
        m_tofi_io_Buffer_MP_Q.push(buffer);
    }

    //[QMP Mode] - Pre-allocate intput/output buffers for the tofi compute queue.
    // [ depth = 512×512×2 = 524,288 | ab = 524,288 | conf = 512×512×4 = 1,048,576 ]
    // Total = 2,097,152 bytes per buffer
    // Allocated: 3 × 2,097,152 = 6,291,456 bytes (~6 MB)
    uint32_t depthSize_qmp =
        TOFI_WIDTH_QMP * TOFI_HEIGHT_QMP * 2;   /* | Depth Frame ( W * H * 2 (type: uint16_t)) |   */
    uint32_t abSize_qmp = depthSize_qmp;                /* | AB Frame ( W * H * 2 (type: uint16_t)) |      */
    uint32_t confSize_qmp =
        TOFI_WIDTH_QMP * TOFI_HEIGHT_QMP * 4;   /* | Confidance Frame ( W * H * 4 (type: float)) | */

    m_tofiBufferSize_qmp = depthSize_qmp + abSize_qmp + confSize_qmp;

    for(int i = 0; i < MAX_QUEUE_SIZE; ++i) {
        auto buffer = std::shared_ptr<uint16_t>(
            new uint16_t[m_tofiBufferSize_qmp], std::default_delete<uint16_t[]>());
        if (!buffer) {
            LOG(ERROR) << "setVideoProperties: Failed to allocate ToFi buffer!";
            return aditof::Status::GENERIC_ERROR;
        } else {
            LOG(INFO) << "m_tofi_io_Buffer_Q[" << i << "] address: " << static_cast<void*>(buffer.get());
        }
        m_tofi_io_Buffer_QMP_Q.push(buffer);
    }

    LOG(INFO) << __func__ << " Done!!";
    return status;
}