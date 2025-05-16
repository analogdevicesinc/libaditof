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

#include "v4l_buffer_access_interface.h"

#include "tofi/tofi_compute.h"
#include "tofi/tofi_config.h"
#include "tofi/tofi_util.h"
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

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

class BufferProcessor : public aditof::V4lBufferAccessInterface {
  public:
    BufferProcessor();
    ~BufferProcessor();

  public:
    aditof::Status open();
    aditof::Status setInputDevice(VideoDev *inputVideoDev);
    aditof::Status setVideoProperties(int frameWidth, int frameHeight);
    aditof::Status setProcessorProperties(uint8_t *iniFile,
                                          uint16_t iniFileLength,
                                          uint8_t *calData,
                                          uint16_t calDataLength, uint16_t mode,
                                          bool ispEnabled);
    aditof::Status processBuffer(uint16_t *buffer);
    TofiConfig *getTofiCongfig();
    aditof::Status getDepthComputeVersion(uint8_t &enabled);
    aditof::Status captureFrames(std::vector<uint8_t> &buffer);
    aditof::Status processFrames(uint8_t *captured_buffer, uint16_t *buffer);
    void storeProcessedFrame(uint16_t *processedFrame, size_t size);
    void startThreads();
    void stopThread();
    bool getProcessedFrame(uint16_t *frameOut);

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

  private:
    bool m_vidPropSet;
    bool m_processorPropSet;

    uint16_t m_outputFrameWidth;
    uint16_t m_outputFrameHeight;

    uint16_t *m_processedBuffer;
    uint8_t *pdata_user_space = nullptr;
    static const int BUFFER_SIZE = 10;
    std::array<std::vector<uint16_t>, BUFFER_SIZE> processedFramesBuffer;
    int bufferWriteIndex = 0;
    std::mutex bufferMutex;
    std::mutex mtx;
    std::queue<std::vector<uint8_t>> frameQueue;
    std::mutex queueMutex;
    std::condition_variable cvNotFull;
    std::condition_variable cvNotEmpty;
    std::condition_variable cvProcessedBufferNotFull;
    std::condition_variable cvProcessedBufferNotEmpty;
    std::mutex processedBufferMutex;

    bool frameCaptured = false;
    bool frameProcessed = true; // Start as true so capture can begin
    std::atomic<bool> stopThreads{false};
    int bufferCount = 0;

    std::thread captureThread;
    std::thread processThread;

    TofiConfig *m_tofiConfig;
    TofiComputeContext *m_tofiComputeContext;
    TofiXYZDealiasData m_xyzDealiasData[11];

    struct v4l2_capability m_videoCap;
    struct v4l2_format m_videoFormat;
    const char *m_videoDeviceName = OUTPUT_DEVICE;

    struct VideoDev *m_inputVideoDev;
    struct VideoDev *m_outputVideoDev;
};
