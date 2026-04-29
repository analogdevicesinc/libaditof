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
#include "v4l2_buffer_manager.h"
#include "buffer_processor.h" // for VideoDev definition
#include <aditof/log.h>
#include <cerrno>
#include <cstring>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

/**
 * @brief Wrapper for ioctl that retries on EINTR with validation.
 *
 * Repeatedly calls ioctl until it succeeds or fails with an error other than EINTR.
 * Validates the file descriptor before attempting ioctl calls and retries up to 3 times.
 *
 * @param[in] fh File descriptor
 * @param[in] request ioctl request code
 * @param[in,out] arg Pointer to ioctl argument structure
 *
 * @return ioctl result: 0 on success, -1 on error (with errno set)
 */
static int xioctl(int fh, unsigned int request, void *arg) {
    int r;
    int tries = 3;

    // Validate file handle before calling ioctl
    if (fh < 0) {
        LOG(ERROR) << "xioctl called with invalid file descriptor: " << fh;
        errno = EBADF;
        return -1;
    }

    // Note: Argument validation removed - the kernel's ioctl properly validates
    // arguments and returns appropriate error codes (EFAULT, EINVAL, etc.)
    // Some V4L2 ioctls legitimately accept NULL pointers for certain operations

    do {
        r = ioctl(fh, request, arg);
    } while (--tries > 0 && r == -1 && EINTR == errno);

    if (r == -1) {
        LOG(WARNING) << "xioctl failed: fd=" << fh << " request=0x" << std::hex
                     << request << " errno=" << std::dec << errno << " ("
                     << strerror(errno) << ")"
                     << " after " << (4 - tries) << " attempts";
    }

    return r;
}

V4L2BufferManager::V4L2BufferManager(struct VideoDev *videoDevs,
                                     uint8_t numVideoDevs)
    : m_videoDevs(videoDevs), m_numVideoDevs(numVideoDevs) {}

aditof::Status V4L2BufferManager::waitForBuffer(struct VideoDev *dev) {
    fd_set fds;
    struct timeval tv;
    int r;

    if (dev == nullptr)
        dev = &m_videoDevs[0];

    FD_ZERO(&fds);
    FD_SET(dev->fd, &fds);

    tv.tv_sec = 20;
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

aditof::Status V4L2BufferManager::dequeueInternalBuffer(struct v4l2_buffer &buf,
                                                        struct VideoDev *dev) {
    using namespace aditof;
    Status status = Status::OK;

    if (dev == nullptr)
        dev = &m_videoDevs[0];

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
        LOG(WARNING) << "Not enough buffers available";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status
V4L2BufferManager::getInternalBuffer(uint8_t **buffer, uint32_t &buf_data_len,
                                     const struct v4l2_buffer &buf,
                                     struct VideoDev *dev) {
    if (dev == nullptr)
        dev = &m_videoDevs[0];

    *buffer = static_cast<uint8_t *>(dev->videoBuffers[buf.index].start);
    buf_data_len = buf.bytesused;

    return aditof::Status::OK;
}

aditof::Status V4L2BufferManager::enqueueInternalBuffer(struct v4l2_buffer &buf,
                                                        struct VideoDev *dev) {
    if (dev == nullptr)
        dev = &m_videoDevs[0];

    if (xioctl(dev->fd, VIDIOC_QBUF, &buf) == -1) {
        LOG(WARNING) << "VIDIOC_QBUF error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return aditof::Status::GENERIC_ERROR;
    }

    return aditof::Status::OK;
}

aditof::Status V4L2BufferManager::getDeviceFileDescriptor(int &fileDescriptor) {
    using namespace aditof;
    struct VideoDev *dev = &m_videoDevs[0];

    if (dev->fd != -1) {
        fileDescriptor = dev->fd;
        return Status::OK;
    }

    return Status::INVALID_ARGUMENT;
}

aditof::Status V4L2BufferManager::cleanupBuffers(struct VideoDev *dev) {
    using namespace aditof;

    if (dev == nullptr)
        dev = &m_videoDevs[0];

    if (!dev->videoBuffers) {
        return Status::OK; // Nothing to clean up
    }

    // Unmap all buffers
    for (unsigned int i = 0; i < dev->nVideoBuffers; i++) {
        if (dev->videoBuffers[i].start &&
            dev->videoBuffers[i].start != MAP_FAILED) {
            if (munmap(dev->videoBuffers[i].start,
                       dev->videoBuffers[i].length) == -1) {
                LOG(WARNING) << "munmap error for buffer " << i << ": "
                             << strerror(errno);
                // Continue cleanup despite error
            }
        }
    }

    // Free the buffer array
    free(dev->videoBuffers);
    dev->videoBuffers = nullptr;
    dev->nVideoBuffers = 0;

    LOG(INFO) << "Cleaned up video buffers for device";
    return Status::OK;
}
