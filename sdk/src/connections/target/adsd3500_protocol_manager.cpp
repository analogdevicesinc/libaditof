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
#include "adsd3500_protocol_manager.h"
#include "buffer_processor.h" // for VideoDev definition
#include "platform/platform_impl.h"
#include <aditof/log.h>
#include <cstring>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

// Protocol constants
#define ADSD3500_BURST_CMD_HEADER_SIZE 15
#define ADSD3500_BURST_RESPONSE_HEADER_SIZE 3
#define ADSD3500_STATUS_READ_DELAY_US 2000
#define ADSD3500_BURST_CMD_SHORT_DELAY_US 1000
#define ADSD3500_BURST_CMD_LONG_DELAY_US 5000
#define ADSD3500_PAYLOAD_READ_DELAY_US 30000
#define ADSD3500_PAYLOAD_WRITE_DELAY_US 100000

#define V4L2_CID_AD_DEV_CHIP_CONFIG                                            \
    (aditof::platform::Platform::getInstance().getV4L2ChipConfigControlId())

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

Adsd3500ProtocolManager::Adsd3500ProtocolManager(
    struct VideoDev *videoDevs,
    std::array<uint8_t, ADSD3500_CTRL_PACKET_SIZE> &ctrlBuf,
    std::recursive_mutex &mutex)
    : m_videoDevs(videoDevs), m_ctrlBuf(ctrlBuf), m_mutex(mutex) {}

aditof::Status
Adsd3500ProtocolManager::adsd3500_read_cmd(uint16_t cmd, uint16_t *data,
                                           unsigned int usDelay) {
    using namespace aditof;
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    struct VideoDev *dev = &m_videoDevs[0];
    Status status = Status::OK;
    struct v4l2_ext_control extCtrl;
    struct v4l2_ext_controls extCtrls;
    uint8_t *buf = m_ctrlBuf.data();
    memset(&extCtrl, 0, sizeof(struct v4l2_ext_control));
    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 1;
    buf[1] = 0;
    buf[2] = 2;
    buf[3] = uint8_t(cmd >> 8);
    buf[4] = uint8_t(cmd & 0xFF);
    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    buf[0] = 0;
    buf[1] = 0;
    buf[2] = 2;

    extCtrl.p_u8 = buf;

    usleep(usDelay);

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    if (xioctl(dev->sfd, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not get control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    *data = (uint16_t)(extCtrl.p_u8[3] << 8) + (uint16_t)(extCtrl.p_u8[4]);

    return status;
}

aditof::Status
Adsd3500ProtocolManager::adsd3500_write_cmd(uint16_t cmd, uint16_t data,
                                            unsigned int usDelay) {
    using namespace aditof;
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    struct VideoDev *dev = &m_videoDevs[0];
    Status status = Status::OK;
    struct v4l2_ext_control extCtrl;
    struct v4l2_ext_controls extCtrls;
    uint8_t *buf = m_ctrlBuf.data();
    memset(&extCtrl, 0, sizeof(struct v4l2_ext_control));
    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 1;
    buf[1] = 0;
    buf[2] = 4;
    buf[3] = uint8_t(cmd >> 8);
    buf[4] = uint8_t(cmd & 0xFF);
    buf[5] = uint8_t(data >> 8);
    buf[6] = uint8_t(data & 0xFF);
    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    if (usDelay)
        usleep(usDelay);

    return status;
}

aditof::Status Adsd3500ProtocolManager::adsd3500_read_payload_cmd(
    uint32_t cmd, uint8_t *readback_data, uint16_t payload_len) {
    using namespace aditof;
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    struct VideoDev *dev = &m_videoDevs[0];
    Status status = Status::OK;

    //switch to burst mode
    uint32_t switchCmd = 0x0019;
    uint16_t switchPayload = 0x0000;

    status = adsd3500_write_cmd(switchCmd, switchPayload);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to switch to burst mode!";
        return status;
    }
    struct v4l2_ext_control extCtrl;
    struct v4l2_ext_controls extCtrls;
    uint8_t *buf = m_ctrlBuf.data();
    memset(buf, 0, ADSD3500_CTRL_PACKET_SIZE);
    memset(&extCtrl, 0, sizeof(struct v4l2_ext_control));
    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;

    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 0x01;
    buf[1] = 0x00;
    buf[2] = 0x10;

    buf[3] = 0xAD;
    buf[6] = uint8_t(cmd & 0xFF);

    uint32_t checksum = 0;
    for (int i = 0; i < 7; i++) {
        checksum += buf[i + 4];
    }
    memcpy(buf + 11, &checksum, 4);

    // Validate buffer has room for command header + initial readback byte (used as mode/param)
    // Command packet structure: 15 byte header + payload data
    if (1 > ADSD3500_CTRL_PACKET_SIZE - ADSD3500_BURST_CMD_HEADER_SIZE) {
        LOG(ERROR) << "Command buffer too small for burst command header";
        return Status::INVALID_ARGUMENT;
    }

    // Validate the actual payload length that will be read back fits in response packet
    // Response packet structure: 3 byte header + payload data
    if (payload_len >
        ADSD3500_CTRL_PACKET_SIZE - ADSD3500_BURST_RESPONSE_HEADER_SIZE) {
        LOG(ERROR) << "Payload length " << payload_len << " exceeds maximum "
                   << (ADSD3500_CTRL_PACKET_SIZE -
                       ADSD3500_BURST_RESPONSE_HEADER_SIZE);
        return Status::INVALID_ARGUMENT;
    }

    // Copy initial readback byte (typically contains mode or parameter to read)
    memcpy(buf + ADSD3500_BURST_CMD_HEADER_SIZE, readback_data, 1);
    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    if (cmd == 0x13)
        usleep(ADSD3500_BURST_CMD_SHORT_DELAY_US);
    else if (cmd == 0x19)
        usleep(ADSD3500_BURST_CMD_LONG_DELAY_US);

    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 0x00;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);

    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    if (xioctl(dev->sfd, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not get control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    if (payload_len >
        ADSD3500_CTRL_PACKET_SIZE - ADSD3500_BURST_RESPONSE_HEADER_SIZE) {
        LOG(ERROR) << "Payload length " << payload_len << " exceeds maximum "
                   << (ADSD3500_CTRL_PACKET_SIZE -
                       ADSD3500_BURST_RESPONSE_HEADER_SIZE);
        return Status::INVALID_ARGUMENT;
    }

    // Extract payload from response packet (skip 3-byte header)
    memcpy(readback_data, extCtrl.p_u8 + ADSD3500_BURST_RESPONSE_HEADER_SIZE,
           payload_len);

    //If we use the read ccb command we need to keep adsd3500 in burst mode
    if (cmd == 0x13) {
        return status;
    }

    //switch to standard mode
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    uint8_t switchBuf[] = {0x01, 0x00, 0x10, 0xAD, 0x00, 0x00, 0x10,
                           0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00};

    memcpy(extCtrl.p_u8, switchBuf, 19);

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << " (switch to standard mode)"
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status
Adsd3500ProtocolManager::adsd3500_read_payload(uint8_t *payload,
                                               uint16_t payload_len) {
    using namespace aditof;
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    struct VideoDev *dev = &m_videoDevs[0];
    Status status = Status::OK;
    struct v4l2_ext_control extCtrl;
    struct v4l2_ext_controls extCtrls;
    uint8_t *buf = m_ctrlBuf.data();
    memset(buf, 0, ADSD3500_CTRL_PACKET_SIZE);
    memset(&extCtrl, 0, sizeof(struct v4l2_ext_control));
    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;

    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 0x00;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);

    extCtrl.p_u8 = buf;

    usleep(ADSD3500_PAYLOAD_READ_DELAY_US);

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " to read payload with length: " << payload_len
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    if (xioctl(dev->sfd, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not get control: 0x" << std::hex << extCtrl.id
                     << " to read payload with length: " << payload_len
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    if (payload_len >
        ADSD3500_CTRL_PACKET_SIZE - ADSD3500_BURST_RESPONSE_HEADER_SIZE) {
        LOG(ERROR) << "Payload length " << payload_len << " exceeds maximum "
                   << (ADSD3500_CTRL_PACKET_SIZE -
                       ADSD3500_BURST_RESPONSE_HEADER_SIZE);
        return Status::INVALID_ARGUMENT;
    }

    memcpy(payload, extCtrl.p_u8 + ADSD3500_BURST_RESPONSE_HEADER_SIZE,
           payload_len);

    return status;
}

aditof::Status Adsd3500ProtocolManager::adsd3500_write_payload_cmd(
    uint32_t cmd, uint8_t *payload, uint16_t payload_len) {
    using namespace aditof;
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    struct VideoDev *dev = &m_videoDevs[0];
    Status status = Status::OK;

    //switch to burst mode
    uint32_t switchCmd = 0x0019;
    uint16_t switchPayload = 0x0000;

    status = adsd3500_write_cmd(switchCmd, switchPayload);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to switch to burst mode!";
        return status;
    }
    struct v4l2_ext_control extCtrl;
    struct v4l2_ext_controls extCtrls;
    uint8_t *buf = m_ctrlBuf.data();
    memset(&extCtrl, 0, sizeof(struct v4l2_ext_control));
    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    payload_len += 16;
    buf[0] = 0x01;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);

    payload_len -= 16;
    buf[3] = 0xAD;
    buf[4] = uint8_t(payload_len >> 8);
    buf[5] = uint8_t(payload_len & 0xFF);
    buf[6] = uint8_t(cmd & 0xFF);

    uint32_t checksum = 0;
    for (int i = 0; i < 7; i++) {
        checksum += buf[i + 4];
    }
    memcpy(buf + 11, &checksum, 4);

    // Validate payload fits in command packet after header
    if (payload_len >
        ADSD3500_CTRL_PACKET_SIZE - ADSD3500_BURST_CMD_HEADER_SIZE) {
        LOG(ERROR) << "Payload length " << payload_len << " exceeds maximum "
                   << (ADSD3500_CTRL_PACKET_SIZE -
                       ADSD3500_BURST_CMD_HEADER_SIZE);
        return Status::INVALID_ARGUMENT;
    }

    memcpy(buf + ADSD3500_BURST_CMD_HEADER_SIZE, payload, payload_len);
    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    //switch to standard mode
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    uint8_t switchBuf[] = {0x01, 0x00, 0x10, 0xAD, 0x00, 0x00, 0x10,
                           0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00};

    memcpy(extCtrl.p_u8, switchBuf, 19);

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << " (switch to standard mode)"
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status
Adsd3500ProtocolManager::adsd3500_write_payload(uint8_t *payload,
                                                uint16_t payload_len) {
    using namespace aditof;
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    struct VideoDev *dev = &m_videoDevs[0];
    Status status = Status::OK;
    struct v4l2_ext_control extCtrl;
    struct v4l2_ext_controls extCtrls;
    uint8_t *buf = m_ctrlBuf.data();
    memset(&extCtrl, 0, sizeof(struct v4l2_ext_control));
    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 1;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);

    // Validate payload fits after response header
    if (payload_len >
        ADSD3500_CTRL_PACKET_SIZE - ADSD3500_BURST_RESPONSE_HEADER_SIZE) {
        LOG(ERROR) << "Payload length " << payload_len << " exceeds maximum "
                   << (ADSD3500_CTRL_PACKET_SIZE -
                       ADSD3500_BURST_RESPONSE_HEADER_SIZE);
        return Status::INVALID_ARGUMENT;
    }

    memcpy(buf + ADSD3500_BURST_RESPONSE_HEADER_SIZE, payload, payload_len);
    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " to write payload with length: " << payload_len
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    usleep(ADSD3500_PAYLOAD_WRITE_DELAY_US);

    return status;
}
