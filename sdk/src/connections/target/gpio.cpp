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
#include "gpio.h"
#include <aditof/log.h>

#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace aditof;
Gpio::Gpio(const std::string &charDeviceName, int gpioNumber)
    : m_charDevName{charDeviceName}, m_lineHandle{-1},
      m_gpioNumber(gpioNumber) {}

int Gpio::open(int openType) {
    int ret;
    int fd;

    fd = ::open(m_charDevName.c_str(), O_RDONLY);
    if (fd <= 0) {
        LOG(ERROR) << "Failed to open gpio!";
        return errno;
    }

    struct gpiohandle_request request;
    request.lineoffsets[0] = m_gpioNumber;
    request.flags = openType;
    request.lines = 1;
    ret = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &request);
    ::close(fd);
    if (ret == -1) {
        LOG(ERROR) << "ioctl failed with error: " << errno;
        return errno;
    }
    m_lineHandle = request.fd;

    return ret;
}

int Gpio::openForWrite() { return this->open(GPIOHANDLE_REQUEST_OUTPUT); }

int Gpio::openForRead() { return this->open(GPIOHANDLE_REQUEST_INPUT); }

int Gpio::close() {
    int ret = 0;

    if (m_lineHandle != -1) {
        ret = ::close(m_lineHandle);
        m_lineHandle = -1;
    }

    return ret;
}

int Gpio::readValue(int &value) {
    struct gpiohandle_data data;
    int ret;

    if (m_lineHandle == -1) {
        LOG(ERROR) << "The Gpio object is not initialized!";
        return -EBADFD;
    }

    data.values[0] = value;
    ret = ioctl(m_lineHandle, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);
    if (ret == -1) {
        return errno;
    }
    value = data.values[0];

    return ret;
}

int Gpio::writeValue(int value) {
    struct gpiohandle_data data;
    int ret;

    if (m_lineHandle == -1) {
        LOG(ERROR) << "The Gpio object is not initialized!";
        return -EBADFD;
    }

    data.values[0] = value;
    ret = ioctl(m_lineHandle, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
    if (ret == -1) {
        return errno;
    }

    return ret;
}
