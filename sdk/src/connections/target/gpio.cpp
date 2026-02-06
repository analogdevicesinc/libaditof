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

/**
 * @brief Constructs a Gpio object for a specific GPIO line.
 *
 * Initializes the GPIO interface with the character device path and GPIO line number.
 * The GPIO must be opened (openForRead or openForWrite) before use.
 *
 * @param[in] charDeviceName Path to the GPIO character device (e.g., "/dev/gpiochip0")
 * @param[in] gpioNumber GPIO line number (offset) within the chip
 */
Gpio::Gpio(const std::string &charDeviceName, int gpioNumber)
    : m_charDevName{charDeviceName}, m_lineHandle{-1},
      m_gpioNumber(gpioNumber) {}

/**
 * @brief Opens the GPIO line with the specified access mode.
 *
 * Opens the GPIO character device and requests a line handle with the specified flags
 * (input or output). Uses ioctl GPIO_GET_LINEHANDLE_IOCTL to obtain the line handle.
 *
 * @param[in] openType GPIO handle request flags (GPIOHANDLE_REQUEST_INPUT or GPIOHANDLE_REQUEST_OUTPUT)
 *
 * @return 0 on success, errno on failure
 */
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

/**
 * @brief Opens the GPIO line for output (write) operations.
 *
 * Convenience method that opens the GPIO with GPIOHANDLE_REQUEST_OUTPUT flag.
 *
 * @return 0 on success, errno on failure
 */
int Gpio::openForWrite() { return this->open(GPIOHANDLE_REQUEST_OUTPUT); }

/**
 * @brief Opens the GPIO line for input (read) operations.
 *
 * Convenience method that opens the GPIO with GPIOHANDLE_REQUEST_INPUT flag.
 *
 * @return 0 on success, errno on failure
 */
int Gpio::openForRead() { return this->open(GPIOHANDLE_REQUEST_INPUT); }

/**
 * @brief Closes the GPIO line handle.
 *
 * Releases the GPIO line handle obtained during open. After closing, the GPIO
 * must be reopened before it can be used again.
 *
 * @return 0 on success, error code on failure
 */
int Gpio::close() {
    int ret = 0;

    if (m_lineHandle != -1) {
        ret = ::close(m_lineHandle);
        m_lineHandle = -1;
    }

    return ret;
}

/**
 * @brief Reads the current value of the GPIO line.
 *
 * Reads the logic level of the GPIO line using ioctl GPIOHANDLE_GET_LINE_VALUES_IOCTL.
 * The GPIO must be opened for reading before calling this function.
 *
 * @param[out] value Variable to receive the GPIO value (0 for low, 1 for high)
 *
 * @return 0 on success, -EBADFD if GPIO not initialized, errno on ioctl failure
 */
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

/**
 * @brief Writes a value to the GPIO line.
 *
 * Sets the logic level of the GPIO line using ioctl GPIOHANDLE_SET_LINE_VALUES_IOCTL.
 * The GPIO must be opened for writing before calling this function.
 *
 * @param[in] value GPIO value to write (0 for low, 1 for high)
 *
 * @return 0 on success, -EBADFD if GPIO not initialized, errno on ioctl failure
 */
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
