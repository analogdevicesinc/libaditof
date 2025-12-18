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
#ifndef GPIO_H
#define GPIO_H

#include <string>

namespace aditof {

enum api_Values {
    API_NOT_DEFINED,
    FIND_SENSORS,
    OPEN,
    START,
    STOP,
    GET_AVAILABLE_MODES,
    GET_MODE_DETAILS,
    SET_MODE,
    SET_MODE_BY_INDEX,
    GET_FRAME,
    GET_AVAILABLE_CONTROLS,
    SET_CONTROL,
    GET_CONTROL,
    INIT_TARGET_DEPTH_COMPUTE,
    ADSD3500_READ_CMD,
    ADSD3500_WRITE_CMD,
    ADSD3500_READ_PAYLOAD_CMD,
    ADSD3500_READ_PAYLOAD,
    ADSD3500_WRITE_PAYLOAD_CMD,
    ADSD3500_WRITE_PAYLOAD,
    ADSD3500_GET_STATUS,
    GET_INTERRUPTS,
    HANG_UP,
    GET_DEPTH_COMPUTE_PARAM,
    SET_DEPTH_COMPUTE_PARAM,
    SET_SENSOR_CONFIGURATION,
    GET_INI_ARRAY
};

enum protocols { PROTOCOL_EXAMPLE, PROTOCOL_COUNT };

class Gpio {
  public:
    Gpio(const std::string &charDeviceName, int gpioNumber);
    int openForRead();
    int openForWrite();
    int close();
    int writeValue(int value);
    int readValue(int &value);

  private:
    int open(int openType);

  private:
    std::string m_charDevName;
    int m_lineHandle;
    int m_gpioNumber;
};

} // namespace aditof

#endif /* GPIO_H */
