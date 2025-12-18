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
#ifndef UTILS_INI_H
#define UTILS_INI_H

#include <aditof/status_definitions.h>

#include <map>
#include <string>

namespace aditof {

class UtilsIni {
  public:
    /**
     * Get key-value pairs from ini file
     * @param[in] iniFileName - the name of the ini file to be opened and parsed
     * @param[out] iniKeyValPairs - map with parameter names and their values extracted from ini file
     * @return Status
     * @see Status
    */
    static Status
    getKeyValuePairsFromIni(const std::string &iniFileName,
                            std::map<std::string, std::string> &iniKeyValPairs);

    /**
     * Get key-value pairs from a string
     * @param[in] iniStr - the string with key-value pairs
     * @param[out] iniKeyValPairs - map with parameter names and their values extracted from string
     * @return Status
     * @see Status
    */
    static Status getKeyValuePairsFromString(
        const std::string &iniStr,
        std::map<std::string, std::string> &iniKeyValPairs);
};
} // namespace aditof

#endif // UTILS_H
