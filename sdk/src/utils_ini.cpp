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
#include "utils_ini.h"
#include <aditof/log.h>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#ifdef _WIN32
#include <Windows.h>
#elif __unix
#include <unistd.h>
#endif

using namespace std;
using namespace aditof;

Status UtilsIni::getKeyValuePairsFromIni(const string &iniFileName,
                                         map<string, string> &iniKeyValPairs) {

    ifstream iniStream(iniFileName);
    if (!iniStream.is_open()) {
        LOG(ERROR) << "Failed to open: " << iniFileName;
        return Status::UNREACHABLE;
    }

    iniKeyValPairs.clear();

    string line;
    while (getline(iniStream, line)) {
        size_t equalPos = line.find('=');
        if (equalPos == string::npos) {
            LOG(WARNING) << "Unexpected format on this line:\n"
                         << line << "\nExpecting 'key=value' format";
            continue;
        }
        string key = line.substr(0, equalPos);
        string value = line.substr(equalPos + 1);
        if (!value.empty()) {
            iniKeyValPairs.emplace(key, value);
        } else {
            LOG(WARNING) << "No value found for parameter: " << key;
        }
    }

    iniStream.close();

    return Status::OK;
}

Status UtilsIni::getKeyValuePairsFromString(
    const std::string &iniStr,
    std::map<std::string, std::string> &iniKeyValPairs) {
    iniKeyValPairs.clear();
    stringstream ss(iniStr);
    string line;
    char delimiter = '\n';
    while (getline(ss, line, delimiter)) {
        size_t equalPos = line.find('=');
        if (equalPos == string::npos) {
            LOG(WARNING) << "Unexpected format on this line:\n"
                         << line << "\nExpecting 'key=value' format";
            continue;
        }
        string key = line.substr(0, equalPos);
        string value = line.substr(equalPos + 1);
        if (!value.empty()) {
            iniKeyValPairs.emplace(key, value);
        } else {
            LOG(WARNING) << "No value found for parameter: " << key;
        }
    }
    return Status::OK;
}
