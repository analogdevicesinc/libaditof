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

/**
 * @brief Parses an INI configuration file and extracts key-value pairs.
 *
 * Reads the specified INI file and parses it line by line, extracting key-value
 * pairs in the format "key=value". Each line is expected to contain one key-value
 * pair separated by an equals sign. Lines with unexpected format or missing values
 * are logged as warnings and skipped. The resulting map is cleared before parsing.
 *
 * @param iniFileName The path to the INI file to parse.
 * @param iniKeyValPairs Output map where parsed key-value pairs will be stored.
 *                       The map is cleared at the beginning of the function.
 * @return Status::OK on success, Status::UNREACHABLE if the file cannot be opened.
 *
 * @note Lines without an equals sign or with empty values are skipped with warnings.
 */
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

/**
 * @brief Parses an INI-formatted string and extracts key-value pairs.
 *
 * Processes the provided string as if it were INI file content, parsing it
 * line by line to extract key-value pairs in the format "key=value". Lines
 * are delimited by newline characters. Lines with unexpected format or missing
 * values are logged as warnings and skipped. The resulting map is cleared
 * before parsing.
 *
 * @param iniStr The INI-formatted string to parse (lines separated by '\n').
 * @param iniKeyValPairs Output map where parsed key-value pairs will be stored.
 *                       The map is cleared at the beginning of the function.
 * @return Status::OK on success (no failure conditions in current implementation).
 *
 * @note Lines without an equals sign or with empty values are skipped with warnings.
 */
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
