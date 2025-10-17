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

#ifndef LOG_H
#define LOG_H

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>

//glog/logging.h defines this namespace
//which is required by protobuf library
namespace google {
#ifdef _MSC_VER
typedef signed __int8 int8;
typedef __int16 int16;
typedef __int32 int32;
typedef __int64 int64;

typedef unsigned __int8 uint8;
typedef unsigned __int16 uint16;
typedef unsigned __int32 uint32;
typedef unsigned __int64 uint64;
#else
typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef int64_t int64;

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;
#endif

static void InitGoogleLogging(char *val){};
} // namespace google

class Log {
  public:
    Log(const std::string &x, const std::string &file, int line) {

        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
                          now.time_since_epoch()) %
                      std::chrono::seconds(1);
        std::stringstream ss;
        ss << x << std::put_time(std::localtime(&now_time), "%Y%m%d %H:%M:%S")
           << "." << std::setfill('0') << std::setw(6) << micros.count() << " "
           << getpid() << " " << getFileName(file) << ":" << line << "] ";

        buffer << ss.str();
    }
    template <typename T>
    Log &operator<<(const T &msg) {
        buffer << msg;
        return *this;
    }

    ~Log() { std::cerr << buffer.str() << std::endl; }

    Log &operator<<(std::ostream &(*manip)(std::ostream &)) {
        // Ignore std::endl to avoid double newline
        return *this;
    }

  private:
    std::stringstream buffer;

    std::string getFileName(const std::string &path) {
        size_t pos = path.find_last_of("/");
        return (pos == std::string::npos) ? path : path.substr(pos + 1);
    }
    std::string currentDate() {
        std::time_t now_time = std::chrono::system_clock::to_time_t(
            std::chrono::system_clock::now());
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_time), "%Y%m%d");
        return ss.str();
    }
};

static int FLAGS_alsologtostderr;
static int FLAGS_logtostderr;

#define INFO "I"
#define ERROR "E"
#define WARNING "W"

#ifdef NDEBUG
#define DLOG(x) if(false) Log(x, __FILE__, __LINE__)
#else
#define DLOG(x) Log(x, __FILE__, __LINE__)
#endif

#define LOG(x) Log(x, __FILE__, __LINE__)

#endif // LOG_COUT_H