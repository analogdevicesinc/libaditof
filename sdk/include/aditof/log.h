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
#ifdef USE_GLOG

#include <glog/logging.h>

#else

#ifndef LOG_H
#define LOG_H

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#ifdef _WIN32
#include <process.h>
#define getpid _getpid
#else
#include <unistd.h>
#endif

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

[[maybe_unused]] static inline void InitGoogleLogging(char *val) { (void)val; }
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

[[maybe_unused]] static int FLAGS_alsologtostderr = 0;
[[maybe_unused]] static int FLAGS_logtostderr = 0;

#define INFO "I"
#define ERROR "E"
#define WARNING "W"

#ifdef NDEBUG
#define DLOG(x)                                                                \
    if (false)                                                                 \
    Log(x, __FILE__, __LINE__)
#else
#define DLOG(x) Log(x, __FILE__, __LINE__)
#endif

#define LOG(x) Log(x, __FILE__, __LINE__)

#endif // LOG_COUT_H

#endif // USE_GLOG