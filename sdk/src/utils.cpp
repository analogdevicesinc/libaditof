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
#include <iostream>
#include <string>

#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
#include <libgen.h> // for dirname
#include <limits.h>
#include <unistd.h>
#endif
#include "aditof/utils.h"

#include <sys/stat.h>
#include <sys/types.h>
#ifdef _WIN32
#include <direct.h>
#endif
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>

using namespace std;
using namespace aditof;

const std::string recordingFolder = "recordings/";

/**
 * @brief Splits a string into tokens based on a delimiter character.
 *
 * Parses the input string and separates it into substrings (tokens) wherever
 * the delimiter character is found. All tokens, including the last one after
 * the final delimiter, are appended to the output vector. The output vector
 * is not cleared before appending.
 *
 * @param s The input string to split.
 * @param delimiter The character to use as a separator.
 * @param tokens Output vector where the extracted tokens will be appended.
 *
 * @note The tokens vector is not cleared; new tokens are appended to it.
 */
void Utils::splitIntoTokens(const string &s, const char delimiter,
                            vector<string> &tokens) {
    string::size_type start = 0;
    for (string::size_type end = 0;
         (end = s.find(delimiter, end)) != string::npos; ++end) {
        tokens.push_back(s.substr(start, end - start));
        start = end + 1;
    }
    tokens.push_back(s.substr(start));
}

/**
 * @brief Retrieves the directory path of the currently running executable.
 *
 * Determines the folder containing the currently executing binary. This is
 * useful for locating configuration files, resources, or other assets
 * relative to the executable location.
 *
 * @return The absolute path to the executable's directory. Returns "." (current
 *         directory) if the path cannot be determined or on unsupported platforms.
 *
 * @note Platform-specific implementations:
 *       - Windows: Uses GetModuleFileNameA()
 *       - Linux: Reads /proc/self/exe symlink
 *       - Other platforms: Returns "."
 */
std::string Utils::getExecutableFolder() {
#if defined(_WIN32)
    char path[MAX_PATH];
    GetModuleFileNameA(nullptr, path, MAX_PATH);
    std::string fullPath(path);
    size_t pos = fullPath.find_last_of("\\/");
    return (pos != std::string::npos) ? fullPath.substr(0, pos) : ".";
#elif defined(__linux__)
    char path[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", path, PATH_MAX);
    if (count == -1)
        return ".";
    path[count] = '\0';
    std::string fullPath(path);
    size_t pos = fullPath.find_last_of("/\\");
    return (pos != std::string::npos) ? fullPath.substr(0, pos) : ".";
#else
    return "."; // Unsupported platform
#endif
}

/**
 * @brief Checks if a folder exists at the specified path.
 *
 * Tests whether the given path points to an existing directory in the filesystem.
 * Uses platform-agnostic stat() to check both existence and directory status.
 *
 * @param path The filesystem path to check.
 * @return true if the path exists and is a directory, false otherwise.
 */
bool Utils::folderExists(const std::string &path) {
    struct stat info;
    return (stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR));
}

/**
 * @brief Creates a new folder at the specified path.
 *
 * Attempts to create a directory at the given path. If the directory already
 * exists, the function returns true (success). Uses platform-specific APIs
 * to create the folder with appropriate permissions.
 *
 * @param path The filesystem path where the folder should be created.
 * @return true if the folder was created or already exists, false if creation
 *         failed for any other reason.
 *
 * @note On Linux/macOS, the folder is created with permissions 0755 (rwxr-xr-x).
 */
bool Utils::createFolder(const std::string &path) {
#ifdef _WIN32
    return _mkdir(path.c_str()) == 0 || errno == EEXIST;
#else
    return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
#endif
}

/**
 * @brief Generates a unique filename with timestamp and random component.
 *
 * Creates a filename combining a prefix, UTC timestamp, random hexadecimal
 * suffix, and file extension. The format is:
 * "<prefix>YYYYMMDD_HHMMSS_<8-digit-hex><extension>"
 *
 * This ensures filenames are unique and sortable by creation time. The random
 * component further reduces collision probability when multiple files are
 * created in quick succession.
 *
 * @param prefix The filename prefix (e.g., "frame_", "snapshot_").
 * @param extension The file extension including the dot (e.g., ".bin", ".jpg").
 * @return A unique filename string in the format described above.
 *
 * @note Uses UTC time to ensure consistency across time zones.
 *
 * Example output: "frame_20260206_143052_a3f5b8c1.bin"
 */
std::string Utils::generateFileName(const std::string &prefix,
                                    const std::string &extension) {
    // Get current UTC time
    std::time_t now = std::time(nullptr);
    std::tm utc_tm;
#ifdef _WIN32
    gmtime_s(&utc_tm, &now); // Windows
#else
    gmtime_r(&now, &utc_tm); // Linux/macOS
#endif

    std::ostringstream oss;

    // Format time: YYYYMMDD_HHMMSS
    oss << prefix;
    oss << std::put_time(&utc_tm, "%Y%m%d_%H%M%S");

    // Generate random 8-digit hex
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint32_t> dis(0, 0xFFFFFFFF);
    uint32_t randNum = dis(gen);
    oss << "_" << std::hex << std::setw(8) << std::setfill('0') << randNum;

    oss << extension;

    return oss.str();
}

/**
 * @brief Splits a full Linux file path into directory path and filename.
 *
 * The directory path (if present) is returned with a trailing '/'.
 *
 * @param[in]  fullPath  Full path to split (e.g., "/home/user/file.txt" or "file.txt").
 * @param[out] path      Output directory portion. Cleared if no '/' is present.
 * @param[out] filename  Output filename portion. Empty if @p fullPath ends with '/'.
 */
void Utils::splitPath(const std::string &fullPath, std::string &path,
                      std::string &filename) {
    auto pos = fullPath.find_last_of('/');
    if (pos == std::string::npos) {
        path.clear();
        filename = fullPath;
        return;
    }
    path = fullPath.substr(0, pos + 1); // keep trailing '/'
    filename = fullPath.substr(pos + 1);
}

/**
 * @brief Gets the default folder for recording outputs.
 *
 * Returns the "recordings" directory located alongside the running
 * executable. This does not create the directory; it only provides the path.
 *
 * @return Absolute path to the default recordings directory.
 */
std::string Utils::getDefaultRecordingFolder() {
    std::string defaultFolder = getExecutableFolder() + "/" + recordingFolder;
    return defaultFolder;
}

bool Utils::generateRecordingPath(std::string &filePath) {

    std::string folderName = filePath;
    std::string fileBaseName = aditof::Utils::generateFileName();

    if (filePath.empty()) {
        folderName = Utils::getDefaultRecordingFolder();
    } else {
        folderName = folderName + "/" + recordingFolder;
    }

    if (!aditof::Utils::folderExists(folderName)) {
        if (!aditof::Utils::createFolder(folderName)) {
            return false;
        }
    }

    filePath = folderName + fileBaseName;

    return true;
}