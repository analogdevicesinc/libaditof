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
#include "aditof/log.h"
#include "aditof/utils.h"

#include <sys/stat.h>
#include <sys/types.h>
#ifdef _WIN32
#include <direct.h>
#else
#include <dirent.h>
#include <mntent.h>
#include <sys/sysmacros.h>
#include <unistd.h>
#endif
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <limits.h>
#include <random>
#include <sstream>
#include <sys/stat.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

using namespace std;
using namespace aditof;

const std::string recordingFolder = "recordings/";

#ifndef _WIN32
// detect_storage_class.cpp
// Compile:
//   g++ -std=c++14 detect_storage_class.cpp -o detect_storage_class
// For standalone test (includes main), add -DSTANDALONE_DETECT_STORAGE
//   g++ -std=c++14 -DSTANDALONE_DETECT_STORAGE detect_storage_class.cpp -o detect_storage_class
class StorageDetector {
  public:
    // Return codes:
    //  -1 : error (couldn't resolve device / other failure)
    //   0 : unknown
    //   1 : Micro-SD/SD (mmc/mmcblk detected)
    //   2 : NVMe or HDD (nvme or rotational HDD)
    int detect(const std::string &path) {
        clear_state();
        src_ = findDeviceForPath(path);
        if (src_.empty())
            return -1;

        devname_ = basenameOfDevice(src_);
        base_ = baseBlockName(devname_);

        // read rotational flag
        std::string rotPath = "/sys/block/" + base_ + "/queue/rotational";
        rotational_ = readFile(rotPath);
        if (rotational_.empty()) {
            rotPath = "/sys/class/block/" + base_ + "/queue/rotational";
            rotational_ = readFile(rotPath);
        }
        rotational_.erase(
            std::remove_if(rotational_.begin(), rotational_.end(), ::isspace),
            rotational_.end());

        // sysfs device link for additional hints
        std::string devlink = "/sys/block/" + base_ + "/device";
        char linkbuf[PATH_MAX];
        ssize_t r = readlink(devlink.c_str(), linkbuf, sizeof(linkbuf) - 1);
        if (r > 0) {
            linkbuf[r] = '\0';
            std::string rel(linkbuf);
            devlink_full_ = std::string("/sys/block/") + base_ + "/" + rel;
        } else {
            std::string alt = "/sys/class/block/" + base_ + "/device";
            r = readlink(alt.c_str(), linkbuf, sizeof(linkbuf) - 1);
            if (r > 0) {
                linkbuf[r] = '\0';
                devlink_full_ = std::string("/sys/class/block/") + base_ + "/" +
                                std::string(linkbuf);
            }
        }

        bool likely_mmc = false;
        bool likely_nvme = false;
        if (base_.rfind("mmcblk", 0) == 0)
            likely_mmc = true;
        if (base_.rfind("nvme", 0) == 0)
            likely_nvme = true;

        if (!devlink_full_.empty() &&
            devlink_full_.find("/mmc") != std::string::npos)
            likely_mmc = true;
        if (!devlink_full_.empty() &&
            devlink_full_.find("/nvme") != std::string::npos)
            likely_nvme = true;

        if (likely_mmc)
            return 1;
        if (likely_nvme)
            return 2;
        if (!rotational_.empty() && rotational_ == "1")
            return 2; // rotational => HDD

        return 0; // ambiguous / unknown (e.g. non-rotational USB / SATA SSD / SD in USB reader)
    }

    // Diagnostics getters:
    std::string lastResolvedDevice() const { return src_; }
    std::string lastBaseDevice() const { return base_; }
    std::string lastRotational() const { return rotational_; }
    std::string lastDevlink() const { return devlink_full_; }

  private:
    // State
    std::string src_;
    std::string devname_;
    std::string base_;
    std::string rotational_;
    std::string devlink_full_;

    void clear_state() {
        src_.clear();
        devname_.clear();
        base_.clear();
        rotational_.clear();
        devlink_full_.clear();
    }

    static std::string readFile(const std::string &path) {
        std::ifstream f(path.c_str());
        if (!f)
            return {};
        std::string s;
        std::getline(f, s);
        return s;
    }

    static bool fileExists(const std::string &path) {
        struct stat st;
        return stat(path.c_str(), &st) == 0;
    }

    static bool isBlockDevice(const std::string &path) {
        struct stat st;
        if (stat(path.c_str(), &st) != 0)
            return false;
        return S_ISBLK(st.st_mode);
    }

    static std::string canonicalizePath(const std::string &p) {
        char buf[PATH_MAX];
        if (realpath(p.c_str(), buf))
            return std::string(buf);
        return std::string();
    }

    static std::string resolveDevSymlink(const std::string &devpath) {
        struct stat st;
        if (lstat(devpath.c_str(), &st) != 0)
            return {};
        if (!S_ISLNK(st.st_mode))
            return {}; // not a symlink

        char buf[PATH_MAX];
        ssize_t r = readlink(devpath.c_str(), buf, sizeof(buf) - 1);
        if (r <= 0)
            return {};
        buf[r] = '\0';
        std::string link(buf);

        std::string target;
        if (!link.empty() && link[0] != '/') {
            std::string dir;
            auto pos = devpath.find_last_of('/');
            if (pos == std::string::npos)
                dir = ".";
            else
                dir = devpath.substr(0, pos);
            target = dir + "/" + link;
        } else {
            target = link;
        }

        std::string canon = canonicalizePath(target);
        if (!canon.empty())
            return canon;
        return target;
    }

    static std::string resolveByDiskBy(const std::string &prefix,
                                       const std::string &id) {
        std::string p = prefix + "/" + id;
        if (!fileExists(p))
            return {};
        std::string canon = canonicalizePath(p);
        if (!canon.empty())
            return canon;
        return p;
    }

    static std::string findBlockDeviceByDevNumbers(dev_t devnum) {
        unsigned int maj = major(devnum);
        unsigned int min = minor(devnum);
        std::string want = std::to_string(maj) + ":" + std::to_string(min);

        const char *sysclass = "/sys/class/block";
        DIR *d = opendir(sysclass);
        if (!d)
            return {};
        struct dirent *ent;
        while ((ent = readdir(d)) != nullptr) {
            if (ent->d_name[0] == '.')
                continue;
            std::string devfile =
                std::string(sysclass) + "/" + ent->d_name + "/dev";
            std::string content = readFile(devfile);
            if (!content.empty()) {
                content.erase(
                    std::remove_if(content.begin(), content.end(), ::isspace),
                    content.end());
                if (content == want) {
                    closedir(d);
                    return std::string("/dev/") + ent->d_name;
                }
            }
        }
        closedir(d);
        return {};
    }

    static std::string findDeviceForPath(const std::string &path) {
        struct stat stTarget;
        if (stat(path.c_str(), &stTarget) != 0) {
            return {};
        }

        FILE *mnt = setmntent("/proc/mounts", "r");
        if (!mnt) {
            return {};
        }

        const struct mntent *ent;
        std::string bestFsname;
        size_t bestLen = 0;
        std::string bestMountPoint;
        while ((ent = getmntent(mnt)) != nullptr) {
            struct stat st;
            if (stat(ent->mnt_dir, &st) != 0)
                continue;
            if (st.st_dev == stTarget.st_dev) {
                size_t len =
                    std::string(ent->mnt_dir ? ent->mnt_dir : "").size();
                if (len > bestLen) {
                    bestLen = len;
                    bestFsname = ent->mnt_fsname ? ent->mnt_fsname : "";
                    bestMountPoint = ent->mnt_dir ? ent->mnt_dir : "";
                }
            }
        }
        endmntent(mnt);

        if (bestFsname.empty())
            return {};

        // If fsname is absolute path like /dev/mmcblk0p2
        if (!bestFsname.empty() && bestFsname[0] == '/') {
            if (isBlockDevice(bestFsname))
                return bestFsname;
            if (fileExists(bestFsname)) {
                std::string resolved = resolveDevSymlink(bestFsname);
                if (!resolved.empty() && isBlockDevice(resolved))
                    return resolved;
            }
        }

        // Handle UUID=, LABEL=, PARTUUID=, PARTLABEL=
        std::vector<std::pair<std::string, std::string>> byprefix = {
            {"UUID=", "/dev/disk/by-uuid"},
            {"LABEL=", "/dev/disk/by-label"},
            {"PARTUUID=", "/dev/disk/by-partuuid"},
            {"PARTLABEL=", "/dev/disk/by-partlabel"},
        };
        for (size_t i = 0; i < byprefix.size(); ++i) {
            const auto &pre = byprefix[i].first;
            const auto &prefixdir = byprefix[i].second;
            if (bestFsname.rfind(pre, 0) == 0) {
                std::string id = bestFsname.substr(pre.size());
                std::string d = resolveByDiskBy(prefixdir, id);
                if (!d.empty()) {
                    if (isBlockDevice(d))
                        return d;
                    return d;
                }
            }
        }

        // /dev/root special
        if (bestFsname == "/dev/root") {
            std::string resolved = resolveDevSymlink("/dev/root");
            if (!resolved.empty() && isBlockDevice(resolved))
                return resolved;
        }

        // Fallback: match mountpoint dev numbers to /sys/class/block/*/dev
        if (!bestMountPoint.empty()) {
            struct stat stMount;
            if (stat(bestMountPoint.c_str(), &stMount) == 0) {
                std::string dd = findBlockDeviceByDevNumbers(stMount.st_dev);
                if (!dd.empty())
                    return dd;
            }
        }

        return {};
    }

    static std::string basenameOfDevice(const std::string &devpath) {
        auto pos = devpath.find_last_of('/');
        if (pos == std::string::npos)
            return devpath;
        return devpath.substr(pos + 1);
    }

    static std::string baseBlockName(const std::string &name) {
        if (name.rfind("mmcblk", 0) == 0)
            return name;
        if (std::regex_match(name, std::regex("^nvme\\d+n\\d+$")))
            return name;

        std::smatch m;
        if (std::regex_match(name, m, std::regex("^(.+?)p(\\d+)$"))) {
            return m[1].str();
        }
        if (std::regex_match(name, m, std::regex("^([a-z]+)(\\d+)$"))) {
            return m[1].str();
        }
        return name;
    }
};
#endif // !_WIN32

#ifdef STANDALONE_DETECT_STORAGE
int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " /path/to/check\n";
        return 2;
    }
    StorageDetector sd;
    int rc = sd.detect(argv[1]);
    std::cout << "detect('" << argv[1] << "') = " << rc << "\n";
    std::cout << "resolved device: " << sd.lastResolvedDevice() << "\n";
    std::cout << "base device:     " << sd.lastBaseDevice() << "\n";
    std::cout << "rotational:      " << sd.lastRotational() << "\n";
    std::cout << "devlink:         " << sd.lastDevlink() << "\n";
    switch (rc) {
    case -1:
        std::cout << "Error resolving device or insufficient info\n";
        break;
    case 0:
        std::cout << "Unknown (ambiguous/non-rotational non-mmc device)\n";
        break;
    case 1:
        std::cout << "Micro-SD / SD (mmc)\n";
        break;
    case 2:
        std::cout << "NVMe or HDD\n";
        break;
    }
    return 0;
}
#endif

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
 * @brief Creates a new folder at the specified path, including all parent directories.
 *
 * Recursively creates all directories in the given path. If any directory in the path
 * already exists, the function continues without error. Uses platform-specific APIs
 * to create folders with appropriate permissions.
 *
 * @param path The filesystem path where the folder should be created (e.g., "output/recordings/").
 * @return true if all folders were created or already exist, false if creation
 *         failed for any reason.
 *
 * @note On Linux/macOS, folders are created with permissions 0755 (rwxr-xr-x).
 *       Handles both Windows (backslash) and Linux/macOS (forward slash) path separators.
 */
bool Utils::createFolder(const std::string &path) {
    if (path.empty()) {
        return true;
    }

    std::string currentPath;
    std::string pathToCreate = path;

    // Normalize path separators for the current platform
#ifdef _WIN32
    for (char &c : pathToCreate) {
        if (c == '/') {
            c = '\\';
        }
    }
#else
    for (char &c : pathToCreate) {
        if (c == '\\') {
            c = '/';
        }
    }
#endif

    // Remove trailing separator if present
    if (!pathToCreate.empty() &&
        (pathToCreate.back() == '/' || pathToCreate.back() == '\\')) {
        pathToCreate.pop_back();
    }

    // Iterate through path and create each directory
#ifdef _WIN32
    char separator = '\\';
#else
    char separator = '/';
#endif

    for (size_t i = 0; i < pathToCreate.length(); ++i) {
        if (pathToCreate[i] == separator) {
            currentPath = pathToCreate.substr(0, i);
            if (!currentPath.empty() && !folderExists(currentPath)) {
#ifdef _WIN32
                if (_mkdir(currentPath.c_str()) != 0 && errno != EEXIST) {
                    return false;
                }
#else
                if (mkdir(currentPath.c_str(), 0755) != 0 && errno != EEXIST) {
                    return false;
                }
#endif
            }
        }
    }

    // Create the final directory
    if (!folderExists(pathToCreate)) {
#ifdef _WIN32
        if (_mkdir(pathToCreate.c_str()) != 0 && errno != EEXIST) {
            return false;
        }
#else
        if (mkdir(pathToCreate.c_str(), 0755) != 0 && errno != EEXIST) {
            return false;
        }
#endif
    }

    return true;
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

/**
 * @brief Expands the tilde (~) in a path to the user's home directory.
 *
 * On Linux/Unix systems, expands ~ at the beginning of a path to the value
 * of the HOME environment variable. On Windows, uses USERPROFILE.
 * Handles both ~ and ~/remaining/path formats.
 *
 * @param[in] path The path that may contain a tilde.
 * @return The expanded path with ~ replaced by home directory, or original
 *         path if ~ is not present or HOME/USERPROFILE cannot be determined.
 */
static std::string expandTildePath(const std::string &path) {
    if (path.empty() || path[0] != '~') {
        return path;
    }

    const char *homeDir = nullptr;

#ifdef _WIN32
    // Windows: use USERPROFILE
    homeDir = getenv("USERPROFILE");
#else
    // Linux/macOS: use HOME
    homeDir = getenv("HOME");
#endif

    if (!homeDir) {
        return path; // Can't expand, return as-is
    }

    std::string home(homeDir);

    // Handle just "~"
    if (path.length() == 1) {
        return home;
    }

    // Handle "~/..." or "~\" format
    if (path[1] == '/' || path[1] == '\\') {
        return home + path.substr(1);
    }

    // Path starts with ~ but not followed by / or \, return as-is
    return path;
}

bool Utils::generateRecordingPath(std::string &filePath,
                                  bool strict_storage_check) {

    std::string folderName = filePath;
    std::string fileBaseName = aditof::Utils::generateFileName();

    if (filePath.empty()) {
        folderName = Utils::getDefaultRecordingFolder();
    } else {
        folderName = folderName + "/" + recordingFolder;
    }

    // Expand tilde (~) to home directory on Linux/Unix
    folderName = expandTildePath(folderName);

    if (!aditof::Utils::folderExists(folderName)) {
        if (!aditof::Utils::createFolder(folderName)) {
            LOG(ERROR) << "Failed to create folder: " << folderName;
            return false;
        }
    }

#ifndef _WIN32
    StorageDetector sd;
    int storageType = sd.detect(folderName);

    if (storageType != 2 && strict_storage_check) {
        LOG(WARNING) << "Recording folder '" << folderName
                     << "' is not on NVMe or HDD storage. Detected type: "
                     << storageType
                     << " (1=mmc/SD, 2=NVMe/HDD, 0=unknown, -1=error)";
    }
#endif

    filePath = folderName + fileBaseName;

    return true;
}

/**
 * @brief Recursively removes a folder and all its contents.
 *
 * This function removes a folder directory and all files/subdirectories contained
 * within it. Works on both Linux and Windows.
 *
 * @param[in] path  The path to the folder to remove.
 * @return true if removal was successful, false otherwise.
 */
bool Utils::removeFolder(const std::string &path) {
    if (path.empty()) {
        LOG(ERROR) << "Cannot remove folder: path is empty";
        return false;
    }

#ifdef _WIN32
    // Windows implementation using RemoveDirectoryW
    // Note: RemoveDirectoryW only works on empty directories,
    // so we need to recursively delete contents first

    WIN32_FIND_DATAA findData;
    HANDLE findHandle;

    // First, delete all files and subdirectories
    std::string searchPath = path;
    if (searchPath.back() != '\\' && searchPath.back() != '/') {
        searchPath += "\\";
    }
    std::string searchPattern = searchPath + "*";

    findHandle = FindFirstFileA(searchPattern.c_str(), &findData);
    if (findHandle == INVALID_HANDLE_VALUE) {
        // Folder might be empty or doesn't exist
        if (RemoveDirectoryA(path.c_str())) {
            return true;
        }
        LOG(ERROR) << "Failed to remove folder: " << path;
        return false;
    }

    do {
        std::string fileName = findData.cFileName;
        if (fileName == "." || fileName == "..") {
            continue;
        }

        std::string fullPath = searchPath + fileName;

        if (findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
            // Recursively remove subdirectory
            if (!removeFolder(fullPath)) {
                FindClose(findHandle);
                return false;
            }
        } else {
            // Delete file
            if (!DeleteFileA(fullPath.c_str())) {
                LOG(ERROR) << "Failed to delete file: " << fullPath;
                FindClose(findHandle);
                return false;
            }
        }
    } while (FindNextFileA(findHandle, &findData));

    FindClose(findHandle);

    // Now remove the empty folder
    if (!RemoveDirectoryA(path.c_str())) {
        LOG(ERROR) << "Failed to remove directory: " << path;
        return false;
    }

    return true;

#elif defined(__linux__)
    // Linux implementation using dirent and unlink/rmdir
    DIR *dir = opendir(path.c_str());
    if (!dir) {
        LOG(ERROR) << "Failed to open directory: " << path;
        return false;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string entryName = entry->d_name;
        if (entryName == "." || entryName == "..") {
            continue;
        }

        std::string fullPath = path;
        if (fullPath.back() != '/') {
            fullPath += "/";
        }
        fullPath += entryName;

        struct stat entryStat;
        if (stat(fullPath.c_str(), &entryStat) == -1) {
            LOG(ERROR) << "Failed to stat: " << fullPath;
            closedir(dir);
            return false;
        }

        if (S_ISDIR(entryStat.st_mode)) {
            // Recursively remove subdirectory
            if (!removeFolder(fullPath)) {
                closedir(dir);
                return false;
            }
        } else {
            // Delete file
            if (unlink(fullPath.c_str()) == -1) {
                LOG(ERROR) << "Failed to delete file: " << fullPath;
                closedir(dir);
                return false;
            }
        }
    }

    closedir(dir);

    // Remove the empty directory
    if (rmdir(path.c_str()) == -1) {
        LOG(ERROR) << "Failed to remove directory: " << path;
        return false;
    }

    return true;

#else
    LOG(ERROR) << "removeFolder() not implemented for this platform";
    return false;
#endif
}