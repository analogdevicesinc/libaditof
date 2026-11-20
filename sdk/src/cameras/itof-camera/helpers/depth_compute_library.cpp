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
#ifdef ADITOF_TOFI_DLOPEN
// System headers first: only the library API may become hidden below.
#include <cctype>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
// Hidden, so the loaded libraries never bind to these forwarding definitions.
#pragma GCC visibility push(hidden)
#include "tofi/tofi_compute.h"
#include "tofi/tofi_config.h"
#include "tofi/tofi_error.h"
#pragma GCC visibility pop
#endif

#include "depth_compute_library.h"

#include <aditof/log.h>
#include <dlfcn.h>
#include <map>
#include <mutex>
#include <string>
#include <unistd.h>
#include <vector>

namespace aditof {
namespace depth_compute_library {
namespace {

// TEMPORARY (two depth compute libraries): lens scatter-only builds of the
// library compute no standard (ISP) depth.
bool runsStandardDepth(bool hasLensScatter, bool hasModeFusion) {
    return !hasLensScatter || hasModeFusion;
}

#ifdef ADITOF_TOFI_DLOPEN

enum Kind { Standard = 0, LensScatter = 1 };

struct Library {
    bool tried = false;
    void *compute = nullptr;
    void *config = nullptr;
};

std::mutex g_mutex;
Library g_libraries[2];
int g_selected = Standard;
// Configs, calibration configs and compute contexts -> library that made them.
std::map<const void *, int> g_owner;

std::string libaditofDir() {
    Dl_info info;
    if (dladdr(reinterpret_cast<void *>(&libaditofDir), &info) &&
        info.dli_fname) {
        const std::string path(info.dli_fname);
        const size_t slash = path.rfind('/');
        if (slash != std::string::npos) {
            return path.substr(0, slash);
        }
    }
    return "";
}

std::vector<std::string> candidateDirs(int kind) {
    std::vector<std::string> dirs;
    const char *env =
        getenv(kind == LensScatter ? "ADITOF_TOFI_LENS_SCATTER_LIBDIR"
                                   : "ADITOF_TOFI_LIBDIR");
    if (env && *env) {
        dirs.emplace_back(env);
    }
    const std::string own = libaditofDir();
    if (!own.empty()) {
        dirs.push_back(kind == LensScatter ? own + "/lens_scatter" : own);
    }
    dirs.emplace_back(kind == LensScatter
                          ? ADITOF_TOFI_LENS_SCATTER_LIBDIR_DEFAULT
                          : ADITOF_TOFI_LIBDIR_DEFAULT);
    return dirs;
}

// Callers hold g_mutex.
Library &load(int kind) {
    Library &lib = g_libraries[kind];
    if (lib.tried) {
        return lib;
    }
    lib.tried = true;
    for (const auto &dir : candidateDirs(kind)) {
        const std::string computePath = dir + "/libtofi_compute.so";
        const std::string configPath = dir + "/libtofi_config.so";
        if (dir.empty() || access(computePath.c_str(), R_OK) != 0 ||
            access(configPath.c_str(), R_OK) != 0) {
            continue;
        }
        // Local, so both builds (same symbol names) can be loaded together.
        void *config = dlopen(configPath.c_str(), RTLD_NOW | RTLD_LOCAL);
        void *compute = config
                            ? dlopen(computePath.c_str(), RTLD_NOW | RTLD_LOCAL)
                            : nullptr;
        if (!compute) {
            LOG(WARNING) << "Could not load the depth compute library in "
                         << dir << ": " << dlerror();
            if (config) {
                dlclose(config);
            }
            continue;
        }
        lib.compute = compute;
        lib.config = config;
        LOG(INFO) << "Depth compute library ("
                  << (kind == LensScatter ? "lens scatter" : "standard")
                  << "): " << dir;
        break;
    }
    return lib;
}

void *lookup(const Library &lib, const char *name) {
    if (!lib.compute) {
        return nullptr;
    }
    void *address = dlsym(lib.compute, name);
    return address ? address : dlsym(lib.config, name);
}

bool has(int kind, const char *name) {
    return lookup(load(kind), name) != nullptr;
}

// Library for algorithm, or -1. Callers hold g_mutex.
int kindFor(DepthAlgorithm algorithm) {
    const bool lensScatter = has(Standard, "InitLensScatterCuda");
    const bool modeFusion = has(Standard, "ModeFusionProcess");
    switch (algorithm) {
    case DepthAlgorithm::None:
        return (load(Standard).compute &&
                runsStandardDepth(lensScatter, modeFusion))
                   ? Standard
                   : -1;
    case DepthAlgorithm::ModeFusion:
        return modeFusion ? Standard : -1;
    case DepthAlgorithm::LensScatter:
        if (has(LensScatter, "InitLensScatterCuda")) {
            return LensScatter;
        }
        return lensScatter ? Standard : -1;
    }
    return -1;
}

template <typename F>
F resolve(const char *name, const void *owned) {
    std::lock_guard<std::mutex> lock(g_mutex);
    int kind = g_selected;
    if (owned) {
        auto it = g_owner.find(owned);
        if (it != g_owner.end()) {
            kind = it->second;
        }
    }
    F function = reinterpret_cast<F>(lookup(load(kind), name));
    if (!function) {
        LOG(ERROR) << name << " is missing from the depth compute library";
    }
    return function;
}

void remember(const void *owned, const void *made) {
    if (!made) {
        return;
    }
    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = owned ? g_owner.find(owned) : g_owner.end();
    g_owner[made] = (it != g_owner.end()) ? it->second : g_selected;
}

void forget(const void *made) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_owner.erase(made);
}

#endif // ADITOF_TOFI_DLOPEN

} // namespace

#ifdef ADITOF_TOFI_DLOPEN

Status select(DepthAlgorithm algorithm) {
    std::lock_guard<std::mutex> lock(g_mutex);
    const int kind = kindFor(algorithm);
    if (kind < 0) {
        return Status::UNAVAILABLE;
    }
    g_selected = kind;
    return Status::OK;
}

bool supports(DepthAlgorithm algorithm) {
    std::lock_guard<std::mutex> lock(g_mutex);
    return kindFor(algorithm) >= 0;
}

void *symbol(const char *name) {
    std::lock_guard<std::mutex> lock(g_mutex);
    return lookup(load(g_selected), name);
}

#else // The library is linked: only what it has is available.

Status select(DepthAlgorithm algorithm) {
    return supports(algorithm) ? Status::OK : Status::UNAVAILABLE;
}

bool supports(DepthAlgorithm algorithm) {
    const bool lensScatter = symbol("InitLensScatterCuda") != nullptr;
    const bool modeFusion = symbol("ModeFusionProcess") != nullptr;
    switch (algorithm) {
    case DepthAlgorithm::None:
        return runsStandardDepth(lensScatter, modeFusion);
    case DepthAlgorithm::ModeFusion:
        return modeFusion;
    case DepthAlgorithm::LensScatter:
        return lensScatter;
    }
    return false;
}

void *symbol(const char *name) { return dlsym(RTLD_DEFAULT, name); }

#endif // ADITOF_TOFI_DLOPEN

} // namespace depth_compute_library
} // namespace aditof

#ifdef ADITOF_TOFI_DLOPEN

// Forward the library API libaditof uses to the library that owns the object,
// or to the selected one for new objects.
using namespace aditof::depth_compute_library;

extern "C" {

uint32_t GetVersion(char *depth_compute_version) {
    auto f = resolve<decltype(&GetVersion)>("GetVersion", nullptr);
    return f ? f(depth_compute_version) : ADI_TOFI_ERROR;
}

TofiConfig *InitTofiConfig(ConfigFileData *p_cal_file_data,
                           ConfigFileData *p_config_file_data,
                           ConfigFileData *p_ini_file_data, uint16_t mode,
                           uint32_t *p_status) {
    auto f = resolve<decltype(&InitTofiConfig)>("InitTofiConfig", nullptr);
    if (!f) {
        *p_status = ADI_TOFI_ERROR;
        return nullptr;
    }
    TofiConfig *config =
        f(p_cal_file_data, p_config_file_data, p_ini_file_data, mode, p_status);
    if (config) {
        remember(nullptr, config);
        remember(config, config->p_tofi_cal_config);
    }
    return config;
}

TofiConfig *InitTofiConfig_isp(ConfigFileData *p_ini_file_data, uint16_t mode,
                               uint32_t *p_status,
                               TofiXYZDealiasData *p_xyz_dealias_data) {
    auto f =
        resolve<decltype(&InitTofiConfig_isp)>("InitTofiConfig_isp", nullptr);
    if (!f) {
        *p_status = ADI_TOFI_ERROR;
        return nullptr;
    }
    TofiConfig *config = f(p_ini_file_data, mode, p_status, p_xyz_dealias_data);
    if (config) {
        remember(nullptr, config);
        remember(config, config->p_tofi_cal_config);
    }
    return config;
}

void FreeTofiConfig(TofiConfig *p_tofi_cal_config) {
    auto f =
        resolve<decltype(&FreeTofiConfig)>("FreeTofiConfig", p_tofi_cal_config);
    if (p_tofi_cal_config) {
        forget(p_tofi_cal_config->p_tofi_cal_config);
        forget(p_tofi_cal_config);
    }
    if (f) {
        f(p_tofi_cal_config);
    }
}

uint32_t TofiSetINIParams(void *p_config_params, int params_group,
                          const void *p_tofi_cal_config) {
    auto f = resolve<decltype(&TofiSetINIParams)>("TofiSetINIParams",
                                                  p_tofi_cal_config);
    return f ? f(p_config_params, params_group, p_tofi_cal_config)
             : ADI_TOFI_ERROR;
}

uint32_t TofiGetINIParams(void *p_config_params, int params_group,
                          const void *p_tofi_cal_config) {
    auto f = resolve<decltype(&TofiGetINIParams)>("TofiGetINIParams",
                                                  p_tofi_cal_config);
    return f ? f(p_config_params, params_group, p_tofi_cal_config)
             : ADI_TOFI_ERROR;
}

TofiComputeContext *InitTofiCompute(const void *p_tofi_cal_config,
                                    uint32_t *p_status) {
    auto f = resolve<decltype(&InitTofiCompute)>("InitTofiCompute",
                                                 p_tofi_cal_config);
    if (!f) {
        *p_status = ADI_TOFI_ERROR;
        return nullptr;
    }
    TofiComputeContext *context = f(p_tofi_cal_config, p_status);
    remember(p_tofi_cal_config, context);
    return context;
}

int TofiCompute(const uint16_t *const input_frame,
                TofiComputeContext *const p_tofi_compute_context,
                TemperatureInfo *p_temperature) {
    auto f =
        resolve<decltype(&TofiCompute)>("TofiCompute", p_tofi_compute_context);
    return f ? f(input_frame, p_tofi_compute_context, p_temperature)
             : ADI_TOFI_ERROR;
}

void FreeTofiCompute(TofiComputeContext *p_tofi_compute_context) {
    auto f = resolve<decltype(&FreeTofiCompute)>("FreeTofiCompute",
                                                 p_tofi_compute_context);
    forget(p_tofi_compute_context);
    if (f) {
        f(p_tofi_compute_context);
    }
}

} // extern "C"

#endif // ADITOF_TOFI_DLOPEN
