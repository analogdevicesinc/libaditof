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
#include "camera_configuration.h"

#include <aditof/log.h>
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <json.h>
#include <limits.h>
#include <list>
#include <sstream>
#include <unistd.h>

namespace aditof {

CameraConfiguration::CameraConfiguration()
    : m_mipiOutputSpeed(-1), m_isdeskewEnabled(-1),
      m_enableTempCompensation(-1), m_enableEdgeConfidence(-1),
      m_enableMetaDatainAB(-1), m_fsyncMode(-1), m_dropFirstFrame(true) {}

CameraConfiguration::~CameraConfiguration() {}

std::string CameraConfiguration::autoDiscoverConfigFile() {
    const std::string defaultConfigName = "adcam_config.json";

    // 1. Check for config file in binary directory (where executable is located)
#ifdef __linux__
    char binaryPath[PATH_MAX];
    ssize_t len =
        readlink("/proc/self/exe", binaryPath, sizeof(binaryPath) - 1);
    if (len != -1) {
        binaryPath[len] = '\0';

        // Extract directory path by finding the last '/'
        std::string binPath(binaryPath);
        size_t lastSlash = binPath.find_last_of('/');
        if (lastSlash != std::string::npos) {
            std::string binDir = binPath.substr(0, lastSlash + 1);
            std::string configPath = binDir + defaultConfigName;

            std::ifstream checkFile(configPath.c_str());
            if (checkFile.good()) {
                checkFile.close();
                return configPath;
            }
        }
    }
#endif

    // 2. Check environment variable ADCAM_CONFIG_PATH
    const char *envConfigPath = std::getenv("ADCAM_CONFIG_PATH");
    if (envConfigPath != nullptr) {
        std::string envPath(envConfigPath);
        std::ifstream checkEnvFile(envPath.c_str());
        if (checkEnvFile.good()) {
            checkEnvFile.close();
            return envPath;
        } else {
            LOG(WARNING) << "ADCAM_CONFIG_PATH environment variable set to '"
                         << envPath << "' but file not found or not readable";
        }
    }

    // 3. Not found - return empty string to proceed with factory defaults
    return "";
}

Status
CameraConfiguration::loadDepthParamsFromJsonFile(const std::string &pathFile,
                                                 const int16_t mode_in_use) {

    Status status = Status::OK;

    m_depth_params_map.clear();

    // Parse json
    std::ifstream ifs(pathFile.c_str());
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));

    json_object *config_json = json_tokener_parse(content.c_str());
    if (config_json != NULL) {

        json_object *errata1 = NULL;
        double errata1val = 1;
        if (json_object_object_get_ex(config_json, "errata1", &errata1)) {
            if (json_object_is_type(errata1, json_type_int) ||
                json_object_is_type(errata1, json_type_double)) {
                errata1val = json_object_get_double(errata1);
            }
        }
        if (errata1val == 1) {
            m_dropFirstFrame = true;
        } else {
            m_dropFirstFrame = false;
        }

        json_object *fsyncMode = NULL;
        if (json_object_object_get_ex(config_json, "fsyncMode", &fsyncMode)) {
            if (json_object_is_type(fsyncMode, json_type_int) ||
                json_object_is_type(fsyncMode, json_type_double)) {
                m_fsyncMode = json_object_get_int(fsyncMode);
            }
        }

        json_object *mipiOutputSpeed = NULL;
        if (json_object_object_get_ex(config_json, "mipiOutputSpeed",
                                      &mipiOutputSpeed)) {
            if (json_object_is_type(mipiOutputSpeed, json_type_int) ||
                json_object_is_type(mipiOutputSpeed, json_type_double)) {
                m_mipiOutputSpeed = json_object_get_int(mipiOutputSpeed);
            }
        }

        json_object *isdeskewEnabled = NULL;
        if (json_object_object_get_ex(config_json, "isdeskewEnabled",
                                      &isdeskewEnabled)) {
            if (json_object_is_type(isdeskewEnabled, json_type_int) ||
                json_object_is_type(isdeskewEnabled, json_type_double)) {
                m_isdeskewEnabled = json_object_get_int(isdeskewEnabled);
            }
        }

        json_object *enableTempCompensation = NULL;
        if (json_object_object_get_ex(config_json, "enableTempCompensation",
                                      &enableTempCompensation)) {
            if (json_object_is_type(enableTempCompensation, json_type_int) ||
                json_object_is_type(enableTempCompensation, json_type_double)) {
                m_enableTempCompensation =
                    json_object_get_int(enableTempCompensation);
            }
        }

        json_object *enableEdgeConfidence = NULL;
        if (json_object_object_get_ex(config_json, "enableEdgeConfidence",
                                      &enableEdgeConfidence)) {
            if (json_object_is_type(enableEdgeConfidence, json_type_int) ||
                json_object_is_type(enableEdgeConfidence, json_type_double)) {
                m_enableEdgeConfidence =
                    json_object_get_int(enableEdgeConfidence);
            }
        }

        json_object *dmsSequence = NULL;
        if (json_object_object_get_ex(config_json, "dynamicModeSwitching",
                                      &dmsSequence)) {
            if (json_object_is_type(dmsSequence, json_type_array)) {

                m_configDmsSequence.clear();

                int arraylen = json_object_array_length(dmsSequence);
                for (int i = 0; i < arraylen; i++) {
                    json_object *dmsPair =
                        json_object_array_get_idx(dmsSequence, i);
                    json_object *dmsMode = NULL;
                    json_object *dmsRepeat = NULL;

                    if (json_object_object_get_ex(dmsPair, "mode", &dmsMode) &&
                        json_object_object_get_ex(dmsPair, "repeat",
                                                  &dmsRepeat)) {
                        if ((json_object_is_type(dmsMode, json_type_int) ||
                             json_object_is_type(dmsMode, json_type_double)) &&
                            (json_object_is_type(dmsRepeat, json_type_int) ||
                             json_object_is_type(dmsRepeat,
                                                 json_type_double))) {
                            m_configDmsSequence.emplace_back(
                                std::make_pair(json_object_get_int(dmsMode),
                                               json_object_get_int(dmsRepeat)));
                        }
                    }
                }
            }
        }

        // Note: Mode iteration requires external available modes list
        // This will be provided by CameraItof via setDepthParamsForMode
        // For now, parse all numeric keys as modes
        json_object_object_foreach(config_json, key, val) {
            // Skip non-mode keys
            if (std::string(key) == "errata1" ||
                std::string(key) == "fsyncMode" ||
                std::string(key) == "mipiOutputSpeed" ||
                std::string(key) == "isdeskewEnabled" ||
                std::string(key) == "enableTempCompensation" ||
                std::string(key) == "enableEdgeConfidence" ||
                std::string(key) == "dynamicModeSwitching") {
                continue;
            }

            // Try to parse key as mode number
            int mode = -1;
            try {
                mode = std::stoi(key);
            } catch (...) {
                continue; // Skip non-numeric keys
            }

            // Filter by mode_in_use if specified
            if (mode_in_use >= 0 && mode != mode_in_use) {
                continue;
            }

            json_object *depthframeType = val;

            json_object *dept_compute_group_keys = NULL;
            json_object_object_get_ex(depthframeType, "depth-compute",
                                      &dept_compute_group_keys);

            std::map<std::string, std::string> iniKeyValPairs;

            if (dept_compute_group_keys) {
                json_object_object_foreach(dept_compute_group_keys, key2,
                                           val2) {

                    std::string value = "";

                    if (json_object_is_type(val2, json_type_string)) {
                        value = std::string(json_object_get_string(val2));
                    } else {
                        std::ostringstream stream;
                        stream << std::fixed << std::setprecision(1)
                               << json_object_get_double(val2);
                        value = stream.str();
                        std::size_t found = value.find(".0");
                        if (found != std::string::npos) {
                            value = std::to_string(json_object_get_int(val2));
                        }
                    }
                    iniKeyValPairs.emplace(std::string(key2), value);
                }
            }

            json_object *configuration_param_keys = NULL;
            json_object_object_get_ex(depthframeType,
                                      "configuration-parameters",
                                      &configuration_param_keys);

            if (configuration_param_keys) {
                json_object_object_foreach(configuration_param_keys, key3,
                                           val3) {

                    std::string value = "";

                    if (json_object_is_type(val3, json_type_string)) {
                        value = std::string(json_object_get_string(val3));
                    } else {
                        std::ostringstream stream;
                        stream << std::fixed << std::setprecision(1)
                               << json_object_get_double(val3);
                        value = stream.str();
                        std::size_t found = value.find(".0");
                        if (found != std::string::npos) {
                            value = std::to_string(json_object_get_int(val3));
                        }
                    }
                    iniKeyValPairs.emplace(std::string(key3), value);
                }
            }
            m_depth_params_map.emplace(mode, iniKeyValPairs);
        }
        json_object_put(config_json);
    }

    return status;
}

Status CameraConfiguration::saveDepthParamsToJsonFile(
    const std::string &savePathFile) {

    Status status = Status::OK;

    json_object *rootjson = json_object_new_object();

    json_object_object_add(rootjson, "errata1",
                           json_object_new_int(m_dropFirstFrame ? 1 : 0));

    json_object_object_add(rootjson, "fsyncMode",
                           json_object_new_int(m_fsyncMode));
    json_object_object_add(rootjson, "mipiOutputSpeed",
                           json_object_new_int(m_mipiOutputSpeed));
    json_object_object_add(rootjson, "isdeskewEnabled",
                           json_object_new_int(m_isdeskewEnabled));
    json_object_object_add(rootjson, "enableTempCompensation",
                           json_object_new_int(m_enableTempCompensation));
    json_object_object_add(rootjson, "enableEdgeConfidence",
                           json_object_new_int(m_enableEdgeConfidence));

    std::list<std::string> depth_compute_keys_list = {
        "abThreshMin",         "radialThreshMax",
        "radialThreshMin",     "depthComputeIspEnable",
        "partialDepthEnable",  "interleavingEnable",
        "bitsInPhaseOrDepth",  "bitsInAB",
        "bitsInConf",          "confThresh",
        "phaseInvalid",        "inputFormat",
        "jblfABThreshold",     "jblfApplyFlag",
        "jblfExponentialTerm", "jblfGaussianSigma",
        "jblfMaxEdge",         "jblfWindowSize"};

    for (auto pfile = m_depth_params_map.begin();
         pfile != m_depth_params_map.end(); pfile++) {

        std::map<std::string, std::string> iniKeyValPairs = pfile->second;

        if (status == Status::OK) {
            json_object *json = json_object_new_object();
            json_object *dept_compute_group_keys = json_object_new_object();
            json_object *configuration_param_keys = json_object_new_object();

            for (auto item = iniKeyValPairs.begin();
                 item != iniKeyValPairs.end(); item++) {
                double valued = strtod(item->second.c_str(), NULL);

                auto it = std::find_if(
                    std::begin(depth_compute_keys_list),
                    std::end(depth_compute_keys_list),
                    [&](const std::string key) { return item->first == key; });
                if (depth_compute_keys_list.end() != it) {
                    if (isConvertibleToDouble(item->second)) {
                        json_object_object_add(dept_compute_group_keys,
                                               item->first.c_str(),
                                               json_object_new_double(valued));
                    } else {
                        json_object_object_add(
                            dept_compute_group_keys, item->first.c_str(),
                            json_object_new_string(item->second.c_str()));
                    }
                } else {
                    if (isConvertibleToDouble(item->second)) {
                        json_object_object_add(configuration_param_keys,
                                               item->first.c_str(),
                                               json_object_new_double(valued));
                    } else {
                        json_object_object_add(
                            configuration_param_keys, item->first.c_str(),
                            json_object_new_string(item->second.c_str()));
                    }
                }
            }
            json_object_object_add(json, "depth-compute",
                                   dept_compute_group_keys);
            json_object_object_add(json, "configuration-parameters",
                                   configuration_param_keys);
            json_object_object_add(rootjson,
                                   std::to_string(pfile->first).c_str(), json);
        }
    }

    const char *json_str =
        json_object_to_json_string_ext(rootjson, JSON_C_TO_STRING_PRETTY);

    FILE *fp = fopen(savePathFile.c_str(), "w");
    if (fp == NULL) {
        LOG(WARNING) << " Unable to open the file. " << savePathFile.c_str();
        json_object_put(rootjson);
        return Status::GENERIC_ERROR;
    }
    fputs(json_str, fp);
    fclose(fp);

    json_object_put(rootjson);

    return status;
}

Status CameraConfiguration::getDepthParamsForMode(
    uint16_t mode, std::map<std::string, std::string> &params) const {

    auto it = m_depth_params_map.find(mode);
    if (it != m_depth_params_map.end()) {
        params = it->second;
        return Status::OK;
    }
    return Status::INVALID_ARGUMENT;
}

Status CameraConfiguration::setDepthParamsForMode(
    uint16_t mode, const std::map<std::string, std::string> &params) {

    m_depth_params_map[mode] = params;
    return Status::OK;
}

void CameraConfiguration::clearDepthParams() { m_depth_params_map.clear(); }

void CameraConfiguration::cacheDepthParams() {
    m_depth_params_map_cache = m_depth_params_map;
}

Status CameraConfiguration::restoreCachedDepthParams() {
    if (m_depth_params_map_cache.empty()) {
        return Status::GENERIC_ERROR;
    }
    m_depth_params_map = m_depth_params_map_cache;
    return Status::OK;
}

bool CameraConfiguration::isConvertibleToDouble(const std::string &str) {
    bool result = false;
    try {
        std::stod(str);
        result = true;
    } catch (...) {
    }
    return result;
}

} // namespace aditof
