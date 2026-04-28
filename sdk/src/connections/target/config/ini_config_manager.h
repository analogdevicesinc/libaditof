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
#ifndef INI_CONFIG_MANAGER_H
#define INI_CONFIG_MANAGER_H

#include "sensor-tables/ini_file_definitions.h"
#include <aditof/status_definitions.h>
#include <map>
#include <string>
#include <vector>

// Forward declaration
struct IniTableEntry;
enum class SensorImagerType;

/**
 * @class IniConfigManager
 * @brief Manages INI configuration parameter operations.
 *
 * Responsible for:
 * - Reading/writing INI parameters
 * - Converting INI structures to string format
 * - Merging CCB mode parameters with default INI settings
 * - Retrieving mode-specific default parameters
 */
class IniConfigManager {
  public:
    /**
     * @brief Constructs INI configuration manager.
     *
     * @param iniFileStructList Reference to list of INI file structures
     * @param ccbmINIContent Reference to CCB mode INI content
     * @param ccbmEnabled Reference to CCB mode enabled flag
     * @param imagerType Reference to imager type
     */
    IniConfigManager(std::vector<iniFileStruct> &iniFileStructList,
                     std::vector<IniTableEntry> &ccbmINIContent,
                     bool &ccbmEnabled, SensorImagerType &imagerType);

    ~IniConfigManager() = default;

    /**
     * @brief Retrieves INI configuration parameters.
     *
     * @param p_config_params Pointer to configuration parameters structure
     * @param params_group Parameter group identifier
     * @param p_tofi_cal_config Pointer to calibration configuration
     * @return Status::OK on success
     */
    aditof::Status getIniParamsImpl(void *p_config_params, int params_group,
                                    const void *p_tofi_cal_config);

    /**
     * @brief Sets INI configuration parameters.
     *
     * @param p_config_params Pointer to configuration parameters structure
     * @param params_group Parameter group identifier
     * @param p_tofi_cal_config Pointer to calibration configuration
     * @return Status::OK on success
     */
    aditof::Status setIniParamsImpl(void *p_config_params, int params_group,
                                    const void *p_tofi_cal_config);

    /**
     * @brief Retrieves default INI parameters for imager and mode.
     *
     * @param imager Imager type name
     * @param mode Mode name
     * @param params Map to receive parameter key-value pairs
     * @return Status::OK on success
     */
    aditof::Status
    getDefaultIniParamsForMode(const std::string &imager,
                               const std::string &mode,
                               std::map<std::string, std::string> &params);

    /**
     * @brief Merges CCB mode parameters with default INI settings.
     *
     * @param iniFileStructList Vector of INI file structures to merge
     * @return Status::OK on success
     */
    aditof::Status
    mergeIniParams(std::vector<iniFileStruct> &iniFileStructList);

    /**
     * @brief Converts INI structure to string format.
     *
     * @param iniStruct INI structure to convert
     * @param inistr String to receive converted content
     * @return Status::OK on success
     */
    aditof::Status convertIniParams(iniFileStruct &iniStruct,
                                    std::string &inistr);

    /**
     * @brief Retrieves INI parameters as string for specific mode.
     *
     * @param mode Mode number
     * @param iniStr String to receive INI parameters
     * @return Status::OK on success
     */
    aditof::Status getIniParamsArrayForMode(int mode, std::string &iniStr);

  private:
    std::vector<iniFileStruct>
        &m_iniFileStructList; ///< Reference to INI file structures
    std::vector<IniTableEntry>
        &m_ccbmINIContent; ///< Reference to CCB mode INI content
    bool &m_ccbmEnabled;   ///< Reference to CCB mode enabled flag
    SensorImagerType
        &m_imagerType; ///< Reference to sensor imager type identifier
};

#endif // INI_CONFIG_MANAGER_H
