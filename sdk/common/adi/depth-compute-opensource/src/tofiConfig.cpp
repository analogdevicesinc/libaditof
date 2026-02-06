/*******************************************************************************
 * Copyright 2025(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#include "tofi_config.h"
#include "tofi_error.h"

#include <sstream>

/**
 * @brief Initializes a TofiConfig structure for open-source depth computation.
 *
 * This function allocates and initializes a TofiConfig object with default values,
 * setting all pointers to null and enabling the open-source depth computation path.
 * This is used when the proprietary depth compute library is not available.
 *
 * @param[in] p_cal_file_data Pointer to calibration file data (unused in open-source implementation)
 * @param[in] p_config_file_data Pointer to configuration file data (unused in open-source implementation)
 * @param[in] p_ini_file_data Pointer to INI file data (unused in open-source implementation)
 * @param[in] mode Frame mode identifier
 * @param[out] p_status Pointer to status variable for error reporting
 *
 * @return Pointer to newly allocated and initialized TofiConfig structure
 */
TofiConfig *InitTofiConfig(ConfigFileData *p_cal_file_data,
                           ConfigFileData *p_config_file_data,
                           ConfigFileData *p_ini_file_data, uint16_t mode,
                           uint32_t *p_status) {
    XYZTable xyzObj;
    xyzObj.p_x_table = 0;
    xyzObj.p_y_table = 0;
    xyzObj.p_z_table = 0;

    TofiConfig *Obj = new TofiConfig;
    Obj->n_cols = 0;
    Obj->n_rows = 0;
    Obj->p_cal_gain_block = 0;
    Obj->p_cal_reg_block = 0;
    Obj->p_camera_intrinsics = 0;
    Obj->p_lsdac_block = 0;
    Obj->p_tofi_cal_config = 0;
    Obj->p_tofi_config_str = 0;
    Obj->xyz_table = xyzObj;

    depthComputeOpenSourceEnabled = 1;

    return Obj;
};

/**
 * @brief Searches for a key in INI file content and returns its associated value.
 *
 * This function parses INI file content (provided as an input stream) to find a line
 * starting with the specified key, then extracts and returns the value following the
 * '=' delimiter. The stream is reset to the beginning before searching.
 *
 * @param[in,out] iniContent Input stream containing INI file content (seeked to beginning)
 * @param[in] key Key string to search for at the start of lines
 *
 * @return String value associated with the key, or empty string if key not found
 */
std::string iniFileContentFindKeyAndGetValue(std::istream &iniContent,
                                             const std::string &key) {
    iniContent.clear();
    iniContent.seekg(0, std::ios::beg);

    std::string line;
    while (getline(iniContent, line)) {
        if (line.compare(0, key.length(), key) == 0) {
            size_t equalPos = line.find('=');
            if (equalPos != std::string::npos) {
                return line.substr(equalPos + 1);
            }
        }
    }

    return "";
}

/**
 * @brief Initializes a TofiConfig structure for ISP-based depth computation.
 *
 * This function creates a TofiConfig object specifically for hardware ISP configurations.
 * It parses the INI file to extract bit depth information for depth, AB (amplitude/brightness),
 * and confidence channels, then encodes this metadata into the dealias data structure.
 * The function stores configuration and dealias data pointers in the TofiConfig for later use.
 *
 * @param[in] p_ini_file_data Pointer to INI file data containing bit depth configuration
 * @param[in] mode Frame mode index to select appropriate dealias data
 * @param[out] p_status Pointer to status variable for error reporting
 * @param[in] p_xyz_dealias_data Array of TofiXYZDealiasData structures indexed by mode
 *
 * @return Pointer to newly allocated and initialized TofiConfig structure for ISP mode
 */
TofiConfig *InitTofiConfig_isp(ConfigFileData *p_ini_file_data, uint16_t mode,
                               uint32_t *p_status,
                               TofiXYZDealiasData *p_xyz_dealias_data) {
    XYZTable xyzObj;
    xyzObj.p_x_table = 0;
    xyzObj.p_y_table = 0;
    xyzObj.p_z_table = 0;

    ConfigFileData *configFileObj = new ConfigFileData;
    *configFileObj = *p_ini_file_data;

    // We create a stream object with the content of the .ini file
    // Then we extract the number of bits for: Depth, AB, Confidence
    std::string s((char *)configFileObj->p_data, configFileObj->size);
    std::istringstream is(s);
    uint16_t nb_depth =
        std::stoi(iniFileContentFindKeyAndGetValue(is, "bitsInPhaseOrDepth"));
    uint16_t nb_ab =
        std::stoi(iniFileContentFindKeyAndGetValue(is, "bitsInAB"));
    uint16_t nb_conf =
        std::stoi(iniFileContentFindKeyAndGetValue(is, "bitsInConf"));

    TofiXYZDealiasData *dealiasDataObj = new TofiXYZDealiasData;
    *dealiasDataObj = p_xyz_dealias_data[mode];

    // We sneak the number of bits for Depth, AB, Conf into this variable that is not going to be used anywhere in tofiCompute.cpp (hopefully)
    dealiasDataObj->Freq[0] = (nb_depth << 0) | (nb_ab << 5) | (nb_conf << 10);

    TofiConfig *Obj = new TofiConfig;
    Obj->n_cols = 0;
    Obj->n_rows = 0;
    Obj->p_cal_gain_block = 0;
    Obj->p_cal_reg_block = 0;
    Obj->p_camera_intrinsics = 0;
    Obj->p_lsdac_block = 0;
    Obj->p_tofi_cal_config = reinterpret_cast<const void *>(
        dealiasDataObj); // Not nice but couldn't find a way to store all content of p_xyz_dealias_data without changing API
    Obj->p_tofi_config_str = reinterpret_cast<const char *>(configFileObj);
    Obj->xyz_table = xyzObj;

    depthComputeOpenSourceEnabled = 1;

    return Obj;
};

/**
 * @brief Retrieves XYZ dealias data from calibration block data.
 *
 * This function is a stub in the open-source implementation. In the full implementation,
 * it would extract dealias parameters from the camera calibration block (CCB) data.
 *
 * @param[in] ccb_data Pointer to camera calibration block data
 * @param[out] p_xyz_data Pointer to TofiXYZDealiasData structure to populate
 *
 * @return 0 on success (always succeeds in open-source stub)
 */
uint32_t GetXYZ_DealiasData(ConfigFileData *ccb_data,
                            TofiXYZDealiasData *p_xyz_data) {
    return 0;
};

/**
 * @brief Frees memory allocated for a TofiConfig structure and its associated data.
 *
 * This function deallocates all heap memory associated with a TofiConfig object,
 * including the config file data and dealias data that were allocated during
 * initialization (particularly from InitTofiConfig_isp).
 *
 * @param[in] p_tofi_cal_config Pointer to TofiConfig structure to be freed
 */
void FreeTofiConfig(TofiConfig *p_tofi_cal_config) {
    ConfigFileData *configFileObj =
        (ConfigFileData *)p_tofi_cal_config->p_tofi_config_str;
    TofiXYZDealiasData *dealiasDataObj =
        (TofiXYZDealiasData *)p_tofi_cal_config->p_tofi_cal_config;

    delete configFileObj;
    delete dealiasDataObj;
    delete p_tofi_cal_config;
};

/**
 * @brief Sets INI configuration parameters for the ToFi computation.
 *
 * This function is a stub in the open-source implementation. In the full implementation,
 * it would update configuration parameters in the specified parameter group.
 *
 * @param[in,out] p_config_params Pointer to configuration parameters structure to update
 * @param[in] params_group Parameter group identifier
 * @param[in] p_tofi_cal_config Pointer to ToFi calibration configuration
 *
 * @return 0 on success (always succeeds in open-source stub)
 */
uint32_t TofiSetINIParams(void *p_config_params, int params_group,
                          const void *p_tofi_cal_config) {
    return 0;
};

/**
 * @brief Retrieves INI configuration parameters from the ToFi computation.
 *
 * This function is a stub in the open-source implementation. In the full implementation,
 * it would read configuration parameters from the specified parameter group.
 *
 * @param[out] p_config_params Pointer to configuration parameters structure to populate
 * @param[in] params_group Parameter group identifier
 * @param[in] p_tofi_cal_config Pointer to ToFi calibration configuration
 *
 * @return 0 on success (always succeeds in open-source stub)
 */
uint32_t TofiGetINIParams(void *p_config_params, int params_group,
                          const void *p_tofi_cal_config) {
    return 0;
};