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
#ifndef ADSD3500_CHIP_CONFIG_MANAGER_H
#define ADSD3500_CHIP_CONFIG_MANAGER_H

#include "aditof/depth_sensor_interface.h"
#include "aditof/sensor_definitions.h"
#include "sensor-tables/ini_file_definitions.h"
#include <string>
#include <unordered_map>
#include <vector>

// Forward declarations
class Adsd3500ProtocolManager;
namespace aditof {
class Adsd3500ModeSelector;
}

struct IniTableEntry;
enum class SensorImagerType;
enum class CCBVersion;

/**
 * @brief Manages ADSD3500 chip configuration discovery and mode setup.
 *
 * Responsible for:
 * - Reading firmware version and chip capabilities from ADSD3500
 * - Discovering imager type (ADSD3100/3030/ADTF3080/3066) and CCB version
 * - Reading available modes from chip NVM (CCBM) or using SDK defaults
 * - Populating INI configuration parameters for each mode
 * - Configuring frame content (depth, AB, confidence, XYZ, metadata)
 */
class Adsd3500ChipConfigManager {
  public:
    /**
     * @brief Constructs the chip configuration manager.
     *
     * @param protocolManager Reference to protocol manager for chip communication
     * @param modeSelector Reference to mode selector for mode table access
     * @param imagerType Reference to sensor imager type
     * @param ccbVersion Reference to CCB version
     * @param fwVersion Reference to firmware version string
     * @param controls Reference to sensor control map
     * @param chipId Reference to chip ID
     */
    Adsd3500ChipConfigManager(
        Adsd3500ProtocolManager &protocolManager,
        aditof::Adsd3500ModeSelector &modeSelector,
        SensorImagerType &imagerType, CCBVersion &ccbVersion,
        std::string &fwVersion,
        std::unordered_map<std::string, std::string> &controls,
        uint16_t &chipId);

    /**
     * @brief Queries the ADSD3500 chip to discover configuration.
     *
     * Performs the following steps:
     * 1. Reads firmware version and determines CCB version and imager type
     * 2. Reads available modes from chip NVM (if CCBM supported) or uses SDK defaults
     * 3. Populates INI configuration structures for each mode
     * 4. Configures frame content based on INI parameters (AB, confidence, XYZ)
     * 5. Merges INI parameters across all modes
     *
     * @param[out] availableModes Vector to populate with discovered modes
     * @param[out] ccbmINIContent Vector to populate with CCBM INI table entries
     * @param[out] iniFileStructList Vector to populate with INI file structures
     * @param[out] bitsInAB Vector to populate with AB bit depths per mode
     * @param[out] bitsInConf Vector to populate with confidence bit depths per mode
     * @param[out] ccbmEnabled Flag to set if CCBM is enabled
     *
     * @return Status::OK on success, error status on failure
     */
    aditof::Status queryChipConfiguration(
        std::vector<aditof::DepthSensorModeDetails> &availableModes,
        std::vector<IniTableEntry> &ccbmINIContent,
        std::vector<iniFileStruct> &iniFileStructList,
        std::vector<uint8_t> &bitsInAB, std::vector<uint8_t> &bitsInConf,
        bool &ccbmEnabled);

  private:
    /**
     * @brief Discovers firmware version, CCB version, and imager type from chip.
     *
     * @return Status::OK on success
     */
    aditof::Status discoverChipCapabilities();

    /**
     * @brief Reads available modes from chip NVM via CCBM.
     *
     * @param[out] availableModes Vector to populate with mode details
     * @param[out] ccbmINIContent Vector to populate with INI table entries
     *
     * @return Status::OK on success
     */
    aditof::Status readModesFromCCBM(
        std::vector<aditof::DepthSensorModeDetails> &availableModes,
        std::vector<IniTableEntry> &ccbmINIContent);

    /**
     * @brief Reads available modes from SDK defaults (CCBM disabled).
     *
     * @param[out] availableModes Vector to populate with mode details
     *
     * @return Status::OK on success
     */
    aditof::Status readModesFromSDK(
        std::vector<aditof::DepthSensorModeDetails> &availableModes);

    /**
     * @brief Creates INI parameter structures for all modes based on imager type.
     *
     * @param[in] availableModes Vector of available mode details
     * @param[out] iniFileStructList Vector to populate with INI structures
     *
     * @return Status::OK on success
     */
    aditof::Status createIniParamsForModes(
        const std::vector<aditof::DepthSensorModeDetails> &availableModes,
        std::vector<iniFileStruct> &iniFileStructList);

    /**
     * @brief Configures frame content for each mode based on INI parameters.
     *
     * Determines which frames to include (raw, depth, AB, confidence, XYZ, metadata)
     * based on the INI configuration for each mode.
     *
     * @param[in,out] availableModes Vector of mode details to update
     * @param[in] iniFileStructList Vector of INI file structures
     * @param[out] bitsInAB Vector to populate with AB bit depths
     * @param[out] bitsInConf Vector to populate with confidence bit depths
     *
     * @return Status::OK on success
     */
    aditof::Status configureFrameContent(
        std::vector<aditof::DepthSensorModeDetails> &availableModes,
        const std::vector<iniFileStruct> &iniFileStructList,
        std::vector<uint8_t> &bitsInAB, std::vector<uint8_t> &bitsInConf);

    /**
     * @brief Merges INI parameters across all modes.
     *
     * @param[in,out] iniFileStructList Vector of INI file structures to merge
     * @param[in] ccbmEnabled Whether CCBM is enabled
     * @param[in] ccbmINIContent Vector of CCBM INI table entries
     *
     * @return Status::OK on success
     */
    aditof::Status
    mergeIniParams(std::vector<iniFileStruct> &iniFileStructList,
                   bool ccbmEnabled,
                   const std::vector<IniTableEntry> &ccbmINIContent);

  private:
    Adsd3500ProtocolManager &m_protocolManager;
    aditof::Adsd3500ModeSelector &m_modeSelector;
    SensorImagerType &m_imagerType;
    CCBVersion &m_ccbVersion;
    std::string &m_fwVersion;
    std::unordered_map<std::string, std::string> &m_controls;
    uint16_t &m_chipId;
};

#endif // ADSD3500_CHIP_CONFIG_MANAGER_H
