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
#ifndef CAMERA_CONFIGURATION_H
#define CAMERA_CONFIGURATION_H

#include <aditof/status_definitions.h>
#include <map>
#include <string>
#include <vector>

namespace aditof {

/**
 * @class CameraConfiguration
 * @brief Manages camera configuration loading, parsing, and storage.
 *
 * Responsible for:
 * - Auto-discovering configuration files from filesystem or environment
 * - Parsing JSON configuration files (depth compute params, global settings)
 * - Storing and retrieving per-mode depth processing parameters
 * - Managing global camera settings (MIPI speed, deskew, temp compensation, etc.)
 *
 * Single Responsibility: Configuration file I/O and parameter management
 */
class CameraConfiguration {
  public:
    CameraConfiguration();
    ~CameraConfiguration();

    /**
     * @brief Auto-discovers JSON configuration file from binary path or environment.
     *
     * Search order:
     * 1. Binary directory (same dir as executable): adcam_config.json
     * 2. Environment variable: ADCAM_CONFIG_PATH
     * 3. Returns empty string if not found
     *
     * @return Path to discovered JSON file, or empty string if not found
     */
    std::string autoDiscoverConfigFile();

    /**
     * @brief Loads depth parameters from a JSON configuration file.
     *
     * Parses JSON and populates depth parameter map with mode-specific settings.
     * Also extracts global settings (MIPI speed, deskew, temp compensation,
     * dynamic mode switching sequences, etc.).
     *
     * @param[in] pathFile Path to the JSON configuration file
     * @param[in] mode_in_use If >= 0, load only this mode; if < 0, load all modes
     * @return Status::OK on success, error codes on failure
     */
    Status loadDepthParamsFromJsonFile(const std::string &pathFile,
                                       const int16_t mode_in_use = -1);

    /**
     * @brief Saves current depth parameters to a JSON configuration file.
     *
     * Exports all depth processing parameters and global settings to JSON.
     * Parameters are organized by mode with depth-compute and configuration sections.
     *
     * @param[in] savePathFile Path where JSON file should be written
     * @return Status::OK on success, error codes on failure
     */
    Status saveDepthParamsToJsonFile(const std::string &savePathFile);

    /**
     * @brief Retrieves depth parameters for a specific mode.
     *
     * @param[in] mode Mode number
     * @param[out] params Map of parameter name -> value
     * @return Status::OK if mode found, error otherwise
     */
    Status
    getDepthParamsForMode(uint16_t mode,
                          std::map<std::string, std::string> &params) const;

    /**
     * @brief Sets depth parameters for a specific mode.
     *
     * @param[in] mode Mode number
     * @param[in] params Map of parameter name -> value
     * @return Status::OK on success
     */
    Status
    setDepthParamsForMode(uint16_t mode,
                          const std::map<std::string, std::string> &params);

    /**
     * @brief Clears all stored depth parameters.
     */
    void clearDepthParams();

    /**
     * @brief Caches current depth parameters for later reset.
     */
    void cacheDepthParams();

    /**
     * @brief Restores depth parameters from cached values.
     *
     * @return Status::OK if cache available, error otherwise
     */
    Status restoreCachedDepthParams();

    // Global configuration getters/setters
    int16_t getMipiOutputSpeed() const { return m_mipiOutputSpeed; }
    void setMipiOutputSpeed(int16_t speed) { m_mipiOutputSpeed = speed; }

    int16_t getDeskewEnabled() const { return m_isdeskewEnabled; }
    void setDeskewEnabled(int16_t enabled) { m_isdeskewEnabled = enabled; }

    int16_t getTempCompensation() const { return m_enableTempCompensation; }
    void setTempCompensation(int16_t enabled) {
        m_enableTempCompensation = enabled;
    }

    int16_t getEdgeConfidence() const { return m_enableEdgeConfidence; }
    void setEdgeConfidence(int16_t enabled) {
        m_enableEdgeConfidence = enabled;
    }

    int16_t getMetadataInAB() const { return m_enableMetaDatainAB; }
    void setMetadataInAB(int16_t enabled) { m_enableMetaDatainAB = enabled; }

    int16_t getFsyncMode() const { return m_fsyncMode; }
    void setFsyncMode(int16_t mode) { m_fsyncMode = mode; }

    bool getDropFirstFrame() const { return m_dropFirstFrame; }
    void setDropFirstFrame(bool drop) { m_dropFirstFrame = drop; }

    const std::vector<std::pair<uint8_t, uint8_t>> &getDmsSequence() const {
        return m_configDmsSequence;
    }
    void setDmsSequence(const std::vector<std::pair<uint8_t, uint8_t>> &seq) {
        m_configDmsSequence = seq;
    }

    const std::map<std::string, std::string> &getIniKeyValPairs() const {
        return m_iniKeyValPairs;
    }
    void setIniKeyValPairs(const std::map<std::string, std::string> &pairs) {
        m_iniKeyValPairs = pairs;
    }

  private:
    /**
     * @brief Checks if a string is convertible to double.
     *
     * @param[in] str String to check
     * @return true if convertible, false otherwise
     */
    bool isConvertibleToDouble(const std::string &str);

    // Depth processing parameters per mode
    std::map<int, std::map<std::string, std::string>> m_depth_params_map;
    std::map<int, std::map<std::string, std::string>> m_depth_params_map_cache;

    // Global configuration settings
    int16_t m_mipiOutputSpeed;
    int16_t m_isdeskewEnabled;
    int16_t m_enableTempCompensation;
    int16_t m_enableEdgeConfidence;
    int16_t m_enableMetaDatainAB;
    int16_t m_fsyncMode;
    bool m_dropFirstFrame;
    std::vector<std::pair<uint8_t, uint8_t>> m_configDmsSequence;
    std::map<std::string, std::string> m_iniKeyValPairs;
};

} // namespace aditof

#endif // CAMERA_CONFIGURATION_H
