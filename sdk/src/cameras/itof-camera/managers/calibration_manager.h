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
#ifndef CALIBRATION_MANAGER_H
#define CALIBRATION_MANAGER_H

#include "tofi/tofi_config.h"
#include "tofi/tofi_util.h"
#include <aditof/depth_sensor_interface.h>
#include <aditof/status_definitions.h>
#include <memory>
#include <string>

namespace aditof {

/**
 * @class CalibrationManager
 * @brief Manages camera calibration data (CCB, intrinsics, XYZ tables).
 *
 * Responsible for:
 * - Reading CCB (Camera Calibration Block) from ADSD3500 EEPROM
 * - Managing camera intrinsic parameters
 * - Managing XYZ dealias tables
 * - Saving/loading calibration data to/from files
 *
 * Single Responsibility: Calibration data management and I/O
 */
class CalibrationManager {
  public:
    CalibrationManager(std::shared_ptr<DepthSensorInterface> depthSensor);
    ~CalibrationManager();

    /**
     * @brief Reads the CCB from ADSD3500 memory.
     *
     * Includes CRC validation to ensure data integrity.
     *
     * @param[out] ccb String populated with CCB data (excluding CRC trailer)
     * @return Status::OK if CCB read successfully and CRC valid
     */
    Status readCCB(std::string &ccb);

    /**
     * @brief Saves CCB to a file.
     *
     * Reads CCB from ADSD3500 and saves to specified path.
     * Includes retry logic for reliability.
     *
     * @param[in] filepath Path where CCB file should be written
     * @return Status::OK on success
     */
    Status saveCCBToFile(const std::string &filepath);

    /**
     * @brief Gets XYZ dealias data for a specific mode.
     *
     * @param[in] mode Mode number
     * @param[out] data XYZ dealias data structure
     * @return Status::OK if mode found
     */
    Status getXYZDealiasData(uint16_t mode, TofiXYZDealiasData &data) const;

    /**
     * @brief Sets XYZ dealias data for a specific mode.
     *
     * @param[in] mode Mode number
     * @param[in] data XYZ dealias data structure
     */
    void setXYZDealiasData(uint16_t mode, const TofiXYZDealiasData &data);

    /**
     * @brief Gets raw CCB data.
     *
     * @return Raw CCB data string
     */
    const std::string &getRawCCBData() const { return m_rawCCBData; }

    /**
     * @brief Sets raw CCB data.
     *
     * @param[in] ccbData Raw CCB data string
     */
    void setRawCCBData(const std::string &ccbData) { m_rawCCBData = ccbData; }

    /**
     * @brief Gets XYZ table.
     *
     * @return XYZ table structure
     */
    const XYZTable &getXYZTable() const { return m_xyzTable; }

    /**
     * @brief Sets XYZ table.
     *
     * @param[in] table XYZ table structure
     */
    void setXYZTable(const XYZTable &table) { m_xyzTable = table; }

    /**
     * @brief Cleans up allocated XYZ tables.
     *
     * Frees memory for X, Y, Z lookup tables.
     */
    void cleanupXYZtables();

  private:
    static constexpr int NR_READADSD3500CCB = 3; // Retry attempts for CCB read
    static constexpr uint32_t MAX_CCB_FILE_SIZE =
        10 * 1024 * 1024; // 10 MB safety limit

    std::shared_ptr<DepthSensorInterface> m_depthSensor;
    std::shared_ptr<Adsd3500HardwareInterface> m_adsd3500Hardware;
    std::string m_rawCCBData; // Raw Camera Calibration Block for non-ISP modes
    TofiXYZDealiasData m_xyz_dealias_data[MAX_N_MODES + 1];
    XYZTable m_xyzTable;
};

} // namespace aditof

#endif // CALIBRATION_MANAGER_H
