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
#ifndef ADSD3500_MODE_SELECTOR
#define ADSD3500_MODE_SELECTOR

#include "sensor-tables/driver_configuration_table.h"

#include <aditof/camera_definitions.h>
#include <aditof/log.h>
#include <aditof/status_definitions.h>

#include <map>
#include <unordered_map>

namespace aditof {

class Adsd3500ModeSelector {
  public:
    Adsd3500ModeSelector();
    ~Adsd3500ModeSelector() = default;

    //functions to set which table configuration to use
    aditof::Status setConfiguration(const std::string &configuration);

    aditof::Status getAvailableModeDetails(
        std::vector<DepthSensorModeDetails> &m_depthSensorModeDetails);

    //populate table with hardcoded values depending on input
    aditof::Status
    getConfigurationTable(DepthSensorModeDetails &configurationTable);

    //this function should update the table with driver details
    aditof::Status
    updateConfigurationTable(DepthSensorModeDetails &configurationTable);

    //Functions used to set mode, number of bits, pixel format, etc
    aditof::Status setControl(const std::string &control,
                              const std::string &value);
    aditof::Status getControl(const std::string &control, std::string &value);
    std::string make_key(int &depthbits, int &confbits, int &ABbits);
    aditof::Status init_bitsPerPixelTable();

  private:
    std::string m_configuration;
    std::vector<std::string> m_availableConfigurations;
    std::map<std::string, std::string> m_controls;
    std::vector<aditof::DepthSensorModeDetails> m_tableInUse;
    std::unordered_map<std::string, bool> m_bitsPerPixelTable;
};
} // namespace aditof

#endif
