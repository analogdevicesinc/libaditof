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
#include "sensor_control_registry.h"
#include <aditof/log.h>

namespace aditof {

SensorControlRegistry::SensorControlRegistry() {
    // Default controls registered by Adsd3500Sensor
    registerControl("abAveraging", "0", false);
    registerControl("depthEnable", "0", false);
    registerControl("phaseDepthBits", "0", false);
    registerControl("abBits", "0", false);
    registerControl("confidenceBits", "0", false);
    registerControl("fps", "0", false);
    registerControl("imagerType", "", true); // Read-only
    registerControl("inputFormat", "", false);
    registerControl("netlinktest", "0", false);
    registerControl("depthComputeOpenSource", "0", true); // Read-only
    registerControl("disableCCBM", "0", false);
    registerControl("availableCCBM", "0", true); // Read-only
    registerControl("lensScatterCompensationEnabled", "0", false);
    registerControl("enableRotation", "0", false);
    // Target mode for runtime bit configuration: Specifies which mode's bit arrays
    // to update when abBits/confidenceBits are set before setMode().
    // -1 = use current mode, >= 0 = specific mode index.
    // Must be set before configureSensorModeDetails() to take effect.
    registerControl("targetModeNumber", "-1", false);
}

Status SensorControlRegistry::registerControl(const std::string &name,
                                              const std::string &defaultValue,
                                              bool isReadOnly) {
    if (hasControl(name)) {
        LOG(WARNING) << "Control '" << name << "' already registered";
        return Status::INVALID_ARGUMENT;
    }

    m_controlValues[name] = defaultValue;
    m_readOnlyFlags[name] = isReadOnly;

    return Status::OK;
}

Status SensorControlRegistry::setControl(const std::string &control,
                                         const std::string &value) {
    if (!hasControl(control)) {
        LOG(WARNING) << "Unsupported control: " << control;
        return Status::INVALID_ARGUMENT;
    }

    if (isReadOnly(control)) {
        LOG(WARNING) << "Control '" << control << "' is read-only";
        return Status::UNAVAILABLE;
    }

    m_controlValues[control] = value;
    return Status::OK;
}

Status SensorControlRegistry::getControl(const std::string &control,
                                         std::string &value) const {
    auto it = m_controlValues.find(control);
    if (it == m_controlValues.end()) {
        LOG(WARNING) << "Unsupported control: " << control;
        return Status::INVALID_ARGUMENT;
    }

    value = it->second;
    return Status::OK;
}

Status SensorControlRegistry::getAvailableControls(
    std::vector<std::string> &controls) const {
    controls.clear();
    controls.reserve(m_controlValues.size());

    for (const auto &item : m_controlValues) {
        controls.emplace_back(item.first);
    }

    return Status::OK;
}

bool SensorControlRegistry::hasControl(const std::string &control) const {
    return m_controlValues.count(control) > 0;
}

bool SensorControlRegistry::isReadOnly(const std::string &control) const {
    auto it = m_readOnlyFlags.find(control);
    if (it == m_readOnlyFlags.end()) {
        return false;
    }
    return it->second;
}

} // namespace aditof
