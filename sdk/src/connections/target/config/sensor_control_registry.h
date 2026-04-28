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
#ifndef SENSOR_CONTROL_REGISTRY_H
#define SENSOR_CONTROL_REGISTRY_H

#include <aditof/status_definitions.h>
#include <string>
#include <unordered_map>
#include <vector>

namespace aditof {

/**
 * @brief Manages sensor control parameters with validation and access control.
 *
 * Centralizes control parameter management, providing:
 * - Type-safe control registration
 * - Read-only vs read-write enforcement
 * - Value validation
 * - Enumeration of available controls
 *
 * Extracted from Adsd3500Sensor to follow Single Responsibility Principle.
 */
class SensorControlRegistry {
  public:
    /**
     * @brief Defines control metadata
     */
    struct ControlDescriptor {
        std::string name;
        std::string defaultValue;
        bool isReadOnly;
    };

    SensorControlRegistry();
    ~SensorControlRegistry() = default;

    /**
     * @brief Registers a new control parameter.
     *
     * @param name Control name (e.g., "fps", "depthEnable")
     * @param defaultValue Initial value
     * @param isReadOnly If true, setControl will fail for this control
     * @return Status::OK on success, Status::INVALID_ARGUMENT if already exists
     */
    Status registerControl(const std::string &name,
                           const std::string &defaultValue,
                           bool isReadOnly = false);

    /**
     * @brief Sets a control value.
     *
     * @param control Control name
     * @param value New value
     * @return Status::OK on success, Status::INVALID_ARGUMENT if unknown,
     *         Status::UNAVAILABLE if read-only
     */
    Status setControl(const std::string &control, const std::string &value);

    /**
     * @brief Gets a control value.
     *
     * @param control Control name
     * @param value Output parameter for control value
     * @return Status::OK on success, Status::INVALID_ARGUMENT if unknown
     */
    Status getControl(const std::string &control, std::string &value) const;

    /**
     * @brief Gets all available control names.
     *
     * @param controls Output vector of control names
     * @return Status::OK on success
     */
    Status getAvailableControls(std::vector<std::string> &controls) const;

    /**
     * @brief Checks if a control exists.
     *
     * @param control Control name
     * @return true if control is registered
     */
    bool hasControl(const std::string &control) const;

    /**
     * @brief Checks if a control is read-only.
     *
     * @param control Control name
     * @return true if control is read-only, false otherwise or if not found
     */
    bool isReadOnly(const std::string &control) const;

    /**
     * @brief Gets internal map for legacy compatibility.
     *
     * @deprecated This method is provided for backward compatibility with code
     *             that directly accesses the control map. New code should use
     *             getControl/setControl methods.
     * @return Reference to internal control values map
     */
    std::unordered_map<std::string, std::string> &getLegacyMap() {
        return m_controlValues;
    }

    const std::unordered_map<std::string, std::string> &getLegacyMap() const {
        return m_controlValues;
    }

  private:
    std::unordered_map<std::string, std::string> m_controlValues;
    std::unordered_map<std::string, bool> m_readOnlyFlags;
};

} // namespace aditof

#endif // SENSOR_CONTROL_REGISTRY_H
