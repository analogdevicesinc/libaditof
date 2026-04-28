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
#ifndef ADSD3500_INTERRUPT_MANAGER_H
#define ADSD3500_INTERRUPT_MANAGER_H

#include <aditof/depth_sensor_interface.h>
#include <aditof/sensor_definitions.h>
#include <aditof/status_definitions.h>
#include <unordered_map>

// Forward declarations
class Adsd3500ProtocolManager;

/**
 * @class Adsd3500InterruptManager
 * @brief Manages ADSD3500 hardware interrupt callbacks and status.
 *
 * Responsible for:
 * - Registering/unregistering interrupt callbacks
 * - Handling interrupt events from hardware
 * - Reading and converting chip/imager status
 * - Invoking registered callbacks on interrupt events
 */
class Adsd3500InterruptManager {
  public:
    /**
     * @brief Constructs interrupt manager with references to status variables.
     *
     * @param protocolManager Protocol manager for reading status registers
     * @param chipStatus Reference to chip status variable
     * @param imagerStatus Reference to imager status variable
     * @param interruptAvailable Reference to interrupt available flag
     * @param interruptCallbackMap Reference to interrupt callback map
     */
    Adsd3500InterruptManager(
        Adsd3500ProtocolManager *protocolManager, int &chipStatus,
        int &imagerStatus, bool &interruptAvailable,
        std::unordered_map<void *, aditof::SensorInterruptCallback>
            &interruptCallbackMap);

    ~Adsd3500InterruptManager() = default;

    /**
     * @brief Registers an interrupt callback for ADSD3500 events.
     *
     * @param cb Callback function to invoke on interrupt events
     * @return Status::OK on success, Status::UNAVAILABLE if interrupts not available
     */
    aditof::Status
    adsd3500_register_interrupt_callback(aditof::SensorInterruptCallback &cb);

    /**
     * @brief Unregisters an interrupt callback.
     *
     * @param cb Callback function to remove from interrupt events
     * @return Status::OK on success
     */
    aditof::Status
    adsd3500_unregister_interrupt_callback(aditof::SensorInterruptCallback &cb);

    /**
     * @brief Handles ADSD3500 hardware interrupts.
     *
     * Reads interrupt status, updates chip/imager status, and invokes all registered callbacks.
     *
     * @param signalValue Signal value from the interrupt
     * @return Status::OK on success
     */
    aditof::Status adsd3500InterruptHandler(int signalValue);

    /**
     * @brief Converts numeric status ID to Adsd3500Status enumeration.
     *
     * @param status Numeric status code from ADSD3500 hardware
     * @return Corresponding Adsd3500Status enum value
     */
    static aditof::Adsd3500Status convertIdToAdsd3500Status(int status);

  private:
    Adsd3500ProtocolManager
        *m_protocolManager;     ///< Protocol manager for status reads
    int &m_chipStatus;          ///< Reference to chip status
    int &m_imagerStatus;        ///< Reference to imager status
    bool &m_interruptAvailable; ///< Reference to interrupt available flag
    std::unordered_map<void *, aditof::SensorInterruptCallback>
        &m_interruptCallbackMap; ///< Reference to interrupt callback map
};

#endif // ADSD3500_INTERRUPT_MANAGER_H
