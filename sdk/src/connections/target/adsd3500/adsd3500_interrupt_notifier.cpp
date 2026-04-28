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
#include "adsd3500_interrupt_notifier.h"
#include "adsd3500_sensor.h"
#include <aditof/log.h>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define SIGETX 44
#define USER_TASK _IOW('A', 1, int32_t *)

std::vector<std::weak_ptr<Adsd3500Sensor>> Adsd3500InterruptNotifier::m_sensors;

/**
 * @brief Returns the singleton instance of Adsd3500InterruptNotifier.
 *
 * Implements the singleton pattern to ensure only one interrupt notifier exists.
 * The instance is created on first call and persists for the application lifetime.
 *
 * @return Reference to the singleton Adsd3500InterruptNotifier instance
 */
Adsd3500InterruptNotifier &Adsd3500InterruptNotifier::getInstance() {
    static auto &&notifier = Adsd3500InterruptNotifier();
    return (notifier);
}

/**
 * @brief Signal handler for ADSD3500 hardware interrupts from kernel.
 *
 * This function is registered as the signal handler for SIGETX (signal 44). When the
 * kernel driver raises an interrupt, this handler notifies all subscribed Adsd3500Sensor
 * instances by calling their interrupt handler methods with the signal value.
 *
 * @param[in] n Signal number received
 * @param[in] info Signal information structure containing interrupt details
 * @param[in] unused Unused context parameter (required by signal handler signature)
 */
void Adsd3500InterruptNotifier::signalEventHandler(int n, siginfo_t *info,
                                                   void *unused) {
    if (n == SIGETX) {
        int signal_value = info->si_int;
        DLOG(INFO) << "Received signal " << info->si_int << " from kernel";

        for (auto sensor : m_sensors) {
            if (std::shared_ptr<Adsd3500Sensor> sptr = sensor.lock()) {
                sptr->adsd3500InterruptHandler(signal_value);
            }
        }
    }
}

/**
 * @brief Enables ADSD3500 hardware interrupt notifications.
 *
 * Registers a signal handler (signalEventHandler) for SIGETX and communicates with the
 * kernel driver via /proc/adsd3500/value to subscribe to interrupt events. Uses ioctl
 * USER_TASK to register the application process with the kernel driver for interrupt
 * delivery.
 *
 * @return Status::OK if interrupts are successfully enabled,
 *         Status::UNAVAILABLE if debugfs interface cannot be opened or ioctl fails
 */
aditof::Status Adsd3500InterruptNotifier::enableInterrupts() {
    // Subscribe to the ADSD3500 interrupt
    struct sigaction act;
    int32_t number;

    sigemptyset(&act.sa_mask);
    act.sa_flags = (SA_SIGINFO | SA_RESTART);
    act.sa_sigaction = Adsd3500InterruptNotifier::signalEventHandler;
    sigaction(SIGETX, &act, NULL);

    m_interruptsAvailable = false;

    const char *debugfs_name = "/proc/adsd3500/value";
    int debug_fd = ::open(debugfs_name, O_RDWR);
    if (debug_fd < 0) {
        LOG(WARNING) << "Failed to open " << debugfs_name << ", "
                     << "Interrupts support will not be available!";
        return aditof::Status::UNAVAILABLE;
    }

    if (ioctl(debug_fd, USER_TASK, (int32_t *)&number)) {
        LOG(WARNING) << "Failed to register the application process with the "
                        "kernel driver. IOCTL failed.";
        close(debug_fd);
    }

    m_interruptsAvailable = true;

    return aditof::Status::OK;
}

/**
 * @brief Disables ADSD3500 hardware interrupt notifications.
 *
 * This is a stub implementation that currently performs no action but returns success.
 *
 * @return Status::OK on success
 */
aditof::Status Adsd3500InterruptNotifier::disableInterrupts() {
    return aditof::Status::OK;
}

/**
 * @brief Checks if hardware interrupts are available and enabled.
 *
 * Returns the status of interrupt availability based on successful enableInterrupts() call.
 *
 * @return true if interrupts are enabled and available, false otherwise
 */
bool Adsd3500InterruptNotifier::interruptsAvailable() {
    return m_interruptsAvailable;
}

/**
 * @brief Subscribes a sensor to receive interrupt notifications.
 *
 * Adds the sensor to the list of subscribers that will be notified when hardware
 * interrupts are received. Uses weak_ptr to avoid circular dependencies and allow
 * sensors to be destroyed without explicit unsubscription.
 *
 * @param[in] sensor Weak pointer to the Adsd3500Sensor to subscribe
 */
void Adsd3500InterruptNotifier::subscribeSensor(
    std::weak_ptr<Adsd3500Sensor> sensor) {
    m_sensors.emplace_back(sensor);
}

/**
 * @brief Unsubscribes a sensor from interrupt notifications.
 *
 * This is a stub implementation that currently performs no action. Sensors are
 * automatically removed when their weak_ptr expires.
 *
 * @param[in] sensor Weak pointer to the Adsd3500Sensor to unsubscribe
 */
void Adsd3500InterruptNotifier::unsubscribeSensor(
    std::weak_ptr<Adsd3500Sensor> sensor) {}
