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
#include <aditof/status_definitions.h>
#include <memory>
#include <signal.h>
#include <vector>

class Adsd3500Sensor;

class Adsd3500InterruptNotifier final {
  private:
    Adsd3500InterruptNotifier() = default;

  public:
    Adsd3500InterruptNotifier(const Adsd3500InterruptNotifier &) = delete;
    ~Adsd3500InterruptNotifier() = default;
    void operator=(const Adsd3500InterruptNotifier &) = delete;
    Adsd3500InterruptNotifier(Adsd3500InterruptNotifier &&) noexcept = default;
    Adsd3500InterruptNotifier &
    operator=(Adsd3500InterruptNotifier &&) noexcept = default;

  public:
    static Adsd3500InterruptNotifier &getInstance();
    aditof::Status enableInterrupts();
    aditof::Status disableInterrupts();
    bool interruptsAvailable();
    void subscribeSensor(std::weak_ptr<Adsd3500Sensor> sensor);
    void unsubscribeSensor(std::weak_ptr<Adsd3500Sensor> sensor);

  private:
    static void signalEventHandler(int n, siginfo_t *info, void *unused);

  private:
    static std::vector<std::weak_ptr<Adsd3500Sensor>> m_sensors;
    bool m_interruptsAvailable;
};
