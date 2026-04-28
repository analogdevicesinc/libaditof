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
#ifndef ADSD3500_COMMAND_IMPL_H
#define ADSD3500_COMMAND_IMPL_H

#include "adsd3500_command_interface.h"
#include "adsd3500_protocol_manager.h"
#include <memory>

namespace aditof {

/**
 * @brief Concrete implementation of ADSD3500 command interface.
 *
 * Wraps Adsd3500ProtocolManager to provide command interface abstraction.
 * Delegates all protocol operations to the underlying protocol manager.
 */
class Adsd3500CommandImpl : public Adsd3500CommandInterface {
  public:
    explicit Adsd3500CommandImpl(Adsd3500ProtocolManager *protocolManager);
    ~Adsd3500CommandImpl() override = default;

    Status readCommand(uint16_t cmd, uint16_t *data,
                       unsigned int usDelay = 0) override;
    Status writeCommand(uint16_t cmd, uint16_t data,
                        unsigned int usDelay = 0) override;
    Status readPayload(uint8_t *payload, uint32_t payloadSize) override;
    Status writePayload(const uint8_t *payload, uint32_t payloadSize) override;
    int getChipStatus() const override;
    int getImagerStatus() const override;

  private:
    Adsd3500ProtocolManager *m_protocolManager;
    int m_chipStatus;
    int m_imagerStatus;
};

} // namespace aditof

#endif // ADSD3500_COMMAND_IMPL_H
