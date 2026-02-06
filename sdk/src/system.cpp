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
#include "system_impl.h"
#include <aditof/camera.h>
#include <aditof/system.h>

namespace aditof {

/**
 * @brief Default constructor for System.
 *
 * Creates a new System object and initializes the internal SystemImpl
 * implementation. The System class is the main entry point for discovering
 * and enumerating ToF cameras in the ADITOF SDK. This class follows the
 * PIMPL (Pointer to IMPLementation) idiom to hide implementation details
 * and maintain ABI stability.
 */
System::System() : m_impl(new SystemImpl) {}

/**
 * @brief Destructor for System.
 *
 * Cleans up the System object and releases the internal implementation.
 */
System::~System() = default;

/**
 * @brief Move constructor for System.
 *
 * Transfers ownership of the internal implementation from another System object.
 * The source System is left in a valid but unspecified state after the move.
 */
System::System(System &&) noexcept = default;

/**
 * @brief Move assignment operator for System.
 *
 * Transfers ownership of the internal implementation from another System object.
 * The source System is left in a valid but unspecified state after the move.
 *
 * @return A reference to this System object.
 */
System &System::operator=(System &&) noexcept = default;

/**
 * @brief Retrieves a list of available cameras based on the specified URI.
 *
 * Discovers and enumerates ToF cameras accessible via different connection methods:
 * - Local/target cameras (when built with ON_TARGET flag)
 * - Network cameras (URI format: "ip:<address>[:port]")
 * - Offline/replay cameras (URI format: "offline:<path>")
 *
 * This is the main method for discovering cameras in the ADITOF SDK. The URI
 * parameter determines the discovery mechanism. An empty URI will discover
 * local cameras if built for a target platform.
 *
 * @param cameraList Output vector where discovered Camera objects will be stored.
 *                   The vector is cleared before populating.
 * @param uri Connection URI specifying how to discover cameras:
 *            - Empty string or no prefix: local/target cameras (requires ON_TARGET)
 *            - "ip:<address>[:port]": network cameras at the specified IP address
 *            - "offline:<path>": offline replay from recorded data (requires HAS_OFFLINE)
 * @return Status::OK on success, Status::GENERIC_ERROR if enumeration fails or
 *         the requested connection mode is not supported in the current build.
 */
Status System::getCameraList(std::vector<std::shared_ptr<Camera>> &cameraList,
                             const std::string &uri) const {
    return m_impl->getCameraList(cameraList, uri);
}

} // namespace aditof
