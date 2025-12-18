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
#ifndef SYSTEM_H
#define SYSTEM_H

#include "sdk_exports.h"
#include "status_definitions.h"

#include <memory>
#include <string>
#include <vector>

class SystemImpl;

namespace aditof {

class Camera;

/**
 * @class System
 * @brief The TOF system that manages the cameras.
 */
class System {
  public:
    /**
     * @brief Constructor
     */
    SDK_API System();

    /**
     * @brief Destructor
     */
    SDK_API ~System();

    // Make System movable and non-copyable
    /**
     * @brief Move constructor
     */
    SDK_API System(System &&op) noexcept;

    /**
     * @brief Move assignment
     */
    SDK_API System &operator=(System &&op) noexcept;

  public:
    /**
     * @brief Populates the given list with Camera objects that correspond to
     * the available cameras.
     * @param[out] cameraList - A container to be set with the available cameras
     * @param[in] uri - A uniform resource identifier (URI) for specifying the
     * type of connectivity with the camera and the identification of the camera.
     * For no remote connection, the uri parameter should be omitted
     * For network connectivity, the URI format must be: "ip:ip-address" where
     * the ip-address is the address of the system to which the camera is
     * attached to. For example: "ip:10.43.0.1" or "ip:192.186.1.2", etc.
     * @return Status
     */
    SDK_API Status
    getCameraList(std::vector<std::shared_ptr<Camera>> &cameraList,
                  const std::string &uri = "") const;

  private:
    std::unique_ptr<SystemImpl> m_impl;
};

} // namespace aditof

#endif // SYSTEM_H
