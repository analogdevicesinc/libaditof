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
#include <aditof/frame_operations.h>

namespace aditof {

bool operator==(const FrameDataDetails &lhs, const FrameDataDetails &rhs) {
    return (lhs.type == rhs.type) && (lhs.width == rhs.width) &&
           (lhs.height == rhs.height);
}

bool operator!=(const FrameDataDetails &lhs, const FrameDataDetails &rhs) {
    return !(lhs == rhs);
}

bool operator==(const FrameDetails &lhs, const FrameDetails &rhs) {
    return (lhs.type == rhs.type) && (lhs.cameraMode == rhs.cameraMode) &&
           (lhs.dataDetails == rhs.dataDetails);
}

bool operator!=(const FrameDetails &lhs, const FrameDetails &rhs) {
    return !(lhs == rhs);
}

} // namespace aditof
