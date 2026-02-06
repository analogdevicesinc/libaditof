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

/**
 * @brief Equality operator for FrameDataDetails.
 *
 * Compares two FrameDataDetails objects for equality. Two FrameDataDetails
 * are considered equal if their type, width, and height are all identical.
 *
 * @param lhs The left-hand side FrameDataDetails object.
 * @param rhs The right-hand side FrameDataDetails object.
 * @return true if the objects are equal, false otherwise.
 */
bool operator==(const FrameDataDetails &lhs, const FrameDataDetails &rhs) {
    return (lhs.type == rhs.type) && (lhs.width == rhs.width) &&
           (lhs.height == rhs.height);
}

/**
 * @brief Inequality operator for FrameDataDetails.
 *
 * Compares two FrameDataDetails objects for inequality. Returns the opposite
 * of the equality operator.
 *
 * @param lhs The left-hand side FrameDataDetails object.
 * @param rhs The right-hand side FrameDataDetails object.
 * @return true if the objects are not equal, false otherwise.
 */
bool operator!=(const FrameDataDetails &lhs, const FrameDataDetails &rhs) {
    return !(lhs == rhs);
}

/**
 * @brief Equality operator for FrameDetails.
 *
 * Compares two FrameDetails objects for equality. Two FrameDetails are
 * considered equal if their type, cameraMode, and all dataDetails elements
 * are identical.
 *
 * @param lhs The left-hand side FrameDetails object.
 * @param rhs The right-hand side FrameDetails object.
 * @return true if the objects are equal, false otherwise.
 */
bool operator==(const FrameDetails &lhs, const FrameDetails &rhs) {
    return (lhs.type == rhs.type) && (lhs.cameraMode == rhs.cameraMode) &&
           (lhs.dataDetails == rhs.dataDetails);
}

/**
 * @brief Inequality operator for FrameDetails.
 *
 * Compares two FrameDetails objects for inequality. Returns the opposite
 * of the equality operator.
 *
 * @param lhs The left-hand side FrameDetails object.
 * @param rhs The right-hand side FrameDetails object.
 * @return true if the objects are not equal, false otherwise.
 */
bool operator!=(const FrameDetails &lhs, const FrameDetails &rhs) {
    return !(lhs == rhs);
}

} // namespace aditof
