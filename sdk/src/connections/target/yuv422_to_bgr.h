/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Analog Devices, Inc.
 * All rights reserved.
 */
#ifndef YUV422_TO_BGR_H
#define YUV422_TO_BGR_H

#ifdef HAS_RGB_CAMERA

#include "aditof/ar0234_sensor.h"

namespace aditof {

/**
 * @brief Converts a packed UYVY or YUYV frame to BGR in place.
 *
 * On success frame.data holds width x height x 3 bytes, format is BGR and
 * stride is width * 3; a BGR frame is left unchanged.
 * @return false for any other format, an odd width or a short buffer.
 */
bool convertYuv422ToBgr(RGBFrame &frame, RGBYuvMatrix matrix, bool fullRange);

} // namespace aditof

#endif // HAS_RGB_CAMERA

#endif // YUV422_TO_BGR_H
