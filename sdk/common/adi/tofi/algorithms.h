/*******************************************************************************
 * Copyright 2025(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include <cstdint>

#include "tofi_camera_intrinsics.h"
#include "tofi_config.h"

class Algorithms {
  public:
    static uint32_t GenerateXYZTables(
        const float **pp_x_table, const float **pp_y_table,
        const float **pp_z_table, CameraIntrinsics *p_intr_data,
        uint32_t n_sensor_rows, uint32_t n_sensor_cols, uint32_t n_out_rows,
        uint32_t n_out_cols, uint32_t n_offset_rows, uint32_t n_offset_cols,
        uint8_t row_bin_factor, uint8_t col_bin_factor, uint8_t iter);

    static uint32_t ComputeXYZ(const uint16_t *p_depth, XYZTable *p_xyz_data,
                               int16_t *p_xyz_image, uint32_t n_rows,
                               uint32_t n_cols);
};

#endif // ALGORITHMS_H
