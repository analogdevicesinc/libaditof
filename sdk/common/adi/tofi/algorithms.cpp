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
#include "algorithms.h"
#include "opencv_undistort.h"

#include <cstring>
#include <math.h>
#include <memory>

uint32_t Algorithms::GenerateXYZTables(
    const float **pp_x_table, const float **pp_y_table,
    const float **pp_z_table, CameraIntrinsics *p_intr_data,
    uint32_t n_sensor_rows, uint32_t n_sensor_cols, uint32_t n_out_rows,
    uint32_t n_out_cols, uint32_t n_offset_rows, uint32_t n_offset_cols,
    uint8_t row_bin_factor, uint8_t col_bin_factor, uint8_t iter) {
    uint32_t n_cols = n_sensor_cols / col_bin_factor;
    uint32_t n_rows = n_sensor_rows / row_bin_factor;

    // Use unique_ptr with custom deleter for C-style arrays (malloc compatibility)
    std::unique_ptr<float[], decltype(&free)> p_xp_unique(
        static_cast<float*>(malloc(n_rows * n_cols * sizeof(float))), free);
    std::unique_ptr<float[], decltype(&free)> p_yp_unique(
        static_cast<float*>(malloc(n_rows * n_cols * sizeof(float))), free);
    std::unique_ptr<float[], decltype(&free)> p_z_unique(
        static_cast<float*>(malloc(n_rows * n_cols * sizeof(float))), free);

    if (!p_xp_unique || !p_yp_unique || !p_z_unique) {
        return -1;
    }

    float *p_xp = p_xp_unique.get();
    float *p_yp = p_yp_unique.get();
    float *p_z = p_z_unique.get();

    // Adjust values based on optical center and focal length
    float cx = p_intr_data->cx / row_bin_factor;
    float cy = p_intr_data->cy / col_bin_factor;
    // float codx = p_intr_data->codx;
    //float cody = p_intr_data->cody;

    float r_min = sqrt((float)(n_rows * n_rows + n_cols * n_cols));

    // Generate the initial x,y tables using the positional
    // index and crop the unused pixels from the maximum in
    // each dimension
    for (uint32_t i = 0; i < n_cols; i++) {
        // Each value in a row increments by one
        p_xp[i] = (float)i;
    }
    // Replicate the rows
    for (uint32_t j = 0; j < n_rows; j++) {
        memmove(&p_xp[j * n_cols], p_xp, n_cols * sizeof(float));
    }

    for (uint32_t j = 0; j < n_rows; j++) {
        // Each row is one more than the last
        float value = (float)j;
        for (uint32_t i = 0; i < n_cols; i++) {
            // Every value in a row is the same
            p_yp[j * n_cols + i] = value;
        }
    }

    UndistortPoints(p_xp, p_yp, p_xp, p_yp, p_intr_data, iter, n_rows, n_cols,
                    row_bin_factor, col_bin_factor);

    for (uint32_t j = 0; j < n_rows; j++) {
        for (uint32_t i = 0; i < n_cols; i++) {
            int idx = j * n_cols + i;
            float xp = p_xp[idx];
            float yp = p_yp[idx];
            p_z[idx] = sqrtf(xp * xp + yp * yp + 1);
            //Check for invalid values
            if (isnan(xp) || isnan(yp) || isnan(p_z[idx]) || p_z[idx] == 0) {
                // Calculate the coordinates relative to the center pixel
                float ix = (float)i - cx;
                float iy = (float)j - cy;
                float r = sqrt(ix * ix + iy * iy);
                // Find the minimum radius with an invalid number
                if (r < r_min) {
                    r_min = r;
                }
            }
        }
    }
    //Add a 2 pixel buffer
    r_min -= 2;

    //Filter for invalid pixels
    for (uint32_t j = 0; j < n_rows; j++) {
        for (uint32_t i = 0; i < n_cols; i++) {
            int idx = j * n_cols + i;
            float ix = (float)i - cx;
            float iy = (float)j - cy;
            float r = sqrt(ix * ix + iy * iy);
            if (r >= r_min) {
                // zero if the pixel is outside the valid radius
                p_xp[idx] = 0;
                p_yp[idx] = 0;
                p_z[idx] = 0;
            }
        }
    }

    // Add a 2 pixel buffer
    r_min -= 2;

    // Allocate cropped tables (these will be transferred to caller via release())
    std::unique_ptr<float[], decltype(&free)> p_xp_cropped(
        static_cast<float*>(malloc(n_out_rows * n_out_cols * sizeof(float))), free);
    std::unique_ptr<float[], decltype(&free)> p_yp_cropped(
        static_cast<float*>(malloc(n_out_rows * n_out_cols * sizeof(float))), free);
    std::unique_ptr<float[], decltype(&free)> p_z_cropped(
        static_cast<float*>(malloc(n_out_rows * n_out_cols * sizeof(float))), free);

    if (!p_xp_cropped || !p_yp_cropped || !p_z_cropped) {
        return -1;
    }

    for (uint32_t j = 0; j < n_out_rows; j++) {
        for (uint32_t i = 0; i < n_out_cols; i++) {
            int idx = (j + n_offset_rows) * n_cols + i + n_offset_cols;
            int crop_idx = j * n_out_cols + i;
            float x = p_xp[idx];
            float y = p_yp[idx];
            float z = p_z[idx];
            if (z != 0) {
                p_xp_cropped[crop_idx] = x / z;
                p_yp_cropped[crop_idx] = y / z;
                p_z_cropped[crop_idx] = 1 / z;
            }
        }
    }

    // Original tables are automatically freed when unique_ptrs go out of scope
    // Transfer ownership to caller via release()
    // NOTE: Caller MUST call free() on these pointers to avoid memory leak.
    // These are malloc-allocated for C API compatibility.
    *pp_x_table = p_xp_cropped.release();
    *pp_y_table = p_yp_cropped.release();
    *pp_z_table = p_z_cropped.release();

    return 0;
}

uint32_t Algorithms::ComputeXYZ(const uint16_t *p_depth, XYZTable *p_xyz_data,
                                int16_t *p_xyz_image, uint32_t n_rows,
                                uint32_t n_cols) {

    for (uint32_t pixel_id = 0; pixel_id < n_rows * n_cols; pixel_id++) {
        p_xyz_image[3 * pixel_id + 0] = (int16_t)(floorf(
            p_xyz_data->p_x_table[pixel_id] * (float)p_depth[pixel_id] + 0.5f));

        p_xyz_image[3 * pixel_id + 1] = (int16_t)(floorf(
            p_xyz_data->p_y_table[pixel_id] * (float)p_depth[pixel_id] + 0.5f));

        p_xyz_image[3 * pixel_id + 2] = (int16_t)((
            p_xyz_data->p_z_table[pixel_id] * (float)p_depth[pixel_id] + 0.5f));
    }

    return 0;
}
