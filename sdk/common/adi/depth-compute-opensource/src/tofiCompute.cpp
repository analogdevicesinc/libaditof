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
#include <cstdint>
#include <iostream>
#include <math.h>
#include <string.h>

#include "algorithms.h"
#include "tofi_compute.h"
#include "tofi_error.h"

#define GEN_XYZ_ITERATIONS 20

typedef struct {
    int n_depth;
    int n_ab;
    int n_conf;
    XYZTable xyz_table;
} PrivateData;

/**
 * @brief Initializes a ToFi compute context for depth frame processing.
 *
 * This function creates and initializes a TofiComputeContext structure for processing
 * depth frames from the ISP. It extracts bit depth configuration for depth, AB, and
 * confidence channels from the calibration data, then generates XYZ lookup tables for
 * 3D point cloud computation using camera intrinsics and distortion parameters.
 *
 * @param[in] p_tofi_cal_config Pointer to ToFi calibration configuration (cast from TofiXYZDealiasData)
 * @param[out] p_status Pointer to status variable for error reporting
 *
 * @return Pointer to newly allocated TofiComputeContext on success, nullptr on failure
 */
TofiComputeContext *InitTofiCompute(const void *p_tofi_cal_config,
                                    uint32_t *p_status) {
    TofiComputeContext *Obj = new TofiComputeContext;
    PrivateData *privDataObj = new PrivateData;

    // Extract the number of bits for: Depth, AB, Confidence
    TofiXYZDealiasData *ccb_data = (TofiXYZDealiasData *)p_tofi_cal_config;
    uint16_t bits = ccb_data->Freq[0];
    privDataObj->n_depth = bits & 0x001F;
    privDataObj->n_ab = (bits & 0x03E0) >> 5;
    privDataObj->n_conf = (bits & 0x3C00) >> 10;

    // Generate the X, Y, Z tables
    memset(&privDataObj->xyz_table, 0, sizeof(privDataObj->xyz_table));

    int n_cols = ccb_data->n_cols;
    int n_rows = ccb_data->n_rows;

    int status = Algorithms::GenerateXYZTables(
        &privDataObj->xyz_table.p_x_table, &privDataObj->xyz_table.p_y_table,
        &privDataObj->xyz_table.p_z_table, &(ccb_data->camera_intrinsics),
        ccb_data->n_sensor_rows, ccb_data->n_sensor_cols, n_rows, n_cols,
        ccb_data->n_offset_rows, ccb_data->n_offset_cols,
        ccb_data->row_bin_factor, ccb_data->col_bin_factor, GEN_XYZ_ITERATIONS);
    if (status != 0 || !privDataObj->xyz_table.p_x_table ||
        !privDataObj->xyz_table.p_y_table || !privDataObj->xyz_table.p_z_table)
        return nullptr;

    // Set context
    Obj->n_cols = 0;
    Obj->n_rows = 0;
    Obj->p_ab_frame = 0;
    Obj->p_cal_config = (void *)p_tofi_cal_config;
    Obj->p_conf_frame = 0;
    Obj->p_depth16_frame = 0;
    Obj->p_depth_frame = 0;
    Obj->p_tofi_processor_config = (void *)privDataObj;
    Obj->p_xyz_frame = 0;
    return Obj;
};

#define NUM_BITS(Input, n_pos, n_bits) (((1 << n_bits) - 1) & (Input >> n_pos))

/**
 * @brief Deinterleaves packed frame data into separate depth, confidence, and AB channels.
 *
 * This function extracts depth, confidence, and AB (amplitude/brightness) data from
 * the interleaved raw frame buffer produced by the ISP. The bit layout and packing
 * depend on the frame mode, with each pixel's components packed into consecutive bytes.
 * The function uses bit masking and shifting to extract each channel component.
 *
 * @param[in] p_frame_data Pointer to packed/interleaved raw frame data from ISP
 * @param[in] n_bits_in_depth Number of bits used to encode depth per pixel
 * @param[in] n_bits_in_conf Number of bits used to encode confidence per pixel
 * @param[in] n_bits_in_ab Number of bits used to encode AB per pixel
 * @param[in] n_bytes Number of bytes per pixel in the packed format
 * @param[in] width Frame width in pixels
 * @param[in] height Frame height in pixels
 * @param[out] p_depth Pointer to output depth buffer (uint16_t per pixel)
 * @param[out] p_conf Pointer to output confidence buffer (uint16_t per pixel), or NULL to skip
 * @param[out] p_ab Pointer to output AB buffer (uint16_t per pixel), or NULL to skip
 *
 * @return 0 on success
 */
static uint32_t
DeInterleaveDepth(uint8_t *p_frame_data, uint32_t n_bits_in_depth,
                  uint32_t n_bits_in_conf, uint32_t n_bits_in_ab,
                  uint32_t n_bytes, uint32_t width, uint32_t height,
                  uint16_t *p_depth, uint16_t *p_conf, uint16_t *p_ab) {
    uint8_t *input_buffer = p_frame_data;

    uint16_t *out_depth = p_depth;
    uint16_t *out_conf = p_conf;
    uint16_t *out_ab = p_ab;

    uint32_t n_pos_conf = (16 - n_bits_in_depth) ? 16 - n_bits_in_depth : 8;
    uint32_t n_depth_conf = n_bits_in_depth + n_bits_in_conf;
    uint32_t div = n_depth_conf % 8;
    uint32_t n_count_conf = n_bits_in_ab ? n_depth_conf / 8 : 0;
    uint32_t n_pos_ab = div ? 4 : 0;
    uint32_t is_conf = n_depth_conf == 16 ? 0 : 2;
    uint32_t n_ab_count = n_bits_in_ab == 8 ? 0 : n_count_conf + 1;

    uint32_t n_pixels = width * height;

    for (uint32_t pix_id = 0; pix_id < n_pixels; pix_id++) {
        input_buffer = p_frame_data + pix_id * n_bytes;

        uint16_t temp = input_buffer[0] | (uint16_t)(input_buffer[1] << 8);
        out_depth[pix_id] = NUM_BITS(temp, 0, n_bits_in_depth);

        if (out_conf) {
            temp = input_buffer[1] | (uint16_t)(input_buffer[is_conf] << 8);
            out_conf[pix_id] = NUM_BITS(temp, n_pos_conf, n_bits_in_conf);
        }

        if (out_ab) {
            temp = input_buffer[n_count_conf] |
                   (uint16_t)(input_buffer[n_ab_count] << 8);
            out_ab[pix_id] = NUM_BITS(temp, n_pos_ab, n_bits_in_ab);
        }
    }
    return 0;
}

/**
 * @brief Processes raw ToF frame data to extract depth, confidence, AB, and optionally XYZ point cloud.
 *
 * This is the main processing function for depth frames from the ISP. It deinterleaves
 * the packed input frame into separate depth, confidence, and AB channels based on the
 * bit configuration from initialization. If the context's p_xyz_frame pointer is set,
 * it also computes a 3D point cloud from the depth data using precomputed lookup tables.
 *
 * @param[in] input_frame Pointer to raw interleaved frame data from ISP (cast to uint16_t)
 * @param[in,out] p_tofi_compute_context Pointer to TofiComputeContext with output buffers:
 *                                       - p_depth_frame: output depth buffer
 *                                       - p_conf_frame: output confidence buffer
 *                                       - p_ab_frame: output AB buffer
 *                                       - p_xyz_frame: output XYZ point cloud buffer (optional)
 * @param[in] p_temperature Pointer to temperature information (currently unused)
 *
 * @return 0 on success, non-zero on error
 */
int TofiCompute(const uint16_t *const input_frame,
                TofiComputeContext *const p_tofi_compute_context,
                TemperatureInfo *p_temperature) {

    TofiXYZDealiasData *ccb_data =
        (TofiXYZDealiasData *)p_tofi_compute_context->p_cal_config;
    int n_cols = ccb_data->n_cols;
    int n_rows = ccb_data->n_rows;

    PrivateData *p =
        (PrivateData *)p_tofi_compute_context->p_tofi_processor_config;
    int n_depth = p->n_depth;
    int n_ab = p->n_ab;
    int n_conf = p->n_conf;
    int n_sum_bits = n_depth + n_conf + n_ab;
    int n_bytes = n_sum_bits / 8;

    int status = DeInterleaveDepth(
        (uint8_t *)input_frame, n_depth, n_conf, n_ab, n_bytes, n_cols, n_rows,
        p_tofi_compute_context->p_depth_frame,
        (uint16_t *)p_tofi_compute_context->p_conf_frame,
        p_tofi_compute_context->p_ab_frame);

    if (status != 0) {
        std::cout << "Unable to deinterleave frame data !" << std::endl;
    }

    // Compute Point cloud if needed (when a location address to XYZ is provided)
    if (p_tofi_compute_context->p_xyz_frame) {
        status = Algorithms::ComputeXYZ(
            p_tofi_compute_context->p_depth_frame, &p->xyz_table,
            p_tofi_compute_context->p_xyz_frame, n_rows, n_cols);

        if (status != 0) {
            std::cout << "Unable to compute XYZ !" << std::endl;
        }
    }

    return 0;
};

/**
 * @brief Frees memory allocated for a TofiComputeContext and its associated data.
 *
 * This function deallocates all heap memory associated with a TofiComputeContext,
 * including the XYZ lookup tables (x, y, z) and private data structures that were
 * allocated during InitTofiCompute. Must be called to avoid memory leaks.
 *
 * @param[in] p_tofi_compute_context Pointer to TofiComputeContext structure to be freed
 */
void FreeTofiCompute(TofiComputeContext *p_tofi_compute_context) {
    PrivateData *p =
        (PrivateData *)p_tofi_compute_context->p_tofi_processor_config;
    // Free tabels x, y, z
    if (p->xyz_table.p_x_table) {
        free((void *)p->xyz_table.p_x_table);
    }
    if (p->xyz_table.p_y_table) {
        free((void *)p->xyz_table.p_y_table);
    }
    if (p->xyz_table.p_z_table) {
        free((void *)p->xyz_table.p_z_table);
    }
    delete p;
    delete p_tofi_compute_context;
};
