/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this
license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without
modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright
notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote
products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is"
and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are
disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any
direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "opencv_undistort.h"

/**
 * @brief Undistorts image points using camera intrinsic parameters and distortion coefficients.
 *
 * This function compensates for lens distortion by applying an iterative algorithm that uses
 * the camera's intrinsic parameters (focal length, principal point) and distortion coefficients
 * (radial k1-k6 and tangential p1-p2). It converts distorted image coordinates to ideal
 * undistorted coordinates, accounting for binning factors.
 *
 * @param[in] _srcx Pointer to array of source x-coordinates (distorted)
 * @param[in] _srcy Pointer to array of source y-coordinates (distorted)
 * @param[out] _dstx Pointer to array of destination x-coordinates (undistorted, normalized)
 * @param[out] _dsty Pointer to array of destination y-coordinates (undistorted, normalized)
 * @param[in] _cameraMatrix Pointer to camera intrinsics structure containing focal lengths (fx, fy),
 *                          principal point (cx, cy), and distortion coefficients (k1-k6, p1-p2)
 * @param[in] maxcount Maximum number of iterations for the distortion compensation algorithm
 * @param[in] rows Number of rows in the image
 * @param[in] cols Number of columns in the image
 * @param[in] row_bin_factor Row binning factor to adjust focal length and principal point
 * @param[in] col_bin_factor Column binning factor to adjust focal length and principal point
 */
void UndistortPoints(float *_srcx, float *_srcy, float *_dstx, float *_dsty,
                     CameraIntrinsics *_cameraMatrix, int maxcount, int rows,
                     int cols, uint8_t row_bin_factor, uint8_t col_bin_factor) {
    double k1 = _cameraMatrix->k1;
    double k2 = _cameraMatrix->k2;
    double k3 = _cameraMatrix->k3;
    double k4 = _cameraMatrix->k4;
    double k5 = _cameraMatrix->k5;
    double k6 = _cameraMatrix->k6;

    double p2 = _cameraMatrix->p2;
    double p1 = _cameraMatrix->p1;

    double k[14] = {k1, k2, p1, p2, k3, k4, k5, k6, 0, 0, 0, 0, 0, 0};

    double fx = _cameraMatrix->fx / row_bin_factor;
    double fy = _cameraMatrix->fy / col_bin_factor;

    double ifx = 1. / fx;
    double ify = 1. / fy;
    double cx = _cameraMatrix->cx / row_bin_factor;
    double cy = _cameraMatrix->cy / col_bin_factor;

    int n = rows * cols;
    for (int i = 0; i < n; i++) {
        double x, y, x0 = 0, y0 = 0, u, v;

        x = _srcx[i];
        y = _srcy[i];

        u = x;
        v = y;
        x = (x - cx) * ifx;
        y = (y - cy) * ify;

        // if (_distCoeffs) {
        // compensate tilt distortion

        x0 = x;
        y0 = y;

        // compensate distortion iteratively

        for (int j = 0; j < maxcount; j++) {
            double r2 = x * x + y * y;
            double icdist = (1 + ((k[7] * r2 + k[6]) * r2 + k[5]) * r2) /
                            (1 + ((k[4] * r2 + k[1]) * r2 + k[0]) * r2);
            if (icdist < 0) // test: undistortPoints.regression_14583
            {
                x = (u - cx) * ifx;
                y = (v - cy) * ify;
                break;
            }
            double deltaX = 2 * k[2] * x * y + k[3] * (r2 + 2 * x * x) +
                            k[8] * r2 + k[9] * r2 * r2;
            double deltaY = k[2] * (r2 + 2 * y * y) + 2 * k[3] * x * y +
                            k[10] * r2 + k[11] * r2 * r2;
            x = (x0 - deltaX) * icdist;
            y = (y0 - deltaY) * icdist;
        }
        _dstx[i] = (float)x;
        _dsty[i] = (float)y;
    }
}
