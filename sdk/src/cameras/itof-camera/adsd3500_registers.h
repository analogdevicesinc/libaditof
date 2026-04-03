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

#ifndef ADSD3500_REGISTERS_H
#define ADSD3500_REGISTERS_H

/**
 * @file adsd3500_registers.h
 * @brief ADSD3500 Depth ISP Register Address Definitions
 *
 * This header defines all register addresses for the ADSD3500 dual depth processor.
 * Registers are organized by functional category for clarity.
 */

// ============================================================================
// Status and Identification Registers
// ============================================================================

/** Chip status register - bit 0 indicates CCBM support */
#define ADSD3500_REG_CHIP_STATUS 0x0020

/** Chip ID register (standard mode) */
#define ADSD3500_REG_CHIP_ID 0x0112

/** Extended chip ID register (requires 110ms delay) */
#define ADSD3500_REG_CHIP_ID_EXT 0x0116

/** Imager status/error code register */
#define ADSD3500_REG_IMAGER_STATUS 0x0038

// ============================================================================
// Mode and Synchronization Control
// ============================================================================

/** Mode selection register */
#define ADSD3500_REG_MODE_SELECT 0x0040

/** FSYNC toggle mode: 0=manual, 1=auto at fps, 2=slave input */
#define ADSD3500_REG_FSYNC_TOGGLE_MODE 0x0025

/** Manual FSYNC toggle trigger */
#define ADSD3500_REG_FSYNC_TOGGLE 0x0026

/** Burst mode command (switch to burst mode) */
#define ADSD3500_REG_BURST_MODE_CMD 0x0019

// ============================================================================
// Threshold Configuration (Write + Read Register Pairs)
// ============================================================================

/** AB invalidation threshold (write) */
#define ADSD3500_REG_AB_THRESHOLD_SET 0x0010
/** AB invalidation threshold (read) */
#define ADSD3500_REG_AB_THRESHOLD_GET 0x0015

/** Confidence threshold (write) */
#define ADSD3500_REG_CONFIDENCE_THRESHOLD_SET 0x0011
/** Confidence threshold (read) */
#define ADSD3500_REG_CONFIDENCE_THRESHOLD_GET 0x0016

/** Radial threshold minimum (write) */
#define ADSD3500_REG_RADIAL_THRESHOLD_MIN_SET 0x0027
/** Radial threshold minimum (read) */
#define ADSD3500_REG_RADIAL_THRESHOLD_MIN_GET 0x0028

/** Radial threshold maximum (write) */
#define ADSD3500_REG_RADIAL_THRESHOLD_MAX_SET 0x0029
/** Radial threshold maximum (read) */
#define ADSD3500_REG_RADIAL_THRESHOLD_MAX_GET 0x0030

// ============================================================================
// Joint Bilateral Filter (JBLF) Configuration
// ============================================================================

/** JBLF filter enable (write) - 1=enabled, 0=disabled */
#define ADSD3500_REG_JBLF_ENABLE_SET 0x0013
/** JBLF filter enable (read) */
#define ADSD3500_REG_JBLF_ENABLE_GET 0x0017

/** JBLF filter size (write) */
#define ADSD3500_REG_JBLF_SIZE_SET 0x0014
/** JBLF filter size (read) */
#define ADSD3500_REG_JBLF_SIZE_GET 0x0018

/** JBLF maximum edge threshold */
#define ADSD3500_REG_JBLF_MAX_EDGE_THRESHOLD 0x0074

/** JBLF AB threshold */
#define ADSD3500_REG_JBLF_AB_THRESHOLD 0x0075

/** JBLF Gaussian sigma (write) */
#define ADSD3500_REG_JBLF_GAUSSIAN_SIGMA_SET 0x006B
/** JBLF Gaussian sigma (read) */
#define ADSD3500_REG_JBLF_GAUSSIAN_SIGMA_GET 0x0069

/** JBLF exponential term (write) */
#define ADSD3500_REG_JBLF_EXPONENTIAL_TERM_SET 0x006C
/** JBLF exponential term (read) */
#define ADSD3500_REG_JBLF_EXPONENTIAL_TERM_GET 0x006A

// ============================================================================
// MIPI and Communication Settings
// ============================================================================

/** MIPI output speed configuration (write) */
#define ADSD3500_REG_MIPI_OUTPUT_SPEED_SET 0x0031
/** MIPI output speed configuration (read) */
#define ADSD3500_REG_MIPI_OUTPUT_SPEED_GET 0x0034

/** Raw bypass mode - bypasses depth computation when enabled */
#define ADSD3500_REG_RAW_BYPASS_MODE 0x00AC

/** Enable deskew at stream on */
#define ADSD3500_REG_ENABLE_DESKEW 0x00AB

// ============================================================================
// VCSEL and Timing Control
// ============================================================================

/** VCSEL delay (write) */
#define ADSD3500_REG_VCSEL_DELAY_SET 0x0066
/** VCSEL delay (read) */
#define ADSD3500_REG_VCSEL_DELAY_GET 0x0068

/** Frame rate (read) - actual frame rate in fps */
#define ADSD3500_REG_FRAME_RATE_GET 0x0023

// ============================================================================
// Advanced Processing Features
// ============================================================================

/** Enable edge confidence */
#define ADSD3500_REG_ENABLE_EDGE_CONFIDENCE 0x0062

/** Enable phase invalidation */
#define ADSD3500_REG_ENABLE_PHASE_INVALIDATION 0x0072

/** Enable temperature compensation (write) */
#define ADSD3500_REG_ENABLE_TEMP_COMPENSATION 0x0021
/** Temperature compensation status (read) */
#define ADSD3500_REG_TEMP_COMPENSATION_STATUS 0x0076

/** Enable metadata in AB frame (write) */
#define ADSD3500_REG_ENABLE_METADATA_SET 0x0036
/** Enable metadata in AB frame (read) */
#define ADSD3500_REG_ENABLE_METADATA_GET 0x0037

// ============================================================================
// Temperature Sensors
// ============================================================================

/** Sensor temperature readback */
#define ADSD3500_REG_SENSOR_TEMPERATURE 0x0054

/** Laser temperature readback */
#define ADSD3500_REG_LASER_TEMPERATURE 0x0055

// ============================================================================
// Calibration and Configuration Management
// ============================================================================

/** Disable CCBM (Camera Configuration Binary Module) */
#define ADSD3500_REG_DISABLE_CCBM 0x00EF

// ============================================================================
// Dynamic Mode Switching
// ============================================================================

/** Enable dynamic mode switching - allows runtime mode changes */
#define ADSD3500_REG_ENABLE_DYNAMIC_MODE_SWITCHING 0x00C9

/** Dynamic mode switching sequence payload command */
#define ADSD3500_REG_DYNAMIC_MODE_SEQUENCE 0x00CA

/** Dynamic mode switching - sequence 0 (lower 16 bits of 32-bit sequence) */
#define ADSD3500_REG_DMS_SEQUENCE_0 0x0081

/** Dynamic mode switching - sequence 1 (upper 16 bits of 32-bit sequence) */
#define ADSD3500_REG_DMS_SEQUENCE_1 0x0082

/** Dynamic mode switching - repeat count 0 (lower 16 bits) */
#define ADSD3500_REG_DMS_REPEAT_COUNT_0 0x0083

/** Dynamic mode switching - repeat count 1 (upper 16 bits) */
#define ADSD3500_REG_DMS_REPEAT_COUNT_1 0x0084

// ============================================================================
// Special Command Codes
// ============================================================================

/**
 * Special chip command base for mode switching
 * Usage: (ADSD3500_CMD_MODE_SWITCH_BASE + mode_number)
 * Example: 0xDA00 + mode → sends mode-specific command with 0x280F payload
 */
#define ADSD3500_CMD_MODE_SWITCH_BASE 0xDA00

/** Payload value for mode switch commands */
#define ADSD3500_CMD_MODE_SWITCH_PAYLOAD 0x280F

// ============================================================================
// Bit Masks and Constants
// ============================================================================

/** CCBM support bit in chip status register */
#define ADSD3500_CHIP_STATUS_CCBM_SUPPORT_BIT 0x0001

#endif // ADSD3500_REGISTERS_H
