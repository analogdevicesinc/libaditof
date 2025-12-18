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
#ifndef FLOAT_TO_LIN_H
#define FLOAT_TO_LIN_H

static const uint16_t CNT = 2048;

static int16_t lookup[CNT] = {0};

static int16_t getRange(uint16_t idx) {

    bool neg = false;
    if (idx > (CNT / 2)) {
        idx = idx - (CNT / 2);
        neg = true;
    }

    int16_t value = 0;
    if (idx < 256)
        value = idx;
    else if (idx < 384)
        value = ((idx - 256) << 1) + 256;
    else if (idx < 512)
        value = ((idx - 384) << 2) + 512;
    else if (idx < 640)
        value = ((idx - 512) << 3) + 1024;
    else if (idx < 768)
        value = ((idx - 640) << 4) + 2048;
    else if (idx < 896)
        value = ((idx - 768) << 5) + 4096;
    else if (idx < 1024)
        value = ((idx - 896) << 6) + 8192;
    else if (idx == 1024)
        value = 32767;

    if (neg == true)
        value = -value;

    return value;
}

void FloatToLinGenerateTable() {
    for (uint16_t idx = 0; idx < CNT; idx++) {
        lookup[idx] = getRange(idx);
    }
}

int16_t Convert11bitFloat2LinearVal(uint16_t input) {
    return ((input < 2048) ? lookup[input] : 0);
}

#endif //FLOAT_TO_LIN_H