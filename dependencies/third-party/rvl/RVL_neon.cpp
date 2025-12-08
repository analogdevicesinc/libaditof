// ARM64-optimized implementation of RVL
// Optimizations: branch hints, forced inlining, prefetching, streamlined loops
#include "RVL_internal.h"

#if defined(__aarch64__)
#include <arm_neon.h>
#define RVL_PREFETCH(addr) __builtin_prefetch(addr, 0, 3)
#define RVL_LIKELY(x) __builtin_expect(!!(x), 1)
#define RVL_UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
#define RVL_PREFETCH(addr) ((void)0)
#define RVL_LIKELY(x) (x)
#define RVL_UNLIKELY(x) (x)
#endif

namespace RVL {
//--------------------------------------------------------------------------
// Standard RVL (ARM64 optimized)
//--------------------------------------------------------------------------
__attribute__((hot, optimize("O3"))) int
CompressRVL_Standard(short *input, char *output, int numPixels) {
    int *pBuffer = (int *)output;
    int *const bufferStart = pBuffer;

    int word = 0;
    int nibblesWritten = 0;
    int previous = 0;

    const short *const end = input + numPixels;

    while (RVL_LIKELY(input != end)) {
        // Count zeros - use pointer arithmetic
        const short *zeroStart = input;
        while (input != end && *input == 0)
            ++input;
        EncodeVLE((int)(input - zeroStart), pBuffer, word, nibblesWritten);

        // Count nonzeros
        const short *nonzeroStart = input;
        while (input != end && *input != 0)
            ++input;
        int nonzeros = (int)(input - nonzeroStart);
        EncodeVLE(nonzeros, pBuffer, word, nibblesWritten);

        // Encode nonzero values - reset input to start of nonzero run
        const short *p = nonzeroStart;

        // Prefetch ahead for large runs
        if (RVL_UNLIKELY(nonzeros > 32)) {
            RVL_PREFETCH(p + 64);
        }

        // Process 4 at a time (unrolled, no fake SIMD)
        int count = nonzeros;
        while (count >= 4) {
            int c0 = p[0], c1 = p[1], c2 = p[2], c3 = p[3];

            int d0 = c0 - previous;
            int d1 = c1 - c0;
            int d2 = c2 - c1;
            int d3 = c3 - c2;

            EncodeVLE((d0 << 1) ^ (d0 >> 31), pBuffer, word, nibblesWritten);
            EncodeVLE((d1 << 1) ^ (d1 >> 31), pBuffer, word, nibblesWritten);
            EncodeVLE((d2 << 1) ^ (d2 >> 31), pBuffer, word, nibblesWritten);
            EncodeVLE((d3 << 1) ^ (d3 >> 31), pBuffer, word, nibblesWritten);

            previous = c3;
            p += 4;
            count -= 4;
        }

        // Handle remainder
        while (count-- > 0) {
            int current = *p++;
            int delta = current - previous;
            EncodeVLE((delta << 1) ^ (delta >> 31), pBuffer, word,
                      nibblesWritten);
            previous = current;
        }
    }

    // Flush remaining nibbles
    if (nibblesWritten) {
        *pBuffer++ = word << (4 * (8 - nibblesWritten));
    }

    return (int)((char *)pBuffer - (char *)bufferStart);
}

__attribute__((hot, optimize("O3"))) void
DecompressRVL_Standard(char *input, short *output, int numPixels) {
    int *pBuffer = (int *)input;

    int word = 0;
    int nibblesWritten = 0;
    int previous = 0;

    int remaining = numPixels;

    while (RVL_LIKELY(remaining > 0)) {
        // Decode zeros count
        int zeros = DecodeVLE(pBuffer, word, nibblesWritten);
        remaining -= zeros;

        // Write zeros - use NEON for large runs
#if defined(__aarch64__)
        if (zeros >= 8) {
            int16x8_t vzero = vdupq_n_s16(0);
            while (zeros >= 8) {
                vst1q_s16(output, vzero);
                output += 8;
                zeros -= 8;
            }
        }
#endif
        while (zeros-- > 0)
            *output++ = 0;

        // Decode nonzeros count
        int nonzeros = DecodeVLE(pBuffer, word, nibblesWritten);
        remaining -= nonzeros;

        // Prefetch for large runs
        if (RVL_UNLIKELY(nonzeros > 32)) {
            RVL_PREFETCH(pBuffer + 16);
        }

        // Process 4 at a time (unrolled)
        while (nonzeros >= 4) {
            int p0 = DecodeVLE(pBuffer, word, nibblesWritten);
            int p1 = DecodeVLE(pBuffer, word, nibblesWritten);
            int p2 = DecodeVLE(pBuffer, word, nibblesWritten);
            int p3 = DecodeVLE(pBuffer, word, nibblesWritten);

            int d0 = (p0 >> 1) ^ -(p0 & 1);
            int d1 = (p1 >> 1) ^ -(p1 & 1);
            int d2 = (p2 >> 1) ^ -(p2 & 1);
            int d3 = (p3 >> 1) ^ -(p3 & 1);

            int v0 = previous + d0;
            int v1 = v0 + d1;
            int v2 = v1 + d2;
            int v3 = v2 + d3;

            output[0] = (short)v0;
            output[1] = (short)v1;
            output[2] = (short)v2;
            output[3] = (short)v3;

            previous = v3;
            output += 4;
            nonzeros -= 4;
        }

        // Handle remainder
        while (nonzeros-- > 0) {
            int positive = DecodeVLE(pBuffer, word, nibblesWritten);
            int delta = (positive >> 1) ^ -(positive & 1);
            int current = previous + delta;
            *output++ = (short)current;
            previous = current;
        }
    }
}

//--------------------------------------------------------------------------
// Fast RVL - trades compression ratio for speed (ARM64 optimized)
// Format: [zeros:16][nonzeros:16][zigzag deltas:16 each]...
//--------------------------------------------------------------------------
__attribute__((hot, optimize("O3"))) int
CompressRVL_Fast(short *input, char *output, int numPixels) {
    unsigned short *out = (unsigned short *)output;
    unsigned short *const outStart = out;

    const short *const end = input + numPixels;
    int previous = 0;

    while (RVL_LIKELY(input != end)) {
        // Count zeros
        const short *zeroStart = input;
        while (input != end && *input == 0)
            ++input;
        unsigned short zeros = (unsigned short)(input - zeroStart);

        // Count nonzeros
        const short *nonzeroStart = input;
        while (input != end && *input != 0)
            ++input;
        unsigned short nonzeros = (unsigned short)(input - nonzeroStart);

        // Write counts
        *out++ = zeros;
        *out++ = nonzeros;

        // Write zigzag-encoded deltas as 16-bit values
        const short *p = nonzeroStart;
        int count = nonzeros;

        // Process 4 at a time
        while (count >= 4) {
            int c0 = p[0], c1 = p[1], c2 = p[2], c3 = p[3];

            int d0 = c0 - previous;
            int d1 = c1 - c0;
            int d2 = c2 - c1;
            int d3 = c3 - c2;

            out[0] = (unsigned short)((d0 << 1) ^ (d0 >> 31));
            out[1] = (unsigned short)((d1 << 1) ^ (d1 >> 31));
            out[2] = (unsigned short)((d2 << 1) ^ (d2 >> 31));
            out[3] = (unsigned short)((d3 << 1) ^ (d3 >> 31));

            previous = c3;
            p += 4;
            out += 4;
            count -= 4;
        }

        // Handle remainder
        while (count-- > 0) {
            int current = *p++;
            int delta = current - previous;
            *out++ = (unsigned short)((delta << 1) ^ (delta >> 31));
            previous = current;
        }
    }

    return (int)((char *)out - (char *)outStart);
}

__attribute__((hot, optimize("O3"))) void
DecompressRVL_Fast(char *input, short *output, int numPixels) {
    const unsigned short *in = (const unsigned short *)input;
    int previous = 0;
    int remaining = numPixels;

    while (RVL_LIKELY(remaining > 0)) {
        unsigned short zeros = *in++;
        unsigned short nonzeros = *in++;

        // Write zeros - use NEON for large runs
        remaining -= zeros;
#if defined(__aarch64__)
        if (zeros >= 8) {
            int16x8_t vzero = vdupq_n_s16(0);
            while (zeros >= 8) {
                vst1q_s16(output, vzero);
                output += 8;
                zeros -= 8;
            }
        }
#endif
        while (zeros-- > 0)
            *output++ = 0;

        // Decode nonzeros
        remaining -= nonzeros;
        int count = nonzeros;

        // Process 4 at a time
        while (count >= 4) {
            unsigned short e0 = in[0], e1 = in[1], e2 = in[2], e3 = in[3];

            int d0 = (e0 >> 1) ^ -(e0 & 1);
            int d1 = (e1 >> 1) ^ -(e1 & 1);
            int d2 = (e2 >> 1) ^ -(e2 & 1);
            int d3 = (e3 >> 1) ^ -(e3 & 1);

            int v0 = previous + d0;
            int v1 = v0 + d1;
            int v2 = v1 + d2;
            int v3 = v2 + d3;

            output[0] = (short)v0;
            output[1] = (short)v1;
            output[2] = (short)v2;
            output[3] = (short)v3;

            previous = v3;
            in += 4;
            output += 4;
            count -= 4;
        }

        // Handle remainder
        while (count-- > 0) {
            unsigned short encoded = *in++;
            int delta = (encoded >> 1) ^ -(encoded & 1);
            int current = previous + delta;
            *output++ = (short)current;
            previous = current;
        }
    }
}
} // namespace RVL
