// Scalar (portable) implementation of RVL
#include "RVL_internal.h"
#include <cstring>

namespace RVL {
//--------------------------------------------------------------------------
// Standard RVL (original algorithm)
//--------------------------------------------------------------------------
int CompressRVL_Standard(short *input, char *output, int numPixels) {
    int *buffer, *pBuffer;
    buffer = pBuffer = (int *)output;

    int word = 0;
    int nibblesWritten = 0;
    short previous = 0;

    short *end = input + numPixels;
    while (input != end) {
        int zeros = 0, nonzeros = 0;
        for (; (input != end) && !*input; input++, zeros++)
            ;
        EncodeVLE(zeros, pBuffer, word, nibblesWritten); // number of zeros
        for (short *p = input; (p != end) && *p++; nonzeros++)
            ;
        EncodeVLE(nonzeros, pBuffer, word,
                  nibblesWritten); // number of nonzeros
        for (int i = 0; i < nonzeros; i++) {
            short current = *input++;
            int delta = current - previous;
            int positive = (delta << 1) ^ (delta >> 31);
            EncodeVLE(positive, pBuffer, word, nibblesWritten); // nonzero value
            previous = current;
        }
    }

    if (nibblesWritten) // last few values
    {
        *pBuffer++ = word << 4 * (8 - nibblesWritten);
    }

    return int((char *)pBuffer - (char *)buffer); // num bytes
}

void DecompressRVL_Standard(char *input, short *output, int numPixels) {
    int *buffer, *pBuffer;
    buffer = pBuffer = (int *)input;

    int word = 0;
    int nibblesWritten = 0;
    short current, previous = 0;

    int numPixelsToDecode = numPixels;
    while (numPixelsToDecode) {
        int zeros = DecodeVLE(pBuffer, word, nibblesWritten); // number of zeros
        numPixelsToDecode -= zeros;
        for (; zeros; zeros--) {
            *output++ = 0;
        }
        int nonzeros =
            DecodeVLE(pBuffer, word, nibblesWritten); // number of nonzeros
        numPixelsToDecode -= nonzeros;
        for (; nonzeros; nonzeros--) {
            int positive =
                DecodeVLE(pBuffer, word, nibblesWritten); // nonzero value
            int delta = (positive >> 1) ^ -(positive & 1);
            current = previous + delta;
            *output++ = current;
            previous = current;
        }
    }
}

//--------------------------------------------------------------------------
// Fast RVL - trades compression ratio for speed
// Format: [zeros:16][nonzeros:16][zigzag deltas:16 each]...
//--------------------------------------------------------------------------
int CompressRVL_Fast(short *input, char *output, int numPixels) {
    unsigned short *out = (unsigned short *)output;
    unsigned short *const outStart = out;

    const short *const end = input + numPixels;
    short previous = 0;

    while (input != end) {
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
        for (unsigned short i = 0; i < nonzeros; ++i) {
            short current = *p++;
            int delta = current - previous;
            // Zigzag encode: map signed to unsigned
            unsigned short encoded =
                (unsigned short)((delta << 1) ^ (delta >> 15));
            *out++ = encoded;
            previous = current;
        }
    }

    return (int)((char *)out - (char *)outStart);
}

void DecompressRVL_Fast(char *input, short *output, int numPixels) {
    const unsigned short *in = (const unsigned short *)input;
    short previous = 0;
    int remaining = numPixels;

    while (remaining > 0) {
        unsigned short zeros = *in++;
        unsigned short nonzeros = *in++;

        // Write zeros
        remaining -= zeros;
        while (zeros-- > 0)
            *output++ = 0;

        // Decode nonzeros
        remaining -= nonzeros;
        for (unsigned short i = 0; i < nonzeros; ++i) {
            unsigned short encoded = *in++;
            // Zigzag decode
            int delta = (encoded >> 1) ^ -(encoded & 1);
            short current = previous + delta;
            *output++ = current;
            previous = current;
        }
    }
}
} // namespace RVL
