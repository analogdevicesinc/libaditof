// Scalar (original) implementation of RVL
#include "RVL_internal.h"
#include <cstring>

namespace RVL
{
    int CompressRVL(short* input, char* output, int numPixels)
    {
        int *buffer, *pBuffer;
        buffer = pBuffer = (int*)output;

        int word = 0;
        int nibblesWritten = 0;
        short previous = 0;

        short* end = input + numPixels;
        while (input != end)
        {
            int zeros = 0, nonzeros = 0;
            for (; (input != end) && !*input; input++, zeros++);
            EncodeVLE(zeros, pBuffer, word, nibblesWritten); // number of zeros
            for (short* p = input; (p != end) && *p++; nonzeros++);
            EncodeVLE(nonzeros, pBuffer, word, nibblesWritten); // number of nonzeros
            for (int i = 0; i < nonzeros; i++)
            {
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

        return int((char*)pBuffer - (char*)buffer); // num bytes
    }

    void DecompressRVL(char* input, short* output, int numPixels)
    {
        int *buffer, *pBuffer;
        buffer = pBuffer = (int*)input;

        int word = 0;
        int nibblesWritten = 0;
        short current, previous = 0;

        int numPixelsToDecode = numPixels;
        while (numPixelsToDecode)
        {
            int zeros = DecodeVLE(pBuffer, word, nibblesWritten); // number of zeros
            numPixelsToDecode -= zeros;
            for (; zeros; zeros--)
            {
                *output++ = 0;
            }
            int nonzeros = DecodeVLE(pBuffer, word, nibblesWritten); // number of nonzeros
            numPixelsToDecode -= nonzeros;
            for (; nonzeros; nonzeros--)
            {
                int positive = DecodeVLE(pBuffer, word, nibblesWritten); // nonzero value
                int delta = (positive >> 1) ^ -(positive & 1);
                current = previous + delta;
                *output++ = current;
                previous = current;
            }
        }
    }
}
