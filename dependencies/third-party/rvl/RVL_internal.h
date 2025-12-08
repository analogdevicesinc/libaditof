// Shared internal helpers for RVL implementations
#pragma once

#include <cstdint>

namespace RVL {

#if defined(RVL_USE_NEON) && RVL_USE_NEON

//--------------------------------------------------------------------------
// ARM64 NEON-optimized helpers
//--------------------------------------------------------------------------
// These use:
//  - Fast-path for small values (0-7 fit in 1 nibble, 8-63 in 2 nibbles)
//  - Compiler hints (__builtin_expect) for branch prediction
//  - always_inline to ensure inlining on ARM
//--------------------------------------------------------------------------

__attribute__((always_inline)) inline void
EncodeVLE(int value, int *&pBuffer, int &word, int &nibblesWritten) {
    // Fast path: value fits in a single nibble (0-7)
    if (__builtin_expect(value < 8, 1)) {
        word = (word << 4) | value;
        if (__builtin_expect(++nibblesWritten == 8, 0)) {
            *pBuffer++ = word;
            nibblesWritten = 0;
            word = 0;
        }
        return;
    }

    // Fast path: value fits in two nibbles (8-63)
    if (__builtin_expect(value < 64, 1)) {
        int nibble1 = (value & 0x7) | 0x8; // lower 3 bits + continuation
        int nibble2 = (value >> 3);        // upper bits (no continuation)

        word = (word << 4) | nibble1;
        if (__builtin_expect(++nibblesWritten == 8, 0)) {
            *pBuffer++ = word;
            nibblesWritten = 0;
            word = 0;
        }

        word = (word << 4) | nibble2;
        if (__builtin_expect(++nibblesWritten == 8, 0)) {
            *pBuffer++ = word;
            nibblesWritten = 0;
            word = 0;
        }
        return;
    }

    // General path for larger values
    do {
        int nibble = value & 0x7;
        if (__builtin_expect((value >>= 3) != 0, 1))
            nibble |= 0x8;
        word = (word << 4) | nibble;
        if (__builtin_expect(++nibblesWritten == 8, 0)) {
            *pBuffer++ = word;
            nibblesWritten = 0;
            word = 0;
        }
    } while (value);
}

__attribute__((always_inline)) inline int DecodeVLE(int *&pBuffer, int &word,
                                                    int &nibblesWritten) {
// Prefetch next cache line for large sequential decodes
#if defined(__aarch64__)
    __builtin_prefetch(pBuffer + 8, 0, 1);
#endif

    unsigned int nibble;
    int value = 0, bits = 29;

    // First nibble: check if we need to load a new word
    if (__builtin_expect(!nibblesWritten, 0)) {
        word = *pBuffer++;
        nibblesWritten = 8;
    }
    nibble = word & 0xf0000000;
    value |= (nibble << 1) >> bits;
    word <<= 4;
    nibblesWritten--;
    bits -= 3;

    // Fast path: single nibble value (no continuation bit)
    if (__builtin_expect(!(nibble & 0x80000000), 1)) {
        return value;
    }

    // Continue decoding
    do {
        if (__builtin_expect(!nibblesWritten, 0)) {
            word = *pBuffer++;
            nibblesWritten = 8;
        }
        nibble = word & 0xf0000000;
        value |= (nibble << 1) >> bits;
        word <<= 4;
        nibblesWritten--;
        bits -= 3;
    } while (nibble & 0x80000000);

    return value;
}

#elif defined(RVL_USE_SSE) && RVL_USE_SSE

//--------------------------------------------------------------------------
// x86-64 SSE-optimized helpers
//--------------------------------------------------------------------------
// These use:
//  - Fast-path for small values (0-7 fit in 1 nibble, 8-63 in 2 nibbles)
//  - MSVC and GCC/Clang compatible intrinsics
//--------------------------------------------------------------------------

#if defined(_MSC_VER)
#define RVL_LIKELY(x) (x)
#define RVL_UNLIKELY(x) (x)
#define RVL_FORCEINLINE __forceinline
#else
#define RVL_LIKELY(x) __builtin_expect(!!(x), 1)
#define RVL_UNLIKELY(x) __builtin_expect(!!(x), 0)
#define RVL_FORCEINLINE __attribute__((always_inline)) inline
#endif

RVL_FORCEINLINE void EncodeVLE(int value, int *&pBuffer, int &word,
                               int &nibblesWritten) {
    // Fast path: value fits in a single nibble (0-7)
    if (RVL_LIKELY(value < 8)) {
        word = (word << 4) | value;
        if (RVL_UNLIKELY(++nibblesWritten == 8)) {
            *pBuffer++ = word;
            nibblesWritten = 0;
            word = 0;
        }
        return;
    }

    // Fast path: value fits in two nibbles (8-63)
    if (RVL_LIKELY(value < 64)) {
        int nibble1 = (value & 0x7) | 0x8; // lower 3 bits + continuation
        int nibble2 = (value >> 3);        // upper bits (no continuation)

        word = (word << 4) | nibble1;
        if (RVL_UNLIKELY(++nibblesWritten == 8)) {
            *pBuffer++ = word;
            nibblesWritten = 0;
            word = 0;
        }

        word = (word << 4) | nibble2;
        if (RVL_UNLIKELY(++nibblesWritten == 8)) {
            *pBuffer++ = word;
            nibblesWritten = 0;
            word = 0;
        }
        return;
    }

    // General path for larger values
    do {
        int nibble = value & 0x7;
        if (RVL_LIKELY((value >>= 3) != 0))
            nibble |= 0x8;
        word = (word << 4) | nibble;
        if (RVL_UNLIKELY(++nibblesWritten == 8)) {
            *pBuffer++ = word;
            nibblesWritten = 0;
            word = 0;
        }
    } while (value);
}

RVL_FORCEINLINE int DecodeVLE(int *&pBuffer, int &word, int &nibblesWritten) {
    unsigned int nibble;
    int value = 0, bits = 29;

    // First nibble: check if we need to load a new word
    if (RVL_UNLIKELY(!nibblesWritten)) {
        word = *pBuffer++;
        nibblesWritten = 8;
    }
    nibble = word & 0xf0000000;
    value |= (nibble << 1) >> bits;
    word <<= 4;
    nibblesWritten--;
    bits -= 3;

    // Fast path: single nibble value (no continuation bit)
    if (RVL_LIKELY(!(nibble & 0x80000000))) {
        return value;
    }

    // Continue decoding
    do {
        if (RVL_UNLIKELY(!nibblesWritten)) {
            word = *pBuffer++;
            nibblesWritten = 8;
        }
        nibble = word & 0xf0000000;
        value |= (nibble << 1) >> bits;
        word <<= 4;
        nibblesWritten--;
        bits -= 3;
    } while (nibble & 0x80000000);

    return value;
}

#undef RVL_LIKELY
#undef RVL_UNLIKELY
#undef RVL_FORCEINLINE

#else

//--------------------------------------------------------------------------
// Scalar (portable) helpers
//--------------------------------------------------------------------------

inline void EncodeVLE(int value, int *&pBuffer, int &word,
                      int &nibblesWritten) {
    do {
        int nibble = value & 0x7; // lower 3 bits
        if ((value >>= 3) != 0)
            nibble |= 0x8; // more to come
        word <<= 4;
        word |= nibble;
        if (++nibblesWritten == 8) // output word
        {
            *pBuffer++ = word;
            nibblesWritten = 0;
            word = 0;
        }
    } while (value);
}

inline int DecodeVLE(int *&pBuffer, int &word, int &nibblesWritten) {
    unsigned int nibble;
    int value = 0, bits = 29;
    do {
        if (!nibblesWritten) {
            word = *pBuffer++; // load word
            nibblesWritten = 8;
        }
        nibble = word & 0xf0000000;
        value |= (nibble << 1) >> bits;
        word <<= 4;
        nibblesWritten--;
        bits -= 3;
    } while (nibble & 0x80000000);
    return value;
}

#endif // RVL_USE_NEON / RVL_USE_SSE

} // namespace RVL
