#pragma once

namespace RVL {
// Standard RVL compression (~23% ratio, slower)
int CompressRVL_Standard(short *input, char *output, int numPixels);
void DecompressRVL_Standard(char *input, short *output, int numPixels);

// Fast RVL compression (~35% ratio, ~2x faster)
// Format: [zero_count:16][nonzero_count:16][delta0:16][delta1:16]...
int CompressRVL_Fast(short *input, char *output, int numPixels);
void DecompressRVL_Fast(char *input, short *output, int numPixels);

// Default API - selects based on RVL_FAST_MODE compile flag
#if defined(RVL_FAST_MODE) && RVL_FAST_MODE
inline int CompressRVL(short *input, char *output, int numPixels) {
    return CompressRVL_Fast(input, output, numPixels);
}
inline void DecompressRVL(char *input, short *output, int numPixels) {
    DecompressRVL_Fast(input, output, numPixels);
}
#else
inline int CompressRVL(short *input, char *output, int numPixels) {
    return CompressRVL_Standard(input, output, numPixels);
}
inline void DecompressRVL(char *input, short *output, int numPixels) {
    DecompressRVL_Standard(input, output, numPixels);
}
#endif
} // namespace RVL