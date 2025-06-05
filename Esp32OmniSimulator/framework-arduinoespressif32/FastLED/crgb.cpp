/// @file crgb.cpp
/// Utility functions for the red, green, and blue (RGB) pixel struct

#define FASTLED_INTERNAL
#include "crgb.h"
#include "FastLED.h"


CRGBFL CRGBFL::computeAdjustment(uint8_t scale,
                             const CRGBFL &colorCorrection,
                             const CRGBFL &colorTemperature)
{
#if defined(NO_CORRECTION) && (NO_CORRECTION == 1)
    return CRGBFL(scale, scale, scale);
#else
    CRGBFL adj(0, 0, 0);
    if (scale > 0)
    {
        for (uint8_t i = 0; i < 3; ++i)
        {
            uint8_t cc = colorCorrection.raw[i];
            uint8_t ct = colorTemperature.raw[i];
            if (cc > 0 && ct > 0)
            {
                // Optimized for AVR size. This function is only called very infrequently
                // so size matters more than speed.
                uint32_t work = (((uint16_t)cc) + 1);
                work *= (((uint16_t)ct) + 1);
                work *= scale;
                work /= 0x10000L;
                adj.raw[i] = work & 0xFF;
            }
        }
    }
    return adj;
#endif
}

CRGBFL CRGBFL::blend(const CRGBFL &p1, const CRGBFL &p2, fract8 amountOfP2)
{
    return CRGBFL(0, 0, 0);
        //blend8(p1.r, p2.r, amountOfP2),
        //        blend8(p1.g, p2.g, amountOfP2),
        //        blend8(p1.b, p2.b, amountOfP2));
}

CRGBFL CRGBFL::blendAlphaMaxChannel(const CRGBFL &upper, const CRGBFL &lower)
{
    // Use luma of upper pixel as alpha (0..255)
    uint8_t max_component = 0;
    for (int i = 0; i < 3; ++i)
    {
        if (upper.raw[i] > max_component)
        {
            max_component = upper.raw[i];
        }
    }
    // uint8_t alpha = upper.getLuma();
    // blend(lower, upper, alpha) → (lower * (255−alpha) + upper * alpha) / 256
    uint8_t amountOf2 = 255 - max_component;
    return CRGBFL::blend(upper, lower, amountOf2);
}

CRGBFL &CRGBFL::nscale8(uint8_t scaledown)
{
    //nscale8x3(r, g, b, scaledown);
    return *this;
}
