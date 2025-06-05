#ifndef COLORUTILS_H
#define COLORUTILS_H


/// @file colorutils.cpp
/// Utility functions for color fill, palettes, blending, and more

#include <math.h>
#include <stdint.h>


#include "FastLED.h"
#include "crgb.h"
#include "chsv.h"

/// @brief Color correction starting points
typedef enum
{
    /// Typical values for SMD5050 LEDs
    TypicalSMD5050 = 0xFFB0F0 /* 255, 176, 240 */,
    /// @copydoc TypicalSMD5050
    TypicalLEDStrip = 0xFFB0F0 /* 255, 176, 240 */,

    /// Typical values for 8 mm "pixels on a string".
    /// Also for many through-hole 'T' package LEDs.
    Typical8mmPixel = 0xFFE08C /* 255, 224, 140 */,
    /// @copydoc Typical8mmPixel
    TypicalPixelString = 0xFFE08C /* 255, 224, 140 */,

    /// Uncorrected color (0xFFFFFF)
    UncorrectedColor = 0xFFFFFF /* 255, 255, 255 */

} LEDColorCorrection;

/// @brief Color temperature values
/// @details These color values are separated into two groups: black body radiators
/// and gaseous light sources.
///
/// Black body radiators emit a (relatively) continuous spectrum,
/// and can be described as having a Kelvin 'temperature'. This includes things
/// like candles, tungsten lightbulbs, and sunlight.
///
/// Gaseous light sources emit discrete spectral bands, and while we can
/// approximate their aggregate hue with RGB values, they don't actually
/// have a proper Kelvin temperature.
///
/// @see https://en.wikipedia.org/wiki/Color_temperature
typedef enum
{
    // Black Body Radiators
    // @{
    /// 1900 Kelvin
    Candle = 0xFF9329 /* 1900 K, 255, 147, 41 */,
    /// 2600 Kelvin
    Tungsten40W = 0xFFC58F /* 2600 K, 255, 197, 143 */,
    /// 2850 Kelvin
    Tungsten100W = 0xFFD6AA /* 2850 K, 255, 214, 170 */,
    /// 3200 Kelvin
    Halogen = 0xFFF1E0 /* 3200 K, 255, 241, 224 */,
    /// 5200 Kelvin
    CarbonArc = 0xFFFAF4 /* 5200 K, 255, 250, 244 */,
    /// 5400 Kelvin
    HighNoonSun = 0xFFFFFB /* 5400 K, 255, 255, 251 */,
    /// 6000 Kelvin
    DirectSunlight = 0xFFFFFF /* 6000 K, 255, 255, 255 */,
    /// 7000 Kelvin
    OvercastSky = 0xC9E2FF /* 7000 K, 201, 226, 255 */,
    /// 20000 Kelvin
    ClearBlueSky = 0x409CFF /* 20000 K, 64, 156, 255 */,
    // @}

    // Gaseous Light Sources
    // @{
    /// Warm (yellower) flourescent light bulbs
    WarmFluorescent = 0xFFF4E5 /* 0 K, 255, 244, 229 */,
    /// Standard flourescent light bulbs
    StandardFluorescent = 0xF4FFFA /* 0 K, 244, 255, 250 */,
    /// Cool white (bluer) flourescent light bulbs
    CoolWhiteFluorescent = 0xD4EBFF /* 0 K, 212, 235, 255 */,
    /// Full spectrum flourescent light bulbs
    FullSpectrumFluorescent = 0xFFF4F2 /* 0 K, 255, 244, 242 */,
    /// Grow light flourescent light bulbs
    GrowLightFluorescent = 0xFFEFF7 /* 0 K, 255, 239, 247 */,
    /// Black light flourescent light bulbs
    BlackLightFluorescent = 0xA700FF /* 0 K, 167, 0, 255 */,
    /// Mercury vapor light bulbs
    MercuryVapor = 0xD8F7FF /* 0 K, 216, 247, 255 */,
    /// Sodium vapor light bulbs
    SodiumVapor = 0xFFD1B2 /* 0 K, 255, 209, 178 */,
    /// Metal-halide light bulbs
    MetalHalide = 0xF2FCFF /* 0 K, 242, 252, 255 */,
    /// High-pressure sodium light bulbs
    HighPressureSodium = 0xFFB74C /* 0 K, 255, 183, 76 */,
    // @}

    /// Uncorrected temperature (0xFFFFFF)
    UncorrectedTemperature = 0xFFFFFF /* 255, 255, 255 */
} ColorTemperature;

void fill_solid(struct CRGBFL *targetArray, int numToFill, const struct CRGBFL &color)
{
    for (int i = 0; i < numToFill; ++i)
    {
        targetArray[i] = color;
    }
}

void fill_solid(struct CHSV *targetArray, int numToFill, const struct CHSV &color)
{
    for (int i = 0; i < numToFill; ++i)
    {
        targetArray[i] = color;
    }
}

// void fill_solid( struct CRGBFL* targetArray, int numToFill,
// 				 const struct CHSV& hsvColor)
// {
// 	fill_solid<CRGBFL>( targetArray, numToFill, (CRGBFL) hsvColor);
// }

void fill_rainbow(struct CRGBFL *targetArray,
                  int numToFill,
                  uint8_t initialhue,
                  uint8_t deltahue)
{
    CHSV hsv;
    hsv.hue = initialhue;
    hsv.val = 255;
    hsv.sat = 240;
    for (int i = 0; i < numToFill; ++i)
    {
        targetArray[i] = hsv;
        hsv.hue += deltahue;
    }
}

void fill_rainbow(struct CHSV *targetArray,
                  int numToFill,
                  uint8_t initialhue,
                  uint8_t deltahue)
{
    CHSV hsv;
    hsv.hue = initialhue;
    hsv.val = 255;
    hsv.sat = 240;
    for (int i = 0; i < numToFill; ++i)
    {
        targetArray[i] = hsv;
        hsv.hue += deltahue;
    }
}

void fill_rainbow_circular(struct CRGBFL *targetArray,
                           int numToFill,
                           uint8_t initialhue,
                           bool reversed)
{
    if (numToFill == 0)
        return; // avoiding div/0

    CHSV hsv;
    hsv.hue = initialhue;
    hsv.val = 255;
    hsv.sat = 240;

    const uint16_t hueChange =
        65535
        / (uint16_t)
            numToFill; // hue change for each LED, * 256 for precision (256 * 256 - 1)
    uint16_t hueOffset = 0; // offset for hue value, with precision (*256)

    for (int i = 0; i < numToFill; ++i)
    {
        targetArray[i] = hsv;
        if (reversed)
            hueOffset -= hueChange;
        else
            hueOffset += hueChange;
        hsv.hue =
            initialhue
            + (uint8_t)(hueOffset >> 8); // assign new hue with precise offset (as 8-bit)
    }
}

void fill_rainbow_circular(struct CHSV *targetArray,
                           int numToFill,
                           uint8_t initialhue,
                           bool reversed)
{
    if (numToFill == 0)
        return; // avoiding div/0

    CHSV hsv;
    hsv.hue = initialhue;
    hsv.val = 255;
    hsv.sat = 240;

    const uint16_t hueChange =
        65535
        / (uint16_t)
            numToFill; // hue change for each LED, * 256 for precision (256 * 256 - 1)
    uint16_t hueOffset = 0; // offset for hue value, with precision (*256)

    for (int i = 0; i < numToFill; ++i)
    {
        targetArray[i] = hsv;
        if (reversed)
            hueOffset -= hueChange;
        else
            hueOffset += hueChange;
        hsv.hue =
            initialhue
            + (uint8_t)(hueOffset >> 8); // assign new hue with precise offset (as 8-bit)
    }
}

void nblend(CRGBFL *existing, const CRGBFL *overlay, uint16_t count, fract8 amountOfOverlay)
{
    for (uint16_t i = count; i; --i)
    {
        //nblend(*existing, *overlay, amountOfOverlay);
        ++existing;
        ++overlay;
    }
}

CRGBFL blend(const CRGBFL &p1, const CRGBFL &p2, fract8 amountOfP2)
{
    CRGBFL nu(p1);
    //nblend(nu, p2, amountOfP2);
    return nu;
}

CRGBFL *blend(
    const CRGBFL *src1, const CRGBFL *src2, CRGBFL *dest, uint16_t count, fract8 amountOfsrc2)
{
    for (uint16_t i = 0; i < count; ++i)
    {
        dest[i] = blend(src1[i], src2[i], amountOfsrc2);
    }
    return dest;
}


#endif