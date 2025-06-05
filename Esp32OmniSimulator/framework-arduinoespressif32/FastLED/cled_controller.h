#pragma once

#include <stddef.h>

/// @file cled_controller.h
/// base definitions used by led controllers for writing out led data

#include "FastLED.h"
#include "crgb.h"
#include "chsv.h"

/// RGB color channel orderings, used when instantiating controllers to determine
/// what order the controller should send data out in. The default ordering
/// is RGB.
/// Within this enum, the red channel is 0, the green channel is 1, and the
/// blue chanel is 2.
enum EOrder
{
    RGB = 0012, ///< Red,   Green, Blue  (0012)
    RBG = 0021, ///< Red,   Blue,  Green (0021)
    GRB = 0102, ///< Green, Red,   Blue  (0102)
    GBR = 0120, ///< Green, Blue,  Red   (0120)
    BRG = 0201, ///< Blue,  Red,   Green (0201)
    BGR = 0210  ///< Blue,  Green, Red   (0210)
};

// After EOrder is applied this is where W is inserted for RGBW.
enum EOrderW
{
    W3 = 0x3, ///< White is fourth
    W2 = 0x2, ///< White is third
    W1 = 0x1, ///< White is second
    W0 = 0x0, ///< White is first
    WDefault = W3
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// LED Controller interface definition
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// Base definition for an LED controller.  Pretty much the methods that every LED
/// controller object will make available. If you want to pass LED controllers around to
/// methods, make them references to this type, keeps your code saner. However, most
/// people won't be seeing/using these objects directly at all.
/// @note That the methods for eventual checking of background writing of data (I'm
/// looking at you, Teensy 3.0 DMA controller!) are not yet implemented.
class CLEDController
{
public:
    friend class CFastLED;
    CRGBFL *m_Data;            ///< pointer to the LED data used by this controller
    CLEDController *m_pNext; ///< pointer to the next LED controller in the linked list
    CRGBFL m_ColorCorrection;  ///< CRGB object representing the color correction to apply
                              ///< to
                            ///< the strip on show()  @see setCorrection
    CRGBFL m_ColorTemperature; ///< CRGB object representing the color temperature to
                               ///< apply
                              ///< to the strip on show() @see setTemperature
    //EDitherMode m_DitherMode; ///< the current dither mode of the controller
    bool m_enabled = true;
    int m_nLeds; ///< the number of LEDs in the LED data array
    static CLEDController
        *m_pHead; ///< pointer to the first LED controller in the linked list
    static CLEDController
        *m_pTail; ///< pointer to the last LED controller in the linked list

    /// Set all the LEDs to a given color.
    /// @param data the CRGBFL color to set the LEDs to
    /// @param nLeds the number of LEDs to set to this color
    /// @param scale the rgb scaling value for outputting color
    //virtual void showColor(const CRGBFL &data, int nLeds, uint8_t brightness) = 0;

    /// Write the passed in RGB data out to the LEDs managed by this controller.
    /// @param data the rgb data to write out to the strip
    /// @param nLeds the number of LEDs being written out
    /// @param scale the rgb scaling to apply to each led before writing it out
    virtual void show(const struct CRGBFL *data, int nLeds, uint8_t brightness) = 0;


    void setEnabled(bool enabled)
    {
        m_enabled = enabled;
    }

    bool getEnabled()
    {
        return m_enabled;
    }

    CLEDController();


    /// Initialize the LED controller
    virtual void init() = 0;

    /// Clear out/zero out the given number of LEDs.
    /// @param nLeds the number of LEDs to clear
    void clearLeds(int nLeds = -1)
    {
        clearLedDataInternal(nLeds);
        showLeds(0);
    }

    // Compatibility with the 3.8.x codebase.
    void showLeds(uint8_t brightness)
    {
        //void *data = beginShowLeds(m_nLeds);
        showLedsInternal(brightness);
        //endShowLeds(data);
    }

    /// @copybrief show(const struct CRGBFL*, int, CRGBFL)
    ///
    /// Will scale for color correction and temperature. Can accept LED data not attached
    /// to this controller.
    /// @param data the LED data to write to the strip
    /// @param nLeds the number of LEDs in the data array
    /// @param brightness the brightness of the LEDs
    /// @see show(const struct CRGBFL*, int, CRGBFL)
    void showInternal(const struct CRGBFL *data, int nLeds, uint8_t brightness)
    {
        if (m_enabled)
        {
            show(data, nLeds, brightness);
        }
    }

    /// @copybrief showColor(const struct CRGBFL&, int, CRGBFL)
    ///
    /// Will scale for color correction and temperature. Can accept LED data not attached
    /// to this controller.
    /// @param data the CRGBFL color to set the LEDs to
    /// @param nLeds the number of LEDs in the data array
    /// @param brightness the brightness of the LEDs
    /// @see showColor(const struct CRGBFL&, int, CRGBFL)
    void showColorInternal(const struct CRGBFL &data, int nLeds, uint8_t brightness)
    {
        if (m_enabled)
        {
            //showColor(data, nLeds, brightness);
        }
    }

    /// Write the data to the LEDs managed by this controller
    /// @param brightness the brightness of the LEDs
    /// @see show(const struct CRGBFL*, int, uint8_t)
    void showLedsInternal(uint8_t brightness)
    {
        if (m_enabled)
        {
            //show(m_Data, m_nLeds, brightness);
        }
    }

    /// @copybrief showColor(const struct CRGBFL&, int, CRGBFL)
    ///
    /// @param data the CRGBFL color to set the LEDs to
    /// @param brightness the brightness of the LEDs
    /// @see showColor(const struct CRGBFL&, int, CRGBFL)
    void showColorInternal(const struct CRGBFL &data, uint8_t brightness)
    {
        if (m_enabled)
        {
            //showColor(data, m_nLeds, brightness);
        }
    }

    /// Get the first LED controller in the linked list of controllers
    /// @returns CLEDController::m_pHead
    static CLEDController *head()
    {
        return m_pHead;
    }

    /// Get the next controller in the linked list after this one.  Will return NULL at
    /// the end of the linked list.
    /// @returns CLEDController::m_pNext
    CLEDController *next()
    {
        return m_pNext;
    }

    /// Set the default array of LEDs to be used by this controller
    /// @param data pointer to the LED data
    /// @param nLeds the number of LEDs in the LED data
    CLEDController &setLeds(CRGBFL *data, int nLeds)
    {
        //m_Data = data;
        //m_nLeds = nLeds;
        return *this;
    }

    /// Zero out the LED data managed by this controller
    void clearLedDataInternal(int nLeds = -1);

    /// How many LEDs does this controller manage?
    /// @returns CLEDController::m_nLeds
    virtual int size()
    {
        return m_nLeds;
    }

    /// How many Lanes does this controller manage?
    /// @returns 1 for a non-Parallel controller
    virtual int lanes()
    {
        return 1;
    }

    /// Pointer to the CRGBFL array for this controller
    /// @returns CLEDController::m_Data
    CRGBFL *leds()
    {
        return NULL;
    }

    /// Reference to the n'th LED managed by the controller
    /// @param x the LED number to retrieve
    /// @returns reference to CLEDController::m_Data[x]
    //CRGBFL &operator[](int x)
    //{
    //    return nullptr_t;
    //}


};

