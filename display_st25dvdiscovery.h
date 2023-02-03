/***************************************************************************
 *   Copyright (C) 2014 by Terraneo Federico                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/

#ifndef MXGUI_LIBRARY
#error "This is header is private, it can be used only within mxgui."
#error "If your code depends on a private header, it IS broken."
#endif //MXGUI_LIBRARY

#ifndef DISPLAY_ST25DVDISCOVERY_H
#define	DISPLAY_ST25DVDISCOVERY_H

#ifdef _BOARD_ST25DVDISCOVERY

#include <config/mxgui_settings.h>
#include "display.h"
#include "point.h"
#include "color.h"
#include "font.h"
#include "image.h"
#include "iterator_direction.h"
#include "misc_inst.h"
#include "line.h"
#include <cstdio>
#include <cstring>
#include <algorithm>

namespace mxgui {

//This display is 16 bit per pixel, check that the color depth is properly
//configured
#ifndef MXGUI_COLOR_DEPTH_16_BIT
#error The ILI9341 driver requires a color depth of 16bit per pixel
#endif

/** 
  * @brief  ILI9341 Registers  
  */

/* Level 1 Commands */
#define LCD_SWRESET                 0x01   /* Software Reset */
#define LCD_READ_DISPLAY_ID         0x04   /* Read display identification information */
#define LCD_RDDST                   0x09   /* Read Display Status */
#define LCD_RDDPM                   0x0A   /* Read Display Power Mode */
#define LCD_RDDMADCTL               0x0B   /* Read Display MADCTL */
#define LCD_RDDCOLMOD               0x0C   /* Read Display Pixel Format */
#define LCD_RDDIM                   0x0D   /* Read Display Image Format */
#define LCD_RDDSM                   0x0E   /* Read Display Signal Mode */
#define LCD_RDDSDR                  0x0F   /* Read Display Self-Diagnostic Result */
#define LCD_SPLIN                   0x10   /* Enter Sleep Mode */
#define LCD_SLEEP_OUT               0x11   /* Sleep out register */
#define LCD_PTLON                   0x12   /* Partial Mode ON */
#define LCD_NORMAL_MODE_ON          0x13   /* Normal Display Mode ON */
#define LCD_DINVOFF                 0x20   /* Display Inversion OFF */
#define LCD_DINVON                  0x21   /* Display Inversion ON */
#define LCD_GAMMA                   0x26   /* Gamma register */
#define LCD_DISPLAY_OFF             0x28   /* Display off register */
#define LCD_DISPLAY_ON              0x29   /* Display on register */
#define LCD_COLUMN_ADDR             0x2A   /* Colomn address register */
#define LCD_PAGE_ADDR               0x2B   /* Page address register */
#define LCD_GRAM                    0x2C   /* GRAM register */
#define LCD_RGBSET                  0x2D   /* Color SET */
#define LCD_RAMRD                   0x2E   /* Memory Read */
#define LCD_PLTAR                   0x30   /* Partial Area */
#define LCD_VSCRDEF                 0x33   /* Vertical Scrolling Definition */
#define LCD_TEOFF                   0x34   /* Tearing Effect Line OFF */
#define LCD_TEON                    0x35   /* Tearing Effect Line ON */
#define LCD_MAC                     0x36   /* Memory Access Control register*/
#define LCD_VSCRSADD                0x37   /* Vertical Scrolling Start Address */
#define LCD_IDMOFF                  0x38   /* Idle Mode OFF */
#define LCD_IDMON                   0x39   /* Idle Mode ON */
#define LCD_PIXEL_FORMAT            0x3A   /* Pixel Format register */
#define LCD_WRITE_MEM_CONTINUE      0x3C   /* Write Memory Continue */
#define LCD_READ_MEM_CONTINUE       0x3E   /* Read Memory Continue */
#define LCD_SET_TEAR_SCANLINE       0x44   /* Set Tear Scanline */
#define LCD_GET_SCANLINE            0x45   /* Get Scanline */
#define LCD_WDB                     0x51   /* Write Brightness Display register */
#define LCD_RDDISBV                 0x52   /* Read Display Brightness */
#define LCD_WCD                     0x53   /* Write Control Display register*/
#define LCD_RDCTRLD                 0x54   /* Read CTRL Display */
#define LCD_WRCABC                  0x55   /* Write Content Adaptive Brightness Control */
#define LCD_RDCABC                  0x56   /* Read Content Adaptive Brightness Control */
#define LCD_WRITE_CABC              0x5E   /* Write CABC Minimum Brightness */
#define LCD_READ_CABC               0x5F   /* Read CABC Minimum Brightness */
#define LCD_READ_ID1                0xDA   /* Read ID1 */
#define LCD_READ_ID2                0xDB   /* Read ID2 */
#define LCD_READ_ID3                0xDC   /* Read ID3 */

/* Level 2 Commands */
#define LCD_RGB_INTERFACE           0xB0   /* RGB Interface Signal Control */
#define LCD_FRMCTR1                 0xB1   /* Frame Rate Control (In Normal Mode) */
#define LCD_FRMCTR2                 0xB2   /* Frame Rate Control (In Idle Mode) */
#define LCD_FRMCTR3                 0xB3   /* Frame Rate Control (In Partial Mode) */
#define LCD_INVTR                   0xB4   /* Display Inversion Control */
#define LCD_BPC                     0xB5   /* Blanking Porch Control register */
#define LCD_DFC                     0xB6   /* Display Function Control register */
#define LCD_ETMOD                   0xB7   /* Entry Mode Set */
#define LCD_BACKLIGHT1              0xB8   /* Backlight Control 1 */
#define LCD_BACKLIGHT2              0xB9   /* Backlight Control 2 */
#define LCD_BACKLIGHT3              0xBA   /* Backlight Control 3 */
#define LCD_BACKLIGHT4              0xBB   /* Backlight Control 4 */
#define LCD_BACKLIGHT5              0xBC   /* Backlight Control 5 */
#define LCD_BACKLIGHT7              0xBE   /* Backlight Control 7 */
#define LCD_BACKLIGHT8              0xBF   /* Backlight Control 8 */
#define LCD_POWER1                  0xC0   /* Power Control 1 register */
#define LCD_POWER2                  0xC1   /* Power Control 2 register */
#define LCD_VCOM1                   0xC5   /* VCOM Control 1 register */
#define LCD_VCOM2                   0xC7   /* VCOM Control 2 register */
#define LCD_NVMWR                   0xD0   /* NV Memory Write */
#define LCD_NVMPKEY                 0xD1   /* NV Memory Protection Key */
#define LCD_RDNVM                   0xD2   /* NV Memory Status Read */
#define LCD_READ_ID4                0xD3   /* Read ID4 */
#define LCD_PGAMMA                  0xE0   /* Positive Gamma Correction register */
#define LCD_NGAMMA                  0xE1   /* Negative Gamma Correction register */
#define LCD_DGAMCTRL1               0xE2   /* Digital Gamma Control 1 */
#define LCD_DGAMCTRL2               0xE3   /* Digital Gamma Control 2 */
#define LCD_INTERFACE               0xF6   /* Interface control register */

/* Extend register commands */
#define LCD_POWERA                  0xCB   /* Power control A register */
#define LCD_POWERB                  0xCF   /* Power control B register */
#define LCD_DTCA                    0xE8   /* Driver timing control A */
#define LCD_DTCB                    0xEA   /* Driver timing control B */
#define LCD_POWER_SEQ               0xED   /* Power on sequence register */
#define LCD_3GAMMA_EN               0xF2   /* 3 Gamma enable register */
#define LCD_PRC                     0xF7   /* Pump ratio control register */

/* Size of read registers */
#define LCD_READ_ID4_SIZE           3      /* Size of Read ID4 */

#define ILI9341_NO_CURSOR           0xFFFF

class DisplayImpl : public Display
{
public:
    /**
     * \return an instance to this class (singleton)
     */
    static DisplayImpl& instance();
    
    /**
     * Turn the display On after it has been turned Off.
     * Display initial state is On.
     */
    void doTurnOn() override;

    /**
     * Turn the display Off. It can be later turned back On.
     */
    void doTurnOff() override;

    /**
     * Set display brightness. Depending on the underlying driver,
     * may do nothing.
     * \param brt from 0 to 100
     */
    void doSetBrightness(int brt) override;
    
    /**
     * \return a pair with the display height and width
     */
    std::pair<short int, short int> doGetSize() const override;

    /**
     * Write text to the display. If text is too long it will be truncated
     * \param p point where the upper left corner of the text will be printed
     * \param text, text to print.
     */
    void write(Point p, const char *text) override;

    /**
     * Write part of text to the display
     * \param p point of the upper left corner where the text will be drawn.
     * Negative coordinates are allowed, as long as the clipped view has
     * positive or zero coordinates
     * \param a Upper left corner of clipping rectangle
     * \param b Lower right corner of clipping rectangle
     * \param text text to write
     */
    void clippedWrite(Point p, Point a, Point b, const char *text) override;

    /**
     * Clear the Display. The screen will be filled with the desired color
     * \param color fill color
     */
    void clear(Color color) override;

    /**
     * Clear an area of the screen
     * \param p1 upper left corner of area to clear
     * \param p2 lower right corner of area to clear
     * \param color fill color
     */
    void clear(Point p1, Point p2, Color color) override;

    /**
     * This backend does not require it, so it is a blank.
     */
    void beginPixel() override;

    /**
     * Draw a pixel with desired color. You have to call beginPixel() once
     * before calling setPixel()
     * \param p point where to draw pixel
     * \param color pixel color
     */
    void setPixel(Point p, Color color) override;

    /**
     * Draw a line between point a and point b, with color c
     * \param a first point
     * \param b second point
     * \param c line color
     */
    void line(Point a, Point b, Color color) override;

    /**
     * Draw an horizontal line on screen.
     * Instead of line(), this member function takes an array of colors to be
     * able to individually set pixel colors of a line.
     * \param p starting point of the line
     * \param colors an array of pixel colors whoase size must be b.x()-a.x()+1
     * \param length length of colors array.
     * p.x()+length must be <= display.width()
     */
    void scanLine(Point p, const Color *colors, unsigned short length) override;
    
    /**
     * \return a buffer of length equal to this->getWidth() that can be used to
     * render a scanline.
     */
    Color *getScanLineBuffer() override;
    
    /**
     * Draw the content of the last getScanLineBuffer() on an horizontal line
     * on the screen.
     * \param p starting point of the line
     * \param length length of colors array.
     * p.x()+length must be <= display.width()
     */
    void scanLineBuffer(Point p, unsigned short length) override;

    /**
     * Draw an image on the screen
     * \param p point of the upper left corner where the image will be drawn
     * \param i image to draw
     */
    void drawImage(Point p, const ImageBase& img) override;

    /**
     * Draw part of an image on the screen
     * \param p point of the upper left corner where the image will be drawn.
     * Negative coordinates are allowed, as long as the clipped view has
     * positive or zero coordinates
     * \param a Upper left corner of clipping rectangle
     * \param b Lower right corner of clipping rectangle
     * \param i Image to draw
     */
    void clippedDrawImage(Point p, Point a, Point b, const ImageBase& img) override;

    /**
     * Draw a rectangle (not filled) with the desired color
     * \param a upper left corner of the rectangle
     * \param b lower right corner of the rectangle
     * \param c color of the line
     */
    void drawRectangle(Point a, Point b, Color c) override;
    
    /**
     * Pixel iterator. A pixel iterator is an output iterator that allows to
     * define a window on the display and write to its pixels.
     */
    class pixel_iterator
    {
    public:
        /**
         * Default constructor, results in an invalid iterator.
         * Note that since aIncr and sIncr are both zero all the writes will
         * happens to the same memory location, but we need a safe
         * /dev/null-like location where to write, which is dummy
         */
        pixel_iterator() : ctr(0), endCtr(0), aIncr(0), sIncr(0),
                dataPtr(&dummy) {}

        /**
         * Set a pixel and move the pointer to the next one
         * \param color color to set the current pixel
         * \return a reference to this
         */
        pixel_iterator& operator= (Color color)
        {
            *dataPtr=color;

            //This is to move to the adjacent pixel
            dataPtr+=aIncr;
            
            //This is the step move to the next horizontal/vertical line
            if(++ctr>=endCtr)
            {
                ctr=0;
                dataPtr+=sIncr;
            }
            return *this;
        }

        /**
         * Compare two pixel_iterators for equality.
         * They are equal if they point to the same location.
         */
        bool operator== (const pixel_iterator& itr)
        {
            return this->dataPtr==itr.dataPtr;
        }

        /**
         * Compare two pixel_iterators for inequality.
         * They different if they point to different locations.
         */
        bool operator!= (const pixel_iterator& itr)
        {
            return this->dataPtr!=itr.dataPtr;
        }

        /**
         * \return a reference to this.
         */
        pixel_iterator& operator* () { return *this; }

        /**
         * \return a reference to this. Does not increment pixel pointer.
         */
        pixel_iterator& operator++ ()  { return *this; }

        /**
         * \return a reference to this. Does not increment pixel pointer.
         */
        pixel_iterator& operator++ (int)  { return *this; }
        
        /**
         * Must be called if not all pixels of the required window are going
         * to be written.
         */
        void invalidate() {}

    private:
        /**
         * Constructor
         * \param start Upper left corner of window
         * \param end Lower right corner of window
         * \param direction Iterator direction
         * \param disp Display we're associated
         */
        pixel_iterator(Point start, Point end, IteratorDirection direction,
                DisplayImpl *disp) : ctr(0), dataPtr(disp->framebuffer1)
        {
            //Compute the increment in the adjacent direction (aIncr) and in the
            //step direction (sIncr) depending on the direction
            dataPtr+=start.y()*disp->getWidth()+start.x();
            if(direction==RD)
            {
                endCtr=end.x()+1-start.x();
                aIncr=1;
                sIncr=disp->getWidth()-endCtr;
            } else {
                endCtr=end.y()+1-start.y();
                aIncr=disp->getWidth();
                sIncr=-aIncr*endCtr+1;
            }
        }

        unsigned short ctr;           ///< Counter to decide when to step
        unsigned short endCtr;        ///< When ctr==endCtr apply a step
        
        short aIncr;                  ///< Adjacent increment
        int sIncr;                    ///< Step increment           
        Color *dataPtr;               ///< Pointer to framebuffer
        
        static Color dummy;           ///< Invalid iterators write here

        friend class DisplayImpl; //Needs access to ctor
    };

    /**
     * Specify a window on screen and return an object that allows to write
     * its pixels.
     * Note: a call to begin() will invalidate any previous iterator.
     * \param p1 upper left corner of window
     * \param p2 lower right corner (included)
     * \param d increment direction
     * \return a pixel iterator
     */
    pixel_iterator begin(Point p1, Point p2, IteratorDirection d);

    /**
     * \return an iterator which is one past the last pixel in the pixel
     * specified by begin. Behaviour is undefined if called before calling
     * begin()
     */
    pixel_iterator end() const { return last; }
    
    /**
     * Destructor
     */
    ~DisplayImpl() override;

private:
    /**
     * Constructor.
     * Do not instantiate objects of this type directly from application code.
     */
    DisplayImpl();
    
    static void doWritePixel(Color c);
    
    #if defined MXGUI_ORIENTATION_VERTICAL
    static const short int width=240;
    static const short int height=320;
    #elif defined MXGUI_ORIENTATION_HORIZONTAL || \
          defined MXGUI_ORIENTATION_VERTICAL_MIRRORED || \
          defined MXGUI_ORIENTATION_HORIZONTAL_MIRRORED
    #error unsupported orientation
    #else
    #error No orientation defined
    #endif

    /**
     * Pointer to the memory mapped display.
     */
    Color * const framebuffer1;
    Color *buffer; ///< For scanLineBuffer
    pixel_iterator last; ///< Last iterator for end of iteration check
    static const unsigned int bpp=sizeof(Color); ///< Bytes per pixel
    static const int numPixels=width*height; ///< Number of pixels of the display

    Color *buffer2;                   ///< For DMA transfers
    static const int buffer2Size=512; ///< DMA buffer size
};

} //namespace mxgui

#endif //_BOARD_ST25DVDISCOVERY

#endif //DISPLAY_ST25DVDISCOVERY_H
