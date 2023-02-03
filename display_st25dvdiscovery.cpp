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

#include "display_st25dvdiscovery.h"
#include "miosix.h"
#include <cstdarg>

using namespace std;
using namespace miosix;

#ifdef _BOARD_ST25DVDISCOVERY

namespace mxgui {

//Control interface
typedef Gpio<GPIOB_BASE,13> sck;    //SPI2 SCK
typedef Gpio<GPIOC_BASE,3> mosi;   //SPI2 MOSI
typedef Gpio<GPIOB_BASE, 12> csx; //SPI2 CS
typedef Gpio<GPIOC_BASE,1> te;  //Tearing effect output from display, unused
typedef Gpio<GPIOC_BASE, 0> wr; //SPI2 WRX

//
// class DisplayImpl
//
const short int DisplayImpl::width;
const short int DisplayImpl::height;


/**
 * Send and receive a byte through SPI2
 * \param c byte to send
 * \return byte received
 */
static unsigned char spi2sendRev(unsigned char c=0)
{
    SPI2->DR=c;
    while((SPI2->SR & SPI_SR_RXNE)==0) ;
    return SPI2->DR;
}

/**
 * Send and receive a byte through SPI2
 * \param c byte to send
 * \return byte received
 */
static unsigned char spi2sendRev16(unsigned int i=0)
{
    SPI2->DR=i;
    while((SPI2->SR & SPI_SR_RXNE)==0) ;
    return SPI2->DR;
}

/**
 * Send a command to the ILI9341 display controller
 * \param cmd command
 * \param len length of the (optional) argument, or 0 for commands without
 * arguments.
 */
static void sendCmd(unsigned char cmd, int len, ...)
{
    wr::low();
    csx::low();
    spi2sendRev(cmd);
    csx::high();
    delayUs(1);
    wr::high();
    va_list arg;
    va_start(arg,len);
    for(int i=0;i<len;i++)
    {   
        csx::low();
        spi2sendRev(va_arg(arg,int));
        csx::high();
        delayUs(1);
    }
    va_end(arg);
}

/**
 * Set cursor to desired location
 * \param point where to set cursor (0<=x<128, 0<=y<128)
 */
static void setCursor(Point p)
{
    #ifdef MXGUI_ORIENTATION_VERTICAL
    sendCmd(LCD_COLUMN_ADDR, 4, (p.x() & 0xff00) >> 8, p.x() & 0x00ff , 0x00 , 0xf0); // Set column address
    sendCmd(LCD_PAGE_ADDR, 4, (p.y() & 0xff00) >> 8, p.y() & 0x00ff , 0x01, 0x40); // Set row address
    sendCmd(LCD_GRAM, 0); 
    #else //MXGUI_ORIENTATION_HORIZONTAL
    sendCmd(LCD_COLUMN_ADDR, 4, (p.y() & 0xff00) >> 8, p.y() & 0x00ff , 0x01, 0x40); // Set row address
    sendCmd(LCD_PAGE_ADDR, 4, (p.x() & 0xff00) >> 8, p.x() & 0x00ff , 0x00, 0xf0); // Set column address
    sendCmd(LCD_GRAM, 0);
    #endif //Hardware doesn't seem to support mirroring
}

/**
 * Set a hardware window on the screen, optimized for writing text.
 * The GRAM increment will be set to up-to-down first, then left-to-right which
 * is the correct increment to draw fonts
 * \param p1 upper left corner of the window
 * \param p2 lower right corner of the window
 */
static void textWindow(Point p1, Point p2)
{
    #ifdef MXGUI_ORIENTATION_VERTICAL
    sendCmd(LCD_COLUMN_ADDR, 4, (p1.x() & 0xff00) >> 8, p1.x() & 0x00ff , (p2.x() & 0xff00) >> 8, p2.x() & 0x00ff); // Set column address
    sendCmd(LCD_PAGE_ADDR, 4, (p1.y() & 0xff00) >> 8, p1.y() & 0x00ff , (p2.y() & 0xff00) >> 8, p2.y() & 0x00ff); // Set row address
    sendCmd(LCD_MAC, 1, 0x28);
    sendCmd(LCD_GRAM, 0); 
    #else //MXGUI_ORIENTATION_HORIZONTAL
    sendCmd(LCD_COLUMN_ADDR, 4, (p1.y() & 0xff00) >> 8, p1.y() & 0x00ff , (p2.y() & 0xff00) >> 8, p2.y() & 0x00ff); // Set row address
    sendCmd(LCD_PAGE_ADDR, 4, (p1.x() & 0xff00) >> 8, p1.x() & 0x00ff , (p2.x() & 0xff00) >> 8, p2.x() & 0x00ff); // Set column address
    sendCmd(LCD_MAC, 1, 0x2c);
    sendCmd(LCD_GRAM, 0);
    #endif //Hardware doesn't seem to support mirroring
}

/**
 * Set a hardware window on the screen, optimized for drawing images.
 * The GRAM increment will be set to left-to-right first, then up-to-down which
 * is the correct increment to draw images
 * \param p1 upper left corner of the window
 * \param p2 lower right corner of the window
 */
static inline void imageWindow(Point p1, Point p2)
{
    #ifdef MXGUI_ORIENTATION_VERTICAL
    sendCmd(LCD_COLUMN_ADDR, 4,(p1.x() & 0xff00) >> 8, p1.x() & 0x00ff , (p2.x() & 0xff00) >> 8, p2.x() & 0x00ff); // Set column address
    sendCmd(LCD_PAGE_ADDR, 4, (p1.y() & 0xff00) >> 8, p1.y() & 0x00ff , (p2.y() & 0xff00) >> 8, p2.y() & 0x00ff); // Set row address
    sendCmd(LCD_MAC, 1, 0x08);
    sendCmd(LCD_GRAM, 0); 
    #else //MXGUI_ORIENTATION_HORIZONTAL
    sendCmd(LCD_COLUMN_ADDR, 4, (p1.y() & 0xff00) >> 8, p1.y() & 0x00ff , (p2.y()) & 0xff00) >> 8, p2.y() & 0x00ff); // Set row address
    sendCmd(LCD_PAGE_ADDR, 4, (p1.x() & 0xff00) >> 8, p1.x() & 0x00ff , (p2.x() & 0xff00) >> 8, p2.x() & 0x00ff); // Set column address
    sendCmd(LCD_MAC, 1, 0x0c);
    sendCmd(LCD_GRAM, 0);
    #endif //Hardware doesn't seem to support mirroring
}

void registerDisplayHook(DisplayManager& dm)
{
    dm.registerDisplay(&DisplayImpl::instance());
}

DisplayImpl& DisplayImpl::instance()
{
    static DisplayImpl instance;
    return instance;
}

void DisplayImpl::doTurnOn()
{
    sendCmd(LCD_DISPLAY_ON,0); //LCD_DISPLAY_ON
}

void DisplayImpl::doTurnOff()
{
    sendCmd(LCD_DISPLAY_OFF,0); //LCD_DISPLAY_OFF
}

void DisplayImpl::doSetBrightness(int brt) {}

pair<short int, short int> DisplayImpl::doGetSize() const
{
    return make_pair(height,width);
}

void DisplayImpl::write(Point p, const char *text)
{
    font.draw(*this,textColor,p,text);
}

void DisplayImpl::clippedWrite(Point p, Point a, Point b, const char *text)
{
    font.clippedDraw(*this,textColor,p,a,b,text);
}

void DisplayImpl::clear(Color color)
{
    clear(Point(0,0),Point(width-1,height-1),color);
}

void DisplayImpl::clear(Point p1, Point p2, Color color)
{
    if(p1.x()<0 || p2.x()<p1.x() || p2.x()>=width
     ||p1.y()<0 || p2.y()<p1.y() || p2.y()>=height) return;
    if((color & 0xff)==(color>>8))
    {
        //Can use memset
        if(p1.x()==0 && p2.x()==width-1)
        {
            //Can merge lines
            memset(framebuffer1+p1.y()*width,color,(p2.y()-p1.y()+1)*width*bpp);
        } else {
            //Can't merge lines
            Color *ptr=framebuffer1+p1.x()+width*p1.y();
            short len=p2.x()-p1.x()+1;
            for(short i=p1.y();i<=p2.y();i++)
            {
                memset(ptr,color,len*bpp);
                ptr+=width;
            }
        }
    } else {
        //Can't use memset
        if(p1.x()==0 && p2.x()==width-1)
        {
            //Can merge lines
            Color *ptr=framebuffer1+p1.y()*width;
            int numPixels=(p2.y()-p1.y()+1)*width;
            //This loop is worth unrolling
            for(int i=0;i<numPixels/4;i++)
            {
                *ptr++=color;
                *ptr++=color;
                *ptr++=color;
                *ptr++=color;
            }
            for(int i=0;i<(numPixels & 3);i++) *ptr++=color;
        } else {
            //Can't merge lines
            Color *ptr=framebuffer1+p1.x()+width*p1.y();
            short len=p2.x()-p1.x()+1;
            for(short i=p1.y();i<=p2.y();i++)
            {
                for(short j=0;j<len;j++) *ptr++=color;
                ptr+=width-len;
            }
        }
    }
}

void DisplayImpl::setPixel(Point p, Color color)
{
    setCursor(p);
    spi2sendRev16(color);
}

void DisplayImpl::beginPixel() {}

void DisplayImpl::line(Point a, Point b, Color color)
{
    //Horizontal line speed optimization
    if(a.y()==b.y())
    {
        imageWindow(Point(min(a.x(),b.x()),a.y()),
                    Point(max(a.x(),b.x()),a.y()));
        int numPixels=abs(a.x()-b.x());
        for(int i=0;i<=numPixels;i++) doWritePixel(color);
        return;
    }
    //Vertical line speed optimization
    if(a.x()==b.x())
    {
        textWindow(Point(a.x(),min(a.y(),b.y())),
                    Point(a.x(),max(a.y(),b.y())));
        int numPixels=abs(a.y()-b.y());
        for(int i=0;i<=numPixels;i++) doWritePixel(color);
        return;
    }
    //General case
    Line::draw(*this,a,b,color);
}

void DisplayImpl::scanLine(Point p, const Color *colors, unsigned short length) {}

Color *DisplayImpl::getScanLineBuffer() {
	if(buffer == nullptr) buffer = new Color[getWidth()];
	return buffer; 
}

void DisplayImpl::scanLineBuffer(Point p, unsigned short length) {}

void DisplayImpl::drawImage(Point p, const ImageBase& img) {}

void DisplayImpl::clippedDrawImage(Point p, Point a, Point b, const ImageBase& img) {}


void DisplayImpl::drawRectangle(Point a, Point b, Color c)
{
    line(a,Point(b.x(),a.y()),c);
    line(Point(b.x(),a.y()),b,c);
    line(b,Point(a.x(),b.y()),c);
    line(Point(a.x(),b.y()),a,c);
}

DisplayImpl::pixel_iterator DisplayImpl::begin(Point p1, Point p2,
        IteratorDirection d)
{
    if(p1.x()<0 || p1.y()<0 || p2.x()<0 || p2.y()<0) return pixel_iterator();
    if(p1.x()>=width || p1.y()>=height || p2.x()>=width || p2.y()>=height)
        return pixel_iterator();
    if(p2.x()<p1.x() || p2.y()<p1.y()) return pixel_iterator();
 
    //Set the last iterator to a suitable one-past-the last value
    if(d==DR) this->last=pixel_iterator(Point(p2.x()+1,p1.y()),p2,d,this);
    else this->last=pixel_iterator(Point(p1.x(),p2.y()+1),p2,d,this);

    return pixel_iterator(p1,p2,d,this);
}

DisplayImpl::~DisplayImpl() {}

DisplayImpl::DisplayImpl()
    : framebuffer1(reinterpret_cast<unsigned short*>(0xd0600000)),
      buffer(framebuffer1+numPixels)
{
    {
        FastInterruptDisableLock dLock;
        sck::mode(Mode::ALTERNATE);    sck::alternateFunction(5); //SPI2
        mosi::mode(Mode::ALTERNATE);    mosi::alternateFunction(5);
        csx::mode(Mode::OUTPUT);       csx::high();
        wr::mode(Mode::OUTPUT);

        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN ;    
        RCC_SYNC();
    }

    SPI2->CR1=SPI_CR1_SSM   //Sowtware CS
            | SPI_CR1_SSI   //Software CS high
            | SPI_CR1_SPE   //SPI enabled
            | (3<<3)        //Divide input clock by 16: 84/16=5.25MHz
            | SPI_CR1_MSTR; //Master mode
    Thread::sleep(1);
    
    //
    // ILI9341 power up sequence -- begin
    //
    sendCmd(LCD_SLEEP_OUT, 0); 
    Thread::sleep(200);
    sendCmd(LCD_POWER1,1,0x18);                     //LCD_POWER1
    sendCmd(LCD_POWER2,1,0x11);                     //LCD_POWER2
    sendCmd(LCD_VCOM1,2,0x3e,0x15);                //LCD_VCOM1
    sendCmd(LCD_MAC,1,0x28);                     //LCD_MAC
    sendCmd(LCD_TEOFF,0); 
    sendCmd(LCD_PGAMMA,15,0x0f,0x29,0x24,0x0c,0x0e,0x09,0x4e,0x78,0x3c,0x09,0x13,
            0x05,0x17,0x11,0x00);             //LCD_PGAMMA
    sendCmd(LCD_NGAMMA,15,0x00,0x16,0x1b,0x04,0x11,0x07,0x31,0x33,0x42,0x05,0x0c,
            0x0a,0x28,0x2f,0x0f);             //LCD_NGAMMA
    sendCmd(LCD_PIXEL_FORMAT,1, 0x55);          //RGB & MCU 16 bits/pixel
    sendCmd(LCD_COLUMN_ADDR, 4, 0x00, 0x00, (width-1) & 0xff00 >> 8 , (width-1)& 0x00ff); 
    sendCmd(LCD_PAGE_ADDR, 4, 0x00, 0x00, (height-1)& 0xff00 >> 8, (height-1)&0x00ff); 
    sendCmd(LCD_DISPLAY_ON,0);  

    /**
    sendCmd(0xca,3,0xc3,0x08,0x50);           //undocumented command
    sendCmd(LCD_POWERB,3,0x00,0xc1,0x30);           //LCD_POWERB
    sendCmd(LCD_POWER_SEQ,4,0x64,0x03,0x12,0x81);      //LCD_POWER_SEQ
    sendCmd(LCD_DTCA,3,0x85,0x00,0x78);           //LCD_DTCA
    sendCmd(LCD_POWERA,5,0x39,0x2c,0x00,0x34,0x02); //LCD_POWERA
    sendCmd(LCD_PRC,1,0x20);                     //LCD_PRC
    sendCmd(LCD_DTCB,2,0x00,0x00);                //LCD_DTCB
    sendCmd(LCD_FRMCTR1,2,0x00,0x1b);                //LCD_FRMCTR1
    sendCmd(LCD_DFC,2,0x0a,0xa2);                //LCD_DFC
    sendCmd(LCD_POWER1,1,0x18);                     //LCD_POWER1
    sendCmd(LCD_POWER2,1,0x11);                     //LCD_POWER2
    sendCmd(LCD_VCOM1,2,0x3e,0x15);                //LCD_VCOM1
    sendCmd(LCD_MAC,1,0x28);                     //LCD_MAC
    sendCmd(LCD_3GAMMA_EN,1,0x00);                     //LCD_3GAMMA_EN
    sendCmd(LCD_RGB_INTERFACE,1,0xc2);                     //LCD_RGB_INTERFACE
    sendCmd(LCD_DFC,4,0x0a,0xa7,0x27,0x04);      //LCD_DFC
    sendCmd(LCD_COLUMN_ADDR,4,0x00,0x00,0x00,0xef);      //LCD_COLUMN_ADDR
    sendCmd(LCD_PAGE_ADDR,4,0x00,0x00,0x01,0x3f);      //LCD_PAGE_ADDR
    sendCmd(LCD_INTERFACE,3,0x01,0x00,0x06);           //LCD_INTERFACE
    sendCmd(LCD_TEOFF,0);
    sendCmd(LCD_GRAM,0);                          //LCD_GRAM
    Thread::sleep(200);
    sendCmd(LCD_GAMMA,1,0x01);                     //LCD_GAMMA
    sendCmd(LCD_PGAMMA,15,0x0f,0x29,0x24,0x0c,0x0e,0x09,0x4e,0x78,0x3c,0x09,0x13,
            0x05,0x17,0x11,0x00);             //LCD_PGAMMA
    sendCmd(LCD_NGAMMA,15,0x00,0x16,0x1b,0x04,0x11,0x07,0x31,0x33,0x42,0x05,0x0c,
            0x0a,0x28,0x2f,0x0f);             //LCD_NGAMMA
    sendCmd(LCD_PIXEL_FORMAT,1, 0x55);          //RGB & MCU 16 bits/pixel
    sendCmd(LCD_SLEEP_OUT,0);                          //LCD_SLEEP_OUT
    Thread::sleep(200);
    sendCmd(LCD_DISPLAY_ON,0);                          //LCD_DISPLAY_ON
    //
    // ILI9341 power up sequence -- end
    //
    **/

    setTextColor(make_pair(Color(0xffff),Color(0x0000)));
    clear(black);
}

void DisplayImpl::doWritePixel(Color c)
{
    spi2sendRev16(c);
}

Color DisplayImpl::pixel_iterator::dummy;

} //namespace mxgui

#endif //_BOARD_ST25DVDISCOVERY
