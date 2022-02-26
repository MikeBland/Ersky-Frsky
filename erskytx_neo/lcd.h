/****************************************************************************
*  Copyright (c) 2021 by Michael Blandford. All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*  3. Neither the name of the author nor the names of its contributors may
*     be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
*  SUCH DAMAGE.
*
****************************************************************************/



#define PLOT_XOR		0
#define PLOT_BLACK	1
#define PLOT_WHITE	2
#define PLOT_CUSTOM	3

#define FWNUM		5
#define FW			6
#define FH			8

/* lcd common flags */
#define INVERS        0x01
#define BLINK         0x02
#define DBLSIZE       0x04
#define CONDENSED     0x08
#define BOLD          0x80

#define BLINK_ON_PHASE (g_blinkTmr10ms & (1<<6))

// putsChnRaw flags
#define MIX_SOURCE    0x10

// putTxSwr flags
#define TSSI_TEXT			0x20

/* lcd outdez flags */
#define LEADING0      0x0400
#define PREC1         0x20
#define PREC2         0x30 /* 4 modes in 2bits!, now removed */
#define LEFT          0x40 /* align left */

#define NO_UNIT       0x80

#define SCREEN_LINES		8

#define LCD_W	128
#define LCD_H 64

extern uint8_t DisplayBuf[1024] ;
extern volatile uint8_t g_blinkTmr10ms ;
extern uint16_t Lcd_lastPos ;


void refreshDisplay() ;
void setBacklightBrightness( uint32_t percent ) ;
void initBacklight() ;

uint8_t lcd_putc( uint8_t x, uint8_t y, const char c ) ;
uint8_t lcd_putcAtt(uint8_t x,uint8_t y,const char c,uint8_t mode) ;
uint8_t lcd_putsAtt( uint8_t x, uint8_t y, const char *s, uint8_t mode ) ;
void lcd_outhex2(uint8_t x,uint8_t y,uint8_t val) ;
void lcd_outhex4(uint8_t x,uint8_t y,uint16_t val) ;
void lcd_outhex8(uint8_t x,uint8_t y,uint32_t val) ;
void clearDisplay() ;

void lcd_hlineStip( unsigned char x, unsigned char y, int16_t w, uint8_t pat ) ;
void lcd_hline( uint8_t x, uint8_t y, int8_t w ) ;
void lcd_write_bits( uint8_t *p, uint8_t mask ) ;
void lcd_vline( uint8_t x, uint8_t y, int8_t h ) ;
void lcd_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h ) ;
void lcd_hbar( uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t percent ) ;
void pushPlotType( uint8_t type ) ;
void popPlotType() ;
void lcd_line( uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t pat, uint8_t att ) ;
void lcd_outdez( uint8_t x, uint8_t y, int16_t val ) ;
void lcd_outdezAtt( uint8_t x, uint8_t y, int16_t val, uint16_t mode ) ;
void lcd_2_digits( uint8_t x, uint8_t y, uint8_t value, uint16_t attr ) ;
uint8_t lcd_outdezNAtt( uint8_t x, uint8_t y, int32_t val, uint16_t mode, int8_t len ) ;
void lcd_puts_P( uint8_t x, uint8_t y, const char *s ) ;
void lcd_puts_Pleft( uint8_t y, const char *s ) ;
void lcd_putsnAtt(uint8_t x, uint8_t y, const char * s, uint8_t len, uint8_t mode) ;
void lcd_putsn_P(uint8_t x,uint8_t y,const char * s,uint8_t len) ;
void lcd_char_inverse( uint8_t x, uint8_t y, uint8_t w, uint8_t blink ) ;
void lcd_putsAttIdx(uint8_t x,uint8_t y,const char * s,uint8_t idx,uint8_t att) ;
void lcd_bitmap( uint8_t i_x, uint8_t i_y, uint8_t *bitmap, uint8_t w, uint8_t h, uint8_t mode ) ;
void lcd_img( uint8_t i_x, uint8_t i_y, uint8_t *imgdat, uint8_t idx, uint8_t mode ) ;
void lcd_plot( register uint8_t x, register uint8_t y ) ;
void lcd_2_digits( uint16_t x, uint16_t y, uint8_t value, uint16_t attr ) ;
void putsTime(uint16_t x,uint16_t y,int16_t tme,uint8_t att,uint8_t att2 ) ;
void putsVolts(uint8_t x,uint8_t y, uint8_t volts, uint8_t att) ;



