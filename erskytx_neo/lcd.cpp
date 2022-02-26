/****************************************************************************
*  
*  Copyright (c) 2021 by Michael Blandford. All rights reserved.
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

#include <Arduino.h>

#include "lcd.h"
#include <soc/ledc_struct.h>
#include <soc/dport_reg.h>

#define BITMASK(bit) (1<<(bit))

uint8_t DisplayBuf[1024] ;

#define DISPLAY_START (DisplayBuf + 0)
#define DISPLAY_END (DisplayBuf+sizeof(DisplayBuf))
#define DISPLAY_H	64
#define DISPLAY_W 128

uint8_t plotType = PLOT_XOR ;
uint16_t Lcd_lastPos ;

uint8_t plotStack[4] ;
uint8_t plotStackIndex ;
volatile uint8_t g_blinkTmr10ms ;


#include "fontlbm.h"
#define font_5x8_x7f (font)

#include "font_dblsizelbm.h"
#define font_10x16_x7f (font_dblsize)

#include "font12x8lbm.h"


void pushPlotType( uint8_t type )
{
	if ( plotStackIndex < 4 )
	{
		plotStack[plotStackIndex++] = plotType ;
	}
	plotType = type ;
}

void popPlotType()
{
	if ( plotStackIndex )
	{
		plotStackIndex -= 1 ;
	}
	plotType = plotStack[plotStackIndex] ;
}

static uint8_t *dispBufAddress( uint8_t x, uint8_t y )
{
  return &DISPLAY_START[ (y & 0xF8) * 16 + x ];
}

uint8_t lcd_putc(uint8_t x,uint8_t y,const char c )
{
  return lcd_putcAtt(x,y,c,0);
}

uint8_t lcd_putcAtt(uint8_t x,uint8_t y,const char c,uint8_t mode)
{
	int32_t i ;
	uint8_t *p = dispBufAddress( x, y ) ;
	
	if ( c < 22 )		// Move to specific x position (c)*FW
	{
		x = c*FW ;
  	if(mode&DBLSIZE)
		{
			if ( (mode & CONDENSED) )
			{
				x = c*8 ;
			}
		 	else
			{
				x += x ;
			}	 
		}
		return x ;
	}
	x += FW ;
  register uint8_t *q ;
	if( c < 0xC0 )
	{
		q = (uint8_t *) &font_5x8_x7f[(c-0x20)*5] ;
	}
	else
	{
		q = (uint8_t *) &font_5x8_x7f[0] ;
	}

  register bool inv = false ;
	if (mode & INVERS) inv = true ;
	if ( (mode & BLINK) && BLINK_ON_PHASE )
	{
		inv = !inv ;
	}
  

	if(mode&DBLSIZE)
  {
		uint32_t doNormal = 1 ;
	  unsigned char c_mapped = c ;

		if ( (mode & CONDENSED) )
		{
			doNormal = 0 ;
			
			if ( doNormal == 0 )
			{
				if ( (c!=0x2E)) x+=8-FW; //check for decimal point
			/* each letter consists of 8 top bytes followed by
	 		* five bottom by 8 bottom bytes (16 bytes per 
	 		* char) */
				if( c < 0xC0 )
				{
					c_mapped = c - 0x20 ;
					q = (uint8_t *) &font_12x8[c_mapped*14] ;
				}
				else
				{
					q = (uint8_t *) &font_12x8[0] ;
				}
    		
				for( i=7 ; i>=0 ; i-- )
				{
					uint8_t b1 ;
					uint8_t b3 ;

  		  	/*top byte*/
   		  	b1 = *q ;
  		  	/*bottom byte*/
   		  	b3 = *(q+7) ;
   		  	q++;
					if ( i == 0 )
					{
						b1 = 0 ;
						b3 = 0 ;
					}
    		  if(inv)
					{
				    b1=~b1;
				    b3=~b3;
    		  }

    		  if(&p[DISPLAY_W+1] < DISPLAY_END)
					{
    		    p[0]=b1;
    		    p[DISPLAY_W] = b3;
    		    p+=1;
    		  }
    		}
			}
		}
		if ( doNormal )
		{
			if ( (c!=0x2E)) x+=FW; //check for decimal point
		/* each letter consists of ten top bytes followed by
	 	* five bottom by ten bottom bytes (20 bytes per 
	 	* char) */
	  	unsigned char c_mapped ;
			if( c < 0xC0 )
			{
				c_mapped = c - 0x20 ;
				q = (uint8_t *) &font_10x16_x7f[(c_mapped)*20] ;
			}
			else
			{
//				if ( ExtraBigFont )
//				{
//					q = (uint8_t *) &ExtraBigFont[(c-0xC0)*10] ;
//				}
//				else
				{
					q = (uint8_t *) &font_10x16_x7f[0] ;
				}
			}
    	for( i=5 ; i>=0 ; i-- )
			{
				uint8_t b1 ;
				uint8_t b3 ;
				uint8_t b2 ;
				uint8_t b4 ;

				if( c < 0xC0 )
				{
	  	  	/*top byte*/
    	  	b1 = i>0 ? *q : 0;
	  	  	/*bottom byte*/
    	  	b3 = i>0 ? *(q+10) : 0;
	  	  	/*top byte*/
    	  	b2 = i>0 ? *(++q) : 0;
	  	  	/*bottom byte*/
    	  	b4 = i>0 ? *(q+10) : 0;
    	  	q++;
				}
				else
				{
	  	  	/*top byte*/
    	  	b1 = i>0 ? *q++ : 0 ;
	  	  	/*bottom byte*/
    	  	b3 = i>0 ? *q++ : 0 ;
	  	  	/*top byte*/
    	  	b2 = i>0 ? *q++ : 0 ;
	  	  	/*bottom byte*/
    	  	b4 = i>0 ? *q++ : 0 ;
				}
    	  if(inv)
				{
			    b1=~b1;
			    b2=~b2;
			    b3=~b3;
			    b4=~b4;
    	  }

    	  if(&p[DISPLAY_W+1] < DISPLAY_END)
				{
    	    p[0]=b1;
    	    p[1]=b2;
    	    p[DISPLAY_W] = b3;
    	    p[DISPLAY_W+1] = b4;
    	    p+=2;
    	  }
    	}
		}
  }
  else
  {
		uint8_t condense=0;

		if (mode & CONDENSED)
		{
			*p = inv ? ~0 : 0;
			p += 1 ;
			condense=1;
			x += FWNUM-FW ;
		}

		y &= 7 ;
		if ( y )
		{ // off grid
    	for( i=5 ; i!=0 ; i-- )
			{
      	uint16_t b = *q++ ;
				if ( inv )
				{
					b = ~b & 0x00FF ;
				}
				b <<= y ;
      	if(p<DISPLAY_END) *p ^= b ;
      	if(&p[DISPLAY_W] < DISPLAY_END)
				{
	        p[DISPLAY_W] ^= b >> 8 ;
				}
				p += 1 ;
			}
			if ( inv )
			{
				uint16_t b = 0xFF ;
     		if(p<DISPLAY_END) *p ^= ( b <<= y) ;
     		if(&p[DISPLAY_W] < DISPLAY_END) p[DISPLAY_W] ^= b >> 8 ;
			}
		}
		else
		{
			uint8_t oldb = 0 ;
  		for( i=5 ; i!=0 ; i-- )
			{
    		uint8_t b = *q++ ;
  		  if (condense && i==4)
				{
    	    /*condense the letter by skipping column 4 */
    	    continue;
    	  }
 	  		if(p<DISPLAY_END) *p++ = inv ? ~(b|oldb) : (b|oldb) ;
				if (mode & BOLD)
				{
					oldb = b ;
				}
  		}
 			if(p<DISPLAY_END) *p++ = inv ? ~oldb : oldb ;
		}
	}
	return x ;
}

uint8_t lcd_putsAtt( uint8_t x, uint8_t y, const char *s, uint8_t mode )
{
  while(1)
	{
    char c = *s++ ;
    if(!c) break ;
		if ( c == 31 )
		{

			if ( (y += FH) >= DISPLAY_H )	// Screen height
			{
				break ;
			}	
			
			x = 0 ;
		}
		else
		{
    	x = lcd_putcAtt(x,y,c,mode) ;
		}
  }
  return x ;
}


void lcd_outhex2(uint8_t x,uint8_t y,uint8_t val)
{
  register int i ;
  x+=FWNUM*2 ;
  for(i=0; i<2; i++)
  {
    x-=FWNUM ;
    char c = val & 0xf ;
    c = c>9 ? c+'A'-10 : c+'0' ;
    lcd_putcAtt(x,y,c,c>='A'?CONDENSED:0) ;
    val>>=4 ;
  }
}

void lcd_outhex4(uint8_t x,uint8_t y,uint16_t val)
{
  register int i ;
  x+=FWNUM*4 ;
  for(i=0; i<4; i++)
  {
    x -= FWNUM ;
    char c = val & 0xf ;
    c = c>9 ? c+'A'-10 : c+'0' ;
    lcd_putcAtt(x,y,c,c>='A'?CONDENSED:0) ;
    val>>=4 ;
  }
}

void lcd_outhex8(uint8_t x,uint8_t y,uint32_t val)
{
  register int i ;
  x+=FWNUM*8 ;
  for(i=0; i<8; i++)
  {
    x -= FWNUM ;
    char c = val & 0xf ;
    c = c>9 ? c+'A'-10 : c+'0' ;
    lcd_putcAtt(x,y,c,c>='A'?CONDENSED:0) ;
    val>>=4 ;
  }
}

void clearDisplay()
{
	uint8_t *p = DisplayBuf ;
	while ( p < DISPLAY_END )
	{
		*p++ = 0 ;
	}
}

void lcd_bitmap( uint8_t i_x, uint8_t i_y, uint8_t *bitmap, uint8_t w, uint8_t h, uint8_t mode )
{
	uint32_t yb ;
	uint32_t x ;

  bool inv = false ;
	if (mode & INVERS) inv = true ;
	if ( (mode & BLINK) && BLINK_ON_PHASE )
	{
		inv = !inv ;
	}
//	bool inv = (mode & INVERS) ? true : (mode & BLINK ? BLINK_ON_PHASE : false ) ;
  for( yb = 0; yb < h; yb++)
	{
    uint8_t *p = &DISPLAY_START[ (i_y / 8 + yb) * DISPLAY_W + i_x ];
    for(x=0; x < w; x++)
		{
      uint8_t b = *bitmap++ ;
      *p++ = inv ? ~b : b;
    }
  }
}


void lcd_hbar( uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t percent )
{
	uint8_t solid ;
	if ( percent > 100 )
	{
		percent = 100 ;
	}
	solid = (w-2) * percent / 100 ;
	lcd_rect( x, y, w, h ) ;

	if ( solid )
	{
		w = y + h - 1 ;
		y += 1 ;
		x += 1 ;
		while ( y < w )
		{
 			lcd_hline(x, y, solid ) ;
			y += 1 ;			
		}
	}
}

void lcd_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h )
{
	pushPlotType( PLOT_BLACK ) ;
  lcd_vline(x, y, h ) ;
	if ( w > 1 )
	{
  	lcd_vline(x+w-1, y, h ) ;
	}
 	lcd_hline(x+1, y+h-1, w-2 ) ;
 	lcd_hline(x+1, y, w-2 ) ;
	popPlotType() ;
}

void lcd_hlineStip( unsigned char x, unsigned char y, int16_t w, uint8_t pat )
{
  if(w<0) {x+=w; w=-w;}
	uint8_t *p = dispBufAddress( x, y ) ;
	register uint8_t msk = BITMASK(y%8) ;
  while(w)
	{
    if ( p>=DISPLAY_END)
    {
      break ;			
    }
    if(pat&1)
		{
			lcd_write_bits( p, msk ) ;
      pat = (pat >> 1) | 0x80;
    }
		else
		{
      pat = pat >> 1;
    }
    w--;
    p++;
  }
}

void lcd_hline( uint8_t x, uint8_t y, int8_t w )
{
  lcd_hlineStip(x,y,w,0xff);
}

void lcd_write_bits( uint8_t *p, uint8_t mask )
{
  if(p<DISPLAY_END)
	{
		uint8_t temp = *p ;
		if ( plotType != PLOT_XOR )
		{
			temp |= mask ;
		}
		if ( plotType != PLOT_BLACK )
		{
			temp ^= mask ;
		}
		*p = temp ;
	}
}

void lcd_vline( uint8_t x, uint8_t y, int8_t h )
{
  if (h<0) { y+=h; h=-h; }

	uint8_t *p = dispBufAddress( x, y ) ;
  y &= 0x07 ;
	if ( y )
	{
    uint8_t msk = ~(BITMASK(y)-1) ;
    h -= 8-y ;
    if (h < 0)
      msk -= ~(BITMASK(8+h)-1) ;
		lcd_write_bits( p, msk ) ;
    p += DISPLAY_W ;
	}
    
  while( h >= 8 )
	{
		h -= 8 ;
		lcd_write_bits( p, 0xFF ) ;
    p += DISPLAY_W ;
  }
	if ( h > 0 )
	{
  	lcd_write_bits( p, (BITMASK(h)-1) ) ;
	}
}

void lcd_plot( register uint8_t x, register uint8_t y )
{
	uint8_t *p = dispBufAddress( x, y ) ;
	lcd_write_bits( p, BITMASK(y%8) ) ;
}


void lcd_line( uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t pat, uint8_t att )
{
  int dx = x2-x1 ;      /* the horizontal distance of the line */
  int dy = y2-y1 ;      /* the vertical distance of the line */
  int dxabs = abs(dx) ;
  int dyabs = abs(dy) ;
  int sdx = dx > 0 ? 1 : dx < 0 ? -1 : 0 ;
  int sdy = dy > 0 ? 1 : dy < 0 ? -1 : 0 ;
  int x = dyabs>>1 ;
  int y = dxabs>>1 ;
  int px = x1 ;
  int py = y1 ;

	lcd_plot( x2, y2 ) ;

  if (dxabs >= dyabs)
	{
    /* the line is more horizontal than vertical */
    for (int i=0; i<dxabs; i++)
		{
      if ((1<<(px%8)) & pat)
			{
//        lcdDrawPoint(px, py, att);
        lcd_plot(px, py ) ;
      }
      y += dyabs ;
      if (y>=dxabs)
			{
        y -= dxabs ;
        py += sdy ;
      }
      px += sdx ;
    }
  }
  else
	{
    /* the line is more vertical than horizontal */
    for (int i=0; i<dyabs; i++)
		{
      if ((1<<(py%8)) & pat)
			{
//        lcdDrawPoint(px, py, att);
        lcd_plot(px, py ) ;
      }
      x += dxabs;
      if (x >= dyabs)
			{
        x -= dyabs ;
        px += sdx ;
      }
      py += sdy ;
    }
  }
}

void lcd_char_inverse( uint8_t x, uint8_t y, uint8_t w, uint8_t blink )
{
	if ( blink && BLINK_ON_PHASE )
	{
		return ;
	}
	uint8_t end = x + w ;
	uint8_t *p = dispBufAddress( x, y ) ;

	y &= 7 ;
	if ( y )
	{ // off grid
		while ( x < end )
		{
     	uint16_t b = 0xFF ;
			b <<= y ;
			if(p<DISPLAY_END) *p ^= b ;
	    if(&p[DISPLAY_W] < DISPLAY_END) p[DISPLAY_W] ^= b >> 8 ;
			p += 1 ;
			x += 1 ;
		}
	}
	else
	{
		while ( x < end )
		{
			if(p<DISPLAY_END) *p++ ^= 0xFF ;
			x += 1 ;
		}
	}
}

void lcd_outdez( uint8_t x, uint8_t y, int16_t val )
{
  lcd_outdezAtt(x,y,val,0);
}

void lcd_outdezAtt( uint8_t x, uint8_t y, int16_t val, uint16_t mode )
{
  lcd_outdezNAtt( x,y,val,mode,5);
}

void lcd_2_digits( uint8_t x, uint8_t y, uint8_t value, uint16_t attr )
{
	lcd_outdezNAtt( x, y, value, attr + LEADING0, 2 ) ;
}

#define PREC(n) ((n&0x20) ? ((n&0x10) ? 2 : 1) : 0)

uint8_t lcd_outdezNAtt( uint8_t x, uint8_t y, int32_t val, uint16_t mode, int8_t len )
{
  uint8_t fw = FWNUM;
  uint8_t prec = PREC(mode);
  int32_t tmp = abs(val);
  uint8_t xn = 0;
  uint8_t ln = 2;
  char c;
  uint8_t xinc ;
	uint8_t fullwidth = 0 ;
	int32_t i ;
	if ( len < 0 )
	{
		fullwidth = 1 ;
		len = -len ;		
	}
	if (mode & BOLD)
	{
		fw = FW ;
	}
  if (mode & DBLSIZE)
  {
		if ( (mode & CONDENSED) )
		{
    	fw = 8 ;
    	xinc = 8 ;
    	Lcd_lastPos = 8 ;
		}
		else
		{
    	fw += FWNUM ;
    	xinc = 2*FWNUM;
    	Lcd_lastPos = 2*FW;
		}
  }
  else
  {
    xinc = FWNUM ;
    Lcd_lastPos = FW;
  }

  if (mode & LEFT)
	{
    if(val<0)
    {
      x += fw;
    }
    if (tmp >= 1000)
		{
	    if (tmp >= 100000)
  	    x += fw;
	    if (tmp >= 10000)
  	    x += fw;
			x += 3 * fw ;	
		}
		else
		{
	    if (tmp >= 100)
  	    x += fw;
    	if (tmp >= 10)
      	x += fw;
		}
    if ( prec )
    {
      if ( prec == 2 )
      {
        if ( tmp < 100 )
        {
          x += fw;
        }
      }
      if ( tmp < 10 )
      {
        x+= fw;
      }
    }
  }
  else
  {
    x -= xinc;
  }
  Lcd_lastPos += x ;

  for ( i=1; i<=len; i++)
	{
    c = (tmp % 10) + '0';
		lcd_putcAtt(x, y, c, mode);
    if (prec==i)
		{
      if (mode & DBLSIZE)
			{
        xn = x ;
				if ( (mode & CONDENSED) )
				{
					x -= 1 ;
        	xn = x-1 ;
				}
				else
				{
	        if(c=='2' || c=='3' || c=='1') ln++;
	        uint8_t tn = (tmp/10) % 10;
	        if(tn==2 || tn==4)
					{
	          if (c=='4')
						{
	            xn++;
	          }
	          else
						{
	            xn--; ln++;
	          }
	        }
				}
      }
      else
			{
        x -= 2;
				pushPlotType( PLOT_BLACK ) ;
        if (mode & INVERS)
				{
          lcd_vline(x+1, y, 8);
				}
				popPlotType() ;
        lcd_plot(x+1, y+6);
      }
      if (tmp >= 10)
        prec = 0;
    }
    tmp /= 10;
    if (!tmp)
    {
      if (prec)
      {
        if ( prec == 2 )
        {
          if ( i > 1 )
          {
            prec = 0 ;
          }
        }
        else
        {
          prec = 0 ;
        }
      }
      else if (mode & LEADING0)
			{
				if ( fullwidth == 0 )
				{
        	mode -= LEADING0;
				}
			}
      else
        break;
    }
    x-=fw;
  }
  if (xn)
	{
    lcd_hline(xn, y+2*FH-4, ln);
    lcd_hline(xn, y+2*FH-3, ln);
  }
  if(val<0) lcd_putcAtt(x-fw,y,'-',mode);
	return 0 ;		// Stops compiler creating two sets of POPS, saves flash
}

void lcd_puts_P( uint8_t x, uint8_t y, const char *s )
{
  lcd_putsAtt( x, y, s, 0);
}

void lcd_puts_Pleft( uint8_t y, const char *s )
{
  lcd_putsAtt( 0, y, s, 0);
}

void lcd_putsnAtt(uint8_t x, uint8_t y, const char * s, uint8_t len, uint8_t mode)
{
	char c ;
  while( len != 0 )
	{
    c = *s++ ;
    x = lcd_putcAtt( x, y, c, mode ) ;
    len -= 1 ;
  }
}

void lcd_putsn_P(uint8_t x,uint8_t y,const char * s,uint8_t len)
{
  lcd_putsnAtt( x,y,s,len,0);
}

void lcd_putsAttIdx(uint8_t x,uint8_t y,const char * s,uint8_t idx,uint8_t att)
{
	uint8_t length ;
	length = *s++ ;

  lcd_putsnAtt(x,y,s+length*idx,length,att) ;
}


void lcd_2_digits( uint16_t x, uint16_t y, uint8_t value, uint16_t attr )
{
	lcd_outdezNAtt( x, y, value, attr + LEADING0, 2 ) ;
}

void putsTime(uint16_t x,uint16_t y,int16_t tme,uint8_t att,uint8_t att2 )
{
	div_t qr ;

	uint8_t z = FWNUM*6-2 ;
	if ( att&DBLSIZE )
	{
		if ( att&CONDENSED )
		{
			x += 3 ;
			z = FWNUM*5-2 ;
		}
	}
	if ( tme<0 )
	{
		lcd_putcAtt( x - ((att&DBLSIZE) ? z : FWNUM*3),    y, '-',att);
		tme = -tme;
	}

	lcd_putcAtt( x, y, ':',att&att2);
	qr = div( tme, 60 ) ;
	if ( att & DBLSIZE )
	{
		if ( att & CONDENSED )
		{
			x += 2 ;
		}
	}
	lcd_2_digits( x, y, (uint16_t)qr.quot, att ) ;
	if ( att&DBLSIZE )
	{
		if ( att&CONDENSED )
		{
			x += FWNUM*5-4 ;
		}
		else
		{
			x += FWNUM*6-4 ;
		}
	}
	else
	{
		x += FW*3-4 ;
	}
	lcd_2_digits( x, y, (uint16_t)qr.rem, att2 ) ;
}

void putsVolts(uint8_t x,uint8_t y, uint8_t volts, uint8_t att)
{
	uint8_t option = att & NO_UNIT ;
	att &= ~NO_UNIT ;
	lcd_outdezAtt(x, y, volts, att|PREC1) ;
	if(!(option&NO_UNIT)) lcd_putcAtt(Lcd_lastPos, y, 'V', 0 ) ;
}


void initBacklight()
{
	DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_LEDC_CLK_EN) ;
	DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_LEDC_RST);
	
	LEDC.timer_group[0].timer[0].conf.rst = 0 ;
	LEDC.conf.apb_clk_sel = 1;//LS use apb clock
	LEDC.timer_group[0].timer[0].conf.clock_divider = 8000 << 8 ;
	LEDC.timer_group[0].timer[0].conf.duty_resolution = 7 ;
	LEDC.timer_group[0].timer[0].conf.tick_sel = 1 ;//apb clock
	
	LEDC.channel_group[0].channel[0].conf0.timer_sel = 0 ;
  LEDC.channel_group[0].channel[0].conf0.sig_out_en = 1 ;//This is the output enable control bit for channel
  LEDC.channel_group[0].channel[0].conf1.duty_start = 1 ;//When duty_num duty_cycle and duty_scale has been configured. these register won't take effect until set duty_start. this bit is automatically cleared by hardware.

  LEDC.channel_group[0].channel[0].duty.val = 64 << 4 ;	// Half brightness

	LEDC.conf.val = 1 ;	// Replaces above 2 lines?

	//  Output is function 71
	GPIO.enable_w1ts = ( (uint32_t)1 << 19 ) ;
	GPIO.func_out_sel_cfg[19].val = 71 ; 
}

void setBacklightBrightness( uint32_t percent )
{
	if ( percent > 100 )
	{
		percent = 100 ;		
	}
	percent *= 128 ;
	percent /= 100 ;
  LEDC.channel_group[0].channel[0].duty.val = percent << 4 ;
  LEDC.channel_group[0].channel[0].conf1.duty_start = 1 ;//When duty_num duty_cycle and duty_scale has been configured. these register won't take effect until set duty_start. this bit is automatically cleared by hardware.
}





