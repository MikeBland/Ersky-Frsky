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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "erskyTx.h"
#include "logicIo.h"
#include "myeeprom.h"
#include "menus.h"
#include "lcd.h"
#include "en.h"
#include "file.h"
#include "telemetry.h"
#include "audio.h"

#include "lfs.h"

extern lfs_t Lfs ;

#define KEY_UP		TRM_LV_UP
#define KEY_DOWN	TRM_LV_DWN
#define KEY_LEFT	TRM_LH_DWN
#define KEY_RIGHT	TRM_LH_UP


uint8_t M_lastVerticalPosition ;
int8_t scrollLR ;
int8_t scrollUD ;
char LastItem[8] ;
uint8_t StepSize = 20 ;
uint8_t AlphaEdited ;
int16_t ex_chans[NUM_SKYCHNOUT] ;          // Outputs + intermidiates

uint8_t TextIndex ;
uint8_t TextOption ;
uint8_t TextType ;
uint8_t TextResult ;
uint16_t LastFileMoveTime ;
uint8_t Unit ;

struct t_alpha Alpha ;

uint32_t xfillNames( uint32_t index, struct fileControl *fc ) ;

const int8_t TelemIndex[] = { FR_A1_COPY, FR_A2_COPY,	FR_RXRSI_COPY, FR_TXRSI_COPY,	TIMER1, TIMER2,	// 0-5
															FR_ALT_BARO, TELEM_GPS_ALT, FR_GPS_SPEED, FR_TEMP1, FR_TEMP2, FR_RPM,	// 6-11
														  FR_FUEL, FR_A1_MAH, FR_A2_MAH, FR_CELL_MIN,                           // 12-15
															BATTERY, FR_CURRENT, FR_AMP_MAH, FR_CELLS_TOT, FR_VOLTS,              // 16-20
															FR_ACCX, FR_ACCY,	FR_ACCZ, FR_VSPD, V_GVAR1, V_GVAR2,	V_GVAR3,        // 21-27
															V_GVAR4, V_GVAR5, V_GVAR6, V_GVAR7, FR_WATT, FR_RXV, FR_COURSE,       // 28-34
															FR_A3, FR_A4, V_SC1, V_SC2, V_SC3, V_SC4, V_SC5, V_SC6, V_SC7, V_SC8, V_RTC, TMOK,	// 35-46
															FR_AIRSPEED, FR_CELL1,FR_CELL2,FR_CELL3,FR_CELL4,FR_CELL5,FR_CELL6,FR_RBOX_B1_V,FR_RBOX_B1_A,	// 47-55
															FR_RBOX_B2_V,FR_RBOX_B2_A, FR_RBOX_B1_CAP, FR_RBOX_B2_CAP,FR_RBOX_SERVO,FR_RBOX_STATE,	// 56-61
															FR_CELL7, FR_CELL8, FR_CELL9, FR_CELL10, FR_CELL11, FR_CELL12,                          // 62-67
															FR_CUST1,FR_CUST2,FR_CUST3,FR_CUST4,FR_CUST5,FR_CUST6,FMODE,RUNTIME,MODELTIME,					// 68-76
															FR_CELLS_TOTAL1, FR_CELLS_TOTAL2, FR_SBEC_VOLT, FR_SBEC_CURRENT, FR_VFR,			// 77-81
															FR_CUST7,FR_CUST8,FR_CUST9,FR_CUST10 } ;	// 82-85


const uint8_t TelemValid[] = { 
1, 1, 1,
#ifdef ACCESS
1,	// SWR
#else
0,	// SWR
#endif
0, 0, 2, 2, 2, 2, 2, 2, 2, 0, 0, 2, 2, 2, 0, 2, 
2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 2, 1, 2, 2, 2, 3, 3, 3, 
3, 3, 3, 3, 3, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 
2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 2, 2, 2, 2, 2, 2 ,2, 2, 2 } ;


const uint16_t UnitsVoice[] = {SV_FEET,SV_VOLTS,SV_DEGREES,SV_DEGREES,SV_MILAMP_H,SV_AMPS,SV_METRES,SV_WATTS,SV_PERCENT,SV_KMSPH,SV_DB } ;
const uint8_t UnitsText[] = { 'F','V','C','F','m','A','m','W','%', 'k', 'd' } ;

const char s_charTab[]=" ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-.";
#define NUMCHARS (sizeof(s_charTab)-1)

extern const uint8_t SwitchFunctionMap[] ;

int32_t getMovedSwitch() ;

uint8_t yesNoMenuExit( uint8_t event, const char * s )
{
	uint8_t reply = YN_NONE ;
	lcd_puts_Pleft(1*FH, s ) ;	
  lcd_puts_Pleft( 5*FH,PSTR(STR_YES_NO)) ;
  lcd_puts_Pleft( 6*FH,PSTR(STR_MENU_EXIT)) ;

  switch(event)
	{
    case EVT_ENTRY:
      audioDefevent(AU_WARNING1);
    break ;

//    case EVT_KEY_FIRST(KEY_MENU):
		case EVT_KEY_BREAK(KEY_MENU):
			reply = YN_YES ;
    break ;

    case EVT_KEY_FIRST(KEY_EXIT):
			reply = YN_NO ;
    break;
  }
	if ( reply != YN_NONE )
	{
		killEvents(event) ;
    popMenu(false) ;
	}
	return reply ;
}

void dispGvar( uint8_t x, uint8_t y, uint8_t gvar, uint8_t attr )
{
	lcd_putsAtt( x, y, PSTR(STR_GV), attr ) ;
	lcd_putcAtt( x+2*FW, y, gvar+'0', attr ) ;
}

uint8_t locateMappedItem( uint8_t value, uint8_t *options, uint32_t count )
{
	uint32_t c ;
	for ( c = 0 ; c < count ; c += 1 )
	{
		if ( options[c] == value )
		{
			value = c ;
			break ;
		}
	}
	if ( c >= count )
	{
		return 0 ;
	}
	return value ;
	
}

// Check a mapped value
uint8_t checkOutOfOrder( uint8_t value, uint8_t *options, uint32_t count )
{
	uint8_t index = locateMappedItem( value, options, count ) ;
	CHECK_INCDEC_H_MODELVAR( index, 0, count-1 ) ;
	index = options[index] ;
	return index ;
}

void setLastIdx( char *s, uint8_t idx )
{
	uint8_t length ;
	length = (uint8_t) *s++ ;

	ncpystr( (uint8_t *)LastItem, (uint8_t *)s+length*idx, length ) ;
}

void setLastTelemIdx( uint8_t idx )
{
	uint8_t *s ;
	uint32_t x ;
	if ( ( idx >= 69 ) && ( idx <= 74 ) ) // A Custom sensor
	{
		x = idx - 69 ;
		x *= 4 ;
		s = &g_model.customTelemetryNames[x] ;
		if ( *s && (*s != ' ' ) )
		{
			ncpystr( (uint8_t *)LastItem, s, 4 ) ;
			return ;
		}
	}
	if ( ( idx >= 38 ) && ( idx <= 45 ) )	// A Scaler
	{
		x = idx - 38 ;
		s = g_model.Scalers[x].name ;
		if ( *s && (*s != ' ' ) )
		{
			ncpystr( (uint8_t *)LastItem, s, 4 ) ;
			return ;
		}
	}
	setLastIdx( (char *) PSTR(STR_TELEM_ITEMS), idx ) ;
}

int16_t m_to_ft( int16_t metres )
{
	int16_t result ;

  // m to ft *105/32
	result = metres * 3 ;
	metres >>= 2 ;
	result += metres ;
	metres >>= 2 ;
  return result + (metres >> 1 );
}

int16_t c_to_f( int16_t degrees )
{
  degrees += 18 ;
  degrees *= 115 ;
  degrees >>= 6 ;
  return degrees ;
}

uint32_t putTxSwr( uint8_t x, uint8_t y, uint8_t attr )
{
	uint32_t index = 1 ;
//	if ( FrskyTelemetryType == FRSKY_TEL_SPORT )
//	{
//		index = 1 ;
//	}
//	if ( ( index == 0 ) && (attr & TSSI_TEXT) )
//	{
//		return 0 ;
//	}
	lcd_putsAttIdx( x, y, PSTR(STR_TXEQ), index, attr ) ;
	setLastIdx( (char *) PSTR(STR_TXEQ), index ) ;
	return 1 ;
}


void putsAttIdxTelemItems( uint8_t x, uint8_t y, uint8_t index, uint8_t attr )
{
	if ( index == 4 )
	{
		if ( putTxSwr( x, y, attr ) )
		{
			return ;
		}
	}
	attr &= ~TSSI_TEXT ;
	setLastTelemIdx( index ) ;
	lcd_putsAtt(x,y,LastItem,attr) ;
}

void putsChnRaw(uint8_t x,uint8_t y,uint8_t idx,uint8_t att )
{
	uint8_t chanLimit = NUM_SKYXCHNRAW ;
	uint8_t mix = att & MIX_SOURCE ;
	LastItem[0] = '\0' ;
	if ( mix )
	{
		chanLimit += MAX_GVARS + 1 + 1 + NUM_SKYCSW + NUM_SCALERS + 4 + 8 ; // 4 for trims
		att &= ~MIX_SOURCE ;		
	}
  if(idx==0)
	{
		ncpystr( (uint8_t *)LastItem, (uint8_t *) XPSTR("----"), 4 ) ;
	}
  else if(idx<=4)
	{
		const char *ptr = "" ;
		if ( g_model.useCustomStickNames )
		{
			ptr = ( char *)g_eeGeneral.customStickNames+4*(idx-1) ;
		}
		if ( *ptr && (*ptr != ' ' ) )
		{
			ncpystr( (uint8_t *)LastItem, (uint8_t *)ptr, 4 ) ;
		}
		else
		{
			setLastIdx( (char *) PSTR(STR_STICK_NAMES), idx-1 ) ;
		}
	}
	else if(idx<=chanLimit)
	{
		setLastIdx( (char *) PSTR(STR_CHANS_GV), idx-5 ) ;
	}
	else
	{
		if ( mix )
		{
//			idx += TEL_ITEM_SC1-(chanLimit-NUM_SKYXCHNRAW) ;
//			if ( idx - NUM_SKYXCHNRAW > TEL_ITEM_SC1 + NUM_SCALERS )
//			{
//				uint8_t *ptr ;
//				idx -= TEL_ITEM_SC1 + NUM_SCALERS - 8 + NUM_SKYXCHNRAW ;
//				ptr = cpystr( (uint8_t *)LastItem, (uint8_t *)"PPM1" ) ;
//				if ( idx == 9 )
//				{
//					*(ptr-1) = '9' ;
//				}
//				else
//				{
//					*ptr = '0' + idx - 10 ;
//				}
//				*(ptr+1) = '\0' ;
//				lcd_putsAttColour(x,y,LastItem,att, colour, bgColour ) ;
			return ;
//			}
		}
		setLastTelemIdx( idx-NUM_SKYXCHNRAW ) ;
	}
	lcd_putsAtt( x, y, LastItem, att ) ;
}

void putsChnOpRaw( uint16_t x, uint16_t y, uint8_t source, uint8_t switchSource, uint8_t output, uint8_t attr )
{
	putsChnRaw( x, y, source, attr| MIX_SOURCE ) ;
	if ( output )
	{
		if ( ( ( source >= CHOUT_BASE + 1 ) && ( source < CHOUT_BASE + NUM_SKYCHNOUT + 1 ) ) )
//					  ||
//					 ( ( source >= EXTRA_CHANS_BASE + 1 ) && ( source < EXTRA_CHANS_BASE + EXTRA_SKYCHANNELS + 1 ) ) )
		{
			lcd_putsAtt( x, y, XPSTR("OP"), attr ) ;
			LastItem[0] = 'O' ;
			LastItem[1] = 'P' ;
		}
	}
}

uint8_t char2idx(char c)
{
	uint8_t ret ;
    for(ret=0;;ret++)
    {
        char cc= s_charTab[ret] ;
        if(cc==0) return 0;
        if(cc==c) return ret;
    }
}
char idx2char(uint8_t idx)
{
    if(idx < NUMCHARS) return s_charTab[idx] ;
    return ' ';
}

void validateText( uint8_t *text, uint32_t length )
{
	for( uint8_t i=0 ; i<length ; i += 1 ) // makes sure text is valid
  {
		if ( text[i] < ' ' )
		{
			text[i] = ' ' ;
		}
  }
}

void validateName( uint8_t *text, uint32_t length )
{
	for(uint8_t i=0; i<length ; i += 1) // makes sure name is valid
	{
		uint8_t idx = char2idx( text[i] ) ;
		text[i] = idx2char( idx ) ;
	}
}

const char AlphaSource[] =   "0123456789qwertyuiop[asdfghjkl\200zxcvbnm_-. " ;
const char AlphaSource2[] = "!\":?%^&*()QWERTYUIOP]ASDFGHJKL\201ZXCVBNM<>, " ;

static uint8_t Caps = 0 ;

void displayKeys( uint16_t x, uint16_t y, char *text, uint8_t count )
{
	while ( count )
	{
		char c = *text++ ;
		lcd_putc( x+1, y, c ) ;
		lcd_rect( x-2, y-1, 11, 10 ) ;
		count -= 1 ;
		x += 10 ;
	}
}

void putSwitchName(uint8_t x, uint8_t y, int8_t z, uint8_t att)
{
  lcd_putsAttIdx(x, y, PSTR(SWITCHES_STR), z, att);
}

void putHwSwitchName(uint16_t x, uint16_t y, uint8_t z, uint8_t att)
{
  lcd_putsAttIdx(x, y, PSTR(HW_SWITCHES_STR), z, att);
}

void putsMomentDrSwitches(uint16_t x,uint16_t y,int16_t idx1,uint8_t att)
{
  if(abs(idx1)>(HSW_MAX))	 //momentary on-off
	{
  	lcd_putcAtt(x+3*FW,  y,'m',att);
	}			 
  putsDrSwitches( x-1*FW, y, idx1, att ) ;
}

void putsDrSwitches( uint16_t x, uint16_t y, int16_t idx1, uint8_t att)
{
	if ( idx1 == 0 )
	{
    lcd_putsAtt(x+FW, y, XPSTR("---"), att) ;
		return ;
	}
	else if ( idx1 == MAX_SKYDRSWITCH )
	{
    lcd_putsAtt(x+FW, y, PSTR(STR_ON), att) ;
		return ;
	}
	else if ( idx1 == -MAX_SKYDRSWITCH )
	{
    lcd_putsAtt(x+FW,y,PSTR(STR_OFF),att) ;
		return ;
	}
	else if ( idx1 == MAX_SKYDRSWITCH + 1 )
	{
    lcd_putsAtt(x+FW, y, XPSTR("Fmd"), att) ;
		return ;
	}

	if ( idx1 < 0 )
	{
  	lcd_putcAtt(x, y, '!', att) ;
	}
	int8_t z ;
	z = idx1 ;
	if ( z < 0 )
	{
		z = -idx1 ;			
	}
	if ( ( z <= HSW_Ttrmup ) && ( z >= HSW_Etrmdn ) )
	{
		z -= HSW_Etrmdn ;
	  lcd_putsAttIdx(x+FW,y,XPSTR("\003EtdEtuAtdAtuRtdRtuTtdTtu"),z,att) ;
		return ;
	}
	if ( ( z <= HSW_FM6 ) && ( z >= HSW_FM0 ) )
	{
		z -= HSW_FM0 ;
  	lcd_putsAttIdx( x+FW, y, XPSTR("\003FM0FM1FM2FM3FM4FM5FM6"), z, att ) ;
		return ;
	}
	z -= 1 ;

	if ( z > MAX_SKYDRSWITCH )
	{
		z -= HSW_OFFSET - 1 ;
	}
  putSwitchName(x+FW, y, z, att) ;
}


uint32_t checkForMenuEncoderBreak( uint8_t event )
{
	return ( event==EVT_KEY_BREAK(KEY_MENU) ) ;
}

uint8_t telemItemValid( uint8_t index )
{
	uint8_t x ;

	x = TelemValid[index] ;
	if ( x == 3 )
	{
		// A scaler
		uint8_t i = g_model.Scalers[index-TEL_ITEM_SC1].source - 1 ;
  	if( i<CHOUT_BASE+NUM_SKYCHNOUT )
		{
			x = 0 ;
		}
		else
		{
			i -= CHOUT_BASE+NUM_SKYCHNOUT ;
			x = TelemValid[i] ;
			if ( x == 3 )
			{
				x = 0 ;
			}
			else
			{
				index = i ;
			}
		}
	}
	if ( x == 0 )
	{
		return 1 ;
	}
	if ( x == 1 )
	{
		if ( FrskyStreaming )
		{
			int8_t i = TelemIndex[index] ;
			if ( i >= 0 )
			{
				if (TelemetryDataValid[i] )
				{
					return 1 ;
				}
				return 0 ;
			}
			return 1 ;
		}
	}
	else // if ( frskyUsrStreaming )
	{
		int8_t i = TelemIndex[index] ;
		if ( i >= 0 )
		{
			if (TelemetryDataValid[i] )
			{
				return 1 ;
			}
			return 0 ;
		}
		return 1 ;
	}
	return 0 ;	
}

uint16_t getUnitsVoice( uint16_t unit )
{
	return UnitsVoice[unit] ;
}


void voice_telem_item( int8_t index )
{
	int16_t value ;
	uint8_t spoken = 0 ;
	uint16_t unit = 0 ;
	uint8_t num_decimals = 0 ;

	value = get_telemetry_value( index ) ;
	if (telemItemValid( index ) == 0 )
	{
//#ifdef TELEMETRY_LOST
//		if ( frskyStreaming )
//		{
//#endif
		putSystemVoice( SV_NO_TELEM, V_NOTELEM ) ;
//#ifdef TELEMETRY_LOST
//		}
//#endif
		spoken = 1 ;
	}
	index = TelemIndex[index] ;

  switch (index)
	{
		case V_SC1 :
		case V_SC2 :
		case V_SC3 :
		case V_SC4 :
		case V_SC5 :
		case V_SC6 :
		case V_SC7 :
		case V_SC8 :
			value = calc_scaler( index-V_SC1, &unit, &num_decimals ) ;
			unit = getUnitsVoice( unit ) ;
		break ;
		
		case BATTERY:
		case FR_VOLTS :
		case FR_CELLS_TOT :
		case FR_CELLS_TOTAL1 :
		case FR_CELLS_TOTAL2 :
			unit = SV_VOLTS ;			
			num_decimals = 1 ;
		break ;

		case FR_CELL_MIN:
			if ( value > 445 )
			{
				value = 0 ;
			}
		case FR_CELL1:
		case FR_CELL2:
		case FR_CELL3:
		case FR_CELL4:
		case FR_CELL5:
		case FR_CELL6:
		case FR_CELL7:
		case FR_CELL8:
		case FR_CELL9:
		case FR_CELL10:
		case FR_CELL11:
		case FR_CELL12:
		case FR_RBOX_B1_V :
		case FR_RBOX_B2_V :
		case FR_SBEC_VOLT :
			unit = SV_VOLTS ;			
			num_decimals = 2 ;
		break ;
			
		case TIMER1 :
		case TIMER2 :
		{	
			div_t qr ;
			qr = div( value, 60 ) ;
			voiceMinutes( qr.quot ) ;
			value = qr.rem ;
			unit = SV_SECONDS ;			
		}
		break ;

    case V_RTC :
			voice_numeric( Time.hour, 0, 0 ) ;
			voice_numeric( Time.minute, 0, 0 ) ;
			spoken = 1 ;
		break ;
		 
		case V_GVAR1 :
		case V_GVAR2 :
		case V_GVAR3 :
		case V_GVAR4 :
		case V_GVAR5 :
		case V_GVAR6 :
		case V_GVAR7 :
			value = g_model.gvars[index-V_GVAR1].gvar ;
		break ;

		case FR_A3:
		case FR_A4:
		{	
			uint8_t channel = index-FR_A3 ;
			
			uint8_t ltype = g_model.frsky.channels[channel].units3_4 ;
			unit = SV_VOLTS ;			
			num_decimals = 2 ;
			if (ltype == 3)
			{
				unit = SV_AMPS ;
			}
			else if (ltype == 1)
			{
				unit = 0 ;
				num_decimals = 0 ;
			}
		}
		break ;
		 
		case FR_A1_COPY:
    case FR_A2_COPY:
		{	
			uint8_t channel = index-FR_A1_COPY ;
			
			uint8_t ltype = g_model.frsky.channels[channel].units3_4 ;
			value = A1A2toScaledValue( channel, &num_decimals ) ;
			unit = SV_VOLTS ;			
			if (ltype == 3)
			{
				unit = SV_AMPS ;
			}
			else if (ltype == 1)
			{
				unit = 0 ;
				num_decimals = 0 ;
			}
		}
		break ;

    case FR_RXV :
			value = convertRxv( value ) ;			
			unit = SV_VOLTS ;			
			num_decimals = 1 ;
		break ;

		case FR_ALT_BARO:
		case TELEM_GPS_ALT :
      unit = SV_METRES ;
			if (g_model.FrSkyUsrProto == 1)  // WS How High
			{
      	if ( g_model.FrSkyImperial )
        	unit = SV_FEET ;
			}
      else
			{
				if ( g_model.FrSkyImperial )
      	{
	        // m to ft *105/32
  	      value = m_to_ft( value ) ;
    	    unit = SV_FEET ;
      	}
			}
			if ( value < 1000 )
			{
				num_decimals = 1 ;
			}
			else
			{
				value /= 10 ;
			}
		break ;

#ifdef MAVLINK 
// Extra data for Mavlink via FrSky		 
		case FR_WP_DIST:
  		if ( g_model.FrSkyImperial )
  		{
				value = m_to_ft(value) ;
			}
		break ;
// Extra data for Mavlink via FrSky
#endif

		case FR_RBOX_B1_A :
		case FR_RBOX_B2_A :
		case FR_SBEC_CURRENT :
			num_decimals = 2 ;
      unit = SV_AMPS ;
		break ;
		
		case FR_RBOX_B1_CAP :
		case FR_RBOX_B2_CAP :
      unit = SV_MILAMP_H ;
		break ;
		
		case FR_CURRENT :
			num_decimals = 1 ;
      unit = SV_AMPS ;
		break ;

		case FR_AIRSPEED :
			num_decimals = 1 ;
      unit = SV_METRES ;
			if ( g_model.FrSkyImperial )
			{
       	// m to ft *105/32
       	value = m_to_ft( value ) ;
	      unit = 'f' ;
			}
		break ;
			 
		case FR_TEMP1:
		case FR_TEMP2:
			unit = SV_DEGREES ;			
  		if ( g_model.FrSkyImperial )
  		{
				value = c_to_f(value) ;
			}
		break ;

		case FR_WATT :
			unit = SV_WATTS ;
		break ;

		case FR_FUEL :
			unit = SV_PERCENT ;
		break ;

		case FR_VSPD :
			num_decimals = 1 ;
			value /= 10 ;
		break ;

	}

	if ( spoken == 0 )
	{
		voice_numeric( value, num_decimals, unit ) ;
	}
}

uint8_t putsTelemValue(uint16_t x, uint16_t y, int16_t val, uint8_t channel, uint8_t att )
{
	int32_t value ;
    //  uint8_t ratio ;
	uint8_t dplaces ;
//    uint8_t times2 ;
  uint8_t unit = ' ' ;
  uint8_t option = att & NO_UNIT ;
	att &= ~NO_UNIT ;
//		uint8_t ltype = g_model.frsky.channels[channel].type ;

//    if ( scale )
//		{
//			value = valuetoScaledValue( val, channel, &dplaces ) ;
//		}

	value = scale_telem_value( val, channel, &dplaces ) ;
	if ( dplaces == 1 )
	{
		att |= PREC1 ;
	}
	else if ( dplaces == 2 )
	{
		att |= PREC2 ;
	}
//        ratio = g_model.frsky.channels[channel].ratio ;
//        if ( times2 )
//        {
//            ratio <<= 1 ;
//        }
//        value *= ratio ;
//		  	if (g_model.frsky.channels[channel].type == 3/*A*/)
//        {
//            value /= 100 ;
//            att |= PREC1 ;
//        }
//        else if ( ratio < 100 )
//        {
//            value *= 2 ;
//            value /= 51 ;  // Same as *10 /255 but without overflow
//            att |= PREC2 ;
//        }
//        else
//        {
//            value /= 255 ;
//        }
  if ( Unit == 'v' ) //ltype == 0/*v*/) || (ltype == 2/*v*/) )
//    if ( ( ltype == 0/*v*/) || ( ltype == 2/*v*/) )
  {
    lcd_outdezNAtt(x, y, value, att, 5) ;
//			unit = 'v' ;
    if(!(option&NO_UNIT)) lcd_putcAtt(Lcd_lastPos, y, unit, att);
  }
  else
  {
    lcd_outdezAtt(x, y, value, att);
    if ( Unit == 'A')
//	    if ( ltype == 3/*A*/)
		{
//					unit = 'A' ;
     	if(!(option&NO_UNIT)) lcd_putcAtt(Lcd_lastPos, y, unit, att);
		}
  }
	return unit ;
}


int16_t scale_telem_value( int16_t val, uint8_t channel, uint8_t *dplaces )
{
	int32_t value ;
  uint8_t unit = 'v' ;
	uint8_t places = 1 ;
	
	value = val ;
	uint8_t ltype = g_model.frsky.channels[channel].units3_4 ;
	uint16_t ratio = g_model.frsky.channels[channel].lratio ;
	if (ltype == 2/*V*/)	// times 2
	{
    ratio <<= 1 ;
	}
  value *= ratio ;
	if (ltype == 3/*A*/)
  {
		unit = 'A' ;
    value /= 100 ;
  }
  else if ( ratio < 100 )
  {
    value *= 2 ;
    value /= 51 ;  // Same as *10 /255 but without overflow
		places = 2 ;
  }
  else
  {
    value /= 255 ;
  }
	if ( ltype == 1 )
	{
		unit = ' ' ;
  	places = 0 ;
	}
	if ( dplaces )
	{
  	*dplaces = places ;
	}
	Unit = unit ;
	return value ;
}


uint8_t putsTelemetryChannel( uint16_t x, uint16_t y, int8_t channel, int16_t val, uint8_t att, uint8_t style )
{
	uint16_t unit = ' ' ;
	uint16_t xbase = x ;
	uint8_t fieldW = FW ;
	uint8_t displayed = 0 ;
	uint8_t valid = telemItemValid( channel ) | (style & TELEM_CONSTANT ) ;
	uint8_t mappedChannel = channel ;
	if ( style & TELEM_LABEL )
	{
		uint8_t displayed = 0 ;
		int8_t index = TelemIndex[mappedChannel] ;
		uint8_t special = 0 ;

		if ( (index >= V_SC1) && (index < V_SC1 + NUM_SCALERS) )
		{
			special = 1 ;
		}
		if ( (index >= FR_CUST1) && (index <= FR_CUST6) )
		{
			special = 2 ;
		}
		if ( (index >= FR_CUST7) && (index <= FR_CUST10) )
		{
			special = 4 ;
		}

		if ( special )
		{
			uint8_t *p ;
			uint8_t text[6] ;
			if ( special & 1 )
			{
				index -= V_SC1 ;
				p = &g_model.Scalers[index].name[0] ;
			}
			else
			{
				if ( special & 2 )
				{				
					index -= FR_CUST1 ;
					p = &g_model.customTelemetryNames[index*4] ;
				}
				else
				{
					index -= FR_CUST7 ;
					p = &g_model.customTelemetryNames[index*4+24] ;
				}
			}
			text[0] = *p++ ;
			if ( text[0] && (text[0] != ' ') )
			{
				text[1] = *p++ ;
				text[2] = *p++ ;
				text[3] = *p ;
				text[4] = 0 ;
				lcd_putsAtt( x, y, (const char *)text, 0 ) ;
				displayed = 1 ;
			}
		}
		if ( displayed == 0 )
		{
  		putsAttIdxTelemItems( x, y, channel+1, 0 ) ;
		}
		x += 4*FW ;
		if ( att & DBLSIZE )
		{
			x += 4 ;
			y -= FH ;												
			fieldW += FW ;
		}
	}

//	if ( style & TELEM_HIRES )
//	{
//		y += 2*FH ;
//	}

	if (style & TELEM_VALUE_RIGHT)
	{
		att &= ~LEFT ;
	}
	channel = TelemIndex[mappedChannel] ;
  switch (channel)
	{
		case V_SC1 :
		case V_SC2 :
		case V_SC3 :
		case V_SC4 :
		case V_SC5 :
		case V_SC6 :
		case V_SC7 :
		case V_SC8 :
		{
			int16_t cvalue ;
			uint8_t precision ;
			cvalue = calc_scaler( channel-V_SC1, &unit, &precision ) ;
			if ( precision == 1 )
			{
				att |= PREC1 ;
			}
			else if ( precision == 2 )
			{
				att |= PREC2 ;
			}
			// Sort units here
			unit = UnitsText[unit] ;
			if ( (style & TELEM_CONSTANT) == 0)
			{
				val = cvalue ;
			}
		}	
		break ;

    case MODELTIME :
    case RUNTIME :
//		break ;
    case V_RTC :
			style |= TELEM_NOTIME_UNIT ;
		case TIMER1 :
    case TIMER2 :
			style &= ~( TELEM_UNIT | TELEM_UNIT_LEFT ) ;
	
			if ( (att & DBLSIZE) == 0 )
			{
				x -= 4 ;
			}
			if ( style & TELEM_LABEL )
			{
				x += FW+4 ;
			}
//			att &= DBLSIZE ;
	  	if (att & DBLSIZE)
  		{
				if ( (att & CONDENSED) )
				{
					x += 2 ;
				}
			}		
      putsTime(x-FW-3+5, y, val, att, att) ;
			displayed = 1 ;
			if ( !(style & TELEM_NOTIME_UNIT) )
			{
    		unit = channel + 2 + '1';
			}
			xbase -= FW ;
    break ;

		case FR_A3 :
    case FR_A4 :
		{	
    	unit = g_model.frsky.channels[channel-FR_A3].units3_4 ;
			switch ( unit )
			{
				case 1 :
					unit = ' ' ;
				break ;
				case 3 :
					unit = 'A' ;
				break ;
				default :
					unit = 'v' ;
				break ;
			}
			att |= PREC2 ;
		}
    break ;

		case FR_A1_COPY:
    case FR_A2_COPY:
      channel -= FR_A1_COPY ;
      // no break
      // A1 and A2
			if ( valid || BLINK_ON_PHASE )
			{
				unit = putsTelemValue( (style & TELEM_VALUE_RIGHT) ? xbase+62 : x-fieldW, y, val, channel, att|NO_UNIT/*|blink*/ ) ;
			}
			displayed = 1 ;
    break ;

		case FR_RXV:
  		unit = 'v' ;
			att |= PREC1 ;
			val = convertRxv( val ) ;
    break ;

    case FR_TEMP1:
//			if ( ( ( g_model.telemetryProtocol == TELEMETRY_ARDUCOPTER ) || ( g_model.telemetryProtocol == TELEMETRY_ARDUPLANE ) || ( g_model.telemetryProtocol == TELEMETRY_MAVLINK ) ) && (style & TELEM_ARDUX_NAME) )
//			{
//				char *s ;
//				s = arduFlightMode( FrskyHubData[FR_TEMP1] ) ;
////#if defined(PCBX12D) || defined(PCBX10)
////				lcd_putsnAttColour( x, y, s, 100, frskyUsrStreaming ? 0 : BLINK, colour, bgColour ) ;
////#else
//				lcd_putsAtt( x, y, s, frskyUsrStreaming ? 0 : BLINK ) ;
////#endif
//				displayed = 1 ;
//				break ;
//			}
    case FR_TEMP2:
			unit = 'C' ;
  		if ( g_model.FrSkyImperial )
  		{
				val = c_to_f(val) ;
  		  unit = 'F' ;
				x -= fieldW ;
  		}
    break;
  
		case FR_AIRSPEED :
			att |= PREC1 ;
			unit = 'm' ;
			if ( g_model.FrSkyImperial )
			{
       	// m to ft *105/32
       	val = m_to_ft( val ) ;
	      unit = 'f' ;
			}
    break;

		case FR_ALT_BARO:
    case TELEM_GPS_ALT:
      unit = 'm' ;
			if ( g_model.FrSkyImperial )
			{
				if (g_model.FrSkyUsrProto == 0)  // Not WS How High
				{
        	// m to ft *105/32
        	val = m_to_ft( val ) ;
				}
        unit = 'f' ;
			}
			if ( val < 1000 )
			{
				att |= PREC1 ;
			}
			else
			{
				val /= 10 ;
			}
    break;

		case FR_CURRENT :
			att |= PREC1 ;
      unit = 'A' ;
		break ;

		case FR_CELL_MIN:
			if ( val > 445 )
			{
				val = 0 ;
			}
		case FR_CELL1:
		case FR_CELL2:
		case FR_CELL3:
		case FR_CELL4:
		case FR_CELL5:
		case FR_CELL6:
		case FR_CELL7:
		case FR_CELL8:
		case FR_CELL9:
		case FR_CELL10:
		case FR_CELL11:
		case FR_CELL12:
		case FR_RBOX_B1_V :
		case FR_RBOX_B2_V :
		case FR_SBEC_VOLT :
			att |= PREC2 ;
      unit = 'v' ;
		break ;

		case FR_RBOX_B1_A :
		case FR_RBOX_B2_A :
		case FR_SBEC_CURRENT :
			att |= PREC2 ;
      unit = 'A' ;
		break ;
		case FR_RBOX_B1_CAP :
		case FR_RBOX_B2_CAP :
      unit = 'm' ;
		break ;
		case FR_CELLS_TOT :
		case FR_VOLTS :
		case FR_CELLS_TOTAL1 :
		case FR_CELLS_TOTAL2 :
			att |= PREC1 ;
      unit = 'v' ;
		break ;
		case BATTERY:
			att |= PREC1 ;
      unit = 'v' ;
		break ;
		case FR_WATT :
      unit = 'w' ;
		break ;

		case FR_RPM :
  		lcd_outdezNAtt( (style & TELEM_VALUE_RIGHT) ? xbase+62 : x, y, (uint16_t)val, att, 5 ) ;
			displayed = 1 ;
		break ;

		case FR_VSPD :
			att |= PREC1 ;
			val /= 10 ;
		break ;

		case FR_FUEL :
      unit = '%' ;
		break ;

//		case FR_GPS_LAT :
//			displayGPSdata( x, y-FH, FrskyHubData[FR_GPS_LAT], FrskyHubData[FR_GPS_LATd], LEADING0 | att ) ;
//			lcd_putcAtt( x+50, y-FH, FrskyHubData[FR_LAT_N_S], att ) ;
//		break ;
//		case FR_GPS_LONG :
//			displayGPSdata( x, y, FrskyHubData[FR_GPS_LONG], FrskyHubData[FR_GPS_LONGd], LEADING0 | att ) ;
//			lcd_putcAtt( x+50, y, FrskyHubData[FR_LONG_E_W], att ) ;
//		break ;

		default:
    break;
  }

	if ( !displayed )
	{
		if ( valid || BLINK_ON_PHASE || (style & TELEM_CONSTANT) )
		{
  		lcd_outdezAtt( (style & TELEM_VALUE_RIGHT) ? xbase+62 : x, y, val, att ) ;
		}
		else
		{
			Lcd_lastPos = ( (style & TELEM_VALUE_RIGHT) ? xbase+62 : x ) ;	// Put the unit in the correct place
		}
	}

//	if ( style & TELEM_HIRES )
//	{
//		style &= ~TELEM_UNIT_LEFT ;
//	}

	if ( style & ( TELEM_UNIT | TELEM_UNIT_LEFT ) )
	{
		if ( style & TELEM_UNIT_LEFT )
		{
			x = xbase + FW + 4 ;			
			att &= ~DBLSIZE ;			 
		}
		else
		{
			x = Lcd_lastPos ;
		}

//		if ( style & TELEM_HIRES )
//		{
//			style &= ~TELEM_UNIT_LEFT ;
//		}
//		else
//		{
//			att &= ~CONDENSED ;
//		}	
  	lcd_putcAtt( x, y, unit, att ) ;
	}
	return unit ;
}

void menuAlpha(uint8_t event)
{
	struct t_alpha *Palpha = &Alpha ;
	char *keytext ;
	static uint8_t lastPosition ;
	static uint8_t lastChar ;

	lcd_puts_Pleft( 0, Palpha->PalphaHeading ? (char *)Palpha->PalphaHeading : PSTR( STR_NAME ) ) ;
	static MState2 mstate2 ;

//#ifdef PAGE_NAVIGATION 
//	uint8_t cursorMove = 0 ;
//	if ( event == EVT_KEY_FIRST(KEY_LEFT) )
//	{
//		event = 0 ;
//	}
//	if ( event == EVT_KEY_BREAK(KEY_LEFT) )
//	{
//		event = 0 ;
//		cursorMove = 1 ;
//	}
//	if ( event == EVT_KEY_LONG(KEY_LEFT) )
//	{
//		killEvents(event) ;
//		event = 0 ;
//		cursorMove = 2 ;
//	}
//#endif

	mstate2.check_columns( event, 5 ) ;
	keytext = Caps ? (char *)&AlphaSource2[0] : (char *)&AlphaSource[0] ;
	displayKeys( 2, 3*FH-4, keytext, 10 ) ;
	displayKeys( 5, 4*FH-3, &keytext[10], 11 ) ;
	displayKeys( 8, 5*FH-2, &keytext[21], 10 ) ;
	displayKeys( 11, 6*FH-1, &keytext[31], 10 ) ;
	lcd_puts_Pleft( 7*FH, XPSTR("CAP SPACE DEL INS < >") ) ;

	int8_t sub = mstate2.m_posVert ;
	if ( event == EVT_ENTRY )
	{
		Palpha->AlphaIndex = 0 ;
		Palpha->lastSub = sub ;
		AlphaEdited = 0 ;
		mstate2.m_posVert = 1 ;
		lastPosition = 0 ;
		lastChar = Palpha->PalphaText[0] ;
	}
	
	if ( Palpha->lastSub != sub )
	{
		if ( sub == 0 )
		{
			g_posHorz = Palpha->AlphaIndex ;
		}
		Palpha->lastSub = sub ;
	}

	uint8_t index = 0 ;
	switch ( sub )
	{
		case 0 :
			Columns = Palpha->AlphaLength-1 ;
		break ;
		case 1 :
			Columns = 9 ;
		break ;
		case 2 :
			index = 10 ;
			Columns = 10 ;
		break ;
		case 3 :
			index = 21 ;
			Columns = 9 ;
		break ;
		case 4 :
			index = 31 ;
			Columns = 9 ;
		break ;
		case 5 :
			index = 40 ;
			Columns = 5 ;
		break ;
	}
	index += g_posHorz ;

	if  ( checkForMenuEncoderBreak( event ) )
	{
		if ( ( sub == 5 ) && ( g_posHorz != 1 ) )
		{
			if ( g_posHorz == 0 )
			{
				Caps = !Caps ;
			}
			if ( g_posHorz == 2 )
			{
				uint32_t i ;
				for ( i = Palpha->AlphaIndex ; i < (uint32_t)Palpha->AlphaLength-1 ; i += 1 )
				{
					Palpha->PalphaText[i] = Palpha->PalphaText[i+1] ;
				}
				Palpha->PalphaText[Palpha->AlphaLength-1] = Palpha->AlphaHex ? '0' : ' ' ;
				AlphaEdited = 1 ;
   			eeDirty( EditType & (EE_GENERAL|EE_MODEL));
			}
			if ( g_posHorz == 3 )
			{
				if ( Palpha->AlphaIndex < Palpha->AlphaLength-1 )
				{
					uint32_t i ;
					for ( i = Palpha->AlphaLength-1 ; i > Palpha->AlphaIndex ; i -= 1 )
					{
						Palpha->PalphaText[i] = Palpha->PalphaText[i-1] ;
					}
				}
				Palpha->PalphaText[Palpha->AlphaIndex] = Palpha->AlphaHex ? '0' : ' ' ; ;
				AlphaEdited = 1 ;
    		eeDirty( EditType & (EE_GENERAL|EE_MODEL));
			}
			if ( g_posHorz == 4 )
			{
				if ( Palpha->AlphaIndex )
				{
					Palpha->AlphaIndex -= 1 ;
				}
			}
			if ( g_posHorz == 5 )
			{
				if ( Palpha->AlphaIndex < Palpha->AlphaLength-1 )
				{
					Palpha->AlphaIndex += 1 ;
				}
			}
		}
		else if (sub>0)
		{
			char chr = keytext[index] ;
			if ( !( ( sub == 5 ) && ( g_posHorz != 1 ) ) )
			{
				uint32_t set = 1 ;
				if ( Palpha->AlphaHex )
				{
					if ( ( chr >= 'a' ) && ( chr <= 'f' ) )
					{
						chr -= ('a' - 'A') ;
					}
					if ( ( chr < '0' ) || ( chr > 'F' ) || ( (chr >'9') && (chr < 'A') ) )
					{
						set = 0 ;
					}
				}
				if ( set )
				{
					lastPosition = Palpha->AlphaIndex ;
					lastChar = Palpha->PalphaText[lastPosition] ;
					Palpha->PalphaText[Palpha->AlphaIndex] = chr ;
					if ( Palpha->AlphaIndex < Palpha->AlphaLength-1 )
					{
						Palpha->AlphaIndex += 1 ;
					}
					AlphaEdited = 1 ;
    			eeDirty( EditType & (EE_GENERAL|EE_MODEL));
				}
			}
		}
		else
		{
			mstate2.m_posVert = 1 ;
		}
	}
//#ifdef PAGE_NAVIGATION 
//	if ( cursorMove )
//	{
//		if ( cursorMove == 1 )
//		{
//			if ( Palpha->AlphaIndex < Palpha->AlphaLength-1 )
//			{
//				Palpha->AlphaIndex += 1 ;
//			}
//			else
//			{
//				Palpha->AlphaIndex = 0 ;
//			}
//		}
//		else
//		{
//			if ( Palpha->AlphaIndex )
//			{
//				Palpha->AlphaIndex -= 1 ;
//			}
//			else
//			{
//				Palpha->AlphaIndex = Palpha->AlphaLength-1 ;
//			}
//		}
//	}
//#endif
	
//	if ( event == EVT_KEY_LONG(KEY_MENU) )
//	{
//    killEvents(event) ;
//		Caps = !Caps ;
//	}
	if ( getEventDbl(EVT_KEY_FIRST(KEY_MENU)) > 1 )
	{
  	killEvents(EVT_KEY_FIRST(KEY_MENU)) ;
		Caps = !Caps ;
		Palpha->AlphaIndex = lastPosition ;
		Palpha->PalphaText[lastPosition] = lastChar ;
	}
	
	if ( sub == 0 )
	{
		Palpha->AlphaIndex = g_posHorz ;
	}

  lcd_putsnAtt( 1, FH, (char *)Palpha->PalphaText, Palpha->AlphaLength, 0 ) ;
	if ( ( sub != 0 ) || ( BLINK_ON_PHASE ) )
	{
		lcd_rect( Palpha->AlphaIndex*FW, FH-1, 7, 9 ) ;
	}

	if ( sub>0 )
	{
		if ( g_posHorz > Columns )
		{
			g_posHorz = Columns ;
		}
		uint16_t x = g_posHorz * 10 + 1 ;
		uint16_t w = 9 ;
		if ( sub == 2 )
		{
			x += 3 ;
		}
		else if ( sub == 3 )
		{
			x += 6 ;
		}
		else if ( sub == 4 )
		{
			x += 9 ;
		}
		else if ( sub == 5 )
		{
			if ( g_posHorz == 0 )
			{
				x = 0 ;
				w = 3*FW ;
			}
			else if ( g_posHorz == 1 )
			{
				w = 5*FW ;
				x = 4*FW ;
			}
			else if ( g_posHorz == 4 )
			{
				w = 1*FW ;
				x = 18*FW ;
			}
			else if ( g_posHorz == 5 )
			{
				w = 1*FW ;
				x = 20*FW ;
			}
			else
			{
				x = (g_posHorz-2) * (4* FW) + 10*FW ;
				w = 3*FW ;
			}
		}
		lcd_char_inverse( x, (sub+2)*FH+sub-5, w, 0 ) ;
	}
	s_editMode = 0 ;
}

void alphaEditName( uint16_t x, uint16_t y, uint8_t *name, uint8_t len, uint16_t type, uint8_t *heading )
{
	if ( ( type & ALPHA_NO_NAME ) == 0 )
	{
		lcd_puts_Pleft( y, PSTR( STR_NAME ) ) ;
	}
	Alpha.AlphaHex = ( type & ALPHA_HEX ) ? 1 : 0 ;
	type &= ~ALPHA_HEX ;
	validateText( name, len ) ;
	lcd_putsnAtt( x, y, (const char *)name, len, 0 ) ;
	if ( type & ~ALPHA_NO_NAME )
	{
		lcd_rect( x-1, y-1, len*FW+2, 9 ) ;
		if ( checkForMenuEncoderBreak( Tevent ) )
		{
			Alpha.AlphaLength = len ;
			Alpha.PalphaText = name ;
			Alpha.PalphaHeading = heading ;
			s_editMode = 0 ;
    	killEvents(Tevent) ;
			Tevent = 0 ;
			pushMenu(menuAlpha) ;
		}
	}
}


void putSwitchName(uint16_t x, uint16_t y, int8_t z, uint8_t att)
{
  lcd_putsAttIdx(x, y, PSTR(SWITCHES_STR), z, att);
}

#define RIGHT_POSITION	127

void DisplayScreenIndex(uint8_t index, uint8_t count, uint8_t attr)
{
	uint8_t x ;
	lcd_outdezAtt(RIGHT_POSITION,0,count,attr);
	x = 1+(RIGHT_POSITION+1)-FW*(count>9 ? 3 : 2) ;
  lcd_putcAtt(x,0,'/',attr);
  lcd_outdezAtt(x-1,0,index+1,attr);
}


uint8_t MState2::check_columns( uint8_t event, uint8_t maxrow)
{
	return check( event, 0, NULL, 0, &Columns, 0, maxrow ) ;
}

uint8_t MAXCOL( uint8_t row, uint8_t *horTab, uint8_t horTabMax)
{
	return (horTab ? *(horTab+min(row, horTabMax)) : (const uint8_t)0) ;
}

#define INC(val,max) if(val<max) {val++;} else {val=0;}
#define DEC(val,max) if(val>0  ) {val--;} else {val=max;}

uint8_t MState2::check(uint8_t event, uint8_t curr, MenuFuncP *menuTab, uint8_t menuTabSize, uint8_t *horTab, uint8_t horTabMax, uint8_t maxrow)
{
	uint8_t l_posHorz ;
	l_posHorz = g_posHorz ;
	
	M_lastVerticalPosition = m_posVert ;
  
	if (menuTab)
	{
    uint8_t attr = m_posVert==0 ? INVERS : 0 ;

    if(m_posVert==0)
    {
//      if( scrollLR && !s_editMode)
//      {
//        int8_t cc = curr - scrollLR ;
//        if(cc<1) cc = menuTabSize-1 ;
//        if(cc>(menuTabSize-1)) cc = 0 ;

//        if(cc!=curr)
//        {
//          chainMenu((MenuFuncP)(&menuTab[cc]));
//					return event ;
//        }

//        scrollLR = 0;
//      }

      if(event==EVT_KEY_FIRST(KEY_LEFT))
      {
        uint8_t index ;
        if(curr>0)
          index = curr ;
        else
          index = menuTabSize ;

       	chainMenu((menuTab[index-1])) ;



				return event ;
      }

      if(event==EVT_KEY_FIRST(KEY_RIGHT))
      {
        uint8_t index ;
        if(curr < (menuTabSize-1))
          index = curr +1 ;
        else
          index = 0 ;
        chainMenu((menuTab[index]));
				return event ;
      }
    }
//		else
//		{
//			if ( s_editMode == 0 ) RotaryState = ROTARY_MENU_UD ;
//		}

		DisplayScreenIndex(curr, menuTabSize, attr);
  }
//	else if ( RotaryState == ROTARY_MENU_LR )
//	{
//		RotaryState = ROTARY_MENU_UD ;
//	}
  uint8_t maxcol = MAXCOL(m_posVert, horTab, horTabMax);
		
// if ( maxrow != 0xFF )
// {
//	if ( RotaryState == ROTARY_MENU_UD )
//	{
//		static uint8_t lateUp = 0 ;
//		if ( lateUp )
//		{
//			lateUp = 0 ;
//			l_posHorz = MAXCOL(m_posVert, horTab, horTabMax) ;
//		}
//		if ( Rotary_diff > 0 )
//		{
//    	INC(l_posHorz,maxcol) ;
//			if ( l_posHorz == 0 )
//			{
//				INC(m_posVert,maxrow);
//			}
//		}
//		else if ( Rotary_diff < 0 )
//		{
//			if ( l_posHorz == 0 )
//			{
//      	DEC(m_posVert,maxrow);
//				lateUp = 1 ;
//				l_posHorz = 0 ;
//			}
//			else
//			{
//      	DEC(l_posHorz,maxcol) ;
//			}
//		}
//		Rotary_diff = 0 ;
//    if(event==EVT_KEY_BREAK(BTN_RE))
//		{
//			RotaryState = ROTARY_VALUE ;
//		}
//	}
//	else if ( RotaryState == ROTARY_VALUE )
//	{
//		if ( s_editMode )
//		{
//			MaskRotaryLong = 1 ;
//		}
//		if ( (event==EVT_KEY_BREAK(BTN_RE)) || ( s_editMode == 0 ) )
//		{
//			RotaryState = ROTARY_MENU_UD ;
//		}
//	}

//	{
//		uint8_t timer = M_longMenuTimer ;
//		if ( menuPressed() )
//		{
//			if ( timer < 255 )
//			{
//				timer += 1 ;
//			}
//		}
//		else
//		{
//			timer = 0 ;
//		}
//		if ( timer > 60 )
//		{
//			s_editMode = 1 ;
//			RotaryState = ROTARY_VALUE ;
//		}
//		M_longMenuTimer = timer ;
//	}
//#ifdef PCBLEM1
//	{
//		uint8_t timer = M_longBtnTimer ;
//		if ( encoderPressed() )
//		{
//			if ( timer < 255 )
//			{
//				timer += 1 ;
//			}
//		}
//		else
//		{
//			timer = 0 ;
//		}
//		if ( timer > 60 )
//		{
//    	if ( ( event == 0 ) || ( event == EVT_KEY_REPT(BTN_RE) ) )
//			{
//				event = EVT_KEY_LONG(KEY_EXIT) ;
//			}			
//		}
//		M_longBtnTimer = timer ;
//	}
//#endif

// } 

  maxcol = MAXCOL(m_posVert, horTab, horTabMax) ;
	EditColumns = maxcol ;
  l_posHorz = min(l_posHorz, maxcol ) ;

  if(!s_editMode)
  {
    if(scrollUD)
    {
      int8_t cc = m_posVert - scrollUD;
      if(cc<1) cc = 0;
      if(cc>=maxrow) cc = maxrow;
      m_posVert = cc;

      l_posHorz = min(l_posHorz, MAXCOL(m_posVert, horTab, horTabMax));
//      BLINK_SYNC ;

      scrollUD = 0;
    }

    if(m_posVert>0 && scrollLR)
    {
      int8_t cc = l_posHorz - scrollLR;
      if(cc<1) cc = 0;
      if(cc>=MAXCOL(m_posVert, horTab, horTabMax)) cc = MAXCOL(m_posVert, horTab, horTabMax);
      l_posHorz = cc;

//      BLINK_SYNC;
      //            scrollLR = 0;
    }
  }

  switch(event)
  {
    case EVT_ENTRY:
        init() ;
        l_posHorz = 0 ;
				M_lastVerticalPosition = m_posVert ;
    case EVT_ENTRY_UP :
        s_editMode = false;
    break;
//    case EVT_KEY_BREAK(BTN_RE):
    case EVT_KEY_FIRST(KEY_MENU):
        if ( (m_posVert > 0) || (!menuTab) )
				{
	 				if ( maxrow != 0xFF )
					{
						s_editMode = !s_editMode;
//						if ( s_editMode )
//						{
//							RotaryState = ROTARY_VALUE ;
//						}
					}
				}
    break;
    case EVT_KEY_LONG(KEY_EXIT):
        s_editMode = false;
        popMenu(true); //return to uppermost, beeps itself
//				event = EVT_EXIT ;
    break;
        //fallthrough
//    case EVT_KEY_LONG(BTN_RE):
//				if ( g_eeGeneral.disableBtnLong )
//				{	
//					break ;
//				}
//				if ( MaskRotaryLong )
//				{
//					break ;
//				}
//				killEvents(event) ;
    case EVT_KEY_BREAK(KEY_EXIT):
        if(s_editMode)
				{
            s_editMode = false;
            break;
        }
//#ifdef PCBLEM1
        popMenu( 0 );  //beeps itself
//#endif  
//				event = EVT_EXIT ;
    break;

    case EVT_KEY_REPT(KEY_RIGHT):  //inc
        if(l_posHorz==maxcol) break;
    case EVT_KEY_FIRST(KEY_RIGHT)://inc
        if(!horTab || s_editMode)break;
        INC(l_posHorz,maxcol);
				if ( maxcol )
				{
					event = 0 ;
//					Tevent = 0 ;
				}
//        BLINK_SYNC;
    break;

    case EVT_KEY_REPT(KEY_LEFT):  //dec
        if(l_posHorz==0) break;
    case EVT_KEY_FIRST(KEY_LEFT)://dec
        if(!horTab || s_editMode)break;
        DEC(l_posHorz,maxcol);
				if ( maxcol )
				{
					event = 0 ;
//					Tevent = 0 ;
				}
//        BLINK_SYNC;
    break;

    case EVT_KEY_REPT(KEY_DOWN):  //inc
        if(m_posVert==maxrow) break;
    case EVT_KEY_FIRST(KEY_DOWN): //inc
        if(s_editMode)break;
        INC(m_posVert,maxrow);
        l_posHorz = min(l_posHorz, MAXCOL(m_posVert, horTab, horTabMax));
//        BLINK_SYNC;
    break;

    case EVT_KEY_REPT(KEY_UP):  //dec
        if(m_posVert==0) break;
    case EVT_KEY_FIRST(KEY_UP): //dec
        if(s_editMode)break;
        DEC(m_posVert,maxrow);
        l_posHorz = min(l_posHorz, MAXCOL(m_posVert, horTab, horTabMax));
//        BLINK_SYNC;
    break;
  }
	s_editing = s_editMode ; // || P1values.p1valdiff ;
//	InverseBlink = (!maxcol || s_editMode) ? BLINK : INVERS ;
	g_posHorz = l_posHorz ;
	InverseBlink = (s_editMode) ? BLINK : INVERS ;
	Columns = 0 ;
//	MaskRotaryLong = 0 ;
	return event ;
}

void menu_lcd_onoff( uint8_t x,uint8_t y, uint8_t value, uint8_t mode )
{
  if (value)
	{
    lcd_putc(x+1, y, '\202');
	}
	lcd_hbar( x, y, 7, 7, mode ? 100 : 0 ) ;
}

void lcd_xlabel_decimal( uint8_t x, uint8_t y, uint16_t value, uint8_t attr, const char *s )
{
  lcd_outdezAtt( x, y, value, attr ) ;
	lcd_puts_Pleft( y, s ) ;
}

uint8_t checkIndexed( uint16_t y, const char *s, uint8_t value, uint8_t edit )
{
	uint16_t x ;
	uint8_t max ;
	if ( s )
	{
		x = *s++ * FW ;
		max = *s++ ;
		if ( value > max )
		{
			value = max ;
		}
		lcd_putsAttIdx( x, y, s, value, edit ? InverseBlink: 0 ) ;
	}
	else
	{
		x = PARAM_OFS ;
		max = 1 ;
		menu_lcd_onoff( x, y, value, edit ) ;
	}
	if ( value > max )
	{
		value = max ;
	}
	if(edit)
	{
		if ( ( EditColumns == 0 ) || ( s_editMode ) )
		{
			value = checkIncDec16( value, 0, max, EditType ) ;
		}
	}
	return value ;
}

uint8_t onoffItem( uint8_t value, uint16_t y, uint8_t condition )
{
	return checkIndexed( y, 0, value, condition ) ;
}

uint8_t offonItem( uint8_t value, uint16_t y, uint8_t condition )
{
	return 1-onoffItem( 1-value, y, condition ) ;
}

uint8_t onoffMenuItem( uint8_t value, uint16_t y, const char *s, uint8_t condition )
{
  lcd_puts_Pleft(y, s);
	return onoffItem( value, y, condition ) ;
}

uint8_t offonMenuItem( uint8_t value, uint16_t y, const char *s, uint8_t condition )
{
	return 1-onoffMenuItem( 1-value, y, s, condition ) ;
}





static uint8_t SingleExpoChan ;

static uint8_t SubMenuCall = 0 ;
static uint8_t UseLastSubmenuIndex = 0 ;
static uint8_t LastSubmenuIndex = 0 ;

uint8_t indexProcess( uint8_t event, MState2 *pmstate, uint8_t extra )
{
	if (event == EVT_ENTRY)
	{
		pmstate->m_posVert = SubmenuIndex - 1 ;
		SubmenuIndex = 0 ;
	}
	if (event == EVT_ENTRY_UP)
	{
		SingleExpoChan = 0 ;
		pmstate->m_posVert = SubmenuIndex - 1 ;
		if ( SubMenuCall )
		{
			pmstate->m_posVert = SubMenuCall & 0x1F ;
			g_posHorz = ( SubMenuCall >> 5 ) & 3 ;
			SubMenuCall = 0 ;
		}
		else
		{
			SubmenuIndex = 0 ;
		}
	}
	
	if ( UseLastSubmenuIndex )
	{
		SubmenuIndex = LastSubmenuIndex & 0x7F ;
		UseLastSubmenuIndex = 0 ;
	}
	
	if ( SubmenuIndex )
	{
    if ( event == EVT_KEY_BREAK(KEY_EXIT) )
//  	if ( ( checkForExitEncoderLong( event ) ) && ( MaskRotaryLong == 0 ) )
		{
      if(s_editMode)
			{
        s_editMode = false;
			}
			else
			{
				pmstate->m_posVert = SubmenuIndex - 1 ;
				SubmenuIndex = 0 ;
				killEvents(event) ;
//				audioDefevent(AU_MENUS) ;
			}
			event = 0 ;
		}
	}
	else
	{
		uint8_t pv = pmstate->m_posVert ;
		if (event == EVT_KEY_FIRST(KEY_RIGHT) )
		{
			if ( pv < ((extra == 8) ? 7 : extra) )
			{
				pv += (extra == 8) ? 8 : 7 ;
			}
		}
		if (event == EVT_KEY_FIRST(KEY_LEFT) )
		{
			if ( pv >= ((extra == 8) ? 8 : 7) )
			{
				pv -= (extra == 8) ? 8 : 7 ;
			}
		}

 		if ( event == EVT_KEY_BREAK(KEY_MENU) )
		{
			SubmenuIndex = pv + 1 ;
			LastSubmenuIndex = SubmenuIndex | 0x80 ;
			pv = 0 ;
			killEvents(event) ;
			event = EVT_ENTRY ;
			Tevent = EVT_ENTRY ;

			g_posHorz = 0 ;
		}
		pmstate->m_posVert = pv ;
	}
	return event ;
}


void displayIndex( const char *strings[], uint8_t extra, uint8_t lines, uint8_t highlight )
{
	uint8_t offset = 7 ;
	if ( extra == 8 )
	{
		offset = 8 ;
		
		lcd_puts_P( 69, 0, strings[7] ) ;
	}
	for ( uint8_t i = 0 ; i < lines ; i += 1 )
	{
		lcd_puts_P( 1, (i+1)*(FH), (strings[i]) ) ;
		if ( i < extra )
		{
			lcd_puts_P( 69, (i+1)*(FH), (strings[i+offset]) ) ;
		}
	} 
	
//	pushPlotType( PLOT_BLACK ) ;
	lcd_vline( 67, 0, 63 ) ;
//	popPlotType() ;

	if ( highlight )
	{
		if ( highlight > 7 )
		{
			lcd_char_inverse( 68, (highlight-offset)*(FH), 60, 0 ) ;
		}
		else
		{
			lcd_char_inverse( 0, (highlight)*(FH), 67, 0 ) ;
		}
	}
}


uint8_t menuPressed()
{
	if ( keys[KEY_MENU].isKilled() )
	{
		return 0 ;
	}
	return readButtons() & 1 ;
}

uint8_t LongMenuTimer ;

int16_t checkIncDec16( int16_t val, int16_t i_min, int16_t i_max, uint8_t i_flags)
{
  int16_t newval = val ;
  uint8_t kpl=KEY_RIGHT, kmi=KEY_LEFT, kother = -1 ;
	uint8_t editAllowed = 1 ;
	
	if ( g_eeGeneral.forceMenuEdit && (s_editMode == 0) && ( i_flags & NO_MENU_ONLY_EDIT) == 0 )
	{
		editAllowed = 0 ;
	}

	uint8_t event = Tevent ;

//  if(event & _MSK_KEY_DBL){
//    uint8_t hlp=kpl;
//    kpl=kmi;
//    kmi=hlp;
//    event=EVT_KEY_FIRST(EVT_KEY_MASK & event);
//  }
  if(event==EVT_KEY_FIRST(kpl) || event== EVT_KEY_REPT(kpl) || (s_editMode && (event==EVT_KEY_FIRST(KEY_UP) || event== EVT_KEY_REPT(KEY_UP))) )
	{
		if ( editAllowed )
		{
			if ( ~AwBits & 0x40000 )	// Enter/Shift pressed
//			if ( menuPressed() )
			{
    		newval += StepSize ;
			}		 
			else
			{
    		newval += 1 ;
			}
			audioDefevent(AU_KEYPAD_UP);
    	kother=kmi;
		}

  }else if(event==EVT_KEY_FIRST(kmi) || event== EVT_KEY_REPT(kmi) || (s_editMode && (event==EVT_KEY_FIRST(KEY_DOWN) || event== EVT_KEY_REPT(KEY_DOWN))) )
	{
		if ( editAllowed )
		{
			if ( ~AwBits & 0x40000 )	// Enter/Shift pressed
//			if ( menuPressed() )
			{
    		newval -= StepSize ;
			}		 
			else
			{
    		newval -= 1 ;
			}
			audioDefevent(AU_KEYPAD_DOWN);
    	kother=kpl;
		}

  }
//  if((kother != (uint8_t)-1) && keyState((EnumKeys)kother))
//	{
//    newval=-val;
//    killEvents(kmi);
//    killEvents(kpl);
//  }
  if(i_min==0 && i_max==1 )
	{
		if ( (event==EVT_KEY_FIRST(KEY_MENU) ) )//|| event==EVT_KEY_BREAK(BTN_RE)) )
		{
      s_editMode = false;
      newval=!val;
     	killEvents(EVT_KEY_FIRST(KEY_MENU)) ; // Allow Dbl for BTN_RE
			event = 0 ;
		}
		else
		{
			newval &= 1 ;
		}
	}
  if ( i_flags & INCDEC_SWITCH )
	{
		if ( s_editMode )
		{
    	int8_t swtch = getMovedSwitch();
//    	int8_t swtch = 0 ;
    	if (swtch)
			{
//	#if defined(PCBSKY) || defined(PCB9XT)
//				swtch = switchUnMap( swtch ) ;
//	#endif
    	  newval = swtch ;
    	}
			else
			{
				// look for trim switches
//				uint32_t i ;

//				for( i = HSW_Etrmdn ; i < HSW_Etrmdn + 8 ; i += 1 )
//				{
//					if ( hwKeyState( i ) )
//					{
//						newval = switchUnMap( i ) ;
//						break ;
//					}
//				}
			}
		}
  }

  //change values based on P1
  if(newval>i_max)
  {
    newval = i_max;
		if ( event != EVT_KEY_REPT(KEY_MENU) )
		{
    	killEvents(event) ;
		}
    audioDefevent(AU_KEYPAD_UP);
  }
  else if(newval < i_min)
  {
    newval = i_min;
		if ( event != EVT_KEY_REPT(KEY_MENU) )
		{
    	killEvents(event) ;
		}
    audioDefevent(AU_KEYPAD_DOWN);
  }
  if(newval != val)
	{
		if ( menuPressed() )
		{
			LongMenuTimer = 255 ;
		}
    if(newval==0)
		{
			if ( event )
			{
   	  	pauseEvents(event);
			}
			if (newval>val)
			{
//				audioDefevent(AU_KEYPAD_UP);
			}
			else
			{
//				audioDefevent(AU_KEYPAD_DOWN);
			}		
    }
    eeDirty(i_flags & (EE_GENERAL|EE_MODEL));
//    checkIncDec_Ret = true;
  }
//  else {
//    checkIncDec_Ret = false;
//  }
	StepSize = 20 ;
  return newval ;
}

int8_t checkIncDec( int8_t i_val, int8_t i_min, int8_t i_max, uint8_t i_flags)
{
  return checkIncDec16(i_val,i_min,i_max,i_flags);
}

int8_t checkIncDecSwitch( int8_t i_val, int8_t i_min, int8_t i_max, uint8_t i_flags)
{
	i_val = switchUnMap( i_val ) ;
  return switchMap( checkIncDec16(i_val,i_min,i_max,i_flags) ) ;
}

int8_t checkIncDec_hm( int8_t i_val, int8_t i_min, int8_t i_max)
{
  return checkIncDec(i_val,i_min,i_max,EE_MODEL);
}

int8_t checkIncDec_hm0( int8_t i_val, int8_t i_max)
{
  return checkIncDec(i_val,0,i_max,EE_MODEL);
}

int8_t checkIncDec_hg( int8_t i_val, int8_t i_min, int8_t i_max)
{
  return checkIncDec(i_val,i_min,i_max,EE_GENERAL);
}

int8_t checkIncDec_hg0( int8_t i_val, int8_t i_max)
{
  return checkIncDec(i_val,0,i_max,EE_GENERAL);
}



#define NUM_MIX_SWITCHES	(4+NUM_SKYCSW)


void menuTextHelp(uint8_t event)
{
	static MState2 mstate2;
	uint8_t item[8] ;
	uint32_t i ;
	uint32_t index ;
	uint32_t type = TextType ;
//#ifdef BIG_SCREEN
	uint32_t width = 4 ;
//#else
//	uint32_t width = 4 ;
//#endif
	uint32_t length = 5 ;
	uint8_t max = NUM_TELEM_ITEMS ;
	uint32_t num_mix_switches = NUM_MIX_SWITCHES ;
	if ( type == TEXT_TYPE_SW_SOURCE )
	{
//#ifdef PCBX7
//		max = NUM_SKYXCHNRAW+NUM_TELEM_ITEMS+NumExtraPots - 1 ;
//#else
// #ifdef PCBX9LITE
//		max = NUM_SKYXCHNRAW+NUM_TELEM_ITEMS+NumExtraPots - 2 ;
// #else // PCBX9LITE
		max = NUM_SKYXCHNRAW+NUM_TELEM_ITEMS ;
// #endif // PCBX9LITE
//#endif // PCBX7
	}
	else if ( type == TEXT_TYPE_MIX_SOURCE )
	{
//#if defined(PCBSKY) || defined(PCB9XT) || defined(PCBX9D)
//		if ( g_eeGeneral.analogMapping & MASK_6POS )
//		{
//			num_mix_switches += 1 ;
//		}
//#endif
////#ifdef PCBX12D
////		num_mix_switches += 1 ;
////#endif
//#ifdef PCBX7
//		max = NUM_SKYXCHNRAW+1+MAX_GVARS+1+NUM_SCALERS+8+NumExtraPots + (num_mix_switches-1) - 1 + EXTRA_SKYCHANNELS - 1 + 4 ;
//#else // PCBX7
// #if defined(PCBX9LITE) || defined(PCBLEM1)
//		max = NUM_SKYXCHNRAW+1+MAX_GVARS+1+NUM_SCALERS+8+NumExtraPots + (num_mix_switches-1) - 1 + EXTRA_SKYCHANNELS - 2 + 4 ;
// #else // PCBX9LITE
		max = NUM_SKYXCHNRAW+1+MAX_GVARS+1+NUM_SCALERS+8 + (num_mix_switches-1) - 1 + 4 - 8 ;
// #endif // PCBX9LITE
//#endif // PCBX7
		if ( TextOption )
		{
			TextOption = 1 ;
		}
	}
	else if ( type == TEXT_TYPE_SW_FUNCTION )
	{
		max = CS_MAXF ;
//#ifdef BIG_SCREEN
		width = 2 ;
//#else
//		width = 2 ;
//#endif
		length = 7 ;
	}

	uint8_t rows = max + 1 ;
	uint8_t lastCol = (rows) % width ;
	if ( lastCol == 0 )
	{
		lastCol = width - 1 ;
	}
	else
	{
		lastCol -= 1 ;
	}

	rows -= 1 ;
	rows /= width ;

	TITLE(XPSTR("SELECT"));

	if ( event == EVT_ENTRY )
	{
//		TextFound = 0 ;
//		TextSearching = 0 ;
		TextResult = 0 ;
//		TextTimer = 40 ;
//		FRESULT result ;
//		result = f_open( &SharedMemory.TextControl.TextFile, "/Helptel.txt", FA_READ ) ;
//		if ( result == FR_OK )
//		{
//			SharedMemory.TextControl.TextFileOpen = 1 ;
//			f_gets( (TCHAR *)SharedMemory.TextControl.TextMenuStore, 30, &SharedMemory.TextControl.TextFile ) ;
//		}
	}
	else
	if ( checkForMenuEncoderBreak( event ) )
	{
//		if ( type == 0 )
//		{
//			if ( TextIndex > TELEM_GAP_START )
//			{
//				TextIndex += 8 ;
//			}
//		}
		TextResult = 1 ;
  	s_editMode = 0 ;
		killEvents(event) ;
		popMenu() ;
	}

	event = mstate2.check_columns( event, rows ) ;
	if ( event == EVT_ENTRY )
	{
		if ( type == TEXT_TYPE_MIX_SOURCE )
		{
			TextIndex -= 1 ;
		}
		else if ( type == TEXT_TYPE_SW_FUNCTION )
		{
			TextIndex = locateMappedItem( TextIndex, (uint8_t *)SwitchFunctionMap, CS_MAXF ) ;
		}
		mstate2.m_posVert = TextIndex / width ;
		g_posHorz = TextIndex % width ;
	}

	uint8_t sub = mstate2.m_posVert ;
	Columns = sub == rows ? lastCol : width - 1 ;
	item[0] = '\0' ;
	
  index = s_pgOfs ;
	int8_t x = sub - index ;
  if(sub<1) index = 0 ;
	else if(x>(SCREEN_LINES-3)) index = sub - (SCREEN_LINES-3) ;
  else if(x<0) index = sub ;
	s_pgOfs = index ;

	index *= width ;
	for ( i = 0 ; i < width * (SCREEN_LINES-2) ; i += 1 )
	{
//		uint16_t x = (i%width) * (length+1)*FW ;
		uint8_t x = (i%4)* 31 ;
		if ( width == 2 )
		{
			x = ( i & 1 ) * 62 ;
		}
		uint16_t y = ((i/width)+1) * FH ;
		uint8_t att = (sub) * width + g_posHorz ;
		if ( index > max )
		{
			break ;
		}
		if ( att == index )
		{
//			if ( TextIndex != att )
//			{
//				TextTimer = 15 ;
//			}
			if ( TextResult == 0 )
			{
				TextIndex = att ;
			}
			att = INVERS ;
		}
		else
		{
			att = 0 ;
		}
		if ( type == TEXT_TYPE_TELE_SOURCE )
		{
			uint32_t idx ;
			idx = index ;
			putsAttIdxTelemItems( x, y, idx, att | TSSI_TEXT ) ;
		}
		else if ( type == TEXT_TYPE_SW_SOURCE )
		{
			putsChnRaw( x, y, index, att | TSSI_TEXT ) ;
		}
		else if ( type == TEXT_TYPE_MIX_SOURCE )
		{
			uint8_t idx ;
			uint8_t swidx ;
			swidx = 0 ;
			idx = index + 1 ;
			if ( idx > MIXSRC_AIL )
			{
				idx += MIXSRC_MAX - MIXSRC_AIL - 1 ;
			}
//			idx = unmapMixSource( idx, &swidx ) ;
////			idx = unmapPots( idx ) ;
////			if ( idx >= MIX_3POS )
////			{
////				if ( idx > (uint32_t)MIX_3POS + (num_mix_switches-1) )
////				{
////					if ( idx < EXTRA_POTS_START )
////					{
////						idx -= (num_mix_switches-1) ;
////					}
////				}
////				else
////				{
////					swidx = idx - MIX_3POS ;
////					idx = MIX_3POS ;
////				}
////			}
			putsChnOpRaw( x, y, idx, swidx, TextOption, att ) ;
		}
		else if ( type == TEXT_TYPE_SW_FUNCTION )
		{
			uint8_t offset = SwitchFunctionMap[index] ;
			lcd_putsAttIdx( x, y, PSTR(CSWITCH_STR), offset, att ) ;
			setLastIdx( (char *) PSTR(CSWITCH_STR), offset ) ;
		}
		if ( att )
		{
			ncpystr( item, (uint8_t *)LastItem, length ) ;
		}
		index += 1 ;
	}
//	if ( TextTimer )
//	{
//		if ( --TextTimer == 0 )
//		{
//			// Start looking in file
//			if ( SharedMemory.TextControl.TextFileOpen )
//			{
//				f_lseek( &SharedMemory.TextControl.TextFile, 0 ) ;
//				TextFound = 0 ;
//				TextSearching = 1 ;
//			}
//		}
//	}
//	if ( TextSearching )
//	{
//		uint32_t i ;
//		for ( i = 0 ; i < 10 ; i += 1 )
//		{
					
//			TCHAR *read ;
//			read = f_gets( (TCHAR *)SharedMemory.TextControl.TextMenuBuffer, 30, &SharedMemory.TextControl.TextFile ) ;
//			if ( !read )
//			{
//				TextSearching = 0 ;
//				break ;
//			}
//			if ( strncmp( (char *)item, (char *)SharedMemory.TextControl.TextMenuBuffer, strlen((char *)item) ) == 0 )
//			{
//				TextFound = strlen((char *)item) ;
//				for ( i =0 ; i < 3 ; i += 1 )
//				{
//					if ( SharedMemory.TextControl.TextMenuBuffer[TextFound++] == ',' )
//					{
//						break ;
//					}
//				}
//				TextSearching = 0 ;
//				break ;
//			}
//		}
//	}

//#ifdef BIG_SCREEN
	lcd_hline( 0, (SCREEN_LINES-1)*FH-1, 400 ) ;
//#else
//	lcd_hline( 0, 55, 127 ) ;
//#endif
//	if ( TextFound )
//	{
//		lcd_putsAtt( 0, (SCREEN_LINES-1)*FH, (char *)&SharedMemory.TextControl.TextMenuBuffer[TextFound], 0 ) ;
//	}
////	else
////	{
////		lcd_putsAtt( 0, 7*FH, (char *)item, 0 ) ;
////	}

////	lcd_outdez( 8*FW, 0, TextIndex ) ;
////	lcd_outdez( 11*FW, 0, TextTimer ) ;
////	lcd_outdez( 14*FW, 0, ttt ) ;
////	lcd_outdez( 17*FW, 0, uuu ) ;

////  lcd_outhex4( 5*FW, 7*FH, strlen((char *)item) ) ;
////  lcd_outhex4( 9*FW, 7*FH, TextMenuBuffer[1] ) ;
////  lcd_outhex4( 13*FW, 7*FH, TextMenuBuffer[2] ) ;
////  lcd_outhex4( 17*FW, 7*FH, TextMenuBuffer[3] ) ;

////	lcd_outdez( 7*FW, 7*FH,  TextDebug1 ) ;
////	lcd_outdez( 10*FW, 7*FH, TextDebug2 ) ;
////	lcd_outdez( 13*FW, 7*FH, TextDebug3 ) ;
////	lcd_putc( 15*FW, 0, TextMenuBuffer[0] ) ;

}


uint8_t evalOffset(int8_t sub)
{
	uint8_t max = (SCREEN_LINES - 2) ;
  uint8_t t_pgOfs = s_pgOfs ;
	int8_t x = sub-t_pgOfs ;
    if(sub<1) t_pgOfs=0;
    else if(x>max) t_pgOfs = sub-max;
    else if(x<max-(SCREEN_LINES - 2)) t_pgOfs = sub-max+(SCREEN_LINES - 2);
		return (s_pgOfs = t_pgOfs) ;
}

void putsChn( uint16_t x, uint16_t y, uint8_t idx1, uint8_t att)
{
	if ( idx1 == 0 )
	{
    lcd_putsnAtt(x,y,PSTR("--- "),4,att);
	}
	else
	{
		uint16_t x1 ;
		x1 = x + 4*FW-2 ;
		if ( idx1 < 10 )
		{
			x1 -= FWNUM ;			
		}
  	lcd_outdezAtt(x1,y,idx1,att);
    lcd_putsnAtt(x,y,PSTR(STR_CH),2,att);
	}
}

void copyFileName( char *dest, char *source, uint32_t size )
{
	uint32_t i ;
	for ( i = 0 ; i < size ; i += 1 )
	{
		char c ;
		c = source[i] ;
		if ( c == '.' )
		{
			break ;
		}
		dest[i] = c ;
	}
	while ( i < size )
	{
		dest[i++] = '\0' ;
	}
}

struct t_filelist FileList ;
extern uint32_t FileSize[] ;

void checkLeavingFilelist()
{
	if ( FileList.directoryOpen )
	{
		lfs_dir_close( &Lfs, &FileList.hdir ) ;
		FileList.directoryOpen = 0 ;
	}
}

#define DISPLAY_CHAR_WIDTH	21

uint32_t fileList(uint8_t event, struct fileControl *fc )
{
	uint32_t limit ;
	uint32_t result = 0 ;
  uint8_t maxhsize ;
	uint32_t i ;
	
	limit = 7 ;
	if ( fc->nameCount < limit )
	{
		limit = fc->nameCount ;						
	}
	maxhsize = 0 ;

	if ( limit == 0 )
	{
		lcd_puts_Pleft( 4*FH, "\005No Files" ) ;
		return 0 ;
	}

	for ( i = 0 ; i < limit ; i += 1 )
	{
		uint32_t x ;
		uint32_t len ;
		len = x = strlen( FileList.Filenames[i] ) ;
		if ( x > maxhsize )
		{
			maxhsize = x ;							
		}
		if ( x > DISPLAY_CHAR_WIDTH )
		{
			if ( ( fc->hpos + DISPLAY_CHAR_WIDTH ) > x )
			{
				x = x - DISPLAY_CHAR_WIDTH ;
			}
			else
			{
				x = fc->hpos ;
			}
			len = DISPLAY_CHAR_WIDTH ;
		}
		else
		{
			x = 0 ;
		}

		lcd_putsn_P( 0, FH+FH*i, &FileList.Filenames[i][x], len ) ;
	}

	if ( ( event == EVT_KEY_REPT(KEY_DOWN) ) || event == EVT_KEY_FIRST(KEY_DOWN) )
	{
		uint16_t now ;
		now = get_tmr10ms() ;
	  if((uint16_t)( now-LastFileMoveTime) > 4) // 50mS
		{
			LastFileMoveTime = now ;
			if ( fc->vpos < limit-1 )
			{
				fc->vpos += 1 ;
			}
			else
			{
				if ( fc->nameCount > limit )
				{
					fc->index += 1 ;
//		CoSchedLock() ;
					fc->nameCount = xfillNames( fc->index, fc ) ;
//  	CoSchedUnlock() ;
				}
			}
		}
	}
	if ( ( event == EVT_KEY_REPT(KEY_UP)) || ( event == EVT_KEY_FIRST(KEY_UP) ) )
	{
		uint16_t now ;
		now = get_tmr10ms() ;
	  if((uint16_t)( now-LastFileMoveTime) > 4) // 50mS
		{
			LastFileMoveTime = now ;
			if ( fc->vpos > 0 )
			{
				fc->vpos -= 1 ;
			}
			else
			{
				if ( fc->index )
				{
					fc->index -= 1 ;
//		CoSchedLock() ;
					fc->nameCount = xfillNames( fc->index, fc ) ;
//  	CoSchedUnlock() ;
				}
			}
		}
	}
	if ( ( event == EVT_KEY_REPT(KEY_RIGHT)) || ( event == EVT_KEY_FIRST(KEY_RIGHT) ) )
	{
		if ( fc->hpos + DISPLAY_CHAR_WIDTH < maxhsize )	fc->hpos += 1 ;
	}
	if ( ( event == EVT_KEY_REPT(KEY_LEFT)) || ( event == EVT_KEY_FIRST(KEY_LEFT) ) )
	{
		if ( fc->hpos )	fc->hpos -= 1 ;
	}
//	if ( ( event == EVT_KEY_LONG(KEY_MENU) ) || ( event == EVT_KEY_BREAK(BTN_RE) ) )
	if ( event == EVT_KEY_LONG(KEY_MENU) )
	{
		// Select file to flash
		killEvents(event);
		result = 1 ;
	}
	if ( event == EVT_KEY_BREAK(KEY_EXIT) )
//	if ( checkForExitEncoderLong( event ) )
	{
		// Select file to flash
		result = 2 ;
	}
	if ( event == EVT_KEY_BREAK(KEY_MENU) )
	{
		// Tag file
		result = 3 ;
	}
	lcd_char_inverse( 0, FH+FH*fc->vpos, DISPLAY_CHAR_WIDTH*FW, 0 ) ;
	return result ;
}

extern void txmit( uint8_t c ) ;

int32_t readBinDir( lfs_dir_t *dj, struct lfs_info *fno, struct fileControl *fc )
{
	int32_t result ;
	uint32_t loop ;
	uint32_t frsk ;

	do
	{
		
		loop = 0 ;
//		fr = f_readdir ( dj, fno ) ;		// First entry
		result = lfs_dir_read( &Lfs, dj, fno );
		if ( result < 0 || fno->name[0] == 0 )
		{
			result = -1 ;
			break ;
		}
		if ( fc->ext[0] )
		{
			frsk = 0 ;
			int32_t len = strlen(fno->name) - 4 ;
			if ( fc->ext[3] )
			{
				len -= 1 ;			
			}
			if ( len < 0 )
			{
				loop = 1 ;
			}
			if ( fno->name[len] != '.' )
			{
				if ( fno->name[len-1] != '.' )
				{
					frsk = 1 ;
				}
				else
				{
					loop = 1 ;
				}
			}
			if ( ( fno->name[len+1] & ~0x20 ) != fc->ext[0] )
			{
				loop = 1 ;
			}
			if ( ( fno->name[len+2] & ~0x20 ) != fc->ext[1] )
			{
				loop = 1 ;
			}
			if ( ( fno->name[len+3] & ~0x20 ) != fc->ext[2] )
			{
				if ( frsk && ( fc->ext[2] == 'K') )
				{
					if ( ( fno->name[len+3] & ~0x20 ) != 'S' )
					{
						loop = 1 ;
					}
				}
				else
				{
					loop = 1 ;
				}
			}
			if ( fc->ext[3] || frsk )
			{
				if ( frsk )
				{
					if ( ( fno->name[len+4] & ~0x20 ) != fc->ext[2] )
					{
						loop = 1 ;
					}
				}
				else if ( ( fno->name[len+4] & ~0x20 ) != fc->ext[3] )
				{
					loop = 1 ;
				}
			}
		}
//		else // looking for a Directory
//		{
//			if ( ( fno->fattrib & AM_DIR ) == 0 )
//			{
//				loop = 1 ;
//			}
//		}	
	} while ( loop ) ;
	return result ;
}


uint32_t xfillNames( uint32_t index, struct fileControl *fc )
{
	uint32_t i ;
	int32_t result ;

//	FileList.Finfo.lfname = FileList.Filenames[0] ;
//	FileList.Finfo.lfsize = 48 ;
//	WatchdogTimeout = 300 ;		// 3 seconds
//	DIR *pDj = &FileList.Dj ;	
	lfs_dir_t *pdir = &FileList.hdir ;
	lfs_dir_seek( &Lfs, pdir, index + 2 ) ;		// Skip . and ..
	for ( i = 0 ; i < 8 ; i += 1 )
	{
//		WatchdogTimeout = 300 ;		// 3 seconds
		result = readBinDir( pdir, &FileList.info, fc ) ;
		ncpystr( (uint8_t *)FileList.Filenames[i], (uint8_t *)FileList.info.name, 48 ) ;
		FileSize[i] = FileList.info.size ;
		if ( result < 0 || FileList.info.name[0] == 0 )
		{
			break ;
		}
	}
	return i ;
}


void setupFileNames( char *dir, struct fileControl *fc, char *ext )
{
	uint32_t result ;
	result = lfs_dir_open( &Lfs, &FileList.hdir, dir ) ;
	if ( result == 0 )
	{
		FileList.directoryOpen = 1 ;
		fc->index = 0 ;
		fc->ext[0] = *ext++ ;
		fc->ext[1] = *ext++ ;
		fc->ext[2] = *ext++ ;
		fc->ext[3] = *ext ;
		fc->index = 0 ;
		fc->nameCount = xfillNames( 0, fc ) ;
		fc->hpos = 0 ;
		fc->vpos = 0 ;
	}
}


