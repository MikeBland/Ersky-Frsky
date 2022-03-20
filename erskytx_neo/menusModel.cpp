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
#include "templates.h"
#include "lcd.h"
#include "en.h"
#include "file.h"
#include "mixer.h"
#include "pxx2.h"
#include "telemetry.h"
#include "audio.h"


#define KEY_UP		TRM_LV_UP
#define KEY_DOWN	TRM_LV_DWN
#define KEY_LEFT	TRM_LH_DWN
#define KEY_RIGHT	TRM_LH_UP

#define NO_HI_LEN 25

#define ACC_NORMAL		0
#define ACC_REG				1
#define ACC_BIND			2
#define ACC_RXOPTIONS		3
#define ACC_TXOPTIONS		4
#define ACC_RX_HARDWARE	5
#define ACC_SHARE				6
#define ACC_SPECTRUM		7
#define ACC_ACCST_BIND			8

#define checkForExitEncoderLong(event) (event == EVT_KEY_BREAK(KEY_EXIT))

extern struct t_elrsConfig ElrsConfig ;

uint8_t AccState ;
uint8_t BindTimer ;

extern const uint8_t SwitchFunctionMap[] = { 0,1,2,3,4,18,21,22,19,5,6,7,8,9,10,11,20,12,13,14,15,16,17} ;

uint8_t VoiceFileType ;
uint8_t FileSelectResult ;
char SelectedVoiceFileName[16] ;

static int8_t s_curItemIdx ;
uint8_t mixToDelete ;
static int8_t s_currDestCh ;
static uint8_t s_moveItemIdx ;
int8_t s_mixMaxSel ;
uint8_t s_moveMode ;
static bool s_currMixInsMode ;
static uint8_t SingleExpoChan ;
static uint8_t s_expoChan ;
uint8_t s_currIdx;
uint8_t s_noHi ;
static uint8_t s_curveChan ;
static uint8_t saveHpos ;
static uint8_t StatusTimer ;

void menuMixer( uint8_t event ) ;
uint8_t putsTelemetryChannel( uint16_t x, uint16_t y, int8_t channel, int16_t val, uint8_t att, uint8_t style ) ;
void editAccessProtocol( uint8_t module, uint8_t event ) ;
int16_t convertTelemConstant( int8_t channel, int8_t value) ;
int16_t m_to_ft( int16_t metres ) ;

const uint8_t UnitsString[] = "\005Feet VoltsDeg_CDeg_FmAh  Amps MetreWattsPcentkms/hdb   " ;
const uint8_t DestString[] = "\005-----BaseMAmps mAh  VoltsFuel RBSV Cus1 Cus2 Cus3 Cus4 Cus5 Cus6 Aspd Cus7 Cus8 Cus9 Cus10" ;
extern const char HyphenString[] = "----" ;

extern const uint8_t GvaString[] = "Global Voice Alerts" ;

void menuVoiceOne(uint8_t event) ;
void menuCurveOne(uint8_t event) ;
uint32_t xfirePacketSend( uint8_t length, uint8_t command, uint8_t *data ) ;
void menuGlobalVoiceAlarm(uint8_t event) ;
int16_t getValue(uint8_t i) ;
void setupFileNames( char *dir, struct fileControl *fc, char *ext ) ;
struct fileControl FileControl = {	0,0,0,0, {0,0,0,0} } ;
uint32_t fileList(uint8_t event, struct fileControl *fc ) ;
extern struct t_filelist FileList ;

uint8_t CS_STATE( uint8_t x ) ;

#define CLIP_NONE		0
#define CLIP_VOICE	1
#define CLIP_SWITCH	2

struct t_clipboard
{
	uint32_t content ;
	union
	{
		VoiceAlarmData clipvoice ;
		X20CSwData clipswitch ;
	} ;
} ;

struct t_clipboard Clipboard ;

// Temporary
extern uint8_t swOn[] ;
extern int16_t g_chans512[] ;

uint32_t checkForMenuEncoderLong( uint8_t event )
{
	uint32_t result ;
	result = ( event==EVT_KEY_LONG(KEY_MENU) ) ;
	if ( result )
	{
  	s_editMode = 0 ;
    killEvents(event);
	}
	return result ;
}


#define WBAR2 (50/2)
void singleBar( uint8_t x0, uint8_t y0, int16_t val )
{
  int16_t limit = (g_model.extendedLimits ? 1280 : 1024);
  int8_t l = (abs(val) * WBAR2 + 512) / limit ;
  if(l>WBAR2)  l =  WBAR2;  // prevent bars from going over the end - comment for debugging

	pushPlotType( PLOT_BLACK ) ;
  lcd_hlineStip(x0-WBAR2,y0,WBAR2*2+1,0x55);
  lcd_vline(x0,y0-2,5);
  if(val>0)
	{
    x0+=1;
  }else{
    x0-=l;
  }
  lcd_hline(x0,y0+1,l);
  lcd_hline(x0,y0-1,l);
		popPlotType() ;
}

const char *get_curve_string()
{
    return PSTR(CURV_STR)	;
}	

int16_t edit_dr_switch( uint16_t x, uint16_t y, int16_t drswitch, uint8_t attr, uint8_t flags, uint8_t event )
{
	if ( flags & EDIT_DR_SWITCH_MOMENT )
	{
		putsMomentDrSwitches( x, y, drswitch, attr ) ;
	}
	else
	{
		putsDrSwitches( x,  y, drswitch, attr ) ;
	}
	if(flags & EDIT_DR_SWITCH_EDIT)
	{
		if ( flags & EDIT_DR_SWITCH_FMODE )
		{
			uint8_t sign = 0 ;
			int16_t mode = drswitch ;
			if ( drswitch < 0 )
			{
				sign = 1 ;
				mode = -drswitch ;
			}
			if ( ( mode <= HSW_FM6 ) && ( mode >= HSW_FM0 ) )
			{
				drswitch = mode - HSW_FM0 + MaxSwitchIndex ;
				if ( sign )
				{
					drswitch = -drswitch ;
				}
			}
			else
			{
				drswitch = switchUnMap( drswitch ) ;
			}
  		drswitch = checkIncDec16( drswitch, -MaxSwitchIndex - MAX_MODES, MaxSwitchIndex + MAX_MODES, EE_MODEL|INCDEC_SWITCH ) ;
			if ( abs(drswitch) >= MaxSwitchIndex )
			{
				sign = 0 ;
				if ( drswitch < 0 )
				{
					sign = 1 ;
					drswitch = -drswitch ;
				}
				drswitch += HSW_FM0 - MaxSwitchIndex ;
				if ( sign )
				{
					drswitch = -drswitch ;
				}
			}
			else
			{
				drswitch = switchMap( drswitch ) ;
			}
		}
		else if ( flags & EDIT_DR_SWITCH_MOMENT )
		{
			if ( drswitch > 255 )
			{
				drswitch = switchUnMap( drswitch - 256 ) + MaxSwitchIndex ;
			}
			else
			{
				drswitch = switchUnMap( drswitch ) ;
			}
  		drswitch = checkIncDec16( drswitch, (1-MaxSwitchIndex), (-2+2*MaxSwitchIndex), EE_MODEL ) ;

			if ( drswitch >= MaxSwitchIndex )
			{
				drswitch = switchMap( drswitch - MaxSwitchIndex + 1 ) + 256 ;
			}
			else
			{
				drswitch = switchMap( drswitch ) ;
			}
		}
		else
		{
			CHECK_INCDEC_MODELSWITCH( drswitch, -MaxSwitchIndex, MaxSwitchIndex) ;
		}
	}
	return drswitch ;
}

void putsTmrMode(uint8_t x, uint8_t y, uint8_t attr, uint8_t timer, uint8_t type )
{
  int8_t tm = g_model.timer[timer].tmrModeA ;
	if ( type < 2 )		// 0 or 1
	{
	  if(tm<TMR_VAROFS)
		{
        lcd_putsnAtt(  x, y, PSTR(STR_TRIGA_OPTS)+3*abs(tm),3,attr) ;
  	}
		else
		{
  		tm -= TMR_VAROFS - 7 ;
      lcd_putsAttIdx(  x, y, get_curve_string(), tm, attr ) ;
			if ( tm < 9 )
			{
				x -= FW ;		
			}
  		lcd_putcAtt(x+3*FW,  y,'%',attr);
		}
	}
	if ( ( type == 2 ) || ( ( type == 0 ) && ( tm == 1 ) ) )
	{
		putsMomentDrSwitches( x, y, g_model.timer[timer].tmrModeB, attr ) ;
	}
}


static void editTimer( uint8_t sub, uint8_t event )
{
	uint8_t subN ;
	uint8_t timer ;
	subN = 0 ;
	if ( sub < 9 )
	{
		timer = 0 ;
	}
	else
	{
		sub -= 9 ;
		timer = 1 ;
	}
	t_TimerMode *ptm ;
	ptm = &g_model.timer[timer] ;
	
	uint8_t y ;
	uint8_t k ;
  
	uint8_t t_pgOfs = evalOffset( sub ) ;
	uint8_t attr ;
	uint8_t doedit ;

	for(uint32_t i=0; i<SCREEN_LINES-1; i++)
	{
    y=(i+1)*FH ;
    k=i+t_pgOfs ;
    attr = sub==k ? InverseBlink : 0 ;
		
    switch( k )
		{
			case 0 :
				lcd_puts_Pleft( y, "Time" ) ;
				ptm = &g_model.timer[timer] ;

 				putsTime(14*FW-1, y, ptm->tmrVal, attr, attr ) ;

				if(sub==subN)	// Use s_editing???
				{
					int16_t temp = 0 ;
					CHECK_INCDEC_H_MODELVAR( temp, -60 ,60 ) ;
					ptm->tmrVal += temp ;
  			  if((int16_t)ptm->tmrVal < 0) ptm->tmrVal=0;
				}
			break ;

			case 1 :
  			lcd_puts_Pleft( y, PSTR(STR_TRIGGERA));
  			if(attr)
				{
					CHECK_INCDEC_H_MODELVAR( ptm->tmrModeA, 0, 1+2+24+8 ) ;
				}
 				putsTmrMode(16*FW, y, attr, timer, 1 ) ;
			break ;
  	
			case 2 :
				lcd_puts_Pleft( y, PSTR(STR_TRIGGERB));
				doedit = attr ? EDIT_DR_SWITCH_MOMENT | EDIT_DR_SWITCH_EDIT : EDIT_DR_SWITCH_MOMENT ;
				ptm->tmrModeB = edit_dr_switch( 16*FW, y, ptm->tmrModeB, attr, doedit, event ) ;
			break ;
	 
			case 3 :
				lcd_puts_Pleft( y, PSTR(STR_TIMER) );
  			if(attr)
				{
					CHECK_INCDEC_H_MODELVAR( ptm->tmrDir, 0, 1 ) ;
				}
  			lcd_putsAttIdx( 10*FW, y, PSTR(STR_COUNT_DOWN_UP), ptm->tmrDir, attr ) ;
			break ;

			case 4 :
				ptm->timerMbeep = onoffMenuItem( ptm->timerMbeep, y, PSTR(STR_MINUTE_BEEP), attr ) ;
			break ;
			
			case 5 :
				ptm->timerCdown = onoffMenuItem( ptm->timerCdown, y, PSTR(STR_BEEP_COUNTDOWN), attr ) ;
			break ;

			case 6 :
  			lcd_puts_Pleft( y, PSTR( STR_RESET_SWITCH ));
				doedit = attr ? EDIT_DR_SWITCH_MOMENT | EDIT_DR_SWITCH_EDIT : EDIT_DR_SWITCH_MOMENT ;
				ptm->timerRstSw = edit_dr_switch( 16*FW, y, ptm->timerRstSw, attr, doedit, event ) ;
			break ;
  	
			case 7 :
				lcd_puts_Pleft( y, XPSTR("Haptic"));
				ptm->timerHaptic = checkIndexed( y, XPSTR("\014\003""\006  NoneMinute Cdown  Both"), ptm->timerHaptic, attr ) ;
			break ;
			
			case 8 :
	  		ptm->autoReset = onoffMenuItem( ptm->autoReset, y, XPSTR("Auto Reset"), attr ) ;
			break ;
		}
	}
}



void menuTimers( uint8_t event)
{
	uint32_t t ;
	uint32_t secs ;
	div_t qr ;
	TITLE( PSTR( STR_TIMER ) ) ;
	static MState2 mstate2 ;
	event = mstate2.check_columns( event, 19-1 ) ;

  uint8_t sub = mstate2.m_posVert ;
	uint8_t subN = 0 ;

	if ( sub < 18 )
	{
		lcd_putcAtt( 6*FW, 0, ( sub < 9 ) ? '1' : '2', BLINK ) ;
		editTimer( sub, event ) ;
	}
	else
	{
		subN = 18 ;
		lcd_puts_Pleft( FH, PSTR( STR_TOTAL_TIME ));
		t = g_model.totalTime ;
		secs = t % 60 ;
		t = t / 60 ;	// minutes
		qr = div( t, 60 ) ;
		lcd_outdezNAtt( 20*FW-3, FH, secs, LEADING0, 2 ) ;
		lcd_putcAtt(17*FW, FH, ':', 0 ) ;
		lcd_outdezNAtt( 17*FW, FH, (uint16_t)qr.rem, LEADING0, 2 ) ;
		lcd_putcAtt(14*FW+3, FH, ':', 0 ) ;
		lcd_outdezAtt( 14*FW+3, FH, (uint16_t)qr.quot, 0 ) ;

    if(sub==subN)
		{
			if (event == EVT_KEY_LONG(KEY_MENU) )
			{
				g_model.totalTime = 0 ;
    		killEvents( event ) ;
			}
		}
	}
}

#define SC1_OFF_0			0

uint16_t scalerDecimal( uint8_t y, uint16_t val, uint8_t attr )
{
  lcd_outdezAtt( 13*FW+SC1_OFF_0, y, val+1, attr ) ;
	if (attr) val = checkIncDec16( val, 0, 2047, EE_MODEL);
	return val ;
}

static uint8_t s_scalerSource ;

void menuScaleOne(uint8_t event)
{
	static MState2 mstate2 ;
	mstate2.check_columns(event, 13-1 ) ;
  lcd_puts_Pleft( 0, XPSTR("SC  =") ) ;
	uint8_t index = s_currIdx ;
  lcd_putc( 2*FW, 0, index+'1' ) ;
	
  uint8_t sub = mstate2.m_posVert ;
	evalOffset( sub ) ;

//#ifdef BIG_SCREEN
//			DisplayOffset = SC1_OFF_0 ;
//#endif
	 
	putsTelemetryChannel( 9*FW+SC1_OFF_0, 0, index+TEL_ITEM_SC1, 0, 0, TELEM_UNIT ) ;

	ScaleData *pscaler ;
	pscaler = &g_model.Scalers[index] ;
	if ( event == EVT_ENTRY )
	{
//		RotaryState = ROTARY_MENU_UD ;
	}
	else if ( event == EVT_ENTRY_UP )
	{
		// Returned from editing
		if ( TextResult )
		{
			if ( s_scalerSource )
			{
				pscaler->exSource = TextIndex ;
			}
			else
			{
				pscaler->source = TextIndex ;
			}
	    eeDirty(EE_MODEL) ;
		}
	}
  
	for ( uint32_t k = 0 ; k < SCREEN_LINES-1 ; k += 1 )
	{
    uint16_t y = (k+1) * FH ;
    uint8_t i = k + s_pgOfs;
		uint8_t attr = (sub==i ? InverseBlink : 0);
		switch(i)
		{
      case 0 :	// Source
			case 7 :	// exSource
			{	
				uint8_t x ;
				
				if ( i == 0 )
				{				
					lcd_puts_Pleft( y, XPSTR("Source") ) ;
					x = pscaler->source ;
				}
				else
				{
					lcd_puts_Pleft( y, XPSTR("ex Source") ) ;
					x = pscaler->exSource ;
				}
				putsChnRaw( 11*FW+SC1_OFF_0, y, x, attr ) ;
				if( attr )
				{
//					x = mapPots( x ) ;
					x = checkIncDec16( x,0,NUM_SKYXCHNRAW+NUM_TELEM_ITEMS-1+1,EE_MODEL);
////					CHECK_INCDEC_H_MODELVAR( x, 0, NUM_SKYXCHNRAW+NUM_TELEM_ITEMS+NumExtraPots ) ;
					if ( i == 0 )
					{
						pscaler->source = x ;
					}
					else
					{
						pscaler->exSource = x ;
					}
					if (event == EVT_KEY_BREAK(KEY_MENU) )
					{
//						// Long MENU pressed
						if ( i == 0 )
						{				
							x = pscaler->source ;
							s_scalerSource = 0 ;
						}
						else
						{
							x = pscaler->exSource ;
							s_scalerSource = 1 ;
						}
						TextIndex = x ;
  				  TextType = TEXT_TYPE_SW_SOURCE ;
  				  killEvents(event) ;
						pushMenu(menuTextHelp) ;
					}
				}
			}
			break ;
			case 1 :	// name
				alphaEditName( 11*FW-2+SC1_OFF_0, y, (uint8_t *)pscaler->name, sizeof(pscaler->name), attr, (uint8_t *)XPSTR( "Scaler Name") ) ;
			break ;
      case 2 :	// offset
				lcd_puts_Pleft( y, PSTR(STR_OFFSET) ) ;
			  lcd_outdezAtt( 13*FW+SC1_OFF_0, y, pscaler->offset, attr ) ;
				if ( attr )
				{
					StepSize = 100 ;
					pscaler->offset = checkIncDec16( pscaler->offset, -32000, 32000, EE_MODEL ) ;
				}
			break ;
      case 3 :	// mult
				lcd_puts_Pleft( y, XPSTR("Multiplier") ) ;
				pscaler->mult = scalerDecimal( y, pscaler->mult, attr ) ;
			break ;
      case 4 :	// div
				lcd_puts_Pleft( y, XPSTR("Divisor") ) ;
				pscaler->div = scalerDecimal( y, pscaler->div, attr ) ;
			break ;
      case 5 :	// mod
				lcd_puts_Pleft( y, XPSTR("Mod Value") ) ;
				pscaler->mod = scalerDecimal( y, pscaler->mod, attr ) ;
			break ;
      case 6 :	// offsetLast
				lcd_puts_Pleft( y, XPSTR("Offset At") ) ;
				lcd_putsAttIdx( 11*FW+SC1_OFF_0, y, XPSTR("\005FirstLast "), pscaler->offsetLast, attr ) ;
  			if( attr ) CHECK_INCDEC_H_MODELVAR( pscaler->offsetLast, 0, 1 ) ;
			break ;
			case 8 :
				lcd_puts_Pleft( y, XPSTR("Function") ) ;
				pscaler->exFunction = checkIndexed( y, XPSTR("\015\006""\010--------Add     SubtractMultiplyDivide  Mod     Min     "), pscaler->exFunction, attr ) ;
			break ;
			case 9 :	// unit
				lcd_puts_Pleft( y, XPSTR("Unit") ) ;
				lcd_putsAttIdx( 11*FW+SC1_OFF_0, y, (char *)XPSTR(UnitsString), pscaler->unit, attr ) ;
  			if( attr ) CHECK_INCDEC_H_MODELVAR( pscaler->unit, 0, 10 ) ;
			break ;
      case 10 :	// sign
				lcd_puts_Pleft( y, XPSTR("Sign") ) ;
  			lcd_putcAtt( 11*FW+SC1_OFF_0, y, pscaler->neg ? '-' : '+', attr ) ;
  			if( attr ) CHECK_INCDEC_H_MODELVAR( pscaler->neg, 0, 1 ) ;
			break ;
      case 11 :	// precision
				lcd_puts_Pleft( y, XPSTR("Decimals") ) ;
				lcd_outdezAtt( 13*FW+SC1_OFF_0, y, pscaler->precision, attr) ;
  			if( attr ) CHECK_INCDEC_H_MODELVAR( pscaler->precision, 0, 2 ) ;
			break ;
      case 12 :	// Dest
				lcd_puts_Pleft( y, XPSTR("Dest") ) ;
#define NUM_SCALE_DESTS		17
				if( attr )
				{
					CHECK_INCDEC_H_MODELVAR( pscaler->dest, 0, NUM_SCALE_DESTS ) ;
				}
				lcd_putsAttIdx( 11*FW+SC1_OFF_0, y, (char *)XPSTR(DestString), pscaler->dest, attr ) ;
			break ;
		}
	}

}


#define SCS_OFF_0			(0)
//#define SCS_OFF_0			(8*FW)


void menuScalers(uint8_t event)
{
	TITLE(XPSTR("Scalers")) ;
	EditType = EE_MODEL ;
	static MState2 mstate2 ;
 	event = mstate2.check_columns(event, NUM_SCALERS-1 ) ;
	
	uint8_t sub = mstate2.m_posVert ;
	uint16_t y = FH ;
	
  uint8_t t_pgOfs = (sub == 7 ) ? 1 : 0 ;
  uint8_t k ;
	
  switch (event)
	{
    case EVT_KEY_FIRST(KEY_MENU) :
//    case EVT_KEY_BREAK(BTN_RE) :
      s_currIdx = sub ;
      killEvents(event);
      pushMenu(menuScaleOne) ;
		break ;
  }

//#ifdef BIG_SCREEN
//			DisplayOffset = SCS_OFF_0 ;
//#endif
	 
	for (uint8_t i=0; i<7; i++ )
	{
    y=(i+1)*FH;
    k=i+t_pgOfs;
  	lcd_puts_Pleft( y, XPSTR("SC\011+") ) ;
  	lcd_putc( 12*FW+4+SCS_OFF_0, y, '*' ) ;
  	lcd_putc( 17*FW+1+SCS_OFF_0, y, '/' ) ;
  	lcd_putc( 2*FW+SCS_OFF_0, y, k+'1' ) ;
		putsChnRaw( 4*FW+SCS_OFF_0, y, g_model.Scalers[k].source, 0 ) ;
		lcd_outdezAtt( 12*FW+3+SCS_OFF_0, y, g_model.Scalers[k].offset, 0) ;
		lcd_outdezAtt( 17*FW-1+SCS_OFF_0, y, g_model.Scalers[k].mult+1, 0) ;
		lcd_outdezAtt( 21*FW+SCS_OFF_0, y, g_model.Scalers[k].div+1, 0) ;
	}
	lcd_char_inverse( 0+SCS_OFF_0, (sub-t_pgOfs+1)*FH, 126, 0 ) ;
}

void menuGlobals(uint8_t event)
{
	TITLE(PSTR(STR_GLOBAL_VARS));
	EditType = EE_MODEL ;
	static MState2 mstate2;
	
	event = mstate2.check_columns(event, MAX_GVARS - 1 ) ;

	uint8_t subN = mstate2.m_posVert ;
	uint8_t subSub = g_posHorz;
	uint16_t y = FH ;

	if ( subN < MAX_GVARS )
	{
		Columns = 2 ;
		for (uint32_t i = 0 ; i<MAX_GVARS ; i += 1 )
		{
  	  lcd_puts_Pleft(y, PSTR(STR_GV));
			lcd_putc( 2*FW, y, i+'1') ;
			for(uint32_t j=0; j<3;j++)
			{
  	    uint8_t attr = ((subN==i && subSub==j) ? InverseBlink : 0);
				uint8_t active = (attr && (s_editMode) ) ;
				GvarData *pgvar ;
				pgvar = &g_model.gvars[i] ;
  	    if ( j == 0 )
				{
  	     	putsDrSwitches( 6*FW, y, pgvar->gvswitch ,attr );
  				if(active) CHECK_INCDEC_MODELSWITCH( pgvar->gvswitch, -MaxSwitchIndex, MaxSwitchIndex) ;
				}
				else if ( j == 1 )
				{
					lcd_putsAttIdx( 12*FW-4, y, PSTR(STR_GV_SOURCE), pgvar->gvsource, attr ) ;
						// STR_GV_SOURCE
  				if(active)
					{ 
						CHECK_INCDEC_H_MODELVAR( pgvar->gvsource, 0, 69 ) ;
					}
				}
				else
				{
					lcd_outdezAtt( 19*FW, y, pgvar->gvar, attr) ;
  				if(active) CHECK_INCDEC_H_MODELVAR(  pgvar->gvar, -125, 125 ) ;
				}
			}
			y += FH ;
		}
	}
}

void menuAdjust(uint8_t event)
{
	TITLE(XPSTR("GVAR Adjust"));
	EditType = EE_MODEL ;
	static MState2 mstate2;
 	event = mstate2.check_columns(event, NUM_GVAR_ADJUST - 1 ) ;
	
	uint8_t sub = mstate2.m_posVert ;
	uint8_t subSub = g_posHorz;
	uint16_t y = FH ;
	
  uint8_t t_pgOfs ;
	t_pgOfs = evalOffset( sub ) ;
  uint8_t k ;
	Columns = 3 ;

	for (uint8_t i=0; i<SCREEN_LINES-1; i++ )
	{
		GvarAdjust *pgvaradj ;
		uint8_t idx ;
    y=(i+1)*FH;
    k=i+t_pgOfs;
		pgvaradj = &g_model.gvarAdjuster[k] ;
		idx = pgvaradj->gvarIndex ;
  	lcd_putc( 0, y, 'A' ) ;
  	lcd_puts_P( 3*FW+2, y, XPSTR("GV") ) ;
		if ( k < 9 )
		{
  		lcd_putc( 1*FW, y, k+'1' ) ;
		}
		else
		{
  		lcd_putc( 1*FW, y, k > 18 ? '2' : '1' ) ;
  		lcd_putc( 2*FW, y, ( (k+1) % 10) +'0' ) ;
		}

		if ( sub==k )
		{
			int8_t value ;
  		lcd_puts_Pleft( 0, XPSTR("\015GV =") ) ;
			if ( idx > 6 )
			{
				value = getTrimValue( CurrentPhase, idx - 7  ) ;
				lcd_putsAttIdx( 13*FW, 0, PSTR(STR_GV_SOURCE), idx-6, 0 ) ;
			}
			else
			{
				value = g_model.gvars[idx].gvar ;
  			lcd_putc( 15*FW, 0, idx+'1' ) ;
			}
			lcd_outdez( 20*FW, 0, value ) ;
		} 
		for( uint32_t j = 0 ; j < 4 ; j += 1 )
		{
      uint8_t attr = ((sub==k && subSub==j) ? InverseBlink : 0);
			uint8_t active = (attr && (s_editMode) ) ;
			if ( j == 0 )
			{
				if ( idx > 6 )
				{
					lcd_putsAttIdx( 3*FW+2, y, PSTR(STR_GV_SOURCE), idx-6, attr ) ;
				}
				else
				{
					lcd_putcAtt( 5*FW+2, y, idx+'1', attr ) ;
				}
				if ( active )
				{
 	        CHECK_INCDEC_H_MODELVAR( pgvaradj->gvarIndex, 0, 10 ) ;
					idx = pgvaradj->gvarIndex ;
				}
			}
			else if ( j == 1 )
			{
				uint32_t old = pgvaradj->function ;
				pgvaradj->function = checkIndexed( y, XPSTR("\007\010""\005-----Add  Set CSet V+/-  Inc/0Dec/0+/Lim-/Lim"), pgvaradj->function, attr ) ;
				if ( pgvaradj->function != old )
				{
					if ( ( old < 4 ) || ( old > 6 ) )
					{
						if ( pgvaradj->function >= 4 )
						{
							pgvaradj->switch_value = 0 ;
						}
					}
					else
					{
						if ( ( pgvaradj->function < 4 ) || ( pgvaradj->function > 6 ) )
						{
							pgvaradj->switch_value = 0 ;
						}
					}
				}
			}
			else if ( j == 2 )
			{
				pgvaradj->swtch = edit_dr_switch( 13*FW-1, y, pgvaradj->swtch, attr, active ? EDIT_DR_SWITCH_EDIT : 0, event ) ;
			}
			else
			{
				if ( ( pgvaradj->function < 4 ) || ( pgvaradj->function > 6 ) )
				{
					if ( pgvaradj->function == 3 )
					{
						lcd_putsAttIdx( 18*FW, y, PSTR(STR_GV_SOURCE), pgvaradj->switch_value, attr ) ;
		  			if(active) CHECK_INCDEC_H_MODELVAR( pgvaradj->switch_value, 0, 69 ) ;
					}
					else
					{
						lcd_outdezAtt( 21*FW, y, pgvaradj->switch_value, attr ) ;
						if ( active )
						{
 	      	  	CHECK_INCDEC_H_MODELVAR( pgvaradj->switch_value, -125, 125 ) ;
						}
					}
				}
				else
				{
					pgvaradj->switch_value = edit_dr_switch( 17*FW, y, pgvaradj->switch_value, attr, active ? EDIT_DR_SWITCH_EDIT : 0, event ) ;
				}
			}
		}
	}
}


static uint8_t editSlowDelay( uint8_t y, uint8_t attr, uint8_t value )
{
  if(attr)  value = checkIncDec16( value, 0, 250, EE_MODEL ) ; //!! bitfield
	uint8_t lval = value ;
  lcd_outdezAtt(FW*18-3,y,lval,attr|PREC1) ;
	return value ;
}

uint8_t getMixerCount()
{
  uint8_t mixerCount = 0 ;
  uint8_t dch ;

  for(uint32_t i = 0 ; i<MAX_SKYMIXERS ; i += 1)
  {
    dch = g_model.mixData[i].destCh ;
    if ((dch!=0) && (dch<=NUM_SKYCHNOUT))
    {
      mixerCount += 1 ;
    }
  }
  return mixerCount ;
}


void menuMixersLimit(uint8_t event)
{
  switch(event)
  {
  case  EVT_KEY_FIRST(KEY_EXIT):
    killEvents(event) ;
    popMenu(true) ;
    pushMenu(menuMixer) ;
    break;
  }
  lcd_puts_Pleft(2*FH, PSTR(STR_MAX_MIXERS)) ;
  lcd_outdezAtt(20*FW, 2*FH, getMixerCount(),0) ;

  lcd_puts_Pleft(4*FH, PSTR(STR_PRESS_EXIT_AB)) ;
}

void deleteMix(uint8_t idx)
{
	while ( idx < MAX_SKYMIXERS-2 )
	{
		g_model.mixData[idx] = g_model.mixData[idx+1] ;
		idx += 1 ;
	}
  memset(&g_model.mixData[MAX_SKYMIXERS-1],0,sizeof(X20MixData));

	eeDirty(EE_MODEL) ;

}

void menuDeleteMix(uint8_t event)
{
	uint8_t action ;
	action = yesNoMenuExit( event, (mixToDelete== 0xFF) ? XPSTR("Clear ALL mixes?") : PSTR(STR_DELETE_MIX) ) ;
  
	switch( action )
	{
    case YN_YES :
			if (mixToDelete == 0xFF)
			{
        clearMixes() ;
			}
			else
			{
      	deleteMix(mixToDelete);
			}	 
      //fallthrough
		case YN_NO :
    break;
  }
}

void insertMix(uint8_t idx, uint8_t copy)
{
  X20MixData *md = &g_model.mixData[idx] ;

	uint8_t xdx ;
	xdx = MAX_SKYMIXERS-1 ;
	while ( xdx > idx )
	{
		g_model.mixData[xdx] = g_model.mixData[xdx-1] ;
		xdx -= 1 ;
	}

	if ( copy )
	{
		*md = *(md-1) ;
	}
	else
	{
   	memset(md,0,sizeof(X20MixData));
   	md->destCh      = s_currDestCh; //-s_mixTab[sub];
   	md->srcRaw      = s_currDestCh; //1;   //
    md->weight      = 100 ;
//		md->lateOffset  = 1 ;
	}
	s_curItemIdx = idx ;
}

bool reachMixerCountLimit()
{
  // check mixers count limit
  if (getMixerCount() >= MAX_SKYMIXERS)
  {
    pushMenu(menuMixersLimit) ;
    return true ;
  }
  else
  {
    return false ;
  }
}

void memswap( void *a, void *b, uint8_t size )
{
	uint8_t *x ;
	uint8_t *y ;
	uint8_t temp ;

	x = (unsigned char *) a ;
	y = (unsigned char *) b ;
	while ( size-- )
	{
		temp = *x ;
		*x++ = *y ;
		*y++ = temp ;
	}
}


void moveMix(uint8_t idx, uint8_t dir) //true=inc=down false=dec=up - Issue 49
{
	X20MixData *src = &g_model.mixData[idx] ;
  if(idx==0 && !dir)
	{
    if (src->destCh>0)
		{
		  src->destCh -= 1 ;
		}
		eeDirty(EE_MODEL) ;
		return ;
	}

  if(idx>MAX_SKYMIXERS || (idx==MAX_SKYMIXERS && dir)) return ;
  uint8_t tdx = dir ? idx+1 : idx-1 ;
	X20MixData *tgt = &g_model.mixData[tdx] ;

  if((src->destCh==0) || (src->destCh>NUM_SKYCHNOUT) || (tgt->destCh>NUM_SKYCHNOUT)) return;

  if(tgt->destCh!=src->destCh)
	{
    if ((dir)  && (src->destCh<NUM_SKYCHNOUT)) src->destCh++ ;
    if ((!dir) && (src->destCh>0))          src->destCh-- ;
		eeDirty(EE_MODEL) ;
    return ;
  }

  //flip between idx and tgt
  memswap( tgt, src, sizeof(X20MixData) ) ;
	s_moveItemIdx = tdx ;
  
	eeDirty(EE_MODEL) ;
}


void put_curve( uint8_t x, uint8_t y, int8_t idx, uint8_t attr )
{
	if ( idx < 0 )
	{
    lcd_putcAtt( x-FW, y, '!', attr ) ;
		idx = -idx + 6 ;
	}
	lcd_putsAttIdx( x, y, PSTR(CURV_STR),idx,attr);
}

void displayNext()
{
	lcd_putsAtt( 17*FW+2, 7*FH, XPSTR("[->]"), BLINK ) ;
}

uint8_t hyphinvMenuItem( uint8_t value, uint16_t y, uint8_t condition )
{
	return checkIndexed( y, PSTR( STR_HYPH_INV), value, condition ) ;
}

int16_t gvarMenuItem( uint8_t x, uint8_t y, int16_t value, int8_t min, int8_t max, uint16_t attr, uint8_t event )
{
  uint8_t invers = attr&(INVERS|BLINK);

  if ( attr & GVAR_100 )
	{
		attr &= ~GVAR_100 ;
		if (value >= 101)
		{
			dispGvar( x-3*FW, y, (uint8_t)value - 100, attr ) ;
	    if (invers) value = checkIncDec16( (uint8_t)value, 101, 107, EE_MODEL);
		}
		else
		{
  	  lcd_outdezAtt(x, y, value, attr ) ;
    	if (invers) CHECK_INCDEC_H_MODELVAR( value, min, max);
		}
		if (invers)
		{
			uint32_t toggle = 0 ;
			if ( event == EVT_TOGGLE_GVAR )
			{
				toggle = 1 ;
			}
			if ( getEventDbl(EVT_KEY_FIRST(KEY_MENU)) > 1 )
			{
    		killEvents(EVT_KEY_FIRST(KEY_MENU)) ;
				toggle = 1 ;
			}
			if ( toggle )
			{
  	  	value = ((value >= 101) ? g_model.gvars[(uint8_t)value-101].gvar : 101);
		    eeDirty(EE_MODEL) ;
			}
		}
		return value ;
	}
  if ( attr & GVAR_250 )
	{
		attr &= ~GVAR_250 ;
		if (value >= 501)
		{
			dispGvar( x-3*FW, y, (uint8_t)value - 500, attr ) ;
  	  if (invers) value = checkIncDec16( value, 501, 505, EE_MODEL);
  	}
		else
		{
	    lcd_outdezAtt(x, y, value, attr ) ;
  	  if (invers) value = checkIncDec16( value, -350, 350, EE_MODEL ) ;
		}
		if (invers)
		{
			uint32_t toggle = 0 ;
			if ( event == EVT_TOGGLE_GVAR )
			{
				toggle = 1 ;
			}
			if ( getEventDbl(EVT_KEY_FIRST(KEY_MENU)) > 1 )
			{
    		killEvents(EVT_KEY_FIRST(KEY_MENU)) ;
				toggle = 1 ;
			}
			if ( toggle )
			{
				s_editMode = 0 ;
  	  	value = ((value >= 501) ? g_model.gvars[value-501].gvar : 501 ) ;
		    eeDirty(EE_MODEL) ;
			}
		}
		return value ;
	}
	if (value >= 126 || value <= -126)
	{
		dispGvar( x-3*FW, y, (uint8_t)value - 125, attr ) ;
    if (invers) value = checkIncDec16( (uint8_t)value, 126, 130, EE_MODEL);
  }
  else
	{
    lcd_outdezAtt(x, y, value, attr ) ;
    if (invers) CHECK_INCDEC_H_MODELVAR( value, min, max);
  }
  
	if (invers)
	{
		uint32_t toggle = 0 ;
		if ( event == EVT_TOGGLE_GVAR )
		{
			toggle = 1 ;
		}
		if ( getEventDbl(EVT_KEY_FIRST(KEY_MENU)) > 1 )
		{
   		killEvents(EVT_KEY_FIRST(KEY_MENU)) ;
			toggle = 1 ;
		}
		if ( toggle )
		{
    	value = ((value >= 126 || value <= -126) ? g_model.gvars[(uint8_t)value-126].gvar : 126);
	    eeDirty(EE_MODEL) ;
		}
	}
	return value;
}


void menuTemplates(uint8_t event)
{
	TITLE( PSTR(STR_TEMPLATES) ) ;
	static MState2 mstate2 ;
	event = mstate2.check_columns( event, NUM_TEMPLATES-1 ) ;
	EditType = EE_MODEL ;

  uint8_t t_pgOfs ;
  uint16_t y = 0;
  uint8_t k = 0;
  uint8_t sub = mstate2.m_posVert ;

  t_pgOfs = evalOffset(sub);

	if ( checkForMenuEncoderLong( event ) )
	{
    //apply mixes or delete
    s_noHi = NO_HI_LEN ;
    applyTemplate(sub) ;
    audioDefevent(AU_WARNING2) ;
  }

  y=1*FH;
  for(uint32_t i=0; i<(SCREEN_LINES - 1); i++)
	{
    k=i+t_pgOfs;
    if(k==NUM_TEMPLATES) break;

    //write mix names here
    lcd_outdezNAtt(3*FW, y, k+1, (sub==k ? INVERS : 0) + LEADING0,2);
    lcd_putsAtt( 4*FW, y, PSTR(n_Templates[k]), (s_noHi ? 0 : (sub==k ? INVERS  : 0)));
    y+=FH;
  }
}

void menuMixOne(uint8_t event)
{
	uint8_t numItems = 15 ;
	X20MixData *md2 = &g_model.mixData[s_curItemIdx] ;

	static MState2 mstate2 ;
	mstate2.check_columns( event, numItems-1 ) ;
	uint16_t x = TITLE( PSTR(STR_EDIT_MIX)) ;
	
//	uint32_t num_mix_switches = NUM_MIX_SWITCHES ;

//	if ( event == EVT_ENTRY )
//	{
//		RotaryState = ROTARY_MENU_UD ;
//	}
	if ( event == EVT_ENTRY_UP )
	{
		SingleExpoChan = 0 ;
		if ( TextResult )
		{
			md2->srcRaw = TextIndex + 1 ;
	    eeDirty(EE_MODEL) ;
		}
	}

	putsChn( x+1*FW, 0, md2->destCh, 0 ) ;
  uint8_t sub = mstate2.m_posVert ;

  evalOffset(sub) ;

  for( uint32_t k = 0 ; k<SCREEN_LINES-1 ; k += 1 )
  {
    uint16_t y = (k+1) * FH ;
    uint8_t i = k + s_pgOfs ;
    uint8_t attr = sub==i ? InverseBlink : 0 ;
////  	uint8_t b ;

    switch(i)
		{
      case 0 :
			{	
        lcd_puts_P( 2*FW,y,PSTR(STR_2SOURCE)) ;
				uint32_t value = md2->srcRaw ;
				putsChnOpRaw( FW*14,y, value, md2->switchSource, md2->disableExpoDr, attr ) ;
				if ( attr )
				{
////					uint8_t x = mapMixSource( value, md2->switchSource ) ;
					uint8_t x = value ;
					if ( x > MIXSRC_AIL )
					{
						x -= MIXSRC_MAX - MIXSRC_AIL - 1 ;
					}
					CHECK_INCDEC_H_MODELVAR( x, 1, NUM_SKYXCHNRAW+1+MAX_GVARS+1+NUM_SCALERS+8) ;
					if ( event == EVT_KEY_BREAK(KEY_MENU) )
					{
						// Long MENU pressed
						TextIndex = x ;
  					TextType = TEXT_TYPE_MIX_SOURCE ;
						TextOption = md2->disableExpoDr ;
  					killEvents(event) ;
						pushMenu(menuTextHelp) ;
					}
////						}	
////					md2->srcRaw = unmapMixSource( x, &md2->switchSource ) ;
					if ( x > MIXSRC_AIL )
					{
						x += MIXSRC_MAX - MIXSRC_AIL - 1 ;
					}
					md2->srcRaw = x ;
						
////  				if ( checkForMenuEncoderLong( event ) )
////					{
////						if ( ( md2->srcRaw) && ( md2->srcRaw <= 4 ) )
////						{
////							SingleExpoChan = 1 ;
////							TextResult = 0 ;
////							s_expoChan = md2->srcRaw-1 ;
////        			pushMenu(menuProcExpoAll);
////						}
////					}
				}
			}
      break ;
      case 1:
			{	
        lcd_puts_P(  2*FW,y,PSTR(STR_2WEIGHT));
				md2->weight = gvarMenuItem( FW*20, y, md2->weight, -125, 125, attr | GVAR_250, event ) ;
			}
      break ;
      case 2:
			{	
        lcd_puts_P( FW, y, PSTR(STR_OFFSET) ) ;
				md2->sOffset = gvarMenuItem( FW*20, y, md2->sOffset, -125, 125, attr | GVAR_250, event ) ;
			}
      break ;
////      case 3:
////				md2->lateOffset = onoffMenuItem( md2->lateOffset, y, PSTR(STR_2FIX_OFFSET), attr ) ;
////      break;
      case 3:
				if ( ( md2->srcRaw <=4 ) )
				{
    			lcd_puts_Pleft( y, PSTR(STR_ENABLEEXPO) ) ;
					md2->disableExpoDr = offonItem( md2->disableExpoDr, y, attr ) ;
				}
				else
				{
					md2->disableExpoDr = onoffMenuItem( md2->disableExpoDr, y, XPSTR("\001Use Output   "), attr ) ;
				}
      break;
      case 4:
					md2->carryTrim = offonMenuItem( md2->carryTrim, y, PSTR(STR_2TRIM), attr ) ;
      break ;
        
			case 5:
				{	
	        lcd_putsAtt(  1*FW, y, PSTR(STR_Curve), ( md2->differential == 0) ? attr : 0 ) ;
	        lcd_putsAtt(  1*FW, y, PSTR(STR_15DIFF), ( md2->differential == 1) ? attr : 0 ) ;
	        lcd_putsAtt(  1*FW, y, XPSTR("\021Expo"), ( md2->differential == 2) ? attr : 0 ) ;
					uint8_t oldvalue =  md2->differential ;
    		  if(attr) CHECK_INCDEC_H_MODELVAR(  md2->differential, 0, 2 ) ;
					if ( md2->differential != oldvalue )
					{
						if ( md2->differential == 2 )		// expo
						{
							if ( ( md2->curve < -100 ) || ( md2->curve > 100 ) )
							{
								md2->curve = 0 ;
							}
						}
						else if ( md2->differential == 0 )		// curve
						{
							if ( ( md2->curve < -MAX_CURVE5-MAX_CURVE9-1-1-1 ) || ( md2->curve > MAX_CURVE5+MAX_CURVE9+7-1+1+1+1 ) )
							{
								md2->curve = 0 ;
							}
						}
//						else	// diff
//						{
//							if ( ( md2->curve < -100 ) || ( md2->curve > 100 ) )
//							{
//								md2->curve = 0 ;
//							}
//						}
					}
				}
      break ;

      case 6:
					if ( md2->differential == 1 )
					{	
		        md2->curve = gvarMenuItem( 12*FW, y, md2->curve, -100, 100, attr /*( m_posHorz==1 ? attr : 0 )*/, event ) ;
					}
					else
					{
						if ( md2->differential == 2 )
						{
            	lcd_outdezAtt(FW*17,y,md2->curve,attr|LEFT);
            	if(attr) CHECK_INCDEC_H_MODELVAR( md2->curve, -100, 100 ) ;
						}
						else
						{
							put_curve( 2*FW, y, md2->curve, attr ) ;
          	  if(attr)
							{
        		  	CHECK_INCDEC_H_MODELVAR( md2->curve, -MAX_CURVE5-MAX_CURVE9-1-1-1, MAX_CURVE5+MAX_CURVE9+7-1+1+1+1);
								if ( event==EVT_KEY_LONG(KEY_MENU) )
								{
									if ( md2->curve>=CURVE_BASE )
									{
          	  	    s_curveChan = md2->curve-CURVE_BASE;
	          		    pushMenu(menuCurveOne);
									}
									if ( md2->curve < 0 )
									{
          	  	    s_curveChan = -md2->curve-1 ;
	          		    pushMenu(menuCurveOne);
									}
								}
          	  }
        		}
					}
      break ;

      case 7:
          lcd_puts_P( 2*FW, y, PSTR(STR_2SWITCH) ) ;
          putsDrSwitches(13*FW, y, md2->swtch, attr) ;
          if(attr) CHECK_INCDEC_MODELSWITCH( md2->swtch, -MaxSwitchIndex, MaxSwitchIndex) ;
      break;

      case 8:
				{	
					uint8_t b = 1 ;
					lcd_puts_P( FW, y, PSTR(STR_MODES) ) ;
//////            lcd_puts_Pleft( y,XPSTR("\001MODES"));
						
					if ( attr )
					{
						Columns = 6 ;
					}
  					
					for ( uint8_t p = 0 ; p<MAX_MODES+1 ; p++ )
					{
						uint8_t z = md2->modeControl ;
    				lcd_putcAtt( (9+p)*(FW+1), y, '0'+p, ( z & b ) ? 0 : INVERS ) ;
						if( attr && ( g_posHorz == p ) )
						{
							lcd_rect( (9+p)*(FW+1)-1, y-1, FW+2, 9 ) ;
							if ( event==EVT_KEY_BREAK(KEY_MENU) ) //|| event==EVT_KEY_BREAK(BTN_RE) ) 
							{
								md2->modeControl ^= b ;
      					eeDirty(EE_MODEL) ;
    						s_editMode = false ;
							}
						}
						b <<= 1 ;
					}
				}
      break ;

////      case 9:
////          lcd_puts_P(  2*FW,y,PSTR(STR_2WARNING));
////					b = md2->mixWarn ;
////          if(b)
////              lcd_outdezAtt(FW*13,y,b,attr|LEFT);
////          else
////              lcd_putsAtt(  FW*13,y,PSTR(STR_OFF),attr);
////          if(attr) { CHECK_INCDEC_H_MODELVAR_0( b, 3); md2->mixWarn = b ; }
////      break;
      case 10:
          lcd_puts_P(  2*FW,y,PSTR(STR_2MULTIPLEX));
          lcd_putsAttIdx(13*FW, y,PSTR(STR_ADD_MULT_REP),md2->mltpx,attr);
          if(attr) CHECK_INCDEC_H_MODELVAR( md2->mltpx, 0, 2); //!! bitfield
      break;
      case 11:
          lcd_puts_P(  2*FW,y,PSTR(STR_2DELAY_DOWN));
					md2->delayUp = editSlowDelay( y, attr, md2->delayUp ) ;	// Sense was wrong
      break;
      case 12:
          lcd_puts_P(  2*FW,y,PSTR(STR_2DELAY_UP));
					md2->delayDown = editSlowDelay( y, attr, md2->delayDown ) ;	// Sense was wrong
      break;
      case 13:
          lcd_puts_P(  2*FW,y,PSTR(STR_2SLOW_DOWN));
					md2->speedDown = editSlowDelay( y, attr, md2->speedDown ) ;
      break;
      case 14:
          lcd_puts_P(  2*FW,y,PSTR(STR_2SLOW_UP));
					md2->speedUp = editSlowDelay( y, attr, md2->speedUp ) ;
      break;
    }
  }
}

void mixpopup( uint8_t event )
{
	uint8_t popaction = doPopup( PSTR(STR_MIX_POPUP), 0x7F, 11, event ) ;
	
  if ( popaction == POPUP_SELECT )
	{
		uint8_t popidx = PopupData.PopupSel ;
		if ( popidx == 1 )
		{
      if ( !reachMixerCountLimit())
      {
				s_currMixInsMode = 1 ;
      	insertMix(++s_curItemIdx, 0 ) ;
  	    s_moveMode=false;
			}
		}
		if ( popidx < 2 )
		{
	    pushMenu(menuMixOne) ;
		}
		else if ( popidx == 4 )		// Delete
		{
			mixToDelete = s_curItemIdx;
			killEvents(event);
			Tevent = 0 ;
			pushMenu(menuDeleteMix);
		}
		else if ( popidx == 5 )		// Clear all
		{
			mixToDelete = 0xFF ;
			killEvents(event);
			Tevent = 0 ;
			pushMenu(menuDeleteMix);
		}
		else if ( popidx == 6 )		// Templates
		{
			pushMenu( menuTemplates ) ;
		}
		else
		{
			if( popidx == 2 )	// copy
			{
     		insertMix(++s_curItemIdx, 1 ) ;
			}
			// PopupIdx == 2 or 3, copy or move
			s_moveMode = 1 ;
		}
		PopupData.PopupActive = 0 ;
	}
	s_moveItemIdx = s_curItemIdx ;

}

void displayVoiceRate( uint8_t x, uint8_t y, uint8_t rate, uint8_t attr )
{
	if ( ( rate >= 4 ) && ( rate <=32 ) )
	{
    lcd_outdezAtt(x+4*FW,y,rate-2, attr ) ;
	}
	else
	{
		if ( rate > 32 )
		{
			rate -= 29 ;
		}
		lcd_putsAttIdx( x, y, XPSTR("\004  ON OFFBOTH ALLONCE"),rate, attr ) ;
	}
}

VoiceAlarmData *voiceAddress( uint32_t index, uint8_t mode )
{
  if ( index >= NUM_VOICE_ALARMS )
	{
		index -= NUM_VOICE_ALARMS ;
		mode = 1 ;
	}	
	return mode ? &g_eeGeneral.gvad[index] : &g_model.vad[index] ;
}

static void deleteVoice(VoiceAlarmData *pvad)
{
	memset( (uint8_t *)pvad, 0, sizeof(*pvad) ) ;
}

void menuPasteVoice(uint8_t event)
{
	uint8_t action ;
	action = yesNoMenuExit( event, XPSTR("Paste Voice Alert?") ) ;
  
	switch( action )
	{
    case YN_YES :
		{	
			*voiceAddress( s_currIdx, 0 ) = Clipboard.clipvoice ;
//			MuteTimer = 5 ;
		}
      //fallthrough
		case YN_NO :
//      pushMenu(menuProcMix);
    break;
  }
}


void menuDeleteVoice(uint8_t event)
{
	uint8_t action ;
	action = yesNoMenuExit( event, XPSTR("Delete Voice Alert?") ) ;
  
	switch( action )
	{
    case YN_YES :
     	deleteVoice(voiceAddress( s_currIdx, 0 )) ;
      //fallthrough
		case YN_NO :
//      pushMenu(menuProcMix);
    break;
  }
}

void moveVoice(uint8_t idx, uint8_t dir, uint8_t mode) //true=inc=down false=dec=up
{
	VoiceAlarmData *psrc ;
	VoiceAlarmData *pdest ;
	uint32_t rows = mode ? NUM_GLOBAL_VOICE_ALARMS-1 : NUM_VOICE_ALARMS-1 ;
	psrc = voiceAddress( idx, mode ) ;
	if(idx==0 && !dir)
	{
		return ;
	}
	if( idx > rows || (idx == rows && dir ) )
	{
		return ;
	}
	uint8_t tidx = dir ? idx+1 : idx-1;
	pdest = voiceAddress( tidx, mode ) ;
  //flip between idx and tgt
  memswap( pdest, psrc, sizeof(VoiceAlarmData) ) ;
	s_curItemIdx = s_moveItemIdx = tidx ;
//	MuteTimer = 5 ;

	if ( mode )
	{
//  	STORE_GENERALVARS;
	}
	else
	{
//		STORE_MODELVARS;
	}
}

void voicepopup( uint8_t event )
{
	uint8_t mask = 0x1D ;
	if ( Clipboard.content == CLIP_VOICE )
	{
		mask = 0x9D ;
	}
	uint8_t popaction = doPopup( PSTR(STR_MIX_POPUP), mask, 11, event ) ;	// Edit,copy,move,Delete
  if ( popaction == POPUP_SELECT )
	{
		uint8_t popidx = PopupData.PopupSel ;
		if ( popidx == 0 )	// Edit
		{
	    pushMenu(menuVoiceOne) ;
		}
		else if ( popidx == 3 )	// Move
		{
			s_moveMode = 1 ;
		}
		else if ( popidx == 2 )	// Copy
		{
			VoiceAlarmData *pvad ;
			pvad = voiceAddress( s_currIdx, 0 ) ;
			Clipboard.clipvoice = *pvad ;
			Clipboard.content = CLIP_VOICE ;
		}
		else if ( popidx == 4 )	// Delete
		{
			killEvents(event);
			Tevent = 0 ;
			pushMenu(menuDeleteVoice);
		}
		else if ( popidx == 7 )	// Paste
		{
			killEvents(event);
			Tevent = 0 ;
			pushMenu(menuPasteVoice) ;
		}
		PopupData.PopupActive = 0 ;
	}
	s_moveItemIdx = s_curItemIdx ;
}

void menuSelectVoiceFile(uint8_t event)
{
	struct fileControl *fc = &FileControl ;
	uint32_t i ;

  TITLE( "Select File" ) ;

  switch(event)
	{
    case EVT_ENTRY:
		{
			char* directory ;
//			i = 0 ;
			directory = "/voice/user" ;
			if ( VoiceFileType == VOICE_FILE_TYPE_NAME )
			{
				directory = "/voice/modelNames" ;
			}
			else if ( VoiceFileType == VOICE_FILE_TYPE_SYSTEM )
			{
				directory = "/voice/system" ;
			}
//			else if ( VoiceFileType == VOICE_FILE_TYPE_MUSIC )
//			{
//				directory = (TCHAR *)"\\music" ;
//				if ( g_eeGeneral.musicType )
//				{
//					i = 1 ;
//				}
//			}
				 
//			setupFileNames( directory, fc, i ? (char *)"\0" : (char *)"WAV" ) ;
			setupFileNames( directory, fc, (char *)"WAV" ) ;
		}
    break ;
	}
	
	i = fileList( event, &FileControl ) ;
	if ( i == 1 )	// Select
	{
		cpystr( (uint8_t *)SelectedVoiceFileName, (uint8_t *) FileList.Filenames[fc->vpos] ) ; 
		killEvents(event) ;
    popMenu() ;
	}
	else if ( i == 2 )	// EXIT
	{
    killEvents(event) ;
    popMenu() ;
	}
	else if ( i == 3 )	// Tag
	{
		if ( VoiceFileType != VOICE_FILE_TYPE_MUSIC )
		{		
			char name[12] ;
  	  killEvents(event) ;
			copyFileName( name, FileList.Filenames[fc->vpos], 8 ) ;
			putNamedVoiceQueue( name, (VoiceFileType == VOICE_FILE_TYPE_NAME) ? VLOC_MNAMES : 
									(VoiceFileType == VOICE_FILE_TYPE_SYSTEM) ? VLOC_SYSTEM :	VLOC_USER ) ;
		}
		else
		{
			cpystr( (uint8_t *)SelectedVoiceFileName, (uint8_t *)FileList.Filenames[fc->vpos] ) ;
			killEvents(event) ;
    	popMenu() ;
			i = 1 ;
		}
	}
	else if ( i == 0 )	// No Files
	{
    if ( event == EVT_KEY_BREAK(KEY_EXIT) )
//		if ( checkForExitEncoderLong( event ) )
		{
			killEvents(event) ;
  	  popMenu(false) ;
		}
	}
	FileSelectResult = i ;
}

// FUnctions need to include ON, OFF and BOTH possibly
void menuVoiceOne(uint8_t event)
{
	TITLE( PSTR( STR_Voice_Alarm ));
	static MState2 mstate2;
	VoiceAlarmData *pvad ;
  if ( s_currIdx >= NUM_VOICE_ALARMS )
	{
		EditType = EE_GENERAL ;
		uint8_t z = s_currIdx - ( NUM_VOICE_ALARMS ) ;
		pvad = &g_eeGeneral.gvad[z] ;
		lcd_putc( 12*FW, 0, 'G' ) ;
		lcd_outdezAtt( 14*FW-1, 0, z+1, 0 ) ;
	}
	else
	{
		pvad = &g_model.vad[s_currIdx] ;
		lcd_outdezAtt( 13*FW, 0, s_currIdx+1, 0 ) ;
	}
	uint32_t rows = pvad->fnameType ? 11-1: 10-1 ;
#ifndef MOVE_VOICE
	if ( Clipboard.content == CLIP_VOICE )
	{
		rows += 1 ;
	}
#endif
	event = mstate2.check_columns( event, rows ) ;
	int8_t sub = mstate2.m_posVert ;
	
	if ( event == EVT_ENTRY )
	{
//		RotaryState = ROTARY_MENU_UD ;
	}
	else if ( event == EVT_ENTRY_UP )
	{
		if ( sub == 0 )
		{
			// Returned from editing
			if ( TextResult )
			{
				pvad->source = TextIndex ;
	  	  eeDirty(EE_MODEL) ;
			}
		}
		else
		{
		 	// From menuProcSelectUvoiceFile
			if ( FileSelectResult == 1 )
			{
		 		copyFileName( (char *)pvad->file.name, SelectedVoiceFileName, 8 ) ;
	   		eeDirty(EE_MODEL) ;		// Save it
			}
		}
	}

	
  for(uint8_t i=0; i<=rows; i++)
  {
		uint8_t j = i ;
		uint8_t y = (i+1)*FH;
    uint8_t attr ;

#define V1P1		6

		if ( pvad->fnameType == 0 )
		{
			if ( j >= V1P1+4 )
			{
				j += 1 ;
			}
		}
    attr = sub==i ? InverseBlink : 0 ;

#ifndef BIG_SCREEN
		if ( sub < V1P1 )
		{
			displayNext() ;
#endif
	    switch(j)
			{
    	  case 0 :	// source
  	  		lcd_puts_Pleft( FH, XPSTR("Source") ) ;
					putsChnRaw( 16*FW, y, pvad->source, attr ) ;
					if ( attr )
					{
						uint8_t x = pvad->source ;
//						CHECK_INCDEC_H_MODELVAR( x, 0,NUM_SKYXCHNRAW+NUM_TELEM_ITEMS+NumExtraPots ) ;
						x = checkIncDec16( x,0,NUM_SKYXCHNRAW+NUM_TELEM_ITEMS-1+1,EditType);
						pvad->source = x ;
//						if ( ( event == EVT_KEY_LONG(KEY_MENU) ) || ( event == EVT_KEY_BREAK(BTN_RE) ) )
						if ( event == EVT_KEY_LONG(KEY_MENU) )
						{
							// Long MENU pressed
							TextIndex = pvad->source ;
  					  TextType = 1 ;
  					  killEvents(event) ;
							pushMenu(menuTextHelp) ;
						}
					}
					if ( pvad->source )
					{
						int16_t value ;
						value = getValue( pvad->source - 1 ) ;
  	  			lcd_puts_Pleft( FH, XPSTR("\007(\016)") ) ;
    				if ( pvad->source > CHOUT_BASE+NUM_SKYCHNOUT )
 						{
							putsTelemetryChannel( 12*FW, FH, pvad->source-CHOUT_BASE-NUM_SKYCHNOUT-1, value, 0, TELEM_NOTIME_UNIT | TELEM_UNIT ) ;
						}
						else
						{
							lcd_outdezAtt( 12*FW, FH, value, 0 ) ;
						}
					}
				break ;

				case 1 :	// func;
  	  		lcd_puts_Pleft( y, XPSTR("Function") ) ;
					lcd_putsAttIdx( 13*FW, y, XPSTR("\007-------v>val  v<val  |v|>val|v|<valv\140=val v=val  v & val|d|>valv%val=0"), pvad->func, attr ) ;	// v1>v2  v1<v2  
	    		if(attr)
					{
      	    CHECK_INCDEC_H_MODELVAR( pvad->func, 0, 9 ) ;
					}	
				break ;

				case 2 :
				{	
  	  		lcd_puts_Pleft( y, XPSTR("Value") ) ;
    			if ( pvad->source > CHOUT_BASE+NUM_SKYCHNOUT )
		 			{
						putsTelemetryChannel( 20*FW, y, pvad->source-CHOUT_BASE-NUM_SKYCHNOUT-1, pvad->offset, attr, TELEM_NOTIME_UNIT | TELEM_UNIT | TELEM_CONSTANT ) ;
					}
					else
					{
						lcd_outdezAtt( FW*20, y, pvad->offset, attr ) ;
					}
					if ( attr )
					{
						pvad->offset = checkIncDec16( pvad->offset, -32000, 32000, EE_MODEL ) ;
					}
				}
				break ;
			
				case 3 :	 // swtch ;
  	  		lcd_puts_Pleft( y, XPSTR("Switch") ) ;
   	  		putsDrSwitches(16*FW, y, pvad->swtch, attr);
	    		if(attr)
					{
      	    CHECK_INCDEC_MODELSWITCH( pvad->swtch, -MaxSwitchIndex, MaxSwitchIndex+1 ) ;
    			}
				break ;

				case 4 :	 // rate ;
  	  		lcd_puts_Pleft( y, XPSTR("Trigger") ) ;
					displayVoiceRate( 16*FW, y, pvad->rate, attr ) ;

	    		if(attr)
					{
      	    CHECK_INCDEC_H_MODELVAR( pvad->rate, 0, 33 ) ;
					}	
				break ;

				case 5 :	 // haptic ;
  	  		lcd_puts_Pleft( y, PSTR( STR_HAPTIC ) ) ;
					lcd_putsAttIdx( 13*FW, y, XPSTR("\007-------Haptic1Haptic2Haptic3"), pvad->haptic, attr ) ;
	    		if(attr)
					{
      	    CHECK_INCDEC_H_MODELVAR( pvad->haptic, 0, 3 ) ;
					}
				break ;
#ifndef BIG_SCREEN
			}

		}
		else
		{
			y = (i-(V1P1-1))*FH ;
	    switch(j)
			{
#endif
				case V1P1 :	 // delay
  	  		lcd_puts_Pleft( y, XPSTR("On Delay") ) ;
  				lcd_outdezAtt(FW*18-3,y,pvad->delay,attr|PREC1);
	    		if(attr)
					{
      	    CHECK_INCDEC_H_MODELVAR( pvad->delay, 0, 50 ) ;
					}
				break ;
				
				case V1P1+1 :	 // vsource:2
  	  		lcd_puts_Pleft( y, XPSTR("Play Source") ) ;
					lcd_putsAttIdx( 14*FW, y, XPSTR("\006No    BeforeAfter "), pvad->vsource, attr ) ;
	    		if(attr)
					{
      	    CHECK_INCDEC_H_MODELVAR( pvad->vsource, 0, 2 ) ;
					}
				break ;

				case V1P1+2 :
  	  		lcd_puts_Pleft( y, XPSTR("On no Telemetry") ) ;
					lcd_putsAttIdx( 17*FW, y, XPSTR("\004PlayMute"), pvad->mute, attr ) ;
	    		if(attr)
					{
      	    CHECK_INCDEC_H_MODELVAR( pvad->mute, 0, 1 ) ;
					}
				break ;

				case V1P1+3 :	 // fnameType:3 ;
				{	
  	  		lcd_puts_Pleft( y, XPSTR("FileType") ) ;
					lcd_putsAttIdx( 14*FW, y, XPSTR("\007-------   NameGV/SC/# EffectSysName"),pvad->fnameType,attr ) ;
					uint8_t previous = pvad->fnameType ;
	    		if(attr)
					{
      	    CHECK_INCDEC_H_MODELVAR( pvad->fnameType, 0, 4 ) ;
					}
					if ( pvad->fnameType != previous )
					{
						if ( pvad->fnameType )
						{
							pvad->file.name[0] = 0 ;
							pvad->file.name[1] = 0 ;
						}
						else
						{
							pvad->file.vfile = 0 ;
						}
					}
				}
				break ;
			
				case V1P1+4 :	 // filename ;
//				if ( pvad->fnameType )
//				{	
  	  		lcd_puts_Pleft( y, pvad->fnameType == 3 ? XPSTR("Sound effect") : XPSTR("Voice File") ) ;
					if ( ( pvad->fnameType == 1 ) || ( pvad->fnameType == 4 ) )	// Name
					{
						if ( pvad->file.name[0] == 0 )
						{
							memset( (uint8_t *)pvad->file.name, ' ', sizeof(pvad->file.name) ) ;
						}
						// Need to edit name here
						alphaEditName( 12*FW, y, (uint8_t *)pvad->file.name, sizeof(pvad->file.name), attr | ALPHA_NO_NAME, (uint8_t *)XPSTR( "FileName") ) ;
						validateName( pvad->file.name, sizeof(pvad->file.name) ) ;
	  				if( attr )
						{
							if ( checkForMenuEncoderLong( event ) )
							{
								VoiceFileType = ( pvad->fnameType == 4 ) ? VOICE_FILE_TYPE_SYSTEM : VOICE_FILE_TYPE_USER ;
      				 	pushMenu( menuSelectVoiceFile ) ;
							}
						} 
					}
					else if ( pvad->fnameType == 2 )	// Number
					{
						uint16_t value = pvad->file.vfile ;
						if ( value )
						{
							if ( value > 500 )
							{
								value -= 500 ;
							}
							else
							{
								value += 15 ;
							}
						}
	  	  		if(attr)
						{
	  	    		value = checkIncDec16( value, 0, 515, EE_MODEL);
						}
						if ( value )
						{
							if ( value > 15 )
							{
								value -= 15 ;
							}
							else
							{
								value += 500 ;
							}
						}
						pvad->file.vfile = value ;
						if ( pvad->file.vfile > 500)
						{
							if ( pvad->file.vfile > 507 )
							{
								lcd_putsAtt( 18*FW, y, XPSTR("SC"), attr ) ;
								lcd_putcAtt( 20*FW, y, pvad->file.vfile - 507 + '0', attr ) ;
							}
							else
							{
								dispGvar( 18*FW, y, pvad->file.vfile - 500, attr ) ;
							}
  					}
  					else
						{
	      	    lcd_outdezAtt( FW*20, y, pvad->file.vfile, attr ) ;
  					}
						if (attr)
						{
							if ( checkForMenuEncoderLong( event ) )
							{
								if ( pvad->file.vfile <= 500)
								{
									putVoiceQueue( pvad->file.vfile | VLOC_NUMUSER  ) ;
								}
							}
						}
					}
					else if ( pvad->fnameType == 3 )	// Audio
					{
						lcd_putsAttIdx(15*FW, y, PSTR(STR_SOUNDS), pvad->file.vfile, attr ) ;
		  	  	if(attr)
						{
							CHECK_INCDEC_H_MODELVAR( pvad->file.vfile, 0, 16 ) ;
						}
					}
				break ;
//				}
				
#ifndef MOVE_VOICE
				case V1P1+5 :	 // Blank ;
  	  		lcd_puts_Pleft( y, XPSTR("Delete") ) ;
					lcd_putsAtt( 12*FW, y, XPSTR("MENU LONG"), attr ) ;
  				if( attr )
					{
						if ( checkForMenuEncoderLong( event ) )
						{
				     	deleteVoice( pvad ) ;
//							if ( i == 10 )
//							{
								mstate2.m_posVert = 0 ;
//							}
	    				eeDirty(EE_MODEL) ;
						}
					}
				break ;
			
				case V1P1+6 :	 // Copy ;
  	  		lcd_puts_Pleft( y, XPSTR("Copy") ) ;
	    		if(attr)
					{
						lcd_char_inverse( 0, y, 24, 0 ) ;
						if ( checkForMenuEncoderLong( event ) )
						{
							Clipboard.clipvoice = *pvad ;
							Clipboard.content = CLIP_VOICE ;
						}
					}
				break ;

				case V1P1+7 :	 // Paste ;
  	  		lcd_puts_Pleft( y-FH, XPSTR("\012Paste") ) ;
	    		if(attr)
					{
						lcd_char_inverse( 60, y-FH, 30, 0 ) ;
						if ( checkForMenuEncoderLong( event ) )
						{
							*pvad = Clipboard.clipvoice ;
							MuteTimer = 5 ;
						}
					}
				break ;
#endif
#ifndef BIG_SCREEN
			}
#endif
		}
	}
}

#define VOI_OFF_0			0

// mode = 0, model alarms
// mode = 1, global alarms
void menuVoice(uint8_t event, uint8_t mode)
{
	uint32_t rows = mode ? NUM_GLOBAL_VOICE_ALARMS : NUM_VOICE_ALARMS + 1 + 1 ;
	TITLE(PSTR(STR_Voice_Alarms)) ;
	static MState2 mstate2 ;
	
#ifdef MOVE_VOICE
	if ( s_moveMode )
	{
//		int8_t moveByRotary ;
//		moveByRotary = qRotary() ;		// Do this now, check_simple destroys rotary data
//		if ( moveByRotary )
//		{
//			if ( moveByRotary > 0 )
//			{
//				event = EVT_KEY_FIRST(KEY_DOWN) ;
//			}
//			else
//			{
//				event = EVT_KEY_FIRST(KEY_UP) ;
//			}
//		}
		uint8_t v = mstate2.m_posVert ;
		if ( ( ( v == 0 ) && ( event == EVT_KEY_FIRST(KEY_UP) ) ) 
				 || ( ( v == (mode ? NUM_GLOBAL_VOICE_ALARMS - 1 : NUM_VOICE_ALARMS - 1 ) ) && ( event == EVT_KEY_FIRST(KEY_DOWN) ) ) )
		{
			event = 0 ;
		}
		Tevent = event ;
	}
	
	if ( !PopupData.PopupActive )
	{
		mstate2.check_columns(event, rows - 1 ) ;
	}
	
#else	
	mstate2.check_columns(event, rows - 1 ) ;
#endif

  int8_t sub = mstate2.m_posVert ;

	evalOffset( sub ) ;

#ifdef BIG_SCREEN
	DisplayOffset = VOI_OFF_0 ;
#endif

  switch (event)
	{
#ifdef MOVE_VOICE
	    case EVT_ENTRY_UP:
	    case EVT_ENTRY:
        s_moveMode = false ;
  	  break;
#endif
    case EVT_KEY_FIRST(KEY_MENU) :
//    case EVT_KEY_BREAK(BTN_RE) :
	    if(sub == NUM_VOICE_ALARMS )
			{
    	  pushMenu(menuGlobalVoiceAlarm) ;
	 	    killEvents(event);
			}
			else if( sub < NUM_VOICE_ALARMS )
			{
	      s_currIdx = sub + (mode ? NUM_VOICE_ALARMS : 0) ;
#ifdef MOVE_VOICE
				s_curItemIdx = s_currIdx ;
				if ( s_moveMode )
				{
	  	  	s_moveMode = false ;
  	  		s_editMode = false ;
//					RotaryState = ROTARY_MENU_UD ;
		 	    killEvents(event);
  	  		break;
				}
				// Else fall through    
				if ( !PopupData.PopupActive )
				{
					PopupData.PopupIdx = 0 ;
					PopupData.PopupActive = 1 ;
 			    killEvents(event);
					event = 0 ;		// Kill this off
				}
#else
    	  pushMenu(menuProcVoiceOne) ;
	 	    killEvents(event);
#endif
			}
		break;
  }

#ifdef MOVE_VOICE
	if ( s_moveMode )
	{
		int8_t dir ;
		uint8_t xevent = event ;
		if ( event == EVT_KEY_REPT(KEY_DOWN) )
		{
			xevent = EVT_KEY_FIRST(KEY_DOWN) ;
		}
		if ( event == EVT_KEY_REPT(KEY_UP) )
		{
			xevent = EVT_KEY_FIRST(KEY_UP) ;
		}
		
		if ( ( dir = (xevent == EVT_KEY_FIRST(KEY_DOWN) ) ) || xevent == EVT_KEY_FIRST(KEY_UP) )
		{
			moveVoice( s_curItemIdx, dir, mode ) ; //true=inc=down false=dec=up - Issue 49
			if ( mode == 0 )
			{
				if ( sub >= NUM_VOICE_ALARMS )
				{
					sub = mstate2.m_posVert = NUM_VOICE_ALARMS - 1 ;
				}
			}
		}
	}
#endif

	uint8_t k = 0 ;
  uint8_t y = 1*FH ;
	for( uint8_t i = 0 ; i < ( mode ? 8 : SCREEN_LINES-1) ; i += 1 )
//  for (uint8_t i = 0 ; i < 7 ; i += 1 )
	{
    k = i + s_pgOfs ;
    uint8_t attr = sub == k ? INVERS : 0 ;
		VoiceAlarmData *pvad ;
      
//		if(y>7*FH) break ;

	  if ( k < NUM_VOICE_ALARMS )
		{
    	if ( mode )
	//		k >= NUM_VOICE_ALARMS + NUM_EXTRA_VOICE_ALARMS )
			{
	//			uint8_t z = k - ( NUM_VOICE_ALARMS + NUM_EXTRA_VOICE_ALARMS ) ;
				lcd_puts_Pleft( y, XPSTR("GVA") ) ;
				lcd_putc( 3*FW+VOI_OFF_0, y, '1' + k ) ;
				pvad = &g_eeGeneral.gvad[k] ;
			}
			else
			{
				lcd_puts_Pleft( y, XPSTR("VA") ) ;
  		  lcd_outdezAtt( (k<9) ? FW*3-1+VOI_OFF_0 : FW*4-2+VOI_OFF_0, y, k+1, 0 ) ;
				pvad = &g_model.vad[k] ;
			}
			putsChnRaw( 5*FW+VOI_OFF_0, y, pvad->source, 0 ) ;
			putsDrSwitches( 9*FW+VOI_OFF_0, y, pvad->swtch, 0 ) ;
			displayVoiceRate( 13*FW+VOI_OFF_0, y, pvad->rate, 0 ) ;
    	switch ( pvad->fnameType )
			{
				case 1 :
					lcd_putc( 19*FW+VOI_OFF_0, y, 'N' ) ;
				break ;
				case 2 :
					lcd_putc( 19*FW+VOI_OFF_0, y, '#' ) ;
				break ;
				case 3 :
					lcd_putc( 19*FW+VOI_OFF_0, y, 'A' ) ;
				break ;
				case 4 :
					lcd_putc( 19*FW+VOI_OFF_0, y, 'S' ) ;
				break ;
			}
		
			if (pvad->haptic)
			{
				lcd_putc( 20*FW+VOI_OFF_0, y, 'H' ) ;
			}
			if ( attr )
			{
#ifdef MOVE_VOICE
				if ( s_moveMode )
				{
					lcd_rect( 0+VOI_OFF_0, y, 127, 8 ) ;
				}
				else
				{
					lcd_char_inverse( 0+VOI_OFF_0, y, 20*FW, 0 ) ;
				}
#else
				lcd_char_inverse( 0+VOI_OFF_0, y, 20*FW, 0 ) ;
#endif
			}
		}
		else
		{
  		if( k == NUM_VOICE_ALARMS )
			{
  		  //last line available - add the global voice alarms line
  		  uint8_t attr = ( sub == NUM_VOICE_ALARMS ) ? INVERS : 0;
  		  lcd_putsAtt( 0+VOI_OFF_0, y, (char *)GvaString, attr ) ;
			}
  		else
			{
  		  lcd_puts_Pleft( y ,XPSTR("Flush Switch") ) ;
  		  uint8_t attr = ( sub == NUM_VOICE_ALARMS + 1 ) ? InverseBlink : 0;
				g_model.voiceFlushSwitch = edit_dr_switch( 17*FW+VOI_OFF_0, y, g_model.voiceFlushSwitch, attr, attr ? EDIT_DR_SWITCH_EDIT : 0, event ) ;
			}
		}
		y += FH ;
  }

#ifdef MOVE_VOICE
	s_curItemIdx = sub ;
	if ( PopupData.PopupActive )
	{
		Tevent = event ;
		voicepopup( event ) ;
    s_editMode = false;
	}
#endif
}

void menuVoiceAlarm(uint8_t event)
{
	EditType = EE_MODEL ;
	menuVoice( event, 0 ) ;
}

void menuGlobalVoiceAlarm(uint8_t event)
{
	EditType = EE_GENERAL ;
	menuVoice( event, 1 ) ;
}


















#define LIM_OFF_0			0
#define LIM_RECT_W		56

void menuLimits(uint8_t event)
{
	TITLE( "Limits" ) ;
	static MState2 mstate2;
	event = mstate2.check_columns(event, NUM_SKYCHNOUT+2-1-1) ;
	Columns = 3 ;
	
	uint16_t y = 0 ;
	uint8_t k = 0 ;
	uint8_t sub = mstate2.m_posVert ;
	uint8_t subSub = g_posHorz;
	uint16_t t_pgOfs ;

	t_pgOfs = evalOffset( sub ) ;

	if ( sub < NUM_SKYCHNOUT )
	{
    lcd_outdez( 13*FW, 0, g_chans512[sub]/2 + 1500 ) ;
	}

//	switch(event)
//	{
//    case EVT_KEY_LONG(KEY_MENU):
//      if(sub>=0 && sub<NUM_SKYCHNOUT+EXTRA_SKYCHANNELS)
//			{
//        int16_t v = g_chans512[sub - t_pgOfs];
//        LimitData *ld = &g_model.limitData[sub];
//        switch (subSub)
//				{
//         	case 0 :
//            ld->offset = (ld->revert) ? -v : v;
//            STORE_MODELVARS;
//          break ;
//        }
//      }
//			if(sub==NUM_SKYCHNOUT+EXTRA_SKYCHANNELS)
//			{
//  		  //last line available - add the "copy trim menu" line
//  		  s_noHi = NO_HI_LEN;
//  		  killEvents(event);
//  			s_editMode = 0 ;
//  		  setStickCenter(1); //if highlighted and menu pressed - copy trims
//			}
//    break;
//	}

  int8_t limit = (g_model.extendedLimits ? 125 : 100);
	for(uint32_t i=0; i<SCREEN_LINES-1; i++)
	{
    y=(i+1)*FH ;
    k=i+t_pgOfs ;
    if(k==NUM_SKYCHNOUT) break ;
    LimitData *ld = &g_model.limitData[k];
    int16_t v = g_chans512[k] - ( (ld->revert) ? -ld->offset : ld->offset) ;
		
    char swVal = '-';  // '-', '<', '>'
		if( v >  50) swVal = ld->revert ? 127 : 126 ;// Switch to raw inputs?  - remove trim!
    if( v < -50) swVal = ld->revert ? 126 : 127 ;
    putsChn(0,y,k+1,0) ;
    lcd_putc(12*FW+FW/2, y, swVal ) ; //'<' : '>'
    
		for(uint32_t j=0; j<4;j++)
		{
      uint8_t attr = ((sub==k && subSub==j) ? InverseBlink : 0);
			uint8_t active = (attr && s_editMode) ;
			int16_t value ;
			int16_t t = 0 ;
			if ( g_model.sub_trim_limit )
			{
				if ( ( t = ld->offset ) )
				{
					if ( t > g_model.sub_trim_limit )
					{
						t = g_model.sub_trim_limit ;
					}
					else if ( t < -g_model.sub_trim_limit )
					{
						t = -g_model.sub_trim_limit ;
					}
				}
			}
			value = t / 10 ;
      switch(j)
      {
        case 0:
          lcd_outdezAtt( 7*FW+3, y,  ld->offset, attr|PREC1);
          if(active)
					{
						StepSize = 50 ;
            ld->offset = checkIncDec16( ld->offset, -1000, 1000, EE_MODEL);
          }
        break;
        case 1:
					value += (int8_t)(ld->min-100) ;
					if ( value < -125 )
					{
						value = -125 ;						
					}
          lcd_outdezAtt( 12*FW, y, value, attr) ;
          if(active)
					{
            ld->min -=  100;
		        CHECK_INCDEC_H_MODELVAR( ld->min, -limit,25);
            ld->min +=  100;
          }
        break;
        case 2:
					value += (int8_t)(ld->max+100) ;
					if ( value > 125 )
					{
						value = 125 ;						
					}
          lcd_outdezAtt( 17*FW, y, value,    attr);
					if ( t )
					{
						lcd_rect( 9*FW-4+LIM_OFF_0, y-1, LIM_RECT_W, 9 ) ;
					}
          if(active)
					{
            ld->max +=  100;
            CHECK_INCDEC_H_MODELVAR( ld->max, -25,limit);
            ld->max -=  100;
          }
        break;
        case 3:
#ifdef BIG_SCREEN
					DisplayOffset = 2*FW ;
#endif
					ld->revert = hyphinvMenuItem( ld->revert, y, attr ) ;
        break;
      }
    }
	}
	if(k==NUM_SKYCHNOUT)
	{
    //last line available - add the "copy trim menu" line
    uint8_t attr = (sub==NUM_SKYCHNOUT) ? INVERS : 0;
		if ( attr )
		{
			Columns = 0 ;
		}
    lcd_putsAtt( 3*FW,y,PSTR(STR_COPY_TRIM),s_noHi ? 0 : attr);
	}
}


#define SAFE_OFF_0		0


void menuSafetySwitches(uint8_t event)
{
	
	TITLE(PSTR(STR_SAFETY_SW)) ;
	EditType = EE_MODEL ;
	static MState2 mstate2;
	event = mstate2.check_columns(event, NUM_SKYCHNOUT-1 ) ;

	uint8_t y = 0 ;
	uint8_t k = 0 ;
	int8_t  sub    = mstate2.m_posVert ;
	uint8_t subSub = g_posHorz ;
  uint16_t t_pgOfs ;

	t_pgOfs = evalOffset( sub ) ;

  lcd_outdez( 20*FW+SAFE_OFF_0, 0, g_chans512[sub]/2 + 1500 ) ;

	for(uint32_t i=0; i<SCREEN_LINES-1; i++)
	{
    y=(i+1)*FH ;
    k=i+t_pgOfs ;
  	X20SafetySwData *sd = &g_model.safetySw[k] ;
		if ( sub==k )
		{
			Columns = 3 ;
		}
   	uint8_t attr = (getSwitch00(sd->swtch) ) ? INVERS : 0 ;

   	putsChn(0+SAFE_OFF_0,y,k+1,attr);
  	for(uint8_t j=0; j<5;j++)
		{
    	attr = ((sub==k && subSub==j) ? InverseBlink : 0);
			uint8_t active = (attr && s_editMode) ;
  	  if (j == 0)
			{
				lcd_putsAttIdx( 5*FW-3+SAFE_OFF_0, y, XPSTR("\001SX"), sd->mode, attr ) ;
	      if(active)
				{
	  	    CHECK_INCDEC_H_MODELVAR( sd->mode, 0, 1 ) ;
  	  	}
			}
	    else if (j == 1)
  	  {
				int8_t max = MaxSwitchIndex ;
				{
         	putsDrSwitches(6*FW+SAFE_OFF_0, y, sd->swtch, attr);
				}
	    	if(active)
				{
          CHECK_INCDEC_MODELSWITCH( sd->swtch, -MaxSwitchIndex, max ) ;
    		}
			}
			else if ( j == 2 )
			{
				int8_t min, max ;
        lcd_putc( 14*FW+SAFE_OFF_0, y, '.' ) ;
				min = -125 ;
				max = 125 ;
        lcd_outdezAtt(  14*FW+SAFE_OFF_0, y, sd->val, attr);
// Option to display current channel value, before limits etc., for failsafe
  	  	if(active)
				{
		      CHECK_INCDEC_H_MODELVAR( sd->val, min, max);
    	  }
			}
			else if ( j == 3 )
			{
        lcd_outdezAtt(  15*FW+3+SAFE_OFF_0, y, sd->tune, attr);
	  	  if(active)
				{
			    CHECK_INCDEC_H_MODELVAR( sd->tune, 0, 9 ) ;
    		}
			}
			else
			{
				if ( sd->mode == 1 )
				{
					Columns = 4 ;
					int8_t temp = sd->source ;
					if ( temp == 0 )
					{
						temp = 3 ;
					}
					else
					{
						if ( temp > 0 )
						{
							temp += 4 ;
						}
						else
						{
							temp -= 4 ;
						}
					}
					if ( temp < 0 )
					{
						temp = - temp ;
						lcd_putc( 16*FW+SAFE_OFF_0, y, '!' ) ;
					}
					putsChnRaw( 17*FW+SAFE_OFF_0, y, temp, attr ) ;
	    	  if(active)
					{
						temp = sd->source ;
						CHECK_INCDEC_H_MODELVAR( temp, -5 , 5 ) ;
						sd->source = temp ;
    		  }
				}
			}
		}
	}
}




void editExpoVals(uint8_t event, uint8_t edit, uint8_t x, uint8_t y, uint8_t which, uint8_t exWt, uint8_t stkRL)
{
  uint8_t  invBlk = (edit) ? InverseBlink : 0 ;
	uint8_t doedit ;
	int8_t *ptr ;			// volatile forces compiler to produce 'better' code
	X20ExpoData *eptr ;

	doedit = edit ? EDIT_DR_SWITCH_EDIT | EDIT_DR_SWITCH_FMODE : EDIT_DR_SWITCH_FMODE ;

	eptr = &g_model.expoData[s_expoChan] ;
  
	if(which==DR_DRSW1)
	{
		eptr->drSw1 = edit_dr_switch( x, y, eptr->drSw1, invBlk, doedit, event ) ;
  }
  else if(which==DR_DRSW2)
	{
		eptr->drSw2 = edit_dr_switch( x, y, eptr->drSw2, invBlk, doedit, event ) ;
  }
  else
	{
		ptr = &eptr->expo[which][exWt][stkRL] ;
    if(exWt==DR_EXPO)
		{
			*ptr = gvarMenuItem( x, y, *ptr, -100, 100, invBlk, event ) ;
    }
    else
		{
			*ptr = gvarMenuItem( x, y, *ptr+100, 0, 100, invBlk, event ) - 100 ;
    }
	}
}

extern int16_t calcExpo( uint8_t channel, int16_t value ) ;


#define WCHART 29
#define X0     (128-WCHART-2 - 2 )
#define Y0     32
#define XD (X0-2)

#define GRAPH_FUNCTION_CURVE		0
#define GRAPH_FUNCTION_EXPO			1

void drawFunction( uint16_t xpos, uint8_t function )
{
  int8_t yv ;
  int8_t prev_yv = 127 ;
	
	pushPlotType( PLOT_BLACK ) ;
	for ( int8_t xv = -WCHART ; xv <= WCHART ; xv++ )
	{
		if ( function == GRAPH_FUNCTION_CURVE )
		{
    	yv = intpol(xv * RESX / WCHART, s_curveChan) * WCHART / RESX ;
		}
		else
		{
    	yv = (calcExpo( s_expoChan, xv * RESX / WCHART) * WCHART + RESX/2) / RESX ;
		}
    if (prev_yv == 127)
		{
			prev_yv = yv ;
		}
		uint16_t len = abs(yv-prev_yv) ;
    if (len <= 1)
		{
    	lcd_plot(xpos + xv, Y0 - yv) ;
		}
		else
		{
      uint8_t tmp = (prev_yv < yv ? 0 : len-1 ) ;
      lcd_vline(xpos+xv, Y0 - yv - tmp, len ) ;
		}	
		if ( yv )
		{
     	lcd_plot(xpos + xv, Y0 ) ;
		}
		prev_yv = yv ;
	}
	popPlotType() ;
}

void menuExpoAll( uint8_t event )
{
	TITLE(PSTR(STR_EXPO_DR));
	EditType = EE_MODEL ;
	static MState2 mstate2;
	uint32_t count = 5-1 ;
	if ( SingleExpoChan )
	{
		count -= 1 ;
	}
	event = mstate2.check_columns( event, count ) ;

	uint8_t stkVal ;
	uint8_t sub = mstate2.m_posVert ;
	int8_t subN ;
	if ( SingleExpoChan )
	{
		sub += 1 ;
	}
	uint8_t l_expoChan = s_expoChan ;
	subN = 0 ;
	{
    uint8_t attr = 0 ;
		if ( sub == subN )
		{
			if ( SingleExpoChan == 0 )
			{
				s_expoChan = l_expoChan = checkIncDec16( s_expoChan, 0, 3, 0 ) ;
				attr = InverseBlink ;
			}
		}		 
		putsChnRaw(0,FH,l_expoChan+1,attr) ;
	}
	
	uint8_t expoDrOn = get_dr_state(l_expoChan);
	switch (expoDrOn)
	{
    case DR_MID:
      lcd_puts_Pleft( FH,PSTR(STR_4DR_MID));
    break;
    case DR_LOW:
      lcd_puts_Pleft( FH,PSTR(STR_4DR_LOW));
    break;
    default: // DR_HIGH:
      lcd_puts_Pleft( FH,PSTR(STR_4DR_HI));
    break;
	}

	subN += 1 ;
	stkVal = DR_BOTH ;
	if(CalibratedStick[l_expoChan]> 100) stkVal = DR_RIGHT;
	if(CalibratedStick[l_expoChan]<-100) stkVal = DR_LEFT;
	if(IS_EXPO_THROTTLE(l_expoChan)) stkVal = DR_RIGHT;

	lcd_puts_Pleft(2*FH,PSTR(STR_2EXPO));
	editExpoVals( event, (stkVal != DR_RIGHT) && (sub==subN), 4*FW, 3*FH, expoDrOn ,DR_EXPO, DR_LEFT ) ;
	editExpoVals( event, (stkVal != DR_LEFT) && (sub==subN), 8*FW, 3*FH, expoDrOn ,DR_EXPO, DR_RIGHT ) ;

	subN += 1 ;
	lcd_puts_Pleft(4*FH,PSTR(STR_2WEIGHT));
	editExpoVals( event, (stkVal != DR_RIGHT) && (sub==subN), 4*FW, 5*FH, expoDrOn ,DR_WEIGHT, DR_LEFT ) ;
	editExpoVals( event, (stkVal != DR_LEFT) && (sub==subN), 8*FW, 5*FH, expoDrOn ,DR_WEIGHT, DR_RIGHT ) ;

	subN += 1 ;
	lcd_puts_Pleft(6*FH,PSTR(STR_DR_SW1));
	editExpoVals( event, sub==subN,5*FW, 6*FH, DR_DRSW1 , 0,0);
	subN += 1 ;
	lcd_puts_Pleft(7*FH,PSTR(STR_DR_SW2));
	editExpoVals( event, sub==subN,5*FW, 7*FH, DR_DRSW2 , 0,0);

	pushPlotType( PLOT_BLACK ) ;
	
	lcd_vline(XD - (IS_EXPO_THROTTLE(s_expoChan) ? WCHART : 0), Y0 - WCHART, WCHART * 2);
	
	drawFunction( XD, GRAPH_FUNCTION_EXPO ) ;

	int16_t x512  = CalibratedStick[s_expoChan];
	int16_t y512 = calcExpo( l_expoChan, x512 ) ;
  
	lcd_outdezAtt( 24*FW, 8*FH,x512*25/((signed) RESXu/4), 0 );
	lcd_outdezAtt( 18*FW, 1*FH,y512*25/((signed) RESXu/4), 0 );
	
	int16_t xv = (x512 * WCHART + RESX/2) / RESX + XD ;
  int16_t yv = Y0 - (y512 * WCHART + RESX/2) / RESX ;

	lcd_vline( xv, yv-6, 13 ) ;
	lcd_hline( xv-6, yv, 13 ) ;
	
	popPlotType() ;
}

void drawCurve( uint16_t offset )
{
  uint8_t cv9 = s_curveChan >= MAX_CURVE5 ;
 	int8_t *crv ;
	uint32_t points = cv9 ? 9 : 5 ;
 	crv = cv9 ? g_model.curves9[s_curveChan-MAX_CURVE5] : g_model.curves5[s_curveChan];
	if ( s_curveChan >= MAX_CURVE5 + MAX_CURVE9 )
	{
		if ( s_curveChan == MAX_CURVE5 + MAX_CURVE9 + MAX_CURVEXY )
		{
			crv = g_model.curve6 ;
			cv9 = 3 ;
			points = 6 ;
		}
		else
		{
			cv9 = 2 ;
			crv = g_model.curvexy[s_curveChan - (MAX_CURVE5 + MAX_CURVE9)] ;
		}
	}
	pushPlotType( PLOT_BLACK ) ;
	lcd_vline(XD, Y0 - WCHART, WCHART * 2);
  
	for(uint32_t i=0; i < points ; i++)
  {
    uint16_t xx ;
		if ( cv9 == 2 )
		{
    	xx = XD-1+crv[i+9]*WCHART/100 ;
		}
		else
		{
			if ( cv9 == 3 )
			{
				xx = XD-1-WCHART+i*WCHART*2/5 ;
			}
			else
			{
				xx = XD-1-WCHART+i*WCHART/(cv9 ? 4 : 2);
			}
		}
    uint16_t yy = Y0-crv[i]*WCHART/100;

    if( (offset==i) || (offset == i+9) )
    {
			lcd_rect( xx-1, yy-2, 5, 5 ) ;
    }
    else
		{
			lcd_rect( xx, yy-1, 3, 3 ) ;
    }
  }

	drawFunction( XD, GRAPH_FUNCTION_CURVE ) ;
	
	popPlotType() ;
}

void menuCurveOne(uint8_t event)
{
  uint8_t cv9 = s_curveChan >= MAX_CURVE5 ;
	uint32_t points = cv9 ? 9 : 5 ;
	if ( s_curveChan >= MAX_CURVE5 + MAX_CURVE9 )
	{
		cv9 = 2 ;
	}
	if ( s_curveChan == MAX_CURVE5 + MAX_CURVE9 + MAX_CURVEXY )
	{
		points = 6 ;
		cv9 = 3 ;
	}
	static int8_t dfltCrv;

	TITLE(PSTR(STR_CURVE)) ;
	static MState2 mstate2 ;
	mstate2.check_columns(event, ( cv9 == 2 ) ? 17 : points ) ;
		
	if ( event == EVT_ENTRY )
	{
		dfltCrv = 0 ;
	}
	lcd_outdezAtt(7*FW, 0, s_curveChan+1, INVERS);

	uint8_t  preset = points ;
	uint8_t  sub    = mstate2.m_posVert ;
	uint8_t blink = InverseBlink ;
	int8_t *crv ;
	if ( cv9 == 2 )
	{ // An xy curve
		crv = g_model.curvexy[s_curveChan - (MAX_CURVE5 + MAX_CURVE9)] ;
		uint32_t i ;
		uint32_t j ;
		uint32_t k ;
		j = sub > 9 ? 2 : 0 ;
		k = sub & 1 ;
		sub >>= 1 ;
		if ( k == 0 )
		{
			sub += 9 ;
		}
		for ( i = 0; i < 7; i++)
		{
  	  uint16_t y = i * FH + 8 ;
  	  uint8_t attr = (k==0) && (sub == j+i+9) ? blink : 0 ;
			lcd_outdezAtt(4 * FW, y, crv[j+i+9], attr);
  	  attr = (k==1) && (sub == j+i) ? blink : 0 ;
    	lcd_outdezAtt(8 * FW, y, crv[j+i], attr);
		}
		int8_t min = -100 ;
		int8_t max = 100 ;
		if ( k == 0) // x value
		{
			if ( sub > 9 )
			{
				min = crv[sub-1] ;
			}
			if ( sub < 17 )
			{
				max = crv[sub+1] ;
			}
		}
		CHECK_INCDEC_H_MODELVAR( crv[sub], min, max ) ;
		if ( sub > 8 )
		{
			sub -= 9 ;
		}
// Draw the curve
		drawCurve( sub ) ;
	}
	else
	{
		crv = cv9 ? g_model.curves9[s_curveChan-MAX_CURVE5] : g_model.curves5[s_curveChan];
		if ( s_curveChan == MAX_CURVE5 + MAX_CURVE9 + MAX_CURVEXY )
		{
			crv = g_model.curve6 ;
		}

		for (uint8_t i = 0; i < points; i++)
		{
  	  uint16_t y = i * FH + 16 ;
  	  uint8_t attr = sub == i ? blink : 0;
  	  lcd_outdezAtt(4 * FW, y, crv[i], attr);
		}
		lcd_putsAtt( 2*FW, 13*FH,PSTR(STR_PRESET), (sub == preset) ? blink : 0);


		if( sub==preset) 
		{
			if ( s_editMode )
			{
				int8_t t ;
				t = dfltCrv ;
	  	  dfltCrv = checkIncDec16( dfltCrv, -4, 4, 0);
	  	  if (dfltCrv != t)
				{
					uint8_t offset = cv9 ? 4 : 2 ;
					if ( points == 6 )
					{
						for (int8_t i = -5 ; i <= 5 ; i += 2 )
						{
						 	crv[(i+5)/2] = i*dfltCrv* 25 / 5 ;
						}
				  }
					else
					{
						for (int8_t i = -offset; i <= offset; i++) crv[i+offset] = i*dfltCrv* 25 / offset ;
					}
//	  	    STORE_MODELVARS;        
	  	  }
			}
		} 
		else  /*if(sub>0)*/
		{
		 CHECK_INCDEC_H_MODELVAR( crv[sub], -100,100);
		}

// Draw the curve
		drawCurve( sub ) ;
	}
}

void menuCurve(uint8_t event)
{
	TITLE( PSTR(STR_CURVES) ) ;
	static MState2 mstate2 ;
	EditType = EE_MODEL ;

	mstate2.check_columns(event,1+MAX_CURVE5+MAX_CURVE9-1-1+1+1+1) ;

  uint8_t sub = mstate2.m_posVert ;

	uint8_t t_pgOfs = evalOffset( sub ) ;

  switch (event)
	{
    case EVT_KEY_FIRST(KEY_RIGHT):
    case EVT_KEY_FIRST(KEY_MENU):
      s_curveChan = sub;
   		killEvents(event);
			Tevent = 0 ;
      pushMenu(menuCurveOne);
    break;
  }

  uint16_t y = 1*FH ;
  for (uint32_t i = 0 ; i < SCREEN_LINES-1; i += 1 )
	{
    uint8_t k = i + t_pgOfs;
    uint8_t attr = sub == k ? INVERS : 0;
    if(y>(SCREEN_LINES-1)*FH) break;
    lcd_putsAtt(   FW*0, y,PSTR(STR_CV),attr);
    lcd_outdezAtt( (k<9) ? FW*3-1 : FW*4-2, y,k+1 ,attr);

    y += FH ;
  }

	s_curveChan = sub ;
	drawCurve( 100 ) ;
}








void menuMixer( uint8_t event )
{
	TITLE(PSTR(STR_MIXER));
	EditType = EE_MODEL ;
	static MState2 mstate2;

	if ( s_moveMode )
	{
//		int8_t moveByRotary ;
//		moveByRotary = qRotary() ;		// Do this now, check_simple destroys rotary data
//		if ( moveByRotary )
//		{
//			if ( moveByRotary > 0 )
//			{
//				event = EVT_KEY_FIRST(KEY_DOWN) ;
//			}
//			else
//			{
//				event = EVT_KEY_FIRST(KEY_UP) ;
//			}
//		}
		uint8_t v = mstate2.m_posVert ;
		if ( ( ( v == 0 ) && ( event == EVT_KEY_FIRST(KEY_UP) ) ) 
				 || ( ( v == s_mixMaxSel ) && ( event == EVT_KEY_FIRST(KEY_DOWN) ) ) )
		{
			event = 0 ;
		}
//		Tevent = event ;
	}
	
	if ( !PopupData.PopupActive )
	{
		mstate2.check_columns(event,s_mixMaxSel-1) ;
	}

  int8_t  sub    = mstate2.m_posVert;
	int8_t	menulong = 0 ;

    switch(event)
    {
	    case EVT_ENTRY:
        s_moveMode=false;
  	  break;
    
			case EVT_KEY_FIRST(KEY_MENU):
//	    case EVT_KEY_BREAK(BTN_RE):
				if ( s_moveMode )
				{
	  	  	s_moveMode = false ;
  	  		s_editMode = false ;
//					RotaryState = ROTARY_MENU_UD ;
  	  		break;
				}
				// Else fall through    
				if ( !PopupData.PopupActive )
				{
		  		killEvents(event);
					Tevent = 0 ;			// Prevent changing weight to/from Gvar
					menulong = 1 ;
				}
  	  break;
    }

		uint8_t t_pgOfs = evalOffset( sub ) ;
    
	if ( PopupData.PopupActive )
	{
		Tevent = 0 ;
	}
		
  uint8_t mix_index = 0 ;
  uint8_t current = 0 ;

	if ( s_moveMode )
	{
		int8_t dir ;
		
		if ( ( dir = (event == EVT_KEY_FIRST(KEY_DOWN) ) ) || event == EVT_KEY_FIRST(KEY_UP) )
		{
			moveMix( s_curItemIdx, dir ) ; //true=inc=down false=dec=up - Issue 49
		}
	}

  for ( uint8_t chan=1 ; chan <= NUM_SKYCHNOUT ; chan += 1 )
	{
    X20MixData *pmd = &g_model.mixData[mix_index] ;
    
    if ( t_pgOfs <= current && current-t_pgOfs < SCREEN_LINES-1)
		{
      putsChn(1, (current-t_pgOfs+1)*FH, chan, 0) ; // show CHx
    }

		uint8_t firstMix = mix_index ;
		
		if (mix_index < MAX_SKYMIXERS && /* pmd->srcRaw && */ pmd->destCh == chan)
		{
    	do
			{
				if (t_pgOfs <= current )
				{
					if ( current-t_pgOfs < SCREEN_LINES-1 )
					{
    	  	  uint8_t y = (current-t_pgOfs+1)*FH ;
    				uint8_t attr = 0 ;

						if ( !s_moveMode && (sub == current) )
						{
							s_curItemIdx = mix_index ;
							s_currDestCh = chan ;		// For insert
							if ( menulong )
							{
								PopupData.PopupIdx = 0 ;
								PopupData.PopupActive = 1 ;
								event = 0 ;		// Kill this off
							}
							if ( PopupData.PopupActive == 0 )
							{
    						attr = INVERS ;
							}
						}
        	  if(firstMix != mix_index) //show prefix only if not first mix
        	 		lcd_putsAttIdx( 4*FW-8, y, XPSTR("\001+*R"),pmd->mltpx,0 ) ;

						putsChnOpRaw( 8*FW ,y, pmd->srcRaw, pmd->switchSource, pmd->disableExpoDr, attr ) ;
						if ( attr )
						{
							uint32_t value = g_chans512[chan-1] ;
							singleBar( 96, 4, value ) ;
					    lcd_outdez( 11*FW, 0, value/2 + 1500 ) ;
						}
//						putsChnOpRaw( 8*FW, y, pmd, 0 ) ;
extern uint8_t swOn[] ;
						attr = 0 ;
						if ( swOn[mix_index] || pmd->srcRaw == MIX_MAX || pmd->srcRaw == MIX_FULL )
						{
							attr |= BOLD ;
						}
						int16_t lweight = pmd->weight ;
						if ( (lweight <= -126) || (lweight >= 126) )
						{
							// A gvar ;
							if ( lweight < 0 )
							{
								lweight += 256 ;
							}
							lweight += 501 - 126 ;
						}
						gvarMenuItem( 7*FW+FW/2, y, lweight, -125, 125, attr | GVAR_250, 0 ) ;

					 	uint16_t x = 0 ;
						if (sub == current)
						{
//							pushPlotType( PLOT_COLOUR ) ;
							lcd_vline( 78+x, 8, 55 ) ;
//							popPlotType() ;
							lcd_puts_P( 13*FW+2+x, FH, XPSTR("Ofst") ) ;
							gvarMenuItem( FW*21+1+x , FH, pmd->sOffset, -125, 125, attr | GVAR_250, event ) ;
//							extendedValueEdit( pmd->sOffset, pmd->extOffset, 0, FH, 0, FW*21+1+x ) ;

							lcd_puts_P( 13*FW+2+x, 2*FH, XPSTR("Sw") ) ;
	            putsDrSwitches( 17*FW+2+x, 2*FH , pmd->swtch, 0 ) ;

// ??? Curve setting, Flight modes
							
						 	uint8_t value = pmd->differential ;
							if ( value == 0 )
							{
								if ( pmd->curve <= -28 )
								{
									value = 2 ;		// Expo
								}
							}
//							if ( value || pmd->curve )
//							{
								lcd_putsAttIdx( 13*FW+2+x, 3*FH, XPSTR("\003CveDifExp"), value, 0 ) ;
								switch ( value )
								{
									case 0 :
										put_curve( 18*FW+2+x, 3*FH, pmd->curve, 0 ) ;
									break ;
									case 1 :
				      	    gvarMenuItem( FW*21+1+x, 3*FH, pmd->curve, -100, 100, 0, 0 ) ;
									break ;
									case 2 :
            				lcd_outdezAtt( FW*21+1+x, 3*FH, pmd->curve + 128, 0 ) ;
									break ;
								}
//							}
							uint8_t b = 1 ;
							uint8_t z = pmd->modeControl ;
							lcd_puts_P( 13*FW+2+x, 4*FH, XPSTR("M") ) ;
							uint32_t modeCount = 0 ;
							uint32_t modeIndex = 0 ;
							for ( uint8_t p = 0 ; p<MAX_MODES+1 ; p++ )
							{
								if ( ( z & b ) == 0 )
								{
    							lcd_putcAtt( (14+p)*FW+2+x, 4*FH, '0'+p, 0 ) ;
									modeCount += 1 ;
									modeIndex = p ;
								}
								b <<= 1 ;
							}
							if ( modeCount == 1 )
							{
								if ( modeIndex )
								{
									modeIndex -= 1 ;
									if ( g_model.phaseData[modeIndex].name[0] )
									{
										lcd_puts_P( 13*FW+2+x, 6*FH, XPSTR("[      ]") ) ;
										lcd_putsnAtt( 14*FW+2+x, 5*FH, g_model.phaseData[modeIndex].name, 6, /*BSS*/ 0 ) ;
									}
								}
							}
							lcd_puts_P( 13*FW+2+x, 6*FH, XPSTR("D    :") ) ;
						  lcd_outdezAtt( FW*18+2+x, 6*FH, pmd->delayUp, PREC1 ) ;
						  lcd_outdezAtt( FW*21+1+x, 6*FH, pmd->delayDown, PREC1 ) ;

							lcd_puts_P( 13*FW+2+x, 7*FH, XPSTR("S    :") ) ;
						  lcd_outdezAtt( FW*18+2+x, 7*FH, pmd->speedDown, PREC1 ) ;
						  lcd_outdezAtt( FW*21+1+x, 7*FH, pmd->speedUp, PREC1 ) ;

						}					 	
						if ( s_moveMode )
						{
							if ( s_moveItemIdx == mix_index )
							{
								lcd_char_inverse( 4*FW+x, y, 76-4*FW, 0 ) ;
								s_curItemIdx = mix_index ;
								sub = mstate2.m_posVert = current ;
							}
						}
					}
					else
					{
						if ( current-t_pgOfs == 7 )
						{
							if ( s_moveMode )
							{
								if ( s_moveItemIdx == mix_index )
								{
									mstate2.m_posVert += 1 ;								
								}
							}
						}
					}
				}
				current += 1 ;
				mix_index += 1 ;
		    pmd = &g_model.mixData[mix_index] ;
    	} while ( (mix_index<MAX_SKYMIXERS && /* pmd->srcRaw && */ pmd->destCh == chan)) ;
		}
		else
		{
			if (sub == current)
			{
				s_currDestCh = chan ;		// For insert
				s_curItemIdx = mix_index ;
				lcd_rect( 0, (current-t_pgOfs+1)*FH-1, 25, 9 ) ;
				if ( menulong )		// Must need to insert here
				{
      		if ( !reachMixerCountLimit())
      		{
      			insertMix(s_curItemIdx, 0 ) ;
  	    		s_moveMode=false;
	      		pushMenu(menuMixOne) ;
						break ;
      		}
				}
			}
			current += 1 ;
		}
	}
//  if ( t_pgOfs <= current && (current-t_pgOfs) < 7)
//	{
//   	lcd_puts_Pleft( 7*FH,XPSTR("Templates\020MENU") ) ;
//		if (sub == s_mixMaxSel )
//		{
//			lcd_char_inverse( 0, 7*FH, 21*FW, 0 ) ;
//			if ( event == EVT_KEY_FIRST(KEY_MENU) || event == EVT_KEY_BREAK(BTN_RE) )
//			{
//				pushMenu( menuProcTemplates ) ;
//			}
//		}
//	}
	
	if ( PopupData.PopupActive )
	{
		Tevent = event ;
		mixpopup( event ) ;
    s_editMode = false;
	}
	s_mixMaxSel = current ;

}

void menuCustomCheck(uint8_t event)
{
	static MState2 mstate2 ;
	mstate2.check_columns(event, 3-1 ) ;
  lcd_puts_Pleft( 0, PSTR( STR_CUSTOM_CHECK ) ) ;
	 
  int8_t sub = mstate2.m_posVert;
	
//	if ( event == EVT_ENTRY )
//	{
//		RotaryState = ROTARY_MENU_UD ;
//	}
	CustomCheckData *pdata ;
	pdata = &g_model.customCheck ;

	lcd_puts_Pleft( FH, XPSTR("Source\037Min.\037Max.") ) ;
	for (uint8_t k = 0 ; k < 3 ; k += 1 )
	{
    uint8_t y = (k+1) * FH ;
    uint8_t i = k ;
		uint8_t attr = (sub==i ? InverseBlink : 0);
		switch(i)
		{
      case 0 :	// Source
			{	
				uint8_t x = pdata->source ;
				putsChnRaw( 11*FW, y, x, attr ) ;
				if( attr ) CHECK_INCDEC_H_MODELVAR( pdata->source, 0, 4 ) ; // NUM_SKYXCHNRAW+NUM_TELEM_ITEMS ) ;
			}
			break ;
			case 1 :
				lcd_outdezAtt( 13*FW, y, pdata->min, attr) ;
				if( attr ) CHECK_INCDEC_H_MODELVAR( pdata->min, -125, 125 ) ;
			break ;
      case 2 :	// offset
				lcd_outdezAtt( 13*FW, y, pdata->max, attr) ;
				if( attr ) CHECK_INCDEC_H_MODELVAR( pdata->max, -125, 125 ) ;
			break ;
		}
	}
}


void menuModelGeneral( uint8_t event )
{
	static MState2 mstate2;
  TITLE( PSTR(STR_General) ) ;
	uint8_t subN = 0 ;
	uint8_t attr = 0 ;

//			lcd_puts_Pleft( 0, XPSTR("\016(ver  )") ) ;
//    	lcd_putc( 19*FW, 0, g_model.modelVersion + '0' ) ;

	mstate2.check_columns(event, 14-1) ;
  int8_t  sub    = mstate2.m_posVert;
  uint8_t t_pgOfs ;

	t_pgOfs = evalOffset( sub ) ;

	uint8_t y = FH ;

	if(t_pgOfs<=subN)
	{
		alphaEditName( 11*FW-2, y, (uint8_t *)g_model.name, sizeof(g_model.name), sub==subN, (uint8_t *)PSTR( "Model Name") ) ;
		validateName( (uint8_t *)g_model.name, sizeof(g_model.name) ) ;
		y += FH ;
	}
	subN += 1 ;

	if(t_pgOfs<=subN)
	{
		lcd_puts_Pleft( y, PSTR(STR_TRIM_INC) ) ;
		g_model.trimInc = checkIndexed( y, PSTR(STR_TRIM_OPTIONS), g_model.trimInc, (sub==subN) ) ;
		y += FH ;
	}
	subN += 1 ;

	if(t_pgOfs<=subN)
	{
		g_model.extendedLimits = onoffMenuItem( g_model.extendedLimits, y, PSTR(STR_E_LIMITS), sub==subN ) ;
    //---------------------------------                           /* ReSt V */
    //   If extended Limits are switched off, min and max limits must be limited to +- 100         
  	if (!g_model.extendedLimits)
  	{
			for( uint32_t i = 0 ; i < NUM_SKYCHNOUT ; i += 1 )
  	  {
  	    LimitData *ld = &g_model.limitData[i] ;
  	    if (ld->min < 0) ld->min = 0;
  	    if (ld->max > 0) ld->max = 0;
  	  }
  	}                                 
    //-----------------------------------                           /* ReSt A */
				
		y += FH ;
	}
	subN += 1 ;
	
	if(t_pgOfs<=subN)
	{
		g_model.thrTrim = onoffMenuItem( g_model.thrTrim, y, PSTR(STR_T_TRIM), ( sub==subN ) ) ;
		y += FH ;
	}
	subN += 1 ;

	if(t_pgOfs<=subN)
	{
		g_model.thrExpo = onoffMenuItem( g_model.thrExpo, y, PSTR(STR_T_EXPO), sub==subN) ;
		y += FH ;
	}
	subN += 1 ;

	if(t_pgOfs<=subN)
	{
  	lcd_puts_Pleft( y, PSTR( STR_CUSTOM_CHECK ) ) ;
		if ( sub == subN )
		{
			lcd_char_inverse( 0, y, 12*FW, 0 ) ;
			if (event == EVT_KEY_BREAK(KEY_MENU) )
			{
  	  	pushMenu( menuCustomCheck ) ;
  			s_editMode = 0 ;
  	  	killEvents(event);
			}
		}
	
		y += FH ;
	}
	subN += 1 ;

	uint16_t states = g_model.modelswitchWarningStates ;
	if(t_pgOfs<=subN)
	{
		uint8_t b = (states & 1) ;
		b = offonMenuItem( b, y, PSTR(STR_SWITCH_WARN), sub==subN ) ;
  	g_model.modelswitchWarningStates = (states & ~1) | b ;
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;


	if(t_pgOfs<=subN)
	{
// Default Switch Enables
		uint32_t enables = 0xFF ;
		uint32_t temp = g_model.modelswitchWarningDisables ;
		if ( temp & 0x0003 )
		{
			enables &= ~0x01 ;
		}
		if ( temp & 0x000C )
		{
			enables &= ~0x02 ;
		}
		if ( temp & 0x0030 )
		{
			enables &= ~0x04 ;
		}
		if ( temp & 0x00C0 )
		{
			enables &= ~0x08 ;
		}
  	lcd_puts_Pleft( y, PSTR(STR_DEFAULT_SW) ) ;
		uint8_t subSub = g_posHorz ;
		if ( sub == subN )
		{
			Columns = 3 ;
		}
  	for( uint32_t i = 0 ; i < 4 ; i += 1 )
		{
			attr = 0 ;
			if ( (enables & (1<<i)) )
			{
				attr = INVERS ;
			}
			if ( sub == subN )
			{
				if ( subSub == i )
				{
					attr = BLINK ;	
				}
			}
			lcd_putsnAtt((13+i)*FW, y, XPSTR("ABCD")+i,1, attr ) ;
		}
		if(sub==subN)
		{
			if ( event == EVT_KEY_BREAK(KEY_MENU) )
			{
				uint32_t shift = subSub ;
  	    killEvents(event) ;
  	  	s_editMode = false ;
  		  enables ^= (1<<(shift)) ;
	//      STORE_MODELVARS;
		    eeDirty(EE_MODEL) ;
			}
		}
		temp = 0 ;
		if ( ( enables & 1 ) == 0 )    temp |= 0x0003 ;
		if ( ( enables & 2 ) == 0 )    temp |= 0x000C ;
		if ( ( enables & 4 ) == 0 )    temp |= 0x0030 ;
		if ( ( enables & 8 ) == 0 )    temp |= 0x00C0 ;
		g_model.modelswitchWarningDisables = temp ;
	
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;

	states >>= 1 ;
	if(t_pgOfs<=subN)
	{
		for ( uint32_t i = 0 ; i < 4 ; i += 1 )
		{
	  	lcd_putc( 2*FW+i*(2*FW+3), y, 'A'+i ) ;
			lcd_putc( 3*FW+i*(2*FW+3), y, PSTR(HW_SWITCHARROW_STR)[states & 0x03] ) ;
		  states >>= 2 ;
		}
extern uint32_t SwitchesStates ;
		if( sub == subN )
		{
			lcd_rect( 2*FW-2, y-1, 10*FW+2+18+1, 9 ) ;
			if ( event==EVT_KEY_FIRST(KEY_MENU) )
			{
				killEvents(event) ;
  			getMovedSwitch() ;	// loads switches_states
		    g_model.modelswitchWarningStates = ((SwitchesStates & 0x00FF) << 1 ) | (g_model.modelswitchWarningStates & 1) ;// states ;
				s_editMode = false ;
	//			STORE_MODELVARS ;
		    eeDirty(EE_MODEL) ;
			}
		}
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;


	if(t_pgOfs<=subN)
	{
		lcd_puts_Pleft( y,PSTR(STR_LIGHT_SWITCH));
  	attr = 0 ;
  	if(sub==subN) { attr = InverseBlink ; }
		g_model.mlightSw = edit_dr_switch( 17*FW, y, g_model.mlightSw, attr, attr ? EDIT_DR_SWITCH_EDIT : 0, event ) ;
		if ( g_model.mlightSw == 0 )
		{
			putsMomentDrSwitches( 14*FW, y, g_eeGeneral.lightSw, 0 ) ;
    	lcd_putcAtt( 12*FW, y, '(', 0 ) ;
    	lcd_putcAtt( 17*FW, y, ')', 0 ) ;
		}
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;

	if(t_pgOfs<=subN)
	{
		g_model.useCustomStickNames = onoffMenuItem( g_model.useCustomStickNames, y, PSTR( STR_CUSTOM_STK_NAMES ), sub==subN) ;
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;

	if(t_pgOfs<=subN)
	{
		attr = 0 ;
		lcd_puts_Pleft( y, PSTR(STR_TRIM_SWITCH)) ;
		if(sub==subN) { attr = InverseBlink ; }
		g_model.trimSw = edit_dr_switch( 17*FW, y, g_model.trimSw, attr, attr ? EDIT_DR_SWITCH_EDIT : 0, event ) ;
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;

	if(t_pgOfs<=subN)
	{
		lcd_puts_Pleft( y, XPSTR(" Modify:")) ;
		g_model.instaTrimToTrims = checkIndexed( y, XPSTR("\015\001""\010SubTrims   Trims"), g_model.instaTrimToTrims, sub==subN ) ;
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;

	if(t_pgOfs<=subN)
	{
  	attr = PREC1 ;
  	lcd_puts_Pleft( y, PSTR(STR_AUTO_LIMITS));
  	if(sub==subN) { attr = InverseBlink | PREC1 ; CHECK_INCDEC_H_MODELVAR( g_model.sub_trim_limit, 0, 100 ) ; }
  	lcd_outdezAtt(  20*FW, y, g_model.sub_trim_limit, attr ) ;
				 
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;

	if(t_pgOfs<=subN)
	{
		g_model.disableThrottleCheck = onoffMenuItem( g_model.disableThrottleCheck, y, XPSTR("Disable Thr Chk"), sub==subN) ;
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;
}


void editOneSwitchItem( uint8_t event, uint32_t item, uint32_t index )
{
	X20CSwData &cs = g_model.customSw[index] ;
	uint8_t cstate = CS_STATE(cs.func) ;
	switch ( item )
	{
		case 0 :
			cs.func = checkOutOfOrder( cs.func, (uint8_t *)SwitchFunctionMap, CS_MAXF ) ;
			if ( event == EVT_KEY_BREAK(KEY_MENU) )
			{
				// Long MENU pressed
				TextIndex = cs.func ;
				TextType = TEXT_TYPE_SW_FUNCTION ;
				saveHpos = item ;
				killEvents(event) ;
				pushMenu(menuTextHelp) ;
			}
	  	if(cstate != CS_STATE(cs.func) )
  		{
  		  cs.v1 = 0 ;
  		  cs.v2 = 0 ;
	  	}
		break ;
		
		case 1 :
      switch (cstate)
			{
      	case (CS_VCOMP):
      	case (CS_VOFS):
				case (CS_U16):
				case (CS_2VAL):
				{
 					uint8_t x = cs.v1u ;
					x = checkIncDec16( x,0,NUM_SKYXCHNRAW+NUM_TELEM_ITEMS-1+1,EE_MODEL) ;
					cs.v1u = x ;
					if ( event == EVT_KEY_BREAK(KEY_MENU) )
					{
						// Long MENU pressed
						TextIndex = x ;
  					TextType = TEXT_TYPE_SW_SOURCE ;
						saveHpos = item ;
  					killEvents(event) ;
						pushMenu(menuTextHelp) ;
					}
				}
   	    break;
      	case (CS_VBOOL):
				case CS_TMONO :
					CHECK_INCDEC_MODELSWITCH( cs.v1, -MaxSwitchIndex, MaxSwitchIndex ) ;
   	    break;
      	case (CS_TIMER):
 					CHECK_INCDEC_H_MODELVAR( cs.v1, -50, 99 ) ;
   	    break;
					
				default:
          break;
      }
		break ;

		case 2:
			switch (cstate)
			{
        case (CS_VOFS):
				case CS_TMONO :
				case (CS_2VAL):
					if ( g_posHorz == 0 )
					{
          	CHECK_INCDEC_H_MODELVAR( cs.v2, -125, 125) ;
					}
					else
					{
						int8_t t = (int8_t)cs.bitAndV3 ;
          	CHECK_INCDEC_H_MODELVAR( t, -125, 125) ;
						cs.bitAndV3 = t ;
					}
        break ;
        case (CS_VBOOL):
          CHECK_INCDEC_MODELSWITCH( cs.v2, -MaxSwitchIndex,MaxSwitchIndex) ;
        break ;
        case (CS_VCOMP):
				{
					uint8_t x = cs.v2u ;
					x = checkIncDec16( x, 0, NUM_SKYXCHNRAW+NUM_TELEM_ITEMS-1+1, EE_MODEL) ;
					cs.v2u = x ;
			if ( event == EVT_KEY_BREAK(KEY_MENU) )
					{
						// Long MENU pressed
						TextIndex = x ;
  					TextType = TEXT_TYPE_SW_SOURCE ;
						saveHpos = item ;
  					killEvents(event) ;
						pushMenu(menuTextHelp) ;
					}
				}
        break ;
        case (CS_TIMER):
            CHECK_INCDEC_H_MODELVAR( cs.v2, -50, 99) ;
        break ;
          
				case (CS_U16):
				{	
					int32_t y ;
					y = (uint8_t) cs.v2 ;
					y |= cs.bitAndV3 << 8 ;
					y -= 32768 ;
					StepSize = 128 ;
					y = checkIncDec16( y, -32768, 32767, EE_MODEL ) ;
					y += 32768 ;
					cs.bitAndV3 = y >> 8 ;
					cs.v2 = y ;
				}
        break ;

        default:
        break ;
			}
		break ;

    case 3:
		{	
			int32_t x ;
			x = cs.andsw ;
	    CHECK_INCDEC_MODELSWITCH( x, -MaxSwitchIndex+1, MaxSwitchIndex-1) ;
			cs.andsw = x ;
		}
		break;
		
		case 4 :
			CHECK_INCDEC_H_MODELVAR( cs.switchDelay, 0, 25 ) ;
		break ;
		
		case 5 :
      CHECK_INCDEC_H_MODELVAR( cs.exfunc, 0, 2 ) ;
		break ;
	}
	
}


void displayLogicalSwitch( uint16_t x, uint16_t y, uint32_t index )
{
	uint8_t attr ;
  attr = (getSwitch00( CSW_INDEX + index + 1 ) ) ? INVERS : 0 ;
  lcd_putc( x, y, 'L' ) ;
  lcd_putcAtt( x + FW, y, index + (index>8 ? 'A'-9: '1'), attr ) ;
}


void menuSwitchOne(uint8_t event)
{
	static MState2 mstate2 ;
	uint8_t attr ;
	uint32_t index ;
	uint8_t blink = InverseBlink ;
	uint32_t rows = 6 ;

	TITLE( XPSTR("SWITCH") ) ;

	mstate2.check_columns(event, rows ) ;

	uint8_t sub = mstate2.m_posVert ;
	index = s_currIdx ;
	X20CSwData &cs = g_model.customSw[index] ;

	if ( event == EVT_ENTRY_UP )
	{
		// Returned from editing
		sub = mstate2.m_posVert = saveHpos ;
	
		if ( TextResult )
		{
			if ( sub == 0 )
			{
				cs.func = SwitchFunctionMap[TextIndex] ;
			}
			else if ( sub == 1 )
			{
				cs.v1u = TextIndex ;
			}
			else
			{
				cs.v2u = TextIndex ;
			}
//	 	  eeDirty(EE_MODEL) ;
		}
	}

	uint8_t cstate = CS_STATE(cs.func) ;

//#ifdef BIG_SCREEN
//	DisplayOffset = SW1_OFF_0 ;
//#endif

	//write SW name here
	displayLogicalSwitch( 7*FW, 0, index ) ;

	lcd_puts_Pleft( FH, XPSTR("Function\037v1\037v2\037AND\037Delay\037Ex.Func\037Copy") ) ;

  if( (cstate == CS_VOFS) || (cstate == CS_2VAL) )
  {
		lcd_puts_Pleft( 3*FH, XPSTR("val") ) ;
		putsChnRaw( 9*FW, 2*FH, cs.v1u, sub == 1 ? blink : 0 ) ;

		uint8_t att = (sub==2) && (g_posHorz==0) ? blink : 0 ;

    if ( cs.v1u > CHOUT_BASE+NUM_SKYCHNOUT )
		{
			int16_t value = convertTelemConstant( cs.v1u-CHOUT_BASE-NUM_SKYCHNOUT-1, cs.v2 ) ;
			putsTelemetryChannel( 9*FW, 3*FH, cs.v1u-CHOUT_BASE-NUM_SKYCHNOUT-1, value, att, TELEM_NOTIME_UNIT | TELEM_UNIT| TELEM_CONSTANT ) ;
		}
    else
		{
    	lcd_outdezAtt( 12*FW, 3*FH, cs.v2, att ) ;
		}
		if ( cstate == CS_2VAL )
		{
			att = (sub==2) && (g_posHorz==1) ? blink : 0 ;
    	if ( cs.v1u > CHOUT_BASE+NUM_SKYCHNOUT )
			{
				int16_t value = convertTelemConstant( cs.v1u-CHOUT_BASE-NUM_SKYCHNOUT-1, (int8_t)cs.bitAndV3 ) ;
				putsTelemetryChannel( 20*FW, 3*FH, cs.v1u-CHOUT_BASE-NUM_SKYCHNOUT-1, value, att, TELEM_NOTIME_UNIT | TELEM_UNIT| TELEM_CONSTANT ) ;
			}
    	else
			{
    		lcd_outdezAtt( 17*FW, 3*FH, (int8_t)cs.bitAndV3, att ) ;
			}
		}
  }
  else if(cstate == CS_VBOOL)
  {
    putsDrSwitches( 9*FW, 2*FH, cs.v1, sub==1 ? blink : 0 ) ;
    putsDrSwitches( 9*FW, 3*FH, cs.v2, sub==2 ? blink : 0 ) ;
  }
  else if(cstate == CS_VCOMP)
  {
    putsChnRaw( 9*FW, 2*FH, cs.v1u, sub==1 ? blink : 0 ) ;
    putsChnRaw( 9*FW, 3*FH, cs.v2u, sub==2 ? blink : 0 ) ;
  }
	else if(cstate == CS_TIMER)
	{
		int8_t x ;
		uint8_t att = 0 ;
		x = cs.v1 ;
		if ( x < 0 )
		{
			x = -x-1 ;
			att = PREC1 ;
		}
	  lcd_puts_Pleft( 2*FH, XPSTR("Off") ) ;
    lcd_outdezAtt( 12*FW, 2*FH, x+1  ,att | (sub==1 ? blink : 0) ) ;
		att = 0 ;
		x = cs.v2 ;
		if ( x < 0 )
		{
			x = -x-1 ;
			att = PREC1 ;
		}
	  lcd_puts_Pleft( 3*FH, XPSTR("On") ) ;
    lcd_outdezAtt( 12*FW, 3*FH, x+1 , att | (sub==2 ? blink : 0 ) ) ;
		cs.exfunc = 0 ;
	}
	else if(cstate == CS_TMONO)
	{
    putsDrSwitches( 9*FW, 2*FH, cs.v1  ,sub==1 ? blink : 0);
		uint8_t att = 0 ;
		int8_t x ;
		x = cs.v2 ;
		if ( x < 0 )
		{
			x = -x-1 ;
			att = PREC1 ;
		}
		lcd_puts_Pleft( 3*FH, XPSTR("Time") ) ;
    lcd_outdezAtt( 12*FW, 3*FH, x+1 , att | (sub==2 ? blink : 0 ) ) ;
		cs.exfunc = 0 ;
	}
	else// cstate == U16
	{
		uint16_t x ;
		x = (uint8_t) cs.v2 ;
		x |= cs.bitAndV3 << 8 ;
    putsChnRaw( 9*FW, 2*FH, cs.v1u, sub==1 ? blink : 0);
    lcd_outdezNAtt( 14*FW, 3*FH, x ,sub==2 ? blink : 0,5);
	}

	attr = 0 ;
	if ( sub == 0 )
	{
		attr = InverseBlink ;
	}
	lcd_putsAttIdx( 9*FW, FH, PSTR(CSWITCH_STR), cs.func, attr ) ;

	putsDrSwitches( 8*FW, 4*FH, cs.andsw, (sub==3 ? blink : 0 ) ) ;

	attr = 0 ;

	lcd_putsAttIdx( 0, 4*FH, XPSTR("\003ANDOR XOR"), cs.exfunc, attr ) ;
	
	if ( sub == 5 )
	{
		attr = blink ;
	}	
	lcd_putsAttIdx( 8*FW, 6*FH, XPSTR("\003ANDOR XOR"), cs.exfunc, attr ) ;

	attr = 0 ;
	if ( sub == 4 )
	{
		attr = blink ;
	}
	lcd_outdezAtt( 12*FW, 5*FH, cs.switchDelay, attr|PREC1 ) ;

	if ( sub <= 5 )
	{
		if ( ( cstate == CS_2VAL ) && ( sub == 2) )
		{
			Columns = 1 ;
		}
		editOneSwitchItem( event, sub, index ) ;
	}
//	TextResult = 0 ;

	if ( Clipboard.content == CLIP_SWITCH )
	{
	 	lcd_puts_Pleft( 7*FH, XPSTR("\012Paste") ) ;
	}

	if ( sub == 6 )
	{
		if ( Clipboard.content == CLIP_SWITCH )
		{
			Columns = 1 ;
		}

		uint8_t subSub = g_posHorz ;
		lcd_char_inverse( subSub ? 60 : 0, 7*FH, 30, 0 ) ;
		if ( checkForMenuEncoderLong( event ) )
		{
			if ( subSub )
			{
				cs = Clipboard.clipswitch ;
			}
			else
			{
				Clipboard.clipswitch = cs ;
				Clipboard.content = CLIP_SWITCH ;
			}
		}
	}
}

#define HELI_OFF_0			0
#define HELI_OFF_17			(17*FW)
#define HELI_OFF_20			(20*FW)

void menuHeli( uint8_t event )
{
	TITLE( PSTR(STR_HELI_SETUP) ) ;
	static MState2 mstate2 ;
	mstate2.check_columns(event, 6-1 ) ;

	uint8_t subN = 0 ;
  uint8_t attr ;
	uint8_t y = FH ;
  uint8_t sub = mstate2.m_posVert ;
	uint8_t blink = InverseBlink ;
	
	lcd_puts_Pleft( y, PSTR(STR_HELI_TEXT));
	g_model.swashType = checkIndexed( y, PSTR(SWASH_TYPE_STR), g_model.swashType, (sub==subN) ) ;
	y += FH ;
	subN += 1 ;
		
	attr = 0 ;
	if(sub==subN)
	{
		attr = blink ;
		uint8_t x = g_model.swashCollectiveSource ;
		CHECK_INCDEC_H_MODELVAR( x, 0, NUM_SKYXCHNRAW ) ;
		g_model.swashCollectiveSource = x  ;
	}
  putsChnRaw(HELI_OFF_17, y, g_model.swashCollectiveSource, attr ) ;
	y += FH ;
	subN += 1 ;

	attr = 0 ;
  if(sub==subN)
	{
		attr = blink ;
		CHECK_INCDEC_H_MODELVAR( g_model.swashRingValue, 0, 100) ;
	}
  lcd_outdezAtt(HELI_OFF_20, y, g_model.swashRingValue, attr ) ;
	y += FH ;
	subN += 1 ;

	g_model.swashInvertELE = hyphinvMenuItem( g_model.swashInvertELE, y, sub==subN ) ;
	y += FH ;
	subN += 1 ;
			
	g_model.swashInvertAIL = hyphinvMenuItem( g_model.swashInvertAIL, y, sub==subN ) ;
	y += FH ;
	subN += 1 ;

	g_model.swashInvertCOL = hyphinvMenuItem( g_model.swashInvertCOL, y, sub==subN ) ;
}





void menuSwitches(uint8_t event)
{
	TITLE(PSTR(STR_CUST_SWITCH));
	EditType = EE_MODEL ;

	static MState2 mstate2 ;
	event = mstate2.check_columns(event, NUM_SKYCSW-1 ) ;

	uint16_t y = 0 ;
	uint8_t k = 0 ;
	uint8_t sub = mstate2.m_posVert ;
	uint8_t t_pgOfs ;

	t_pgOfs = evalOffset( sub ) ;


	for(uint32_t i=0; i<SCREEN_LINES-1; i++)
	{
    y = (i+1) * FH ;
    k = i + t_pgOfs ;
    uint8_t attr ;
		uint8_t m = k ;
    X20CSwData &cs = g_model.customSw[m];

		//write SW names here
		displayLogicalSwitch( 0, y, m ) ;
    
		attr = (sub==k ? INVERS : 0);
		lcd_putsAttIdx( 2*FW+1, y, PSTR(CSWITCH_STR),cs.func, 0);

    uint8_t cstate = CS_STATE(cs.func);

	  if( (cstate == CS_VOFS) || (cstate == CS_2VAL) )
    {
			putsChnRaw(    10*FW-6, y, cs.v1u  , 0);
	    if ( cs.v1u > CHOUT_BASE+NUM_SKYCHNOUT )
 			{
				int16_t value = convertTelemConstant( cs.v1u-CHOUT_BASE-NUM_SKYCHNOUT-1, cs.v2 ) ;
				putsTelemetryChannel( 18*FW-8, y, cs.v1u-CHOUT_BASE-NUM_SKYCHNOUT-1, value, 0, TELEM_NOTIME_UNIT | TELEM_UNIT| TELEM_CONSTANT);
			}
      else
			{
        lcd_outdezAtt( 18*FW-9, y, cs.v2  ,0);
			}
    }
    else if(cstate == CS_VBOOL)
    {
      putsDrSwitches(10*FW-6, y, cs.v1  , 0);
      putsDrSwitches(14*FW-7, y, cs.v2  , 0);
    }
    else if(cstate == CS_VCOMP)
    {
      putsChnRaw(    10*FW-6, y, cs.v1u  , 0);
      putsChnRaw(    14*FW-4, y, cs.v2u  , 0);
    }
		else if(cstate == CS_TIMER)
		{
			int8_t x ;
			uint8_t att = 0 ;
			x = cs.v1 ;
			if ( x < 0 )
			{
				x = -x-1 ;
				att = PREC1 ;
			}
	    lcd_puts_P( 13*FW, y, PSTR("On") ) ;
      lcd_outdezAtt( 13*FW-5, y, x+1  ,att ) ;
			att = 0 ;
			x = cs.v2 ;
			if ( x < 0 )
			{
				x = -x-1 ;
				att = PREC1 ;
			}
      lcd_outdezAtt( 18*FW-3, y, x+1 , att ) ;
		}
		else if(cstate == CS_TMONO)
		{
      putsDrSwitches(10*FW-6, y, cs.v1  , 0);
			uint8_t att = 0 ;
			int8_t x ;
			x = cs.v2 ;
			if ( x < 0 )
			{
				x = -x-1 ;
				att = PREC1 ;
			}
      lcd_outdezAtt( 17*FW-2, y, x+1 , att ) ;
		}
		else// cstate == U16
		{
			uint16_t x ;
			x = cs.v2u ;
			x |= cs.bitAndV3 << 8 ;
      putsChnRaw( 10*FW-6-FW, y, cs.v1  , 0);
      lcd_outdezNAtt( 18*FW-9, y, x  , 0,5);
		}
		
		putsDrSwitches( 18*FW-3, y, cs.andsw, 0 ) ;

//		lcd_puts_P( 22*FW, y, XPSTR("Delay" ) ) ;
//		lcd_outdezAtt( 31*FW, y, cs.switchDelay, PREC1 ) ;

		if ( attr )
		{
 			lcd_char_inverse( 12, y, 127-12, 0 ) ;
			lcd_puts_P( 102, 0, XPSTR("Dy" ) ) ;
			lcd_outdezAtt( 21*FW, 0, cs.switchDelay, PREC1 ) ;
			if ( checkForMenuEncoderBreak( event ) )
			{
				// Long MENU pressed
		    s_currIdx = sub ;
//  			TextType = 0 ;
				pushMenu(menuSwitchOne) ;
  	  	s_editMode = false ;
			}
		}
	}
}

void putsTrimMode( uint8_t x, uint8_t y, uint8_t phase, uint8_t idx, uint8_t att )
{
  int16_t v = getRawTrimValue(phase, idx);

  if (v > TRIM_EXTENDED_MAX)
	{
//		if ( v > TRIM_EXTENDED_MAX+MAX_MODES+1 )
//		{
//    	lcd_putcAtt(x, y, '+', att ) ;
//		}
//		else
//		{
	    uint8_t p = v - TRIM_EXTENDED_MAX - 1;
  	  if (p >= phase) p += 1 ;
    	lcd_putcAtt(x, y, '0'+p, att);
//		}
  }
  else
	{
  	lcd_putsAttIdx( x, y, PSTR(STR_1_RETA), idx, att ) ;
  }
}


void menuPhaseOne(uint8_t event)
{
  PhaseData *phase = &g_model.phaseData[s_currIdx] ;
	TITLE(PSTR(STR_FL_MODE)) ;

	static MState2 mstate2 ;
	mstate2.check_columns(event,6-1) ;
  lcd_putc( 8*FW, 0, '1'+s_currIdx ) ;

  uint8_t sub = mstate2.m_posVert;
  uint8_t editMode = s_editMode;

	for (uint8_t i = 0 ; i < 6 ; i += 1 )
	{
    uint8_t y = (i+2) * FH;
		uint8_t attr = (sub==i ? InverseBlink : 0);
    
		switch(i)
		{
      case 0 : // switch
				lcd_puts_Pleft( y, PSTR(STR_SWITCH) ) ;
				phase->swtch = edit_dr_switch( 10*FW, y, phase->swtch, attr, attr ? EDIT_DR_SWITCH_EDIT : 0, event ) ;
			break;

      case 1 : // switch
				lcd_puts_Pleft( y, PSTR(STR_SWITCH) ) ;
				phase->swtch2 = edit_dr_switch( 10*FW, y, phase->swtch2, attr, attr ? EDIT_DR_SWITCH_EDIT : 0, event ) ;
			break;

      case 2 : // trims
				if ( attr )
				{
					Columns = 3 ;
				}
				lcd_puts_Pleft( y, PSTR(STR_TRIMS) ) ;
        for ( uint8_t t = 0 ; t<4/*NUM_STICKS*/ ; t += 1 )
				{
          putsTrimMode( (10+t)*FW, y, s_currIdx+1, t, (g_posHorz==t) ? attr : 0 ) ;
          if (attr && g_posHorz==t && (editMode>0) )
					{
            int16_t v = phase->trim[t] ;
            if (v < TRIM_EXTENDED_MAX)
						{
							v = TRIM_EXTENDED_MAX;
						}
						int16_t u = v ;
            v = checkIncDec16( v, TRIM_EXTENDED_MAX, TRIM_EXTENDED_MAX+MAX_MODES, EE_MODEL ) ;
            
						if (v != u)
						{
              if (v == TRIM_EXTENDED_MAX) v = 0 ;
//							if ( v == TRIM_EXTENDED_MAX+MAX_MODES+1 )
//							{
//								v = 2000 ;
//							}
  						phase->trim[t] = v ;
            }
          }
        }
      break;
      
			case 3 : // fadeIn
				lcd_puts_Pleft( y, XPSTR("Fade In") ) ;
    		lcd_outdezAtt(14*FW, y, phase->fadeIn * 5, attr | PREC1 ) ;
  			if( attr ) CHECK_INCDEC_H_MODELVAR( phase->fadeIn, 0, 15 ) ;
			break ;
      
			case 4 : // fadeOut
				lcd_puts_Pleft( y, XPSTR("Fade Out") ) ;
		    lcd_outdezAtt(14*FW, y, phase->fadeOut * 5, attr | PREC1 ) ;
  			if( attr ) CHECK_INCDEC_H_MODELVAR( phase->fadeOut, 0, 15 ) ;
			break ;
			
			case 5 : // Name
				alphaEditName( 11*FW-2, y, (uint8_t *)phase->name, sizeof(phase->name), attr, (uint8_t *)XPSTR( "Mode Name") ) ;
			break ;
	  }
	}
}




#define PH_Y				(1*FH)

void menuPhases(uint8_t event)
{
	uint32_t i ;
  uint8_t attr ;
	
	TITLE(PSTR(STR_MODES));
	static MState2 mstate2;
  
	event = mstate2.check_columns( event, 7-1-1 ) ;
	
	uint8_t  sub    = mstate2.m_posVert ;

  switch (event)
	{
    case EVT_KEY_FIRST(KEY_MENU) :
  //  case EVT_KEY_BREAK(BT) :
       s_currIdx = sub ;
       killEvents(event);
       pushMenu(menuPhaseOne) ;
		break;
  }
    
  lcd_putc( 0, PH_Y, 'F' ) ;
  lcd_putc( 1*FW-1, PH_Y, '0' ) ;
	lcd_puts_P( 16*FW+2, PH_Y, XPSTR("RETA") ) ;

  for ( i=0 ; i<MAX_MODES ; i += 1 )
	{
    uint16_t y = PH_Y + FH + i*FH ;
		PhaseData *p = &g_model.phaseData[i] ;
    attr = (i == sub) ? INVERS : 0 ;
    lcd_putc( 0, y, 'F' ) ;
    lcd_putc( 1*FW-1, y, '1'+i ) ;
		lcd_putsnAtt( 2*FW-1, y, g_model.phaseData[i].name, 6, /*BSS*/ attr ) ;
    putsDrSwitches( 8*FW, y, p->swtch, attr ) ;
    putsDrSwitches( 12*FW+1, y, p->swtch2, attr ) ;
    for ( uint8_t t = 0 ; t < 4/*NUM_STICKS*/ ; t += 1 )
		{
			putsTrimMode( 16*FW+2 + t*FW, y, i+1, t, attr ) ;
		}
		if ( p->fadeIn || p->fadeOut )
		{
	    lcd_putcAtt( 20*FW+2, y, '*', attr ) ;
		}
	}	 
	i = getFlightPhase() ;
	pushPlotType( PLOT_BLACK ) ;
	lcd_rect( 0, PH_Y + i*FH-1, 2*FW-1, 9 ) ;
	popPlotType() ;
}

void menuSetFailsafe(uint8_t event)
{
	static MState2 mstate2 ;
//#ifdef ACCESS
//	mstate2.check_columns(event, 24-1+1+1+1 ) ;	
//#else
	mstate2.check_columns(event, 16-1+1+1+1 ) ;	
//#endif
  lcd_puts_Pleft( 0, XPSTR( "Set Failsafe" ) ) ;
  int8_t sub = mstate2.m_posVert ;
	uint16_t y = 0;
	uint8_t k = 0;
	uint8_t t_pgOfs ;
	int32_t value ;
	t_pgOfs = evalOffset( sub ) ;
	uint32_t module = s_currIdx ? 1 : 0 ;
	
	if ( event == EVT_ENTRY )
	{
		StatusTimer = 0 ;
	}
	if ( StatusTimer )
	{
		StatusTimer -= 1 ;
	}

	for(uint8_t i=0; i<SCREEN_LINES-1; i++)
	{
    y=(i+1)*FH;
    k=i+t_pgOfs;
    uint8_t attr = ((sub==k) ? InverseBlink : 0);
		uint8_t active = (attr && s_editMode ) ;

		if ( k == 0 )
		{
		  lcd_puts_P( 0, y, XPSTR( "Mode" ) ) ;
			EditColumns = 1 ;
			g_model.Module[module].failsafeMode = checkIndexed( y, XPSTR("\011\004""\007Not Set     Rx Custom   HoldNoPulse"), g_model.Module[module].failsafeMode, attr ) ;
			EditColumns = 0 ;
		}
		else if ( k == 1 )
		{
			if ( attr )
			{
    		if ( checkForMenuEncoderLong( event ) )
				{
					FailsafeCounter[module] = 6 ;		// Send failsafe values soon
					StatusTimer = 50 ;
				}
			}
		  lcd_putsAtt( 0, y, XPSTR( "Send Now" ), StatusTimer ? 0 : attr ) ;
		}
		else if ( k == 2 )
//		{
//			uint8_t b ;
//			b = g_model.Module[module].failsafeRepeat ;
//     	g_model.Module[module].failsafeRepeat = offonMenuItem( b, y, XPSTR("Repeat Send"), sub == k ) ;
//			if ( b != g_model.Module[module].failsafeRepeat )
//			{
//				if ( b == 0 )
//				{
//					FailsafeCounter[module] = 6 ;		// Send failsafe values soon
//				}
//				else
//				{
//					FailsafeCounter[module] = 0 ;		// Stop sending
//				}
//			}
//		}
//		else if ( k == 3 )
		{
			if ( attr )
			{
    		if ( checkForMenuEncoderLong( event ) )
				{
					uint32_t j ;
//#ifdef ACCESS
//					for ( j = 0 ; j < 24 ; j += 1 )
//#else
					for ( j = 0 ; j < 16 ; j += 1 )
//#endif
					{
						value = g_chans512[j] * 100 / RESX ;
  					if(value > 125)
						{
							value = 125 ;
						}	
  					if(value < -125 )
						{
							value = -125 ;
						}	
//#ifdef ACCESS
//						if ( j >= 16 )
//						{
//							g_model.accessFailsafe[module][j-16] = value ;
//						}
//						else
//						{
//							g_model.Module[module].failsafe[j] = value ;
//						}
//#else
						g_model.Module[module].failsafe[j] = value ;
//#endif
					}
					StatusTimer = 50 ;
//		    	eeDirty(EE_MODEL) ;
				}
			}
		  lcd_putsAtt( 0, y, XPSTR( "Use Actual" ), StatusTimer ? 0 : attr ) ;
		}
		else
		{
			putsChn( 0, y, k-2 ,0 ) ;
//#ifdef ACCESS
//			int8_t *pValue ;
//			pValue = ( (k-3) < 16 ) ? &g_model.Module[module].failsafe[k-3] : &g_model.accessFailsafe[module][k-3-16] ;
//			value = *pValue ;
//			lcd_outdezAtt( FW*7+FAIL_OFF_0+3, y, value, attr ) ;
//			lcd_putc( FW*7+5, y, '%' ) ;
//    	if(active)
//			{
//  	    *pValue = checkIncDec16( value, -125, 125, EE_MODEL ) ;
//    	}
//#else  	  
			value = g_model.Module[module].failsafe[k-3] ;
			lcd_outdezAtt( FW*7+3, y, value, attr ) ;
			lcd_putc( FW*7+5, y, '%' ) ;
    	if(active)
			{
  	    g_model.Module[module].failsafe[k-3] = checkIncDec16( value, -125, 125, EE_MODEL ) ;
    	}
//#endif
		}
	}
}

void menuRangeBind(uint8_t event)
{
	static uint8_t binding = 0 ;
	uint8_t *ptrFlag ;

	ptrFlag = &BindRangeFlag[s_currIdx] ;
	uint8_t flag = *ptrFlag & PXX_BIND ;
  if ( event == EVT_ENTRY )
	{
		binding = flag ? 1 : 0 ;
		BindTimer = 255 ;
//		DsmResponseFlag = 0 ;
	}

	if ( checkForExitEncoderLong( event ) )
	{
    killEvents(event);
		*ptrFlag = 0 ;
		popMenu(false) ;
	}

	if ( binding && ( flag == 0 ) )
	{
		*ptrFlag = 0 ;
		popMenu(false) ;
	}

	lcd_puts_Pleft( 2*FH, (binding) ? PSTR(STR_6_BINDING) : PSTR(STR_RANGE_RSSI) ) ;

	if ( binding == 0 )
	{
//		lcd_outdezAtt( 12 * FW, 4*FH, FrskyHubData[FR_RXRSI_COPY], DBLSIZE);
	}

//	if ( ( ( g_model.Module[0].protocol == PROTO_MULTI ) && ( s_currIdx == 0 ) ) ||
//		( ( g_model.Module[1].protocol == PROTO_MULTI ) && ( s_currIdx == 1 ) ) )
//	{
//		if ( binding )
//		{
//			if ( PrivateData[1] && BindTimer < 170 )
//			{
//				if ( ( PrivateData[0] & 0x08 ) == 0 )
//				{
//					*ptrFlag = 0 ;
//					popMenu(false) ;
//				}
//			}
//		}
//	}

//	if ( ( ( g_model.Module[0].protocol == PROTO_MULTI ) && ( (g_model.Module[0].sub_protocol & 0x3F) == M_DSM ) ) ||
//		( ( g_model.Module[1].protocol == PROTO_MULTI ) && ( (g_model.Module[1].sub_protocol & 0x3F) == M_DSM ) ) )
//	{
//		if ( DsmResponseFlag )
//		{
//			uint32_t module = s_currIdx ;
//			lcd_outhex4( 0, 6*FH, MultiResponseData ) ;

//			lcd_puts_Pleft( 7*FH, XPSTR("DSM2 22mS\015ch") ) ;
//			if ( MultiResponseData & 0x80 )
//			{
//				lcd_putc( 3*FW, 7*FH, 'X' ) ;
//			}
//			if ( MultiResponseData & 0x40 )
//			{
//				lcd_puts_Pleft( 7*FH, "\00511" ) ;
//			}
//  		lcd_outdez( 13*FW-2, 7*FH, MultiResponseData & 0x0F ) ;

//			int8_t x ;
//			uint8_t y ;
			
//			if ( *ptrFlag )
//			{
//				*ptrFlag = 0 ;
//				x = MultiResponseData & 0x0F ;	// # channels
//				y = (MultiResponseData & 0xC0 ) >> 2 ;	// DSMX, 11mS
//				if ( module )
//				{
//					g_model.xoption_protocol = (g_model.xoption_protocol & 0xF0) | x ;
//					y |= g_model.xppmNCH & 0x8F ;
//					g_model.xppmNCH = y ;	// Set DSM2/DSMX
					
//					g_model.Module[module].option_protocol = (g_model.Module[module].option_protocol & 0xF0) | x ;
//					y |= g_model.Module[module].channels & 0x8F ;
//					g_model.Module[module].channels = y ;	// Set DSM2/DSMX
//				}
//				else
//				{
//					g_model.option_protocol = (g_model.option_protocol & 0xF0) | x ;
//					y |= g_model.ppmNCH & 0x8F ;
//					g_model.ppmNCH = y ;	// Set DSM2/DSMX
					
//					g_model.Module[module].option_protocol = (g_model.Module[module].option_protocol & 0xF0) | x ;
//					y |= g_model.Module[module].channels & 0x8F ;
//					g_model.Module[module].channels = y ;	// Set DSM2/DSMX
//				}
//				protocolsToModules() ;
//				STORE_MODELVARS ;
//			}
//		}
//	}
	
//	if ( ( g_model.Module[1].protocol == PROTO_DSM2 ) && ( g_model.Module[1].sub_protocol == DSM_9XR ) )
//	{
//		if ( DsmResponseFlag )
//		{
//			lcd_outhex4( 0, 6*FH, (uint8_t)g_model.dsmMode ) ;

//			lcd_puts_Pleft( 7*FH, XPSTR("DSM2 22mS\015ch") ) ;
//			if ( g_model.dsmMode & 0x01 )
//			{
//				lcd_putc( 3*FW, 7*FH, 'X' ) ;
//			}
//			if ( g_model.dsmMode & 0x02 )
//			{
//				lcd_puts_Pleft( 7*FH, "\00511" ) ;
//			}
//  		lcd_outdez( 13*FW-2, 7*FH, g_model.Module[1].channels ) ;
//			if ( BindTimer > 40 )
//			{
//				BindTimer = 40 ;
//			}
//		}
//	}


	if ( --BindTimer == 0 )
	{
  	audioDefevent(AU_WARNING2) ;
//		if ( DsmResponseFlag )
//		{
//			*ptrFlag = 0 ;
//		}
	}
}


void menuBindOptions(uint8_t event)
{
	struct t_module *pModule = &g_model.Module[s_currIdx] ;
	TITLE(XPSTR("Bind Options"));
	static MState2 mstate2;
	event = mstate2.check_columns( event, 2 ) ;
  switch(event)
  {
	
    case EVT_KEY_BREAK(KEY_MENU):
			if ( mstate2.m_posVert == 0 )
		  {
//#ifdef ACCESS
//				if ( g_model.Module[s_currIdx].protocol == PROTO_ACCESS )
//				{
//					ModuleControl[s_currIdx].bindStep = BIND_START ;
//					ModuleSettings[s_currIdx].mode = MODULE_MODE_BIND ;
//					AccState = ACC_ACCST_BIND ;
//					popMenu( 0 ) ;
//				}
//				else
//#endif
				{
		  		BindRangeFlag[s_currIdx] = PXX_BIND ;		    	//send bind code
					chainMenu( menuRangeBind ) ;
				}
			}
		break ;
	}
	int8_t sub = mstate2.m_posVert ;
	
	lcd_puts_Pleft( 1*FH, PSTR(STR_BIND) ) ;
	if(sub == 0)
	{
		lcd_char_inverse( 0, 1*FH, 4*FW, 0 ) ;
	}
	pModule->highChannels = checkIndexed( 2*FH, XPSTR("\0""\001""\012Chans 1-8 Chans 9-16"), pModule->highChannels, (sub==1) ) ;
	pModule->disableTelemetry = checkIndexed( 3*FH, XPSTR("\0""\001""\014Telemetry   No Telemetry"), pModule->disableTelemetry, (sub==2) ) ;
}

const uint8_t ProtocolOptions[2][5] = { {1,PROTO_PPM}, {2,PROTO_PXX,PROTO_XFIRE} } ;

uint32_t checkProtocolOptions( uint8_t module )
{
//#if defined(PCBX12D) || defined(PCBX10)
//	if ( module == 0 )
//	{
// #ifdef DISABLE_PXX
//  #if defined(PCBT16)
//		g_model.Module[module].protocol = PROTO_MULTI ;
//	#else
//		g_model.Module[module].protocol = PROTO_PPM ;
//	#endif
// #else
//  #ifdef ACCESS
//		g_model.Module[module].protocol = PROTO_ACCESS ;
//  #else
//   #if defined(PCBT16)
//		g_model.Module[module].protocol = PROTO_MULTI ;
//	 #else
//		g_model.Module[module].protocol = PROTO_PXX ;
//  	#endif
//  #endif
// #endif
//		return 1 ;
//	}
//#endif
	uint8_t *options = (uint8_t *) ProtocolOptions[module] ;
	uint32_t count = *options++ ;
	uint8_t index = g_model.Module[module].protocol ;
	uint8_t save = index ;
	
	index = checkOutOfOrder( index, options, count ) ;
	g_model.Module[module].protocol = index ;
	
	if ( save != index )
	{
		g_model.Module[module].sub_protocol = 0 ;
		return 1 ;
	}
	return 0 ;
}

void displayProtocol( uint16_t x, uint16_t y, uint8_t value, uint8_t attr )
{
	if ( value != PROTO_OFF )
	{
		lcd_putsAttIdx( x, y, PSTR(STR_PROT_OPT), value, attr ) ;
	}
	else
	{
		lcd_putsAtt( x, y, XPSTR( "OFF" ), attr ) ;
	}
}

void displayModuleName( uint16_t x, uint16_t y, uint8_t index, uint8_t attr )
{
	lcd_putsAttIdx( x, y, XPSTR("\010InternalExternal"), index, attr ) ;
}


uint8_t EditingModule ;

void editOneProtocol( uint8_t event )
{
	uint32_t need_bind_range = 0 ;
	uint32_t dataItems = 7 ;
	uint32_t module = EditingModule ;
	uint8_t attr = 0 ;
	if ( module )
	{
		module = 1 ;
	}
	struct t_module *pModule = &g_model.Module[module] ;
	uint8_t blink = InverseBlink ;

	switch ( pModule->protocol )
	{
		case PROTO_OFF :
			dataItems = 1 ;
		break ;
//		case PROTO_PPM :
////			dataItems += 4 ;
//		break ;
		case PROTO_PXX :
			dataItems += 4 ;
			need_bind_range |= 5 ;
		break ;
//		case PROTO_MULTI :
//			dataItems += 7 ;
//			need_bind_range |= 5 ;
//		break ;
		case PROTO_XFIRE :
			dataItems -= 3 ;
			need_bind_range |= 5 ;
		break ;
	}

	TITLE( XPSTR("Protocol") ) ;
	static MState2 mstate2 ;

	if (pModule->protocol == PROTO_OFF)
	{
		mstate2.m_posVert = 0 ;
	}

	displayModuleName( 9*FW, 0, module, 0 ) ;

	event = mstate2.check_columns( event, dataItems-1 ) ;
	
	int8_t  sub    = mstate2.m_posVert ;
//	uint8_t subSub = g_posHorz;
  uint8_t t_pgOfs ;

	t_pgOfs = evalOffset( sub ) ;

	uint16_t y = 1*FH;
	
	uint8_t subN = 0 ;

	if(t_pgOfs<=subN)
	{
		uint8_t value ;
		uint8_t newvalue ;
		value = (pModule->protocol == PROTO_OFF) ? 0 : 1 ;
		newvalue = onoffMenuItem( value, y, XPSTR("Enable"), sub==subN ) ;
		if ( newvalue != value )
		{
			value = newvalue ? 1 : PROTO_OFF ;
		}
		else
		{
			value = pModule->protocol ;
		}
		if ( pModule->protocol != value )
		{
			pModule->protocol = value ;
		}
  	y+=FH ;
	} subN++ ;

	if ( pModule->protocol != PROTO_OFF )
	{
		if(t_pgOfs<=subN)
		{
  		lcd_puts_Pleft( y, PSTR(STR_PROTO));//sub==2 ? INVERS:0);
			attr = 0 ;
			if (sub==subN)
			{
				attr |= blink ;
				checkProtocolOptions( module ) ;
			}
			displayProtocol( 12*FW, y, pModule->protocol, attr ) ;
			if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
		}
		subN += 1 ;
		attr = 0 ;
		if ( pModule->protocol == PROTO_PXX )
		{
			if(t_pgOfs<=subN)
			{
				lcd_puts_Pleft( y, XPSTR( "RxNum" ) ) ;
				if (sub==subN)
				{
					attr |= blink ;
				}
	 			lcd_outdezAtt(  20*FW, y,  pModule->pxxRxNum, attr ) ;
				if ( attr )
				{
  			  CHECK_INCDEC_H_MODELVAR( pModule->pxxRxNum, 0, 63 ) ;
				}
				if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
			}
			subN += 1 ;
			if(t_pgOfs<=subN)
			{
				attr = 0 ;
				lcd_puts_Pleft( y, PSTR(STR_TYPE) ) ;
				pModule->sub_protocol = checkIndexed( y, XPSTR("\012\003""\006D16(X)D8(D) LRP   R9M   "), pModule->sub_protocol, (sub==subN) ) ;
				if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
			}
			subN += 1 ;
			if(t_pgOfs<=subN)
			{
				lcd_puts_Pleft( y, XPSTR("Chans") );
				pModule->channels = checkIndexed( y, XPSTR("\012\001""\00216 8"), pModule->channels, (sub==subN) ) ;
				if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
			}
			subN += 1 ;
			if ( pModule->sub_protocol == 0 )	// D16
			{
				if(t_pgOfs<=subN)
				{
					pModule->pxxDoubleRate = onoffMenuItem( pModule->pxxDoubleRate, y, XPSTR("Double Rate"), sub==subN ) ;
				}
				if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
			}
			subN += 1 ;
			if(t_pgOfs<=subN)
			{
				lcd_putsAtt( 0, y, XPSTR("Failsafe"), sub==subN ? INVERS : 0 ) ;
				if ( pModule->failsafeMode == 0 )
				{
	    		lcd_puts_Pleft( y, XPSTR("\012(Not Set)") ) ;
				}
				if ( sub == subN )
				{
					if ( checkForMenuEncoderLong( event ) )
					{
						s_currIdx = module ;
    				pushMenu( menuSetFailsafe ) ;
					}
				}
				if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
			}
			subN += 1 ;
			
			if(t_pgOfs<=subN)
			{
				lcd_puts_Pleft( y, PSTR(STR_COUNTRY) ) ;
				pModule->country = checkIndexed( y, XPSTR("\012\002""\003AmeJapEur"), pModule->country, (sub==subN) ) ;
				if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
			}
			subN += 1 ;
//			else if ( pModule->protocol == PROTO_XFIRE )
//			{
//				lcd_puts_Pleft( y, XPSTR("Enquire") ) ;
//  			if(sub==subN)
//				{
//					lcd_char_inverse( 0, y, 7*FW, 0 ) ;
//					if ( event == EVT_KEY_BREAK(KEY_MENU) )
//					{
//						killEvents(event) ;
//  					s_editMode = 0 ;
////						xfirePacketSend( 2, 0x28, (uint8_t *) "\0\352"  ) ;	//data = 00 EA
//						xfirePacketSend( 4, 0x2D, (uint8_t *) "\356\352\0\0"  ) ;	//data = 00 00
//					}
//				}

//				if ( ElrsConfig.filled )
//				{
//					lcd_puts_Pleft( 6*FH, ElrsConfig.name ) ;
//					if ( ElrsConfig.filled > 3 )
//					{
//						lcd_puts_Pleft( 7*FH, XPSTR("# Configs") ) ;
//						lcd_outdez(  10*FW, 7*FH, ElrsConfig.numConfigs ) ;
//					}
//				}
//				if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
//				subN += 1 ;
//			}
		}
		if(t_pgOfs<=subN)
		{
			uint8_t attr = 0 ;
 			lcd_puts_Pleft( y, PSTR(STR_PPM_1ST_CHAN));
 			if(sub==subN) { attr = blink ; CHECK_INCDEC_H_MODELVAR( pModule->startChannel, 0, 16 ) ; }
			lcd_outdezAtt(  14*FW, y, pModule->startChannel + 1, attr ) ;
		}
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
		subN += 1 ;
	}	
	if ( need_bind_range & 1 )
	{
		if(t_pgOfs<=subN)
		{
			lcd_puts_Pleft( y, PSTR(STR_BIND) ) ;
  		if(sub==subN)
			{
				lcd_char_inverse( 0, y, 4*FW, 0 ) ;
				if ( checkForMenuEncoderLong( event ) )
				{
					s_currIdx = module ;
//						uint32_t subp = pModule->sub_protocol & 0x3F ;
//						subp |= pModule->exsub_protocol << 6 ;
//					if ( ( pModule->protocol == PROTO_PXX ) || ( ( pModule->protocol == PROTO_MULTI ) && (( subp == M_FRSKYX ) || ( subp == M_FRSKYX2 ) || ( subp == M_FRSKYR9 )) ) )
					if ( pModule->protocol == PROTO_PXX )
					{
						pushMenu( menuBindOptions ) ;
					}
					else
					{
		  			BindRangeFlag[module] = PXX_BIND ;		    	//send bind code
						pushMenu( menuRangeBind ) ;
					}
				}
			}
			if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
		}
		subN += 1 ;
	}

	if ( need_bind_range & 4 )
	{
		if(t_pgOfs<=subN)
		{
			lcd_puts_Pleft( y, PSTR(STR_RANGE) ) ;
  		if(sub==subN)
			{
				lcd_char_inverse( 0, y, 11*FW, 0 ) ;
				if ( checkForMenuEncoderLong( event ) )
				{
  			  BindRangeFlag[module] = PXX_RANGE_CHECK ;		    	//send bind code or range check code
					s_currIdx = module ;
					pushMenu(menuRangeBind) ;
				}
			}
			if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
		}
		subN += 1 ;
	}
}


void menuProtocol(uint8_t event)
{
//	editAccessProtocol( 0, event ) ; //, 0 ) ;
	EditingModule = 1 ;
//	pushMenu( editOneProtocol ) ;
	editOneProtocol( event ) ;
}	

void menuTelemetry(uint8_t event)
{
	TITLE(PSTR(STR_TELEMETRY));
	static MState2 mstate2;

	uint8_t subN = 0 ;
	uint8_t blink = InverseBlink ;
	uint8_t sub = mstate2.m_posVert ;
	uint8_t subSub = g_posHorz ;
	uint8_t y = 1*FH ;
  uint8_t t_pgOfs ;
	uint8_t attr ;
	int8_t offset ;
	uint8_t b ;
//	if (sub==subN)
//	{
//		Columns = 1 ;
//	}
  
	event = mstate2.check_columns( event, 11-1 ) ;
	
	t_pgOfs = evalOffset( sub ) ;

	
	if(t_pgOfs<=subN)
	{
		attr = 0 ;
		b = g_model.FrSkyImperial ;
		if(sub==subN)
		{
			attr = blink ;
			CHECK_INCDEC_H_MODELVAR(b,0, 1) ;
			g_model.FrSkyImperial = b ;
		}
		lcd_puts_Pleft( y, XPSTR( "Units") ) ;
		lcd_putsAttIdx( 17*FW, FH, PSTR(STR_MET_IMP), b, attr ) ;
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;
	 
	if(t_pgOfs<=subN)
	{
		lcd_puts_Pleft( y, XPSTR( "RSSI Warn") ) ;
	
		attr = ( ( (sub==subN) && (subSub==0) ) ? InverseBlink : 0) ;
		offset = 45 ; // rssiOffsetValue( 0 ) ;
		lcd_outdezAtt( 15*FW, y, g_model.rssiOrange + offset, attr ) ;
  	if( attr) CHECK_INCDEC_H_MODELVAR( g_model.rssiOrange, -18, 30 ) ;
//	attr = ( ( (sub==subN) && (subSub==1) ) ? InverseBlink : 0) ;
//	b = 1-g_model.enRssiOrange ;
//	menu_lcd_onoff( PARAM_OFS+1, y, b, attr ) ;
//  if( attr) { CHECK_INCDEC_H_MODELVAR_0( b, 1 ) ; g_model.enRssiOrange = 1-b ; }
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;

//	if (sub==subN)
//	{
//		Columns = 1 ;
//	}
	if(t_pgOfs<=subN)
	{
		lcd_puts_Pleft( y, XPSTR( "RSSI Critical") ) ;
		attr = ( ( (sub==subN) && (subSub==0) ) ? InverseBlink : 0) ;
		offset = 42 ; // rssiOffsetValue( 1 ) ;
		lcd_outdezAtt( 15*FW, y, g_model.rssiRed + offset, attr ) ;
		if( attr) CHECK_INCDEC_H_MODELVAR( g_model.rssiRed, -17, 30 ) ;
//	attr = ( ( (sub==subN) && (subSub==1) ) ? InverseBlink : 0) ;
//	b = 1-g_model.enRssiRed ;
//	menu_lcd_onoff( PARAM_OFS+1, y, b, attr ) ;
//  if( attr) { CHECK_INCDEC_H_MODELVAR_0( b, 1 ) ; g_model.enRssiRed = 1-b ; }
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;

	for (int i=0; i<4; i++)
	{
		if(t_pgOfs<=subN)
		{
		
			uint32_t index = i ;
			uint8_t unit ;
    	lcd_puts_Pleft(y, PSTR(STR_A_CHANNEL)) ;
    	lcd_putc(FW, y, '1'+i);
			if ( i < 2 )
			{
    		putsTelemValue(16*FW, y, 255, i, (sub==subN && subSub==0 ? blink:0)|NO_UNIT ) ;
    		putsTelemValue( 21*FW, y, FrskyTelemetry[i].value, i,  NO_UNIT ) ;
    		unit = g_model.frsky.channels[index].units3_4 ;

    		if (sub==subN)
				{
					Columns = 1 ;
					if ( s_editMode)
					{
    		    switch (subSub)
						{
    		    	case 0:
    		        g_model.frsky.channels[i].lratio = checkIncDec16( g_model.frsky.channels[i].lratio, 0, 1000, EE_MODEL);
    		      break;
	  		      case 1:
    		        CHECK_INCDEC_H_MODELVAR( g_model.frsky.channels[i].units3_4, 0, 3) ;
    		      break;
    		    }
					}
    		}
			}
			else
			{
				lcd_outdezAtt( 16*FW, y, g_model.frsky.channels[index].ratio3_4, (sub==subN && subSub==0 ? blink:0)|PREC2 ) ;
				lcd_outdezAtt( 21*FW, y, TelemetryData[FR_A3+(index*1)], PREC2 ) ;
    		unit = g_model.frsky.channels[index].units3_4 ;
    		if (sub==subN)
				{
					Columns = 1 ;
					if ( s_editMode)
					{
    		    switch (subSub)
						{
    		    	case 0:
    		        g_model.frsky.channels[index].ratio3_4 = checkIncDec16( g_model.frsky.channels[index].ratio3_4, 0, 4800, EE_MODEL);
    		      break;
	  		      case 1:
    		        CHECK_INCDEC_H_MODELVAR( g_model.frsky.channels[index].units3_4, 0, 3);
    		      break;
    		    }
					}
    		}
			}
    	lcd_putsAttIdx(16*FW, y, XPSTR("\001v-VA"), unit, (sub==subN && subSub==1 ? blink:0));
			if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
		}
		subN += 1 ;
	}	

	if(t_pgOfs<=subN)
	{
		attr = (sub==subN) ? InverseBlink : 0 ;
		lcd_xlabel_decimal( 16*FW, y, g_model.rxVratio, attr|PREC1, XPSTR( "Rx Voltage") ) ;
  	lcd_putc(Lcd_lastPos, y, 'v' ) ;
		lcd_outdezAtt( 21*FW, y, convertRxv( TelemetryData[FR_RXV] ), PREC1 ) ;
		if( attr) { g_model.rxVratio = checkIncDec16( g_model.rxVratio, 0, 255, EE_MODEL ) ; }
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;
	 
	if(t_pgOfs<=subN)
	{
		uint8_t attr = 0 ;
  	if(sub==subN) { attr = blink ; CHECK_INCDEC_H_MODELVAR( g_model.numBlades, 1, 127 ) ; }
		lcd_xlabel_decimal( 14*FW, y, g_model.numBlades, attr, PSTR(STR_NUM_BLADES) ) ;
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;
	
	if(t_pgOfs<=subN)
	{
  	lcd_puts_Pleft( y, PSTR(STR_GPS_ALTMAIN) ) ;
  	menu_lcd_onoff( PARAM_OFS, y, g_model.FrSkyGpsAlt, sub==subN ) ;
  	if(sub==subN) CHECK_INCDEC_H_MODELVAR( g_model.FrSkyGpsAlt, 0, 1);
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;

	if(t_pgOfs<=subN)
	{
		lcd_puts_Pleft( y, XPSTR("Current Source" ) ) ;
		lcd_putsAttIdx( 15*FW, y, XPSTR("\004----A1  A2  Fas SC1 SC2 SC3 SC4 SC5 SC6 SC7 SC8 "), g_model.currentSource, attr ) ;
   	if(attr)
		{
			CHECK_INCDEC_H_MODELVAR( g_model.currentSource, 0, 3 + NUM_SCALERS ) ;
	  }
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;

}

void menuCustomTelemetry(uint8_t event)
{
	TITLE(PSTR(STR_CUSTOM_DISP)) ;
	static MState2 mstate2 ;

	uint32_t rows = 11 ;
	uint32_t page1limit = 5 ;
	
	event = mstate2.check_columns( event, rows ) ;
	
	uint8_t sub = mstate2.m_posVert ;

	uint8_t *pindex ;
	uint8_t *epindex ;
	uint8_t chr = '1' ;
	pindex = g_model.customDisplayIndex[0] ;
	
	if ( sub <= page1limit )
	{
		lcd_puts_P( 20*FW-4, 7*FH, XPSTR("->") ) ;
	}
	
	uint8_t subN = 0 ;
	if ( sub >= page1limit + 1 )
	{
		subN = page1limit + 1 ;
		pindex = g_model.customDisplayIndex[1] ;
		chr = '2' ;
	}
	lcd_putc( 15*FW, 0, chr ) ;

//	if ( event == EVT_ENTRY_UP )
//	{
//		// Returned from editing
//		if ( TextResult )
//		{
//			uint8_t index = sub - 1 ;
//			if ( index >= page1limit + 1 )
//			{
//				index -= page1limit + 1 ;
//			}
//			pindex[index] = TextIndex ;
//	    eeDirty(EE_MODEL) ;
//		}
//	}

	{
		for (uint8_t j=0; j<6; j++)
		{
			uint8_t x = ( j & 1 ) ? 32 : 0 ;
			uint8_t y = ( j + 2 ) * FH ;
		  uint8_t attr = ((sub==subN) ? InverseBlink : 0);
//			if ( attr )
//			{
//				if ( event == EVT_KEY_LONG(KEY_MENU) )
//				{
//					// Long MENU pressed
////					if ( sub >= page1limit + 1 )
////					{
////						sub -= page1limit + 1 ;
////					}
////					TextIndex = pindex[sub-1] ;
//					TextIndex = pindex[j] ;
//					if ( TextIndex >= TELEM_GAP_START + 8 )
//					{
//						TextIndex -= 8 ;
//					}
//  			  TextType = 0 ;
//  			  killEvents(event) ;
//					pushMenu(menuTextHelp) ;
//				}
//			}
			if ( pindex[j] )
			{
				uint32_t index = pindex[j] - 1 ;
				putsAttIdxTelemItems( x, y, pindex[j], attr | TSSI_TEXT ) ;
//				putsTelemetryChannel( 90, y, index, get_telemetry_value(index), 0, TELEM_UNIT ) ;
				if ( index < 2 )		// A1 or A2
				{
//  				lcd_outdezAtt( 120, y, get_telemetry_value(index), 0 ) ;
					lcd_putc( 98, y, '[' ) ;
					lcd_putc( 122, y, ']' ) ;
				}
			}
			else
			{
  	  	lcd_putsAtt(  x, y, HyphenString, attr ) ;
			}
  		if(sub==subN)
			{
				uint8_t val = pindex[j] ;
				CHECK_INCDEC_H_MODELVAR( val, 0, NUM_TELEM_ITEMS) ;
				pindex[j] = val ;
			}
			subN += 1 ;
		}
	}
}

void menuSensors(uint8_t event)
{
	TITLE(XPSTR("Sensors"));
	static MState2 mstate2 ;
	mstate2.check_columns( event, NUMBER_EXTRA_IDS + 6 + 4 ) ;
	uint8_t sub = mstate2.m_posVert ;
	uint8_t blink = InverseBlink ;
	uint8_t subN = 0 ;

	if ( sub < 6 )
	{
		displayNext() ;
    for( uint32_t i = 0 ; i < 6 ; i += 1 )
		{
			uint8_t y = (1+i) * FH ;
      lcd_putsAttIdx( FW*5, y, PSTR(STR_TELEM_ITEMS), i + 69, 0 ) ;
			alphaEditName( 11*FW, y, &g_model.customTelemetryNames[i*4], 4, sub==subN, (uint8_t *)XPSTR( "Custom Name") ) ;
	 		y += FH ;
			subN += 1 ;
		}
	}
	else if ( sub < 10 )
	{
		subN = 6 ;
		displayNext() ;
    for( uint32_t i = 0 ; i < 4 ; i += 1 )
		{
			uint8_t y = (1+i) * FH ;
      lcd_putsAttIdx( FW*5, y, PSTR(STR_TELEM_ITEMS), i + 83, 0 ) ;
			alphaEditName( 11*FW, y, &g_model.customTelemetryNames[(i+6)*4], 4, sub==subN, (uint8_t *)XPSTR( "Custom Name") ) ;
	 		y += FH ;
			subN += 1 ;
		}
	}
	else	
	{
		subN = 10 ;

 		for( uint32_t j=0 ; j < NUMBER_EXTRA_IDS+1 ; j += 1 )
		{
			uint8_t attr = (sub==subN) ? blink : 0 ;
			uint8_t y = (1+j)*FH ;

			if ( j < g_model.extraSensors )
			{
				lcd_outhex4( 0, y, g_model.extraId[j].id ) ;
				lcd_putsAttIdx( 11*FW, y, (char *)XPSTR(DestString), g_model.extraId[j].dest, attr ) ;
 			  if(attr)
				{
					CHECK_INCDEC_H_MODELVAR( g_model.extraId[j].dest, 0, NUM_SCALE_DESTS ) ;
 			  }
			}
			else
			{
				if ( j < NUMBER_EXTRA_IDS )
				{
					lcd_puts_Pleft( y, HyphenString ) ;
 			  	if(attr)
					{				
						lcd_char_inverse( 0, y, 4*FW, 0 ) ;
					}
				}
				else
				{
					lcd_puts_Pleft( y, XPSTR("Clear All") ) ;
 			  	if(attr)
					{				
						lcd_char_inverse( 0, y, 9*FW, 0 ) ;
						if ( checkForMenuEncoderLong( event ) )
						{
							uint32_t i ;
							for ( i = 0 ; i < NUMBER_EXTRA_IDS ; i += 1 )
							{
								g_model.extraId[i].dest = 0 ;
							}
							g_model.extraSensors = 0 ;
						}
					}
				}
			}
			subN += 1 ;
		}
	}
}

void menuVario(uint8_t event)
{
	TITLE(XPSTR("Vario"));
	static MState2 mstate2 ;
	mstate2.check_columns( event, 6 ) ;
	uint8_t sub = mstate2.m_posVert ;
	uint8_t blink = InverseBlink ;

	uint8_t subN = 0 ;
 	for( uint32_t j = 0 ; j < 7 ; j += 1 )
	{
		uint8_t b ;
		uint8_t attr = (sub==subN) ? blink : 0 ;
		uint8_t y = (1+j)*FH ;

		switch ( j )
		{
			case 0 :
				lcd_puts_Pleft( y, PSTR(STR_VARIO_SRC) ) ;
				lcd_putsAttIdx( 15*FW, y, PSTR(STR_VSPD_A2), g_model.varioData.varioSource, attr ) ;
   		  if(attr)
				{
					CHECK_INCDEC_H_MODELVAR( g_model.varioData.varioSource, 0, 2+NUM_SCALERS ) ;
   		  }
			break ;
				
			case 1 :
				lcd_puts_Pleft( y, PSTR(STR_2SWITCH) ) ;
				g_model.varioData.swtch = edit_dr_switch( 15*FW, y, g_model.varioData.swtch, attr, attr ? EDIT_DR_SWITCH_EDIT : 0, event ) ;
			break ;

			case 2 :
				lcd_puts_Pleft( y, PSTR(STR_2SENSITIVITY) ) ;
 				lcd_outdezAtt( 17*FW, y, g_model.varioData.param, attr) ;
   			if(attr)
				{
					CHECK_INCDEC_H_MODELVAR( g_model.varioData.param, 0, 50 ) ;
	   		}
			break ;

			case 3 :
				lcd_puts_Pleft( y, XPSTR("Base Freq.") ) ;
 				lcd_outdezAtt( 17*FW, y, g_model.varioData.baseFrequency, attr) ;
   			if(attr)
				{
					CHECK_INCDEC_H_MODELVAR( g_model.varioData.baseFrequency, -50, 50 ) ;
	   		}
			break ;

			case 4 :
				lcd_puts_Pleft( y, XPSTR("Offset Freq.") ) ;
 				lcd_outdezAtt( 17*FW, y, g_model.varioData.offsetFrequency, attr) ;
   			if(attr)
				{
					CHECK_INCDEC_H_MODELVAR( g_model.varioData.offsetFrequency, -20, 20 ) ;
	   		}
			break ;

			case 5 :
				lcd_puts_Pleft( y, XPSTR("Volume") ) ;
				if ( g_model.varioData.volume )
				{
 					lcd_outdezAtt( 17*FW, y, g_model.varioData.volume, attr) ;
				}
				else
				{
					lcd_putsAtt( FW*15, y, "Vol", attr ) ;
				}
   			if(attr)
				{
					CHECK_INCDEC_H_MODELVAR( g_model.varioData.volume, 0, NUM_VOL_LEVELS-1 ) ;
	   		}
			break ;

			case 6 :
	      b = g_model.varioData.sinkTones ;
				g_model.varioData.sinkTones = offonMenuItem( b, y, PSTR(STR_SINK_TONES), attr ) ;
			break ;
		}
		subN += 1 ;
	}
}





enum MODEL_INDEX
{
 M_MINDEX	 ,	
 M_MIXER		 ,	
 M_HELI		 ,	
 M_LIMITS	 ,	
 M_EXPO		 ,	
 M_MODES		 ,	
 M_CURVE		 ,	
 M_SWITCHES ,	
// M_MUSIC		 ,	
 M_SAFETY	 ,	
 M_GLOBALS	 ,	
 M_TELEMETRY,
 M_VOICE		 ,	
 M_TIMERS	 ,	
 M_MGENERAL ,	
 M_PROTOCOL	,
 M_MCOUNT
} ;

void menuModelIndex(uint8_t event)
{
	static MState2 mstate ;
	EditType = EE_MODEL ;
	if ( PopupData.PopupActive == 0 )
	{
		event = indexProcess( event, &mstate, M_MCOUNT-1-7 ) ;
		event = mstate.check_columns( event, IlinesCount-1 ) ;
	}
					
	SubMenuFromIndex = 1 ;
	switch ( SubmenuIndex )
	{
		case M_MIXER :
			if ( PopupData.PopupActive == 0 )
			{
				PopupData.PopupIdx = 0 ;
				PopupData.PopupActive = 3 ;
				SubmenuIndex = 0 ;
			}
////      pushMenu(menuMixer) ;
		break ;
		case M_SWITCHES :
      pushMenu(menuSwitches) ;
		break ;
		case M_TELEMETRY :
			if ( PopupData.PopupActive == 0 )
			{
				PopupData.PopupIdx = 0 ;
				PopupData.PopupActive = 2 ;
				SubmenuIndex = 0 ;
			}
		break ;
		case M_HELI :
      pushMenu(menuHeli) ;
		break ;
		case M_LIMITS :
      pushMenu(menuLimits) ;
		break ;
		case M_VOICE :
      pushMenu(menuVoiceAlarm) ;
		break ;
		case M_CURVE :
      pushMenu(menuCurve) ;
		break ;
		case M_TIMERS :
      pushMenu(menuTimers) ;
		break ;
		case M_EXPO :
			SingleExpoChan = 0 ;
			s_expoChan = 0 ;
      pushMenu(menuExpoAll) ;
		break ;
		case M_MODES :
      pushMenu(menuPhases) ;
		break ;
		case M_GLOBALS :
			if ( PopupData.PopupActive == 0 )
			{
				PopupData.PopupIdx = 0 ;
				PopupData.PopupActive = 1 ;
				SubmenuIndex = 0 ;
			}
		break ;
		case M_MGENERAL :
      pushMenu(menuModelGeneral) ;
		break ;
		case M_PROTOCOL :
      pushMenu(menuProtocol) ;
		break ;
		case M_SAFETY :
      pushMenu(menuSafetySwitches) ;
		break ;
////#ifdef BIG_SCREEN
////		case M_MUSIC :
////extern void menuModelMusic(uint8_t event) ;
////      pushMenu(menuModelMusic) ;
////		break ;
////#endif		
////		default :
////			SubMenuFromIndex = 0 ;
////			if ( event == EVT_ENTRY )
////			{
////				audioDefevent(AU_MENUS) ;
////				event = 0 ;
////			}
////		break ;
	}

	uint32_t sub = mstate.m_posVert ;
//	uint32_t y = FH ;
//	uint8_t blink = InverseBlink ;

	switch ( SubmenuIndex )
	{
		case M_MINDEX :
			lcd_putsAtt(0,0,"Model Setup",INVERS) ;
			IlinesCount = M_MCOUNT-1 ;
			sub += 1 ;

static const char *in_Strings[] =
{
	STR_Mixer,
	STR_heli_setup,
	STR_limits,
	STR_Expo,
	STR_Modes,
	STR_Curves,
	STR_Cswitches,
//	STR_Music,
	STR_Safety,
	STR_Globals,
	STR_Telemetry,
	STR_Voice,
	STR_Timer,
	STR_General,
	STR_Protocol
};
			
			if ( PopupData.PopupActive )
			{
				if ( PopupData.PopupActive == 1 )
				{
					sub = M_GLOBALS ;
				}
				else if ( PopupData.PopupActive == 2 )
				{
					sub = M_TELEMETRY ;
				}
				else
				{
					sub = M_MIXER ;
				}
			}
	
			displayIndex( in_Strings, M_MCOUNT-1-7, 7, sub ) ;

			if ( PopupData.PopupActive )
			{
				uint32_t mask ;
				if ( PopupData.PopupActive == 1 )
				{
					mask = 0x0007 ; ;
				}
				else if ( PopupData.PopupActive == 2 )
				{
					mask = 0x0618 ;	// No Logging
				}
				else
				{
					mask = 0x0060 ;
				}
				uint8_t popaction = doPopup( PSTR( STR_POPUP_GLOBALS ), mask, 13, event ) ;
  			if ( popaction == POPUP_SELECT )
				{
					uint8_t popidx = PopupData.PopupSel ;
					if ( popidx == 0 )	// gvars
					{
    	  		pushMenu(menuGlobals) ;
					}
					else if ( popidx == 1 )	// adjusters
					{
    	  		pushMenu(menuAdjust) ;
					}
					else if ( popidx == 2 )	// scalers
					{
    	  		pushMenu(menuScalers) ;
					}
					else if ( popidx == 3 )	// Telemetry
					{
    	  		pushMenu(menuTelemetry) ;
					}
					else if ( popidx == 4 )	// Custom
					{
	   	  		pushMenu(menuCustomTelemetry) ;
					}
					else if ( popidx == 5 )	// Mixer
					{
			      pushMenu(menuMixer) ;
					}
					else if ( popidx == 6 )	// Templates
					{
    	  		pushMenu(menuTemplates) ;
					}
//					else if ( popidx == 7 )	// Logging
//					{
////    	  		pushMenu(menuLogging) ;
//					}
					else if ( popidx == 9 )	// Vario
					{
    	  		pushMenu(menuVario) ;
					}
					else if ( popidx == 10 )	// Sensors
					{
    	  		pushMenu(menuSensors) ;
					}
//					else if ( popidx == 11 )	// HiRes
//					{
//    	  		pushMenu(menuHires) ;
//					}
					SubmenuIndex = sub ;
				}
  			if ( popaction == POPUP_EXIT )
				{
					SubmenuIndex = 0 ;
					mstate.m_posVert = sub - 1 ;
				}
			}

		break ;
	}
}

//void menuRangeBind(uint8_t event)
//{
//	static uint8_t binding = 0 ;
//	uint8_t *ptrFlag ;

//	ptrFlag = &BindRangeFlag[s_currIdx] ;
//	uint8_t flag = *ptrFlag & PXX_BIND ;
//  if ( event == EVT_ENTRY )
//	{
//		binding = flag ? 1 : 0 ;
//		BindTimer = 255 ;
////		DsmResponseFlag = 0 ;
//	}

//  if ( event == EVT_KEY_BREAK(KEY_EXIT) )
//	{
//    killEvents(event);
//		*ptrFlag = 0 ;
//		popMenu(false) ;
//	}

//	if ( binding && ( flag == 0 ) )
//	{
//		*ptrFlag = 0 ;
//		popMenu(false) ;
//	}

//	lcd_puts_Pleft( 2*FH, (binding) ? PSTR(STR_6_BINDING) : PSTR(STR_RANGE_RSSI) ) ;

//	if ( binding == 0 )
//	{
//		lcd_outdezAtt( 12 * FW, 4*FH, TelemetryData[FR_RXRSI_COPY], DBLSIZE);
//	}

////	if ( ( ( g_model.Module[0].protocol == PROTO_MULTI ) && ( s_currIdx == 0 ) ) ||
////		( ( g_model.Module[1].protocol == PROTO_MULTI ) && ( s_currIdx == 1 ) ) )
////	{
////		if ( binding )
////		{
////			if ( PrivateData[1] && BindTimer < 170 )
////			{
////				if ( ( PrivateData[0] & 0x08 ) == 0 )
////				{
////					*ptrFlag = 0 ;
////					popMenu(false) ;
////				}
////			}
////		}
////	}

////	if ( ( ( g_model.Module[0].protocol == PROTO_MULTI ) && ( (g_model.Module[0].sub_protocol & 0x3F) == M_DSM ) ) ||
////		( ( g_model.Module[1].protocol == PROTO_MULTI ) && ( (g_model.Module[1].sub_protocol & 0x3F) == M_DSM ) ) )
////	{
////		if ( DsmResponseFlag )
////		{
////			uint32_t module = s_currIdx ;
////			lcd_outhex4( 0, 6*FH, MultiResponseData ) ;

////			lcd_puts_Pleft( 7*FH, XPSTR("DSM2 22mS\015ch") ) ;
////			if ( MultiResponseData & 0x80 )
////			{
////				lcd_putc( 3*FW, 7*FH, 'X' ) ;
////			}
////			if ( MultiResponseData & 0x40 )
////			{
////				lcd_puts_Pleft( 7*FH, "\00511" ) ;
////			}
////  		lcd_outdez( 13*FW-2, 7*FH, MultiResponseData & 0x0F ) ;

////			int8_t x ;
////			uint8_t y ;
			
////			if ( *ptrFlag )
////			{
////				*ptrFlag = 0 ;
////				x = MultiResponseData & 0x0F ;	// # channels
////				y = (MultiResponseData & 0xC0 ) >> 2 ;	// DSMX, 11mS
////				if ( module )
////				{
////					g_model.xoption_protocol = (g_model.xoption_protocol & 0xF0) | x ;
////					y |= g_model.xppmNCH & 0x8F ;
////					g_model.xppmNCH = y ;	// Set DSM2/DSMX
					
////					g_model.Module[module].option_protocol = (g_model.Module[module].option_protocol & 0xF0) | x ;
////					y |= g_model.Module[module].channels & 0x8F ;
////					g_model.Module[module].channels = y ;	// Set DSM2/DSMX
////				}
////				else
////				{
////					g_model.option_protocol = (g_model.option_protocol & 0xF0) | x ;
////					y |= g_model.ppmNCH & 0x8F ;
////					g_model.ppmNCH = y ;	// Set DSM2/DSMX
					
////					g_model.Module[module].option_protocol = (g_model.Module[module].option_protocol & 0xF0) | x ;
////					y |= g_model.Module[module].channels & 0x8F ;
////					g_model.Module[module].channels = y ;	// Set DSM2/DSMX
////				}
////				protocolsToModules() ;
////				STORE_MODELVARS ;
////			}
////		}
////	}
	
////	if ( ( g_model.Module[1].protocol == PROTO_DSM2 ) && ( g_model.Module[1].sub_protocol == DSM_9XR ) )
////	{
////		if ( DsmResponseFlag )
////		{
////			lcd_outhex4( 0, 6*FH, (uint8_t)g_model.dsmMode ) ;

////			lcd_puts_Pleft( 7*FH, XPSTR("DSM2 22mS\015ch") ) ;
////			if ( g_model.dsmMode & 0x01 )
////			{
////				lcd_putc( 3*FW, 7*FH, 'X' ) ;
////			}
////			if ( g_model.dsmMode & 0x02 )
////			{
////				lcd_puts_Pleft( 7*FH, "\00511" ) ;
////			}
////  		lcd_outdez( 13*FW-2, 7*FH, g_model.Module[1].channels ) ;
////			if ( BindTimer > 40 )
////			{
////				BindTimer = 40 ;
////			}
////		}
////	}


//	if ( --BindTimer == 0 )
//	{
//  	audioDefevent(AU_WARNING2) ;
////		if ( DsmResponseFlag )
////		{
////			*ptrFlag = 0 ;
////		}
//	}
//}


//void menuBindOptions(uint8_t event)
//{
//	struct t_module *pModule = &g_model.Module[s_currIdx] ;
//	TITLE(XPSTR("Bind Options"));
//	static MState2 mstate2;
//	event = mstate2.check_columns( event, 2 ) ;
//  switch(event)
//  {
	
//    case EVT_KEY_BREAK(KEY_MENU):
//			if ( mstate2.m_posVert == 0 )
//		  {
////#ifdef ACCESS
//				if ( g_model.Module[s_currIdx].protocol == PROTO_ACCESS )
//				{
//					ModuleControl[s_currIdx].bindStep = BIND_START ;
//					ModuleSettings[s_currIdx].mode = MODULE_MODE_BIND ;
//					AccState = ACC_ACCST_BIND ;
//					popMenu( 0 ) ;
//				}
////				else
////#endif
////				{
////		  		BindRangeFlag[s_currIdx] = PXX_BIND ;		    	//send bind code
////					chainMenu( menuRangeBind ) ;
////				}
//			}
//		break ;
//	}
//	int8_t sub = mstate2.m_posVert ;
	
//	lcd_puts_Pleft( 1*FH, PSTR(STR_BIND) ) ;
//	if(sub == 0)
//	{
//		lcd_char_inverse( 0, 1*FH, 4*FW, 0 ) ;
//	}
//	pModule->highChannels = checkIndexed( 2*FH, XPSTR("\0""\001""\012Chans 1-8 Chans 9-16"), pModule->highChannels, (sub==1) ) ;
//	pModule->disableTelemetry = checkIndexed( 3*FH, XPSTR("\0""\001""\014Telemetry   No Telemetry"), pModule->disableTelemetry, (sub==2) ) ;
//}


uint8_t AccEntry ;

struct t_accessTelemetry
{
	uint16_t dataCrc ;
	uint16_t startTime ;
	uint8_t dataState ;
	uint8_t dataCount ;
	uint8_t dataReceived ;
	uint8_t AccessPacket[50] ;
} ;

void editAccessRegister( uint8_t module, uint8_t event )
{
	TITLE( XPSTR("Register") ) ;
	static MState2 mstate2 ;

	if ( ( event == EVT_KEY_BREAK(KEY_EXIT) ) || ( event == EVT_KEY_LONG(KEY_EXIT) ) )
	{
		ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
		AccState = ACC_NORMAL ;
		return ;
	}
	event = mstate2.check_columns( event, ( ModuleControl[module].registerStep != REGISTER_START ) ? 3 : 1 ) ;
	uint8_t sub = mstate2.m_posVert ;
	uint8_t subN = 0 ;
  
	uint8_t attr = sub==subN ? InverseBlink : 0 ;
	lcd_puts_Pleft( 1*FH, XPSTR( "Radio ID") ) ;
	EditType = EE_GENERAL ;
	alphaEditName( 11*FW, FH, g_eeGeneral.radioRegistrationID, 8, attr | ALPHA_NO_NAME, (uint8_t *)XPSTR( "Radio ID") ) ;
	EditType = EE_MODEL ;
	subN += 1 ;
	
	lcd_puts_Pleft( 2*FH, XPSTR("Rx UID") ) ;
	attr = sub==subN ? InverseBlink : 0 ;
	if ( attr )
	{
  	CHECK_INCDEC_H_MODELVAR( ModuleControl[module].registerModuleIndex, 0, 2 ) ;
	}
	lcd_outdezAtt( 12*FW, 2*FH, ModuleControl[module].registerModuleIndex, attr ) ;
	subN += 1 ;

	attr = sub==subN ? InverseBlink : 0 ;
  if ( ModuleControl[module].registerStep != REGISTER_START )
//			 ModuleControl[module].registerRxName[0] )
	{
//		lcd_puts_Pleft( 3*FH, XPSTR("Found") ) ;
		alphaEditName( 10*FW, 3*FH, ModuleControl[module].registerRxName, PXX2_LEN_RX_NAME, attr, (uint8_t *)XPSTR( "Rx Name") ) ;
	}
	else
	{
		lcd_puts_Pleft( 3*FH, XPSTR("Waiting") ) ;
	}
	
	subN += 1 ;

	attr = 0 ;
	if ( sub==subN )
	{
		attr = INVERS ;
		if ( checkForMenuEncoderBreak( event ) )
		{
  		if ( ModuleControl[module].registerStep != REGISTER_START )
			{
				ModuleControl[module].registerStep = REGISTER_RX_NAME_SELECTED ;
				ModuleControl[module].timeout = 0 ;
				ModuleSettings[module].mode = MODULE_MODE_REGISTER ;
			}
			killEvents( event ) ;
			s_editMode = false ;
		}
	}
  lcd_putsAtt( 0, 4*FH, XPSTR("REGISTER"), attr ) ;

	if ( ModuleControl[module].registerStep == REGISTER_OK )
	{
		ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
//		popupMessage( XPSTR("Registration Ok") ) ;
		if ( checkForMenuEncoderBreak( event ) )
		{		
			AccState = ACC_NORMAL ;
			killEvents( event ) ;
			s_editMode = false ;
		}
	}
}

void editAccessShare( uint8_t module, uint8_t event )
{
	TITLE( XPSTR("Sharing") ) ;

	// Need to determine when the module is shared to clear the name
	if ( ( event == EVT_KEY_BREAK(KEY_EXIT) ) || ( event == EVT_KEY_LONG(KEY_EXIT) ) )
	{
		ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
		AccState = ACC_NORMAL ;
		return ;
	}

	lcd_putsnAtt( 0, 2*FH, (char *)g_model.Access[module].receiverName[ModuleControl[module].bindReceiverIndex], PXX2_LEN_RX_NAME, 0 ) ;
	 
}

void editAccstAccessBind( uint8_t module, uint8_t event )
{
	TITLE( XPSTR("Bind") ) ;
//	static MState2 mstate2 ;
	
//	if ( event == EVT_ENTRY )
//	{
//		ModuleControl[module].bindReceiverCount = 0 ;
//		ModuleControl[module].timeout = 0 ;
//		ModuleControl[module].bindStep = BIND_START ;
//		ModuleSettings[module].mode = MODULE_MODE_BIND ;
//	}
  if ( event == EVT_ENTRY )
	{
		BindTimer = 255 ;
	}
	if ( event == EVT_KEY_BREAK(KEY_EXIT) )
	{
		ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
		AccState = ACC_NORMAL ;
		return ;
	}
	if ( --BindTimer == 0 )
	{
  	audioDefevent(AU_WARNING2) ;
	}
}

void editAccessBind( uint8_t module, uint8_t event )
{
	TITLE( XPSTR("Bind") ) ;
	static MState2 mstate2 ;

	if ( event == EVT_KEY_BREAK(KEY_EXIT) )
	{
		ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
		AccState = ACC_NORMAL ;
		return ;
	}
  if ( ModuleControl[module].bindReceiverCount )
	{
		lcd_puts_Pleft( FH, XPSTR("Select") ) ;
		uint32_t i ;
		uint32_t y = 2*FH ;

		event = mstate2.check_columns( event, ModuleControl[module].bindReceiverCount - 1 ) ;
		int8_t  sub    = mstate2.m_posVert ;
		uint8_t subN = 0 ;
		for ( i = 0 ; i < ModuleControl[module].bindReceiverCount ; i += 1 )
		{
			uint8_t attr = (sub == subN) ? InverseBlink : 0 ;
			lcd_putsnAtt( 0, y, (char *)ModuleControl[module].bindReceiversNames[i], PXX2_LEN_RX_NAME, attr ) ;
			if ( attr && checkForMenuEncoderBreak( event ) )
			{
				ModuleControl[module].bindReceiverNameIndex = i ;
				ModuleControl[module].bindStep = BIND_RX_NAME_SELECTED ;
				killEvents( event ) ;
				s_editMode = false ;
			}
			subN += 1 ;
			y += FH ;	 
		}
	}
	else
	{
		lcd_puts_Pleft( FH, XPSTR("Waiting") ) ;
	}
	
//	if ( ModuleSettings[module].mode == MODULE_MODE_BIND )
//	{
  	if ( ModuleControl[module].bindStep == BIND_OK )
		{
//			popupMessage( XPSTR("Bind Ok") ) ;
			if ( checkForMenuEncoderBreak( event ) )
			{		
//				ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
				AccState = ACC_NORMAL ;
			}
		}
//	}
}

void menuDeleteReceiver(uint8_t event)
{
	uint8_t action ;
	if ( s_currIdx < 2 )
	{
		action = yesNoMenuExit( event, (ModuleControl[s_currIdx].resetType == 0x01) ? XPSTR("Delete Receiver?") : XPSTR("Reset Receiver?") ) ;

		switch( action )
		{
  	  case YN_YES :
	 			ModuleSettings[s_currIdx].mode = MODULE_MODE_RESET ;
       	memset(g_model.Access[s_currIdx].receiverName[ModuleControl[s_currIdx].bindReceiverIndex], 0, PXX2_LEN_RX_NAME ) ;
			case YN_NO :
  	  break;
  	}
	}
	else
	{
    popMenu(false) ;
	}
}


void accesspopup( uint8_t event, uint8_t module )
{
	uint8_t popaction = doPopup( XPSTR("Bind\0Options\0Share\0Delete\0Reset\0Hardware"), 0x3F, 10, event ) ;
  if ( popaction == POPUP_SELECT )
	{
		uint8_t popidx = PopupData.PopupSel ;
		if ( popidx == 0 )	// Bind
		{
			killEvents(event);
			Tevent = 0 ;
			ModuleControl[module].bindStep = BIND_START ;
			ModuleControl[module].bindReceiverCount = 0 ;
			ModuleControl[module].timeout = 0 ;
			ModuleSettings[module].mode = MODULE_MODE_BIND ;
			AccState = ACC_BIND ;
		}
		else if ( popidx == 1 )	// Options
		{
			killEvents(event);
			Tevent = 0 ;
			ModuleControl[module].optionsState = 0 ;
			AccState = ACC_RXOPTIONS ;
		}
		else if ( popidx == 2 )	// Share
		{
			ModuleSettings[module].mode = MODULE_MODE_SHARE ;
			s_currIdx = module ;
			AccState = ACC_SHARE ;
			killEvents(event);
			Tevent = 0 ;
		}
		else if ( popidx == 3 )	// Delete
		{
			pushMenu(menuDeleteReceiver) ;
			ModuleControl[module].resetType = 0x01 ;
			s_currIdx = module ;
			killEvents(event);
			Tevent = 0 ;
//			pushMenu(menuAccessDelete);
		}
		else if ( popidx == 4 )	// Reset
		{
			pushMenu(menuDeleteReceiver) ;
			ModuleControl[module].resetType = 0xFF ;
			s_currIdx = module ;
			killEvents(event);
			Tevent = 0 ;
		}
		else if ( popidx == 5 )	// Hardware
		{
			ModuleControl[module].step = ModuleControl[module].bindReceiverIndex ;
			ModuleControl[module].timeout = 0 ;
			ModuleSettings[module].mode = MODULE_MODE_GET_HARDWARE_INFO ;
			AccState = ACC_RX_HARDWARE ;
			killEvents(event);
			Tevent = 0 ;
		}
		PopupData.PopupActive = 0 ;
	}
}

void editAccessRxOptions( uint8_t module, uint8_t event )
{
	// Rx UID in ModuleControl[module].bindReceiverIndex	
	TITLE( XPSTR("Rx Options") ) ;
	lcd_putsn_P( 12*FW, 0, (char *)g_model.Access[module].receiverName[ModuleControl[module].step], PXX2_LEN_RX_NAME ) ;
	
	static MState2 mstate2 ;
	EditType = 0 ;
	if ( ( event == EVT_KEY_BREAK(KEY_EXIT) ) || ( event == EVT_KEY_LONG(KEY_EXIT) ) )
	{
		ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
		AccState = ACC_NORMAL ;
		return ;
	}

	if ( ModuleControl[module].optionsState < 2 )
	{
		// Need to read settings from receiver
		// channelMapping
		lcd_puts_Pleft( 4*FH, XPSTR("    Reading") ) ;
		mstate2.m_posVert = 0 ;
		if ( ModuleControl[module].optionsState == 0 )
		{
			ModuleControl[module].rxtxSetupState = RECEIVER_SETTINGS_READ ;
//			ModuleControl[module].receiverSetupTimeout = get_tmr10ms() - 200 ;
			ModuleControl[module].receiverSetupReceiverId = ModuleControl[module].bindReceiverIndex ;
			ModuleSettings[module].mode = MODULE_MODE_RECEIVER_SETTINGS ;
			ModuleControl[module].optionsState = 1 ;
		}
		if ( ModuleControl[module].rxtxSetupState == RECEIVER_SETTINGS_OK )
		{
			ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
			ModuleControl[module].optionsState = 2 ;
		}
		return ;
	}
	
	event = mstate2.check_columns( event, 13 ) ;
	uint8_t  sub = mstate2.m_posVert ;
	uint8_t subN = 0 ;
	uint8_t y = 1*FH;
  uint8_t t_pgOfs ;
	uint8_t before ;
	uint8_t after ;
	uint8_t needWrite = 0 ;

	t_pgOfs = evalOffset( sub ) ;

	for(;;)
	{
		if(t_pgOfs<=subN)
		{
			before = ModuleControl[module].receiverSetupTelemetryDisabled ;
			after = checkIndexed( y, XPSTR("\0""\001""\014Telemetry   No Telemetry"), before, (sub==subN) ) ;
			if ( after != before )
			{
				needWrite = 1 ;
				ModuleControl[module].receiverSetupTelemetryDisabled = after ;
			}
			if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
		} 
		subN += 1 ;
		if(t_pgOfs<=subN)
		{
			before = ModuleControl[module].receiverSetupPwmRate ;
			after = checkIndexed( y, XPSTR("\0""\001""\00418mS 9mS"), before, (sub==subN) ) ;
			if ( after != before )
			{
				needWrite = 1 ;
				ModuleControl[module].receiverSetupPwmRate = after ;
			}
		
			if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
		} 
		subN += 1 ;
		if(t_pgOfs<=subN)
		{
			before = ModuleControl[module].receiverSetupTelePower ;
			after = checkIndexed( y, XPSTR("\0""\001""\011200mW Tele 25mW Tele"), before, (sub==subN) ) ;
			if ( after != before )
			{
				needWrite = 1 ;
				ModuleControl[module].receiverSetupTelePower = after ;
			}
		
			if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
		} 
		subN += 1 ;
		if(t_pgOfs<=subN)
		{
			before = ModuleControl[module].receiverSetupFPort ;
			after = checkIndexed( y, XPSTR("\0""\001""\005SPortFPort"), before, (sub==subN) ) ;
			if ( after != before )
			{
				needWrite = 1 ;
				ModuleControl[module].receiverSetupFPort = after ;
			}
		
			if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
		} 
		subN += 1 ;
		if(t_pgOfs<=subN)
		{
			before = ModuleControl[module].receiverSetupFPort2 ;
			after = checkIndexed( y, XPSTR("\0""\001""\006FPort1FPort2"), before, (sub==subN) ) ;
			if ( after != before )
			{
				needWrite = 1 ;
				ModuleControl[module].receiverSetupFPort2 = after ;
			}
		
			if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
		} 
		subN += 1 ;
		if(t_pgOfs<=subN)
		{
			before = ModuleControl[module].receiverSetupCH56pwm ;
			after = checkIndexed( y, XPSTR("\0""\001""\017CH5/6 PWM      CH5/6 Sbus/port"), before, (sub==subN) ) ;
			if ( after != before )
			{
				needWrite = 1 ;
				ModuleControl[module].receiverSetupCH56pwm = after ;
			}
		
			if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
		} 
		subN += 1 ;
	
		for ( uint32_t i = 0 ; i < 8 ; i += 1 )
		{
			if(t_pgOfs<=subN)
			{
				uint32_t value = (ModuleControl[module].channelMapping[i]+1) & 0x00FF ;
				lcd_puts_Pleft( y, XPSTR("Pin  CH") ) ;
				lcd_putc( 3*FW, y, i+'1' ) ;
				lcd_outdezAtt( 9*FW-2, y, value, (sub == subN) ? InverseBlink : 0 ) ;
			
				// edit value here
				if (sub == subN)
				{
					if ( value )
					{
						before = ModuleControl[module].channelMapping[i] ;
  					after = checkIncDec16( before, 0, 23, EE_MODEL ) ;
						if ( after != before )
						{
							needWrite = 1 ;
							ModuleControl[module].channelMapping[i] = after ;
						}
					}
				}
				if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
			}
			subN += 1 ;
		}
		break ;
	}

	if ( needWrite )
	{
		needWrite = 0 ;
		ModuleControl[module].rxtxSetupState = RECEIVER_SETTINGS_WRITE ;
//		ModuleControl[module].receiverSetupTimeout = get_tmr10ms() ;
		ModuleSettings[module].mode = MODULE_MODE_RECEIVER_SETTINGS ;

	}

}

void editAccessTxOptions( uint8_t module, uint8_t event )
{
	// Rx UID in ModuleControl[module].bindReceiverIndex	
	TITLE( XPSTR("Module Options") ) ;
	static MState2 mstate2 ;
	EditType = 0 ;

	if ( ( event == EVT_KEY_BREAK(KEY_EXIT) ) || ( event == EVT_KEY_LONG(KEY_EXIT) ) )
	{
		ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
		AccState = ACC_NORMAL ;
		return ;
	}

	if ( ModuleControl[module].optionsState < 2 )
	{
		// Need to read settings from receiver
		// channelMapping
		lcd_puts_Pleft( 4*FH, ModuleControl[module].rxtxSetupState == MODULE_SETTINGS_READ ? XPSTR("    Reading") : XPSTR("    Writing") ) ;
		mstate2.m_posVert = 0 ;
		if ( ModuleControl[module].optionsState == 0 )
		{
			ModuleControl[module].moduleRequest = 0 ;
//			ModuleControl[module].timeout = get_tmr10ms() - 100 ;
			ModuleControl[module].rxtxSetupState = MODULE_SETTINGS_READ ;
			ModuleSettings[module].mode = MODULE_MODE_GETSET_TX ;
			ModuleControl[module].optionsState = 1 ;
		}
		if ( ModuleControl[module].rxtxSetupState == MODULE_SETTINGS_OK )
		{
			ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
			ModuleControl[module].optionsState = 2 ;
		}
		return ;
	}
	
	event = mstate2.check_columns( event, 2 ) ;
	uint8_t sub = mstate2.m_posVert ;
	uint8_t subN = 0 ;
	uint8_t y = 1*FH;
  uint8_t t_pgOfs ;
	uint8_t before ;
	uint8_t after ;
//	uint8_t needWrite = 0 ;

	t_pgOfs = evalOffset( sub ) ;

	for(;;)
	{
		if(t_pgOfs<=subN)
		{
			uint8_t attr = 0 ;
			lcd_puts_Pleft( y, XPSTR("Power") ) ;
			if (sub == subN)
			{
				attr = InverseBlink ;
				before = ModuleControl[module].power ;
  			after = checkIncDec16( before, 0, 30, EE_MODEL ) ;
				if ( after != before )
				{
					ModuleControl[module].power = after ;
				}
			}
			lcd_outdezAtt( 15*FW, y, ModuleControl[module].power, attr ) ;
			if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
		} 
		subN += 1 ;
		
		if(t_pgOfs<=subN)
		{
			ModuleControl[module].moduleExtAerial = onoffMenuItem( ModuleControl[module].moduleExtAerial, y, XPSTR("Use External Ant."), sub==subN ) ;
//			g_model.Module[module].externalAntenna = ModuleControl[module].moduleExtAerial ;
		  if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
		}
		subN += 1 ;
		
		if(t_pgOfs<=subN)
		{
			lcd_puts_Pleft( y, XPSTR("Write") ) ;
			if(sub==subN)
			{
				lcd_char_inverse( 0, y, 5*FW, 0 ) ;
				if ( checkForMenuEncoderBreak( event ) )
				{
					ModuleControl[module].optionsState = 0 ;
					ModuleControl[module].moduleRequest = 1 ;
					ModuleSettings[module].mode = MODULE_MODE_GETSET_TX ;
					killEvents( event ) ;
  				s_editMode = false ;
			  }
			}
		  if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
		}
		subN += 1 ;

		break ;
	}
}


void editAccessRxHardware( uint8_t module, uint8_t event )
{
	TITLE( XPSTR("Rx Hardware") ) ;

	lcd_putsn_P( 13*FW, 0, (char *)g_model.Access[module].receiverName[ModuleControl[module].step], PXX2_LEN_RX_NAME ) ;
	if ( ( event == EVT_KEY_BREAK(KEY_EXIT) ) || ( event == EVT_KEY_LONG(KEY_EXIT) ) )
	{
		ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
		AccState = ACC_NORMAL ;
		return ;
	}

	if ( ModuleSettings[module].mode == MODULE_MODE_NORMAL )
	{
		uint32_t index ;
		lcd_puts_Pleft( FH, XPSTR("Hardware") ) ;
		lcd_puts_Pleft( 2*FH, XPSTR("Software") ) ;
		lcd_puts_Pleft( 3*FH, XPSTR("Type") ) ;
		lcd_puts_Pleft( 4*FH, XPSTR("Variant") ) ;
		
		lcd_outhex4( 10*FW-1, FH, ModuleControl[module].rxHwVersion + 0x0100 ) ;
		lcd_outhex4( 10*FW-1, 2*FH, ModuleControl[module].rxSwVersion + 0x0100 ) ;
//		lcd_putc( 12*FW, 3*FH, ModuleControl[module].rxModuleId + '0' ) ;
		index = ModuleControl[module].rxModuleId ;
		if ( index < 30 )
		{
			lcd_puts_P( 10*FW, 3*FH, PXX2receiversNames[index] ) ;
		}
		lcd_outhex4( 10*FW-1, 4*FH, ModuleControl[module].rxVariant ) ;
	}
}

void editAccessSpectrum( uint8_t module, uint8_t event )
{
	uint32_t i ;
	uint32_t j ;
	TITLE( XPSTR("Spectrum Analyser") ) ;
	EditType = 0 ;
	static MState2 mstate2;
	Columns = 1 ;
	event = mstate2.check_columns( event, 0 ) ;
	uint8_t subSub = g_posHorz;
		 
	// Could request to turn off receiver

	// If exiting menu	 
	if ( ( event == EVT_KEY_BREAK(KEY_EXIT) ) || ( event == EVT_KEY_LONG(KEY_EXIT) ) )
	{
		// Need a 1 sec delay
		lcd_puts_Pleft( 3*FH, "\006STOPPING" ) ;
	  refreshDisplay() ;
		ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
		AccState = ACC_NORMAL ;
		for ( i = 0 ; i < 50 ; i += 1 )
		{
//			CoTickDelay(10) ;			// 20mS for now
//			wdt_reset() ;
		}
		killEvents( event ) ;
		return ;
	}

//	if ( ModuleSettings[module].mode != MODULE_MODE_SPECTRUM_ANALYSER )
//	{
//		for ( i = 0 ; i < SPECTRUM_NUM_BARS ; i += 1 )
//		{
//			SharedMemory.SpectrumAnalyser.bars[i] = 0 ;
//		}
//    SharedMemory.SpectrumAnalyser.span = 40000000 ;  // 40MHz
//    SharedMemory.SpectrumAnalyser.freq = 2440000000U ;  // 2440MHz
//    SharedMemory.SpectrumAnalyser.step = SharedMemory.SpectrumAnalyser.span / 128 ; // LCD_W ;
//		ModuleControl[module].step = 0 ;
//		ModuleSettings[module].mode = MODULE_MODE_SPECTRUM_ANALYSER ;
//	}

	// Handle editing the fields
//  for ( i = 0 ; i < 2 ; i += 1 )
//	{
//    uint8_t attr = (subSub == i ? (s_editMode>0 ? INVERS|BLINK : INVERS) : 0);

//    switch (i)
//		{
//      case 0 :
//			{
//        uint16_t frequency = SharedMemory.SpectrumAnalyser.freq / 1000000 ;
//        uint16_t newValue ;
////        lcdDrawText(1, 10, "F:", 0);
////        lcdDrawNumber(lcdLastRightPos + 2, 10, frequency, attr);
////        lcdDrawText(lcdLastRightPos + 2, 10, "MHz", 0);
//				lcd_puts_Pleft( FH, XPSTR( "\002F:\010MHz" ) ) ;
//				lcd_outdezAtt( 8*FW-3, FH, SharedMemory.SpectrumAnalyser.freq / 1000000, attr ) ;
//        if (attr)
//				{
//          newValue = uint32_t(checkIncDec16( frequency, 2400, 2485, 0)) ;
//          SharedMemory.SpectrumAnalyser.freq = (uint32_t) newValue * 1000000 ;
//					if ( newValue != frequency )
//					{
//						for ( j = 0 ; j < SPECTRUM_NUM_BARS ; j += 1 )
//						{
//							SharedMemory.SpectrumAnalyser.bars[j] = 0 ;
//						}
//						ModuleControl[module].step = 0 ;
//					}
//        }
//        break ;
//      }

//      case 1 :
//			{	
//        uint8_t span = SharedMemory.SpectrumAnalyser.span / 1000000 ;
//        uint16_t newValue ;
////        lcdDrawText(lcdLastRightPos + 5, 10, "S:", 0);
////        lcdDrawNumber(lcdLastRightPos + 2, 10, reusableBuffer.spectrumAnalyser.span/1000000, attr);
////        lcdDrawText(lcdLastRightPos + 2, 10, "MHz", 0);
//				lcd_puts_Pleft( FH, XPSTR( "\015S:\022MHz" ) ) ;
// 				lcd_outdezAtt(  17*FW, FH, SharedMemory.SpectrumAnalyser.span / 1000000, attr ) ;
//        if (attr)
//				{
//          newValue = checkIncDec16( span, 1, 80, 0 ) ;
//          SharedMemory.SpectrumAnalyser.span = (uint32_t) newValue * 1000000 ;
//					if ( newValue != span )
//					{
//    				SharedMemory.SpectrumAnalyser.step = SharedMemory.SpectrumAnalyser.span / 128 ; // LCD_W ;
//						for ( j = 0 ; j < SPECTRUM_NUM_BARS ; j += 1 )
//						{
//							SharedMemory.SpectrumAnalyser.bars[j] = 0 ;
//						}
//						ModuleControl[module].step = 0 ;
//					}
//        }
//			}
//      break ;
//    }
//  }
		 
	// Display the data
	pushPlotType( PLOT_BLACK ) ;
  uint8_t peak_y = 1;
//  uint8_t peak_x = 0;
//  for ( i = 0 ; i < 128 /*LCD_W*/ ; i += 1 )
//  {
//    uint8_t h = min<uint8_t >( SharedMemory.SpectrumAnalyser.bars[i] >> 1, 128) ;
//    if (h > peak_y)
//    {
////      peak_x = i;
//      peak_y = h;
//    }
////		lcd_vline( i + X12OFFSET, /*LCD_H*/64-h, h ) ;
//  }
	popPlotType() ;

// Handle the peak bar??
//  int8_t y = max<int8_t>(FH, LCD_H - peak_y - FH);
//  lcdDrawNumber(min<uint8_t>(100, peak_x), y, ((reusableBuffer.spectrumAnalyser.freq - reusableBuffer.spectrumAnalyser.span / 2) + peak_x * (reusableBuffer.spectrumAnalyser.span / 128)) / 1000000, TINSIZE);
//  lcdDrawText(lcdLastRightPos, y, "M", TINSIZE);
		 
		 
}



//const uint8_t ProtocolOptions[2][7] = { {1,PROTO_ACCESS}, {4, PROTO_PPM,PROTO_PXX,PROTO_XFIRE,PROTO_ACCESS} };

//uint32_t checkProtocolOptions( uint8_t module )
//{
//	if ( module == 0 )
//	{
//		g_model.Module[module].protocol = PROTO_ACCESS ;
//		return 1 ;
//	}
//	uint8_t *options = (uint8_t *) ProtocolOptions[module] ;
//	uint32_t count = *options++ ;
//	uint8_t index = g_model.Module[module].protocol ;
//	uint8_t save = index ;
	
//	index = checkOutOfOrder( index, options, count ) ;
//	g_model.Module[module].protocol = index ;
	
//	if ( save != index )
//	{
//		g_model.Module[module].sub_protocol = 0 ;
//		return 1 ;
//	}
//	return 0 ;
//}


uint8_t byteToBCD( uint8_t value )
{
	return ( ( value >> 4) * 10  ) + (value & 0x0F) ;
}


void editAccessProtocol( uint8_t module, uint8_t event ) //, uint8_t start )
{
	static MState2 mstate2 ;
	uint32_t i ;
	
	EditType = EE_MODEL ;	
	
	if ( AccState == ACC_REG )
	{
		editAccessRegister( module, event ) ;
	}
	else if ( AccState == ACC_BIND )
	{
		editAccessBind( module, event ) ;
	}
	else if ( AccState == ACC_ACCST_BIND )
	{
		editAccstAccessBind( module, event ) ;
	}
	else if ( AccState == ACC_RXOPTIONS )
	{
		editAccessRxOptions( module, event ) ;
	}
	else if ( AccState == ACC_TXOPTIONS )
	{
		editAccessTxOptions( module, event ) ;
	}
	else if ( AccState == ACC_RX_HARDWARE )
	{
		editAccessRxHardware( module, event ) ;
	}
	else if ( AccState == ACC_SHARE )
	{
		editAccessShare( module, event ) ;
	}
	else if ( AccState == ACC_SPECTRUM )
	{
		editAccessSpectrum( module, event ) ;
	}
	else
	{
		struct t_module *pModule = &g_model.Module[module] ;
		struct t_access *pAccess = &g_model.Access[module] ;
		TITLE( XPSTR("Protocol") ) ;

		if ( ( event == EVT_ENTRY ) || AccEntry )
		{
			AccState = ACC_NORMAL ;
			mstate2.m_posVert = module ? 1 : 0 ;
			AccEntry = 0 ;
		}
	 
  	uint8_t t_pgOfs ;
		uint8_t blink = InverseBlink ;
		uint32_t rows = 13 ;
		if (pAccess->type)
		{
			rows = 8 ;
			if ( pAccess->type == 3 )
			{
				rows = 6 ;
			}
		}
		if ( pModule->protocol == PROTO_OFF )
		{
			rows = 0 ;
		}
		
		displayModuleName( 9*FW, 0, module, 0 ) ;
	
		if ( !PopupData.PopupActive )
		{
			event = mstate2.check_columns( event, rows ) ;
		}

		uint8_t sub = mstate2.m_posVert ;
		t_pgOfs = evalOffset( sub ) ;

		uint8_t y = 1*FH;

		uint8_t subN = 0 ;

		for(;;)
		{
			if(t_pgOfs<=subN)
			{
				uint8_t value ;
				uint8_t newvalue ;
				value = (pModule->protocol == PROTO_OFF) ? 0 : 1 ;
				newvalue = onoffMenuItem( value, y, XPSTR("Enable"), sub==subN ) ;
				if ( newvalue == 0 )
				{
					pModule->protocol = PROTO_OFF ;
				}
				else
				{
					pModule->protocol = PROTO_ACCESS ;
				}
  			y+=FH ;
			}
			if ( pModule->protocol == PROTO_OFF )
			{
				return  ;
			}

			subN += 1 ;

			if(t_pgOfs<=subN)
			{
				uint8_t attr = 0 ;
				lcd_puts_Pleft( y, PSTR(STR_PROTO));//sub==2 ? INVERS:0);
				if ( sub==subN )
				{
					attr |= blink ;
					checkProtocolOptions( module ) ;
				}
  			lcd_putsAtt( 6*FW, y, XPSTR("ACCESS"), attr ) ;
				if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
			} 
			subN += 1 ;
	
			if(t_pgOfs<=subN)
			{
//				uint8_t attr = 0 ;
				lcd_puts_Pleft( y, XPSTR("Mode"));
				uint32_t newType = pAccess->type == 3 ? 2 : pAccess->type ;
				newType = checkIndexed( y, XPSTR("\013\002""\012ACCESS    ACCST(D16)D8        "), pAccess->type, (sub==subN) ) ;
				if ( newType != pAccess->type )
				{
					pAccess->type = newType == 2 ? 3 : newType ;
				}
				if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
			} 
			subN += 1 ;
	
			if(t_pgOfs<=subN)
			{
				lcd_puts_Pleft( y, XPSTR("Channels"));
				if (pAccess->type == 3)
				{
					pModule->channels = checkIndexed( y, XPSTR("\013\000""\002 8"), pModule->channels, (sub==subN) ) ;
				}
				else if (pAccess->type)
				{
					pModule->channels = checkIndexed( y, XPSTR("\013\001""\00216 8"), pModule->channels, (sub==subN) ) ;
				}
				else
				{
					pAccess->numChannels = checkIndexed( y, XPSTR("\013\002""\002 81624"), pAccess->numChannels, (sub==subN) ) ;
				}
				if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
			}
			subN += 1 ;

			// Could remove this, editable in Register menu
//			if(t_pgOfs<=subN)
//			{
//				uint8_t attr = 0 ;
//				attr = (sub == subN) ? blink : 0 ;
//				alphaEditName( 11*FW, y, g_eeGeneral.radioRegistrationID, sizeof(g_eeGeneral.radioRegistrationID), attr, (uint8_t *)XPSTR( "Password") ) ;
//				if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
//			}
//			subN += 1 ;
			if ( pAccess->type < 3 )
			{
				if(t_pgOfs<=subN)
				{
					uint8_t attr = 0 ;
				  lcd_puts_Pleft( y, XPSTR("RxNum") ) ;
				  if(sub==subN)
					{
						attr = blink ;
  				  CHECK_INCDEC_H_MODELVAR( pModule->pxxRxNum, 0, 63 ) ;
					}
	  		  lcd_outdezAtt(  21*FW, y,  pModule->pxxRxNum, attr ) ;
					if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
				}
				subN += 1 ;
			}

			if (pAccess->type == 0)
			{
				if(t_pgOfs<=subN)
				{
					lcd_puts_Pleft( y, XPSTR("Register") ) ;
				  if(sub==subN)
					{
						lcd_char_inverse( 0, y, 8*FW, 0 ) ;
						if ( checkForMenuEncoderBreak( event ) )
						{
							ModuleControl[module].registerRxName[0] = '\0' ; 
							ModuleControl[module].registerStep = REGISTER_START ;
							ModuleControl[module].timeout = 0 ;
							ModuleSettings[module].mode = MODULE_MODE_REGISTER ;
							AccState = ACC_REG ;
							killEvents( event ) ;
							s_editMode = false ;
					  }
					}
					if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
				}
				subN += 1 ;

				for ( i = 0 ; i < 3 ; i += 1 )
				{
					if(t_pgOfs<=subN)
					{
						uint8_t attr = 0 ;
						uint32_t nameValid = ( ( g_model.Access[module].receiverName[i][0] ) && ( g_model.Access[module].receiverName[i][0] > ' ' ) ) ;
						if (sub==subN)
						{
							attr = InverseBlink ;
							if ( checkForMenuEncoderBreak( event ) )
							{
								if ( nameValid )
								{
									if ( !PopupData.PopupActive )
									{
										PopupData.PopupIdx = 0 ;
										PopupData.PopupActive = 1 ;
 				  				  killEvents(event);
										event = 0 ;		// Kill this off
										ModuleControl[module].bindReceiverIndex = i ;
									}
								
								
									// Handle options - Bind, Options, Share, Delete and Reset
							
									// Temporary, clear name only and reset Rx
	//								g_model.Access[module].receiverName[i][0] = 0 ;

	//								ModuleControl[module].bindReceiverIndex = i ;
	//								ModuleSettings[module].mode = MODULE_MODE_RESET ;


								}
								else
								{
									ModuleControl[module].bindStep = BIND_START ;
									ModuleControl[module].bindReceiverIndex = i ;
									ModuleControl[module].bindReceiverId = i ;
									ModuleControl[module].bindReceiverCount = 0 ;
									ModuleControl[module].timeout = 0 ;
									ModuleSettings[module].mode = MODULE_MODE_BIND ;
									AccState = ACC_BIND ;
								}
  	  					s_editMode = false ;
								killEvents( event ) ;
							}
	//						if ( (ModuleControl[module].bindReceiverIndex == i) && (ModuleSettings[module].mode == MODULE_MODE_RESET) )
	//						{
	//							lcd_puts_P( 19*FW, y, XPSTR("RS") ) ;
	//						}
						}
						lcd_puts_Pleft( y, XPSTR("Receiver") ) ;
						lcd_putc( 8*FW, y, '1' + i ) ;
						if ( nameValid )
						{
							lcd_putsnAtt( 11*FW, y, (char *)g_model.Access[module].receiverName[i], PXX2_LEN_RX_NAME, attr ) ;
						}			
						else
						{
		//					if ( ( attr ) && (AccState == ACC_BIND) )
		//					{
		//						if ( ModuleControl[module].bindReceiverCount )
		//						{
							
		//						}
		//						else
		//						{
		//							lcd_putsAtt( 11*FW, y, XPSTR("----"), BLINK ) ;
		//						}
		//					}
		//					else
		//					{
								lcd_putsAtt( 11*FW, y, XPSTR("BIND"), attr ) ;
		//					}
						}
						if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
					}
					subN += 1 ;
				}
				if((y)>(SCREEN_LINES-1)*FH) break ;
			}

			if ( pAccess->type < 3 )
			{
				if(t_pgOfs<=subN)
				{
					lcd_putsAtt( 0, y, XPSTR("Failsafe"), sub==subN ? INVERS : 0 ) ;
	//				if ( ( PrivateData[1] && ( ( PrivateData[0] & 0x20 ) == 0 ) ) )
	//				{
	//					lcd_putsAtt( FW*9, y, "N/A", BLINK ) ;
	//				}
	//				else
					{
						if ( pModule->failsafeMode == 0 )
						{
	    				lcd_puts_Pleft( y, XPSTR("\012(Not Set)") ) ;
						}
						if ( sub == subN )
						{
							if ( checkForMenuEncoderLong( event ) )
							{
								s_currIdx = module ;
	//    				  pushMenu( menuSetFailsafe ) ;
							}
						}
					}
				  if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
				}
				subN += 1 ;
			}

			if (pAccess->type == 0)
			{
				if(t_pgOfs<=subN)
				{
					lcd_puts_Pleft( y, XPSTR("Hardware") ) ;
					if ( ModuleControl[module].swVersion )
					{
						lcd_putc( 9*FW+FWNUM, y, '.' ) ;
//						lcd_putc( 9*FW+FWNUM*3, y, '.' ) ;
						lcd_putc( 9*FW+FWNUM*4, y, '/' ) ;
						lcd_putc( 9*FW+FWNUM*6, y, '.' ) ;
//						lcd_putc( 9*FW+FWNUM*8, y, '.' ) ;
						if ( ModuleControl[module].hwVersion == 0xFFFF )
						{
							ModuleControl[module].hwVersion = 0 ;
						}
						lcd_putc( 9*FW, y, (ModuleControl[module].hwVersion >> 8) +'1' ) ;
						lcd_outdezAtt( 9*FW+FWNUM*4, y, byteToBCD( ModuleControl[module].hwVersion & 0xFF ), PREC1 ) ;

//							lcd_outhex4( 9*FW, y, ModuleControl[module].hwVersion ) ;
						
						lcd_putc( 9*FW+FWNUM*5, y, (ModuleControl[module].swVersion >> 8) +'1' ) ;
						lcd_outdezAtt( 9*FW+FWNUM*9, y, byteToBCD( ModuleControl[module].swVersion & 0xFF ), PREC1 ) ;

//						lcd_outhex4( 14*FW, y, ModuleControl[module].swVersion ) ;
						
//						lcd_putc( 11*FW, y, '/' ) ;
//						lcd_outdezAtt( 11*FW-1, y, byteToBCD( ModuleControl[module].hwVersion), PREC1 ) ;
//						lcd_outdezAtt( 14*FW, y, byteToBCD( ModuleControl[module].swVersion), PREC1 ) ;
const char *moduleName( uint32_t index ) ;
						if ( y < (SCREEN_LINES-1)*FH )
						{
							lcd_puts_P( 10*FW-4, y+FH, moduleName( ModuleControl[module].moduleId ) ) ;
						}
//						lcd_putc( 20*FW, y, ModuleControl[module].moduleId + '0' ) ;
						if ( ModuleControl[module].variant < 4 )
						{
							lcd_putsAttIdx( 9*FW+FWNUM*9+2, y, XPSTR("\004    FCC LBT FLEX"), ModuleControl[module].variant, 0 ) ;
//							lcd_putsAttIdx( 14*FW+2, y, XPSTR("\006      FCC   EU-LBTFLEX"), ModuleControl[module].variant, 0 ) ;
						}
					}
				  if(sub==subN)
					{
						lcd_char_inverse( 0, y, 8*FW, 0 ) ;
						if ( checkForMenuEncoderBreak( event ) )
						{
							ModuleControl[module].step = -1 ;
							ModuleControl[module].timeout = 0 ;
							ModuleSettings[module].mode = MODULE_MODE_GET_HARDWARE_INFO ;
							killEvents( event ) ;
  	  				s_editMode = false ;
						
// Temporary, kick off a request
//extern void setupPulsesAccess( uint32_t module ) ;
//							setupPulsesAccess( 0 ) ;
						
						}
					}
					if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
				}
				subN += 1 ;

				if(t_pgOfs<=subN)
				{
					lcd_puts_Pleft( y, XPSTR("Options") ) ;
				  if(sub==subN)
					{
						lcd_char_inverse( 0, y, 7*FW, 0 ) ;
						if ( checkForMenuEncoderBreak( event ) )
						{
							ModuleControl[module].optionsState = 0 ;
							AccState = ACC_TXOPTIONS ;
							killEvents( event ) ;
 	  					s_editMode = false ;
					  }
					}
					if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
				}
				subN += 1 ;

				if(t_pgOfs<=subN)
				{
					lcd_puts_Pleft( y, XPSTR("Spectrum") ) ;
				  if(sub==subN)
					{
						lcd_char_inverse( 0, y, 8*FW, 0 ) ;
						if ( checkForMenuEncoderBreak( event ) )
						{
	//						ModuleControl[module].optionsState = 0 ;
							AccState = ACC_SPECTRUM ;
							killEvents( event ) ;
							s_editMode = 0 ;
					  }
					}
			
					if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
				}
				subN += 1 ;
			}
			
			if (pAccess->type )
			{
				if(t_pgOfs<=subN)
				{
					lcd_puts_Pleft( y, PSTR(STR_COUNTRY) ) ;
					pModule->country = checkIndexed( y, XPSTR("\013\001""\003AmeEur"), pModule->country, (sub==subN) ) ;
				  if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
				}subN++;

				if(t_pgOfs<=subN)
				{
					lcd_puts_Pleft( y, PSTR(STR_BIND) ) ;
  		  	if(sub==subN)
					{
						lcd_char_inverse( 0, y, 4*FW, 0 ) ;
						if ( checkForMenuEncoderLong( event ) )
						{
							s_currIdx = module ;
							pushMenu( menuBindOptions ) ;
						}
					}
					if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
				}	subN += 1 ;
			}

			if(t_pgOfs<=subN)
			{
				lcd_puts_Pleft( y, PSTR(STR_RANGE) ) ;
  			if(sub==subN)
				{
					lcd_char_inverse( 0, y, 11*FW, 0 ) ;
					if ( checkForMenuEncoderBreak( event ) )
					{
  		  	  BindRangeFlag[module] = PXX_RANGE_CHECK ;		    	//send bind code or range check code
						s_currIdx = module ;
						pushMenu(menuRangeBind) ;
						killEvents( event ) ;
 	  				s_editMode = false ;
					}
				}
				if((y+=FH)>(SCREEN_LINES-1)*FH) break ;
			}
			subN += 1 ;
			break ;
		}
		if ( PopupData.PopupActive )
		{
			Tevent = event ;
			accesspopup( event, module ) ;
  	  s_editMode = false;
		}
	}
}


	
	

