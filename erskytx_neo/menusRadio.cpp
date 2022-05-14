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
#include "audio.h"
#include "sound.h"

#include "stickslbm.h"

struct t_calib
{
	int16_t midVals[4] ;
	int16_t loVals[4] ;
	int16_t hiVals[4] ;
	uint8_t idxState;
} ;

struct t_calib Cal_data ;

extern uint16_t g_vbat10mV ;
extern uint16_t UsbV ;
extern uint8_t CurrentVolume ;

extern const uint8_t GvaString[] ;
extern uint8_t VoiceFileType ;
extern char SelectedVoiceFileName[] ;
extern uint8_t FileSelectResult ;
extern uint8_t BTjoystickActive ;

extern void startJoystick() ;

void menuVersion(uint8_t event) ;
void menuGlobalVoiceAlarm(uint8_t event) ;
void menuSelectVoiceFile(uint8_t event) ;

static void displayGPSformat( uint16_t x, uint16_t y, uint8_t attr )
{
	lcd_putsAttIdx( x, y, XPSTR("\012DD mm.mmmmDD.dddddd "), g_eeGeneral.gpsFormat, attr ) ;
}

uint32_t calibration( uint8_t event )
{
//	TITLE(PSTR(STR_Calibration)) ;
	lcd_putsAtt( 0, 0, "Calibration", 0 ) ;

	uint32_t numAnalogCals = 4 ;
	 
  for(uint32_t i=0; i < numAnalogCals ; i++)
	{ //get low and high vals for sticks and trims
		int16_t vt = AnalogData[i] ; // anaIn( i ) ;
    Cal_data.loVals[i] = min(vt,Cal_data.loVals[i]);
    Cal_data.hiVals[i] = max(vt,Cal_data.hiVals[i]);
//    if(i>=4)
//		{
//			uint32_t j = i - 4 ;
//			j = 1 << j ;
//			if ( ( g_eeGeneral.potDetents & j ) == 0 )
//				Cal_data.midVals[i] = (Cal_data.loVals[i] + Cal_data.hiVals[i])/2;
//		}
  }

//  scroll_disabled = Cal_data.idxState; // make sure we don't scroll while calibrating

  switch(event)
  {
	  case EVT_ENTRY:
      Cal_data.idxState = 0 ;
    break ;

  	case EVT_KEY_BREAK(KEY_MENU):
      Cal_data.idxState++ ;
      if(Cal_data.idxState==3)
      {
        audioDefevent(AU_MENUS) ;
				ee32StoreGeneral() ;
//        STORE_GENERALVARS ;     //eeWriteGeneral();
        Cal_data.idxState = 0 ;
				return 0 ;
      }
    break ;
  }

	switch(Cal_data.idxState)
  {
  	case 0:
      lcd_puts_Pleft( 3*FH, "\003[MENU] TO START" ) ;//, 15, sub>0 ? INVERS : 0);
    break;

	  case 1: //get mid
      //SET MIDPOINT
    
  		for(uint8_t i=0; i<numAnalogCals; i++)
      {
        Cal_data.loVals[i] =  15000;
        Cal_data.hiVals[i] = -15000;
        Cal_data.midVals[i] = AnalogData[i] ; // anaIn( i );
      }
      lcd_puts_Pleft( 2*FH, "\005SET MIDPOINT" ) ;//, 12, sub>0 ? INVERS : 0);
      lcd_puts_P(3*FW, 3*FH, "[MENU] WHEN DONE" ) ;//, 16, sub>0 ? BLINK : 0);
    break;

	  case 2:
      //MOVE STICKS/POTS
      //[MENU]
//			StickScrollAllowed = 0 ;

      for(uint8_t i=0; i<numAnalogCals; i++)
			{
	      if(abs(Cal_data.loVals[i]-Cal_data.hiVals[i])>50)
				{
          g_eeGeneral.calibMid[i]  = Cal_data.midVals[i] ;
          int16_t v = Cal_data.midVals[i] - Cal_data.loVals[i] ;
          g_eeGeneral.calibSpanNeg[i] = v - v/64 ;
          v = Cal_data.hiVals[i] - Cal_data.midVals[i] ;
          g_eeGeneral.calibSpanPos[i] = v - v/64 ;
        }
			}
  		g_eeGeneral.chkSum = evalChkSum() ;
      lcd_puts_Pleft( 2*FH, "\003MOVE STICKS" ) ; //, 16, sub>0 ? INVERS : 0);
      lcd_puts_P(3*FW, 3*FH, "[MENU] WHEN DONE" ) ; //, 16, sub>0 ? BLINK : 0);
    break;
  }

  doMainScreenGraphics() ;
	
	return 1 ;	 
}


void menuCalib(uint8_t event)
{
	static MState2 mstate2 ;
	mstate2.check_columns( event, 2-1 ) ;

//	if ( event == EVT_KEY_LONG(KEY_LEFT) )
//	{
//		chainMenu( menuProcIndex ) ;
//	}

  mstate2.m_posVert = 0 ; // make sure we don't scroll or move cursor here

	calibration( event ) ;

}

void menuAlarms( uint8_t event )
{
	TITLE( STR_Alarms ) ;
	static MState2 mstate2;
	event = mstate2.check_columns(event, 5-1) ;

	uint32_t sub = mstate2.m_posVert ;
	uint32_t y = FH ;
	uint8_t blink = InverseBlink ;
	uint8_t attr ;
	uint8_t subN = 0 ;
	
	attr = LEFT ;
  lcd_puts_Pleft( y,PSTR(STR_BATT_WARN));
  if(sub==subN)
	{
		attr = blink | LEFT ;
		g_eeGeneral.vBatWarn = checkIncDec16( g_eeGeneral.vBatWarn, 50, 90, EE_GENERAL) ;
	}
  putsVolts(PARAM_OFS, y, g_eeGeneral.vBatWarn, attr);
  y += FH ;
	subN += 1 ;

  attr = 0 ;//LEFT ;
  lcd_puts_Pleft( y,PSTR(STR_INACT_ALARM)) ;
	lcd_putc( PARAM_OFS+2*FW, y, 'm' ) ;
  if(sub==subN)
	{
		attr = blink ;
		g_eeGeneral.inactivityTimer = checkIncDec16( g_eeGeneral.inactivityTimer, -10, 110, EE_GENERAL) ; //0..120minutes
	}
  lcd_outdezAtt(PARAM_OFS+2*FW-2, y, g_eeGeneral.inactivityTimer+10, attr);
  y += FH ;
	subN += 1 ;

	lcd_puts_P( FW, y, PSTR(STR_VOLUME) ) ;
	int8_t value = g_eeGeneral.inactivityVolume + ( NUM_VOL_LEVELS-3 ) ;
	lcd_outdezAtt( PARAM_OFS+2*FW, y, value, (sub==subN) ? blink : 0 ) ;
	if(sub==subN)
	{
		CHECK_INCDEC_H_GENVAR_0( value,NUM_VOL_LEVELS-1);
		g_eeGeneral.inactivityVolume = value - ( NUM_VOL_LEVELS-3 ) ;
	} 	
	y += FH ;
	subN += 1 ;
	
  uint8_t b = g_eeGeneral.disableThrottleWarning;
  g_eeGeneral.disableThrottleWarning = offonMenuItem( b, y, PSTR(STR_THR_WARNING), sub == subN ) ;
 	y += FH ;
	subN += 1 ;

//  b = g_eeGeneral.disableAlarmWarning;
//  g_eeGeneral.disableAlarmWarning = offonMenuItem( b, y, PSTR(STR_ALARM_WARN), sub == subN ) ;
// 	y += FH ;
//	subN += 1 ;

  b = g_eeGeneral.disableRxCheck;
  g_eeGeneral.disableRxCheck = offonMenuItem( b, y, XPSTR("Receiver Warning"), sub == subN ) ;
 	y += FH ;
	subN += 1 ;
	 
}



void menuBluetooth( uint8_t event )
{
	uint8_t value ;
	TITLE( "Bluetooth" ) ;
	static MState2 mstate2;
	event = mstate2.check_columns(event, 1-1) ;
	uint32_t sub = mstate2.m_posVert ;
	uint8_t subN = 0 ;

	value = BTjoystickActive ;
	BTjoystickActive = onoffMenuItem( BTjoystickActive, 2*FH, XPSTR("Enable Joystick"), sub == subN ) ;
	
	if ( BTjoystickActive )
	{
		if ( value == 0 )
		{
			startJoystick() ;
		}
	}
}



void menuGeneral( uint8_t event )
{
	TITLE( STR_General ) ;
	
	static MState2 mstate2;
	mstate2.check_columns(event, 6-1) ;

//	uint16_t attr ;
	uint8_t subN = 0 ;
  int8_t sub = mstate2.m_posVert ;
	uint16_t y = FH ;
	uint8_t blink = InverseBlink ;

	alphaEditName( 11*FW-2, y, (uint8_t *)g_eeGeneral.ownerName, sizeof(g_eeGeneral.ownerName), sub==subN, (uint8_t *)XPSTR( "Owner Name") ) ;
	validateName( (uint8_t *)g_eeGeneral.ownerName, sizeof(g_eeGeneral.ownerName) ) ;
 	y += FH ;
	subN += 1 ;

//	lcd_puts_Pleft( y,PSTR(STR_LANGUAGE));
//	lcd_putsAttIdx( 11*FW, y, XPSTR("\012   ENGLISH  FRANCAIS   DEUTSCH NORWEGIAN   SWEDISH   ITALIAN    POLISHVIETNAMESE   SPANISH"),g_eeGeneral.language,(sub==subN ? blink:0));
//	if(sub==subN) CHECK_INCDEC_H_GENVAR_0( g_eeGeneral.language, 8 ) ;
//	setLanguage() ;
// 	y += FH ;
//	subN += 1 ;

  uint8_t b = g_eeGeneral.disableSplashScreen;
  g_eeGeneral.disableSplashScreen = offonMenuItem( b, y, PSTR(STR_SPLASH_SCREEN), sub == subN ) ;
 	y += FH ;
	subN += 1 ;

  b = g_eeGeneral.hideNameOnSplash;
  g_eeGeneral.hideNameOnSplash = offonMenuItem( b, y, PSTR(STR_SPLASH_NAME), sub == subN ) ;
 	y += FH ;
	subN += 1 ;

//	g_eeGeneral.disableBtnLong = onoffMenuItem( g_eeGeneral.disableBtnLong, y, XPSTR("No ENC. as exit"), sub == subN ) ;
// 	y += FH ;
//	subN += 1 ;

  lcd_puts_Pleft( y, XPSTR("GPS Format") ) ;
  displayGPSformat( 11*FW, y, (sub==subN ? blink:0) ) ;
  if(sub==subN)
	{
		CHECK_INCDEC_H_GENVAR_0(g_eeGeneral.gpsFormat, 1 ) ;
	}
 	y += FH ;
	subN += 1 ;
      
	b = g_eeGeneral.forceMenuEdit ;
	g_eeGeneral.forceMenuEdit = onoffMenuItem( b, y, PSTR(STR_MENU_ONLY_EDIT), sub == subN ) ;
	y += FH ;
	subN += 1 ;

  g_eeGeneral.altMixMenu = onoffMenuItem( g_eeGeneral.altMixMenu, y, XPSTR("Mix Menu Details"), sub == subN ) ;

}

void menuDisplay( uint8_t event )
{
//	TITLE( "Display" ) ;
	lcd_putsAtt(0,0, PSTR(STR_Display), INVERS) ;
	
	static MState2 mstate2;
	mstate2.check_columns(event, 4-1) ;

	uint16_t attr ;
	uint8_t subN = 0 ;
  int8_t sub = mstate2.m_posVert ;
	uint16_t y = FH ;
	uint16_t z ;
	uint8_t blink = InverseBlink ;
	uint32_t i ;

	lcd_puts_Pleft( y, PSTR(STR_BRIGHTNESS) ) ;
	if(sub==subN)
	{
		g_eeGeneral.bright = checkIncDec16( g_eeGeneral.bright, 1, 100, 0 ) ;
		setBacklightBrightness( g_eeGeneral.bright ) ;		
		attr = blink ;
	}
	lcd_outdezAtt( PARAM_OFS+3*FW, y, g_eeGeneral.bright, attr ) ;
	y += FH ;
	subN += 1 ;

	lcd_puts_Pleft( y,PSTR(STR_LIGHT_SWITCH));
	putsDrSwitches(PARAM_OFS-FW,y,g_eeGeneral.lightSw,sub==subN ? blink : 0);
	if(sub==subN)
	{
		int8_t ival ;
		ival = switchUnMap( g_eeGeneral.lightSw ) ;
  	g_eeGeneral.lightSw = switchMap( checkIncDec16(ival, -MaxSwitchIndex, MaxSwitchIndex,EE_GENERAL|INCDEC_SWITCH) ) ;
	}
	y += FH ;
	subN += 1 ;

	for ( i = 0 ; i < 2 ; i += 1 )
	{
		uint8_t b ;
    lcd_puts_Pleft( y,( i == 0) ? PSTR(STR_LIGHT_AFTER) : PSTR(STR_LIGHT_STICK) );
		b = ( i == 0 ) ? g_eeGeneral.lightAutoOff : g_eeGeneral.lightOnStickMove ;

    if(b) {
    	  lcd_outdezAtt(PARAM_OFS, y, b*5,LEFT|(sub==subN ? blink : 0));
    	  lcd_putc(Lcd_lastPos, y, 's');
    }
    else
    	  lcd_putsAtt(PARAM_OFS, y, PSTR(STR_OFF),(sub==subN ? blink:0));
    if(sub==subN) b = checkIncDec16( b, 0, 600/5, EE_GENERAL) ;
		if ( i == 0 )
		{
			g_eeGeneral.lightAutoOff = b ;
		}
		else
		{
			g_eeGeneral.lightOnStickMove = b ;
		}
  	y += FH ;
		subN += 1 ;
	}
}

void menuAudio( uint8_t event )
{
//	TITLE( "Display" ) ;
	lcd_putsAtt(0,0,"Audio Haptic",INVERS) ;
	
	static MState2 mstate2 ;
	uint32_t rows = g_eeGeneral.welcomeType == 2 ? 9 : 8 ;
	mstate2.check_columns(event, rows-1) ;

	uint16_t attr ;
	uint8_t subN = 0 ;
  int8_t sub = mstate2.m_posVert ;
	uint16_t y = FH ;
	uint8_t blink = InverseBlink ;
  uint8_t t_pgOfs ;

	t_pgOfs = evalOffset( sub ) ;

	if(t_pgOfs<=subN)
	{
		lcd_puts_Pleft( y, PSTR(STR_VOLUME));
		CurrentVolume = g_eeGeneral.volume ;
		lcd_outdezAtt( PARAM_OFS+2*FW, y, CurrentVolume, (sub==subN) ? blink : 0 ) ;
  	if(sub==subN)
		{
  	  CurrentVolume = checkIncDec16( CurrentVolume, 0, NUM_VOL_LEVELS-1, EE_GENERAL) ;
			if ( CurrentVolume != g_eeGeneral.volume )
			{
				setVolume( g_eeGeneral.volume = CurrentVolume ) ;
			}
		}
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;

	if(t_pgOfs<=subN)
	{
		uint8_t b ;
  	b = g_eeGeneral.beeperVal ;
  	lcd_puts_Pleft( y,PSTR(STR_BEEPER));
  	lcd_putsAttIdx(PARAM_OFS - FW - 4, y, PSTR(STR_BEEP_MODES),b,(sub==subN ? blink:0)) ;
		if(sub==subN)
		{
			CHECK_INCDEC_H_GENVAR_0( b, 6 ) ;
			g_eeGeneral.beeperVal = b ;
		}
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;
	
	if(t_pgOfs<=subN)
	{
  	lcd_puts_P( 0, y, PSTR(STR_SPEAKER_PITCH) ) ;
  	lcd_outdezAtt( PARAM_OFS+2*FW, y, g_eeGeneral.speakerPitch, (sub==subN ? blink : 0) ) ;
  	if(sub==subN)
		{
			g_eeGeneral.speakerPitch = checkIncDec16( g_eeGeneral.speakerPitch, 0, 20, EE_GENERAL ) ;
		}
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;

	if(t_pgOfs<=subN)
	{
  	lcd_puts_P( 0, y, XPSTR("Haptic Min Run")) ;
  	lcd_outdezAtt(PARAM_OFS+2*FW, y, g_eeGeneral.hapticMinRun+20, (sub==subN ? blink : 0) ) ;
  	if(sub==subN) CHECK_INCDEC_H_GENVAR_0( g_eeGeneral.hapticMinRun, 20 ) ;
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;
	 
	if(t_pgOfs<=subN)
	{
		lcd_putsAtt( 0, y, (char *)XPSTR(GvaString), sub==subN ? INVERS : 0 ) ;
  	if( sub==subN )
		{
			if ( checkForMenuEncoderBreak( Tevent ) )
			{
				pushMenu(menuGlobalVoiceAlarm) ;
			}
		}
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;

	if(t_pgOfs<=subN)
	{
		g_eeGeneral.preBeep = onoffMenuItem( g_eeGeneral.preBeep, y, PSTR(STR_BEEP_COUNTDOWN), sub == subN ) ;
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;

	if(t_pgOfs<=subN)
	{
		g_eeGeneral.minuteBeep = onoffMenuItem( g_eeGeneral.minuteBeep, y, PSTR(STR_MINUTE_BEEP), sub == subN ) ;
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;

	if(t_pgOfs<=subN)
	{
		lcd_puts_Pleft( y, XPSTR( "Welcome Type") ) ;
		g_eeGeneral.welcomeType = checkIndexed( y, XPSTR("\132\002""\006System  NoneCustom"), g_eeGeneral.welcomeType, sub==subN ) ;
		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;

	if(t_pgOfs<=subN)
	{
		if ( g_eeGeneral.welcomeType == 2 )
		{
			// FileName
	 		lcd_puts_Pleft( y, XPSTR( "FileName") ) ;
 			uint8_t attr = sub == subN ? InverseBlink : 0 ;
			alphaEditName( 10*FW-2, y, (uint8_t *)g_eeGeneral.welcomeFileName, sizeof(g_eeGeneral.welcomeFileName), (sub==subN) | ALPHA_NO_NAME, (uint8_t *)XPSTR( "FileName") ) ;
			validateName( g_eeGeneral.welcomeFileName, sizeof(g_eeGeneral.welcomeFileName) ) ;
			if( attr )
			{
				if ( event == EVT_KEY_LONG(KEY_MENU) )
				{
					VoiceFileType = VOICE_FILE_TYPE_USER ;
 				 	pushMenu( menuSelectVoiceFile ) ;
				}
				if ( event == EVT_ENTRY_UP )
				{
					if ( FileSelectResult == 1 )
					{
		 				copyFileName( (char *)g_eeGeneral.welcomeFileName, SelectedVoiceFileName, 8 ) ;
	   				eeDirty(EE_GENERAL) ;		// Save it
					}
				}
			} 
		}

		if((y+=FH)>(SCREEN_LINES-1)*FH) return ;
	}
	subN += 1 ;
		
}

void menuDiagAna( uint8_t event )
{
	uint32_t i ;
	TITLE(PSTR(STR_DiagAna));
	static MState2 mstate2;
	mstate2.check_columns(event, 2-1) ;

  int8_t sub = mstate2.m_posVert ;
	
  for( i = 0 ; i < 6 ; i += 1 )
	{
    uint16_t y=(i+1)*FH ;
		lcd_putsAttIdx( 2*FW, y, XPSTR("\003LH LV RV RH BATUSB"), i, 0 ) ;
		uint32_t index = i ;
		if ( i < 4 )
		{
 			index = stickScramble[g_eeGeneral.stickMode*4+i] ;
			lcd_outdezAtt( 17*FW, y, (int32_t)CalibratedStick[index]*1000/1024, PREC1);
  		lcd_outhex4( 7*FW, y, AnalogData[i] );
		}
		else if ( i == 4 )
		{
			uint8_t attr = PREC2 ;
			if ( sub == 1 )
			{
				attr |= InverseBlink ;
			}
			lcd_outdezAtt( 17*FW, y, g_vbat10mV, attr ) ;
    	lcd_outhex4( 7*FW, y, AnalogData[i] ) ;
			lcd_outhex2( 19*FW, y, g_eeGeneral.vBatCalib ) ;
		}
		else if ( i == 5 )
		{
			lcd_outdezAtt( 17*FW, y, UsbV, PREC2 ) ;
    	lcd_outhex4( 7*FW, y, AnalogData[i] ) ;
		}
	}
  if ( sub == 1 )
  {
		if ( s_editMode )
		{
			int8_t save ;
			save = g_eeGeneral.vBatCalib ;
			g_eeGeneral.vBatCalib = checkIncDec16( save, -127, 127, EE_GENERAL) ;
		}
  }
}

void lcd_img( uint8_t i_x, uint8_t i_y, uint8_t *imgdat, uint8_t idx, uint8_t mode )
{
  uint8_t *q = imgdat ;
  uint8_t w    = *q++ ;
  uint32_t hb   = (*q++ +7) / 8 ;
  uint8_t sze1 = *q++ ;

  q += idx * sze1 ;

	lcd_bitmap( i_x, i_y, q, w, hb, mode ) ;
}


#define RCON_OFF_0 	0

void menuControls( uint8_t event )
{
	TITLE( STR_Controls ) ;
	
	static MState2 mstate2;
	mstate2.check_columns(event, 6-1) ;

	uint8_t subN = 0 ;
  int8_t sub = mstate2.m_posVert ;
	uint8_t y = FH ;
	uint8_t blink = InverseBlink ;

//	if ( sub < 3 )
//	{
//		displayNext() ;
		uint8_t attr = sub==subN ? blink : 0 ;
		lcd_puts_Pleft( y, PSTR(STR_CHAN_ORDER) ) ;//   RAET->AETR
		uint8_t bch = bchout_ar[g_eeGeneral.templateSetup] ;
		for ( uint8_t i = 4 ; i > 0 ; i -= 1 )
		{
			uint8_t letter ;
			letter = *(PSTR(STR_SP_RETA) +(bch & 3) + 1 ) ;
		  lcd_putcAtt( 16*FW+RCON_OFF_0+i*FW, y, letter, attr ) ;
			bch >>= 2 ;
		}
		if(attr) CHECK_INCDEC_H_GENVAR_0( g_eeGeneral.templateSetup, 23 ) ;
 		y += FH ;
		subN += 1 ;

//		uint8_t ct = g_eeGeneral.crosstrim + ( g_eeGeneral.xcrosstrim << 1 ) ;
//  	lcd_puts_Pleft(y, PSTR(STR_CROSSTRIM));
//		ct = checkIndexed( y, XPSTR("\146\002\003OFFON Vtg"), ct, (sub==subN) ) ;
//		g_eeGeneral.crosstrim = ct ;
//		g_eeGeneral.xcrosstrim = ct >> 1 ;
// 		y += FH ;
//		subN += 1 ;

		lcd_puts_Pleft( y, PSTR(STR_MODE) );
		for ( uint32_t i = 0 ; i < 4 ; i += 1 )
		{
			lcd_img((6+4*i)*FW, y, (uint8_t *)sticks, i, 0 ) ;
		}
		y += FH ;
    
		attr = 0 ;
		uint8_t mode = g_eeGeneral.stickMode ;
 		if(sub==subN)
		{
			attr = INVERS ;
			if ( s_editMode )
			{
				attr = BLINK ;
					
				CHECK_INCDEC_H_GENVAR_0( mode,3);
				if ( mode != g_eeGeneral.stickMode )
				{
	//				g_eeGeneral.stickScroll = 0 ;
					g_eeGeneral.stickMode = mode ;							
				}
			}
		}
  	lcd_putcAtt( 3*FW+RCON_OFF_0, y, '1'+g_eeGeneral.stickMode,attr);
  	for(uint8_t i=0; i<4; i++) putsChnRaw( (6+4*i)*FW+RCON_OFF_0, y, modeFixValue( i ), 0 ) ;//sub==3?INVERS:0);
 		y += FH ;
		subN += 1 ;
//	}
//	else
//	{
//		subN = 3 ;
			
    for ( uint32_t i = 0 ; i < 4 ; i += 1 )
		{
      lcd_putsAttIdx( 5*FW, y, PSTR(STR_STICK_NAMES), i, 0 ) ;
//			if ( sub == subN )
//			{
//				SubMenuCall = 0x80 + i + 5 ;
//			}
			alphaEditName( 11*FW, y, &g_eeGeneral.customStickNames[i*4], 4, sub==subN, (uint8_t *)&PSTR(STR_STICK_NAMES)[i*5+1] ) ;
	 		y += FH ;
			subN += 1 ;
		}
//	}
}

void edit_stick_deadband(uint8_t x, uint8_t y, const char *s, uint8_t ch, uint8_t edit)
{
	uint8_t attr = 0 ;
	if(edit)
	{
    attr = InverseBlink ;
    CHECK_INCDEC_H_MODELVAR( g_eeGeneral.stickDeadband[ch], 0, 15 ) ;
  }
  lcd_puts_Pleft( y,PSTR(STR_STICK_DEADBAND));
	lcd_puts_P( x*FW, y, s);
	lcd_outdezAtt( 20*FW, y, g_eeGeneral.stickDeadband[ch], attr) ;
}

void menuHardware( uint8_t event )
{
	TITLE( STR_Hardware ) ;
	
	static MState2 mstate2;
	mstate2.check_columns(event, 4-1) ;

	uint8_t subN = 0 ;
  int8_t sub = mstate2.m_posVert ;
	uint16_t y = FH ;

  edit_stick_deadband(15, y, PSTR(STR_LV), 1, (sub==subN));
	y += FH ;
	subN++;
  edit_stick_deadband(15, y, PSTR(STR_LH), 0, (sub==subN));
  y += FH ;
  subN++;
  edit_stick_deadband(15, y, PSTR(STR_RV), 2, (sub==subN));
  y += FH ;
  subN++;
  edit_stick_deadband(15, y, PSTR(STR_RH), 3, (sub==subN));
  y += FH ;
  subN++;
	
//  lcd_puts_Pleft(y, "Rf Protocol");
//	g_eeGeneral.rfCountry = checkIndexed( y, XPSTR("\146\002\003FCCEU JAP"), g_eeGeneral.rfCountry, (sub==subN) ) ;


}

uint32_t trimPos( uint32_t trimNo )
{
	uint32_t x = 1 ;
  bool tm=keyState((EnumKeys)(TRM_BASE+2*trimNo));
  bool tp=keyState((EnumKeys)(TRM_BASE+2*trimNo+1));
	
	if ( tm )
	{
		x = 0 ;
	}
	if ( tp )
	{
		x = 2 ;
	}
	return x ;
}


void menuDiagKeys( uint8_t event )
{
	TITLE( PSTR(STR_DIAG) ) ;
	
	static MState2 mstate2;
	mstate2.check_columns(event, 2-1) ;

	uint16_t x ;
	x = 8*FW ;

	for( uint32_t i=0 ; i<4 ; i += 1 )
	{
    uint16_t y = i*FH ;
		{
			putHwSwitchName( x, y, i, 0 ) ;
			uint32_t pos = switchPosition( i ) ;
			if ( i < 10 )
			{
				lcd_putc(x+2*FW, y, PSTR(HW_SWITCHARROW_STR)[pos] ) ;
			}
			else
			{
				lcd_putc(x+2*FW, y, '0' +pos ) ;
			}
		}
	}

  lcd_puts_P( 15*FW, 2*FH, "Trims" ) ;

	x = trimPos( 0 ) ;
	lcd_putc(  17*FW, 6*FH, "\177|\176"[x] ) ;
	
	x = trimPos( 1 ) ;
	lcd_putc(  17*FW, 4*FH, PSTR(HW_SWITCHARROW_STR)[2-x] ) ;

//	x = trimPos( 2 ) ;
//	lcd_putc(  15*FW, 4*FH, PSTR(HW_SWITCHARROW_STR)[2-x] ) ;

//	x = trimPos( 3 ) ;
//	lcd_putc(  16*FW, 6*FH, "\177|\176"[x] ) ;

}

void dispMonth( uint8_t x, uint8_t y, uint32_t month, uint8_t attr)
{
	if ( month > 12 )
	{
		month = 0 ;		
	}
  lcd_putsAttIdx( x, y, PSTR(STR_MONTHS),month, attr ) ;
}

void disp_datetime( uint8_t y )
{
	lcd_puts_Pleft( y, XPSTR("  -   -      :  :"));
	lcd_outdezNAtt( 19*FW-2, y, Time.second, LEADING0, 2 ) ;
	lcd_outdezNAtt( 16*FW-1, y, Time.minute, LEADING0, 2 ) ;
	lcd_outdezNAtt( 13*FW,   y, Time.hour, LEADING0, 2 ) ;
	lcd_outdezNAtt( 2*FW,    y, Time.date, LEADING0, 2 ) ;
	lcd_outdezNAtt( 10*FW,   y, Time.year, LEADING0, 4 ) ;
	dispMonth( 3*FW, y, Time.month, 0 ) ;
}

#define DATE_COUNT_ITEMS	7

t_time EntryTime ;

void rtc_settime( t_time *t ) ;

void menuDateTime(uint8_t event)
{
	TITLE(PSTR(STR_DATE_TIME));
	static MState2 mstate2;
	
	mstate2.check_columns(event, DATE_COUNT_ITEMS-1) ;

  switch(event)
  {
    case EVT_ENTRY :
			{	
				t_time *p = &EntryTime ;

				if ( Time.date == 0 )
				{
					Time.date = 1 ;
				}
				if ( Time.month == 0 )
				{
					Time.month = 1 ;
				}
				p->second = Time.second ;
				p->minute = Time.minute ;
				p->hour   = Time.hour ;
				p->date   = Time.date ;
				p->month  = Time.month ;
				p->year   = Time.year ;
				if ( p->year < 2000 )
				{
					p->year = 2000 + ( p->year % 100 ) ;
					
				}
			}
    break ;
		
    case EVT_KEY_LONG(KEY_MENU) :
			rtc_settime( &EntryTime ) ;
      killEvents(event);
			s_editMode = 0 ;
    break ;
	}		 

	disp_datetime( 1*FH ) ;

  int8_t  sub    = mstate2.m_posVert;

	for (uint8_t subN=1; subN<8; subN++)
	{
	  uint16_t attr = ((sub==subN) ? InverseBlink : 0) | LEADING0 ;
		switch ( subN )
		{
			case 1 :
			  lcd_puts_Pleft( 2*FH, PSTR(STR_SEC) );
				lcd_outdezNAtt( 8*FW, 2*FH, EntryTime.second, attr, 2 ) ;
			  if(sub==subN)  EntryTime.second = checkIncDec16( EntryTime.second, 0, 59, 0 ) ;
			break ;
			case 2 :
			  lcd_puts_Pleft( 3*FH, PSTR(STR_MIN_SET) );
				lcd_outdezNAtt( 8*FW, 3*FH, EntryTime.minute, attr, 2 ) ;
			  if(sub==subN)  EntryTime.minute = checkIncDec16( EntryTime.minute, 0, 59, 0 ) ;
			break ;
			case 3 :
			  lcd_puts_Pleft( 4*FH, PSTR(STR_HOUR_MENU_LONG) );
				lcd_outdezNAtt( 8*FW, 4*FH, EntryTime.hour, attr, 2 ) ;
			  if(sub==subN)  EntryTime.hour = checkIncDec16( EntryTime.hour, 0, 23, 0 ) ;
			break ;
			case 4 :
			  lcd_puts_Pleft( 5*FH, PSTR(STR_DATE) );
				lcd_outdezNAtt( 8*FW, 5*FH, EntryTime.date, attr, 2 ) ;
			  if(sub==subN)  EntryTime.date = checkIncDec16( EntryTime.date, 1, 31, 0 ) ;
			break ;
			case 5 :
			  lcd_puts_Pleft( 6*FH, PSTR(STR_MONTH) );
				dispMonth( 6*FW+3, 6*FH, EntryTime.month, attr ) ;
			  if(sub==subN)  EntryTime.month = checkIncDec16( EntryTime.month, 1, 12, 0 ) ;
			break ;
			case 6 :
				lcd_puts_Pleft( 7*FH, PSTR(STR_YEAR) );
				lcd_outdezNAtt( 10*FW-2, 7*FH, EntryTime.year, attr, 4 ) ;
			  if(sub==subN)  EntryTime.year = checkIncDec16( EntryTime.year, 0, 2999, 0 ) ;
			break ;
//			case 7 :
//				lcd_puts_Pleft( 8*FH, "SET" );
//				if(sub==subN)
//				{
//					lcd_char_inverse( 0, 8*FH, 3*FW, 0, FH ) ;
//				}

//			  if(sub==subN)
//				{
//					if ( event == EVT_KEY_BREAK(BTN_RE) )
//					{
//						rtc_settime( &EntryTime ) ;
//      			killEvents(event);
//						s_editMode = 0 ;
//					}
//				}
//			break ;
		}
//		  lcd_puts_P( 14*FW, 7*FH, XPSTR("RTC V") );
	}
}



enum GENERAL_INDEX
{
	M_INDEX,
	M_DISPLAY,
	M_AUDIO,
	M_ALARMS,
	M_GENERAL,
	M_CONTROLS,
	M_HARDWARE,
	M_CALIB,
//#ifdef BLUETOOTH
	M_BLUETOOTH,
//#endif
//	M_TRAINER,
	M_VERSION,
//#ifndef PCBXLITE
// #ifndef PCBT12
//  #ifndef PCBX9LITE
//   #ifndef PCBX12D
//    #ifndef PCBX10
//     #ifndef PCBLEM1
//	M_MODULE,
//     #endif // PCBLEM1
//    #endif // X10
//   #endif // X12
//  #endif // X9Lite
// #endif
//#endif
//	M_EEPROM,
	M_DATE,
	M_DIAGKEYS,
	M_DIAGANA,
	M_COUNT
} ;


void menuRadioIndex(uint8_t event)
{
	static MState2 mstate;
	EditType = EE_GENERAL ;

	event = indexProcess( event, &mstate, M_COUNT-1-7 ) ;

	event = mstate.check_columns( event, IlinesCount-1 ) ;
	
	switch ( SubmenuIndex )
	{
		case M_VERSION :
    	pushMenu(menuVersion) ;
		break ;
		case M_DATE :
    	pushMenu(menuDateTime) ;
		break ;
		case M_AUDIO :
    	pushMenu(menuAudio) ;
		break ;
		case M_CALIB :
    	pushMenu(menuCalib) ;
		break ;
		case M_DISPLAY :
    	pushMenu(menuDisplay) ;
		break ;
		case M_ALARMS :
    	pushMenu(menuAlarms) ;
		break ;
		case M_GENERAL :
    	pushMenu(menuGeneral) ;
		break ;
		case M_CONTROLS :
    	pushMenu(menuControls) ;
		break ;
		case M_HARDWARE :
    	pushMenu(menuHardware) ;
		break ;
		case M_DIAGKEYS :
    	pushMenu(menuDiagKeys) ;
		break ;
		case M_DIAGANA :
    	pushMenu(menuDiagAna) ;
		break ;
		case M_BLUETOOTH :
    	pushMenu(menuBluetooth) ;
		break ;
	}
	
	uint32_t sub = mstate.m_posVert ;

	switch ( SubmenuIndex )
	{
		case 0 :
			lcd_putsAtt(0,0,"Radio Setup",INVERS) ;
			IlinesCount = M_COUNT-1 ;
			sub += 1 ;

static const char *in_Strings[] =
{
	STR_Display,
	STR_AudioHaptic,
	STR_Alarms,
	STR_General,
	STR_Controls,
	STR_Hardware,
	STR_Calibration,
//	#ifdef BLUETOOTH
	STR_Bluetooth,
//	#endif
//	STR_Trainer,
	STR_Version,
//	STR_Eeprom,
	STR_DateTime,
	STR_DiagSwtch,
	STR_DiagAna
} ;

			displayIndex( in_Strings, M_COUNT-1-7, 7, sub ) ;
		break ;
	}
}


