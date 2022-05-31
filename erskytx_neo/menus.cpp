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

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>

#include "erskyTx.h"
#include "lcd.h"
#include "logicIo.h"
#include "myeeprom.h"
#include "menus.h"
#include "file.h"
#include "en.h"
#include "telemetry.h"
#include "audio.h"

#include "lfs.h"
#include "freertos/semphr.h"

extern lfs_t Lfs ;
////lfs_file_t LfsFile ;

#define BASIC		1
#include "basic.h"

#define RSSI_POWER_OFF	1
#define RSSI_STAY_ON		0

struct t_text	TextControl ;
extern struct t_alpha Alpha ;
extern struct fileControl FileControl ;
void setupFileNames( char *dir, struct fileControl *fc, char *ext ) ;
uint32_t fileList(uint8_t event, struct fileControl *fc ) ;
extern struct t_filelist FileList ;
extern uint8_t	CurrentPhase ;

void menuAlpha(uint8_t event) ;


#define STR_MAIN_POPUP			"Model Select\0Model Setup\0Last Menu\0Radio Setup\0Statistics\0Notes\0Zero Alt.\0Zero A1 Offs\0Zero A2 Offs\0Reset GPS\0Help\0Main Display\0Run Script\0Reset Telemetry\0Reset Timer1\0Reset Timer2"

#define KEY_UP		TRM_LV_UP
#define KEY_DOWN	TRM_LV_DWN
#define KEY_LEFT	TRM_LH_DWN
#define KEY_RIGHT	TRM_LH_UP

#define MENU(title, tab, menu, lines_count, ...) \
TITLE(title) ; \
static MState2 mstate2 ; \
static const uint8_t mstate_tab[] = __VA_ARGS__ ; \
event = mstate2.check(event,menu,tab,DIM(tab),mstate_tab,DIM(mstate_tab)-1,lines_count-1)

extern uint16_t AnalogValues[] ;

uint8_t Columns ;
uint8_t s_editMode ;
uint8_t s_editing ;
uint8_t g_posHorz ;
uint8_t EditType ;
uint8_t s_pgOfs ;
uint8_t InverseBlink ;
uint8_t EditColumns ;
uint8_t IlinesCount ;
uint8_t SubmenuIndex ;
uint8_t SubMenuFromIndex ;

struct t_popupData PopupData ;

struct t_timer s_timer[2] ;

// For model select
uint8_t DupIfNonzero = 0 ;
int8_t DupSub ;

extern uint8_t VoiceCheckFlag100mS ;

void perOut(int16_t *chanOut, uint8_t att ) ;

void doMainScreenGraphics() ;
void menuCalib(uint8_t event) ;
void menuRadioIndex(uint8_t event) ;
void menuModelIndex(uint8_t event) ;
void menuModelSelect(uint8_t event) ;
void processSwitches( void ) ;

extern uint16_t WatchdogTimeout ;

void menuBattery(uint8_t event) ;
void menuStatistic(uint8_t event) ;
void menuStatistic2(uint8_t event) ;
void menuDebug(uint8_t event) ;
void menuBoot(uint8_t event) ;

enum EnumTabStat
{
	e_battery,
	e_stat1,
	e_stat2,
	e_debug,
  e_Boot
} ;

MenuFuncP menuTabStat[] =
{
	menuBattery,
	menuStatistic,
	menuStatistic2,
	menuDebug,
	menuBoot,
}	;

uint16_t TelSent ;
uint8_t MainDisplayIndex ;

const uint8_t HeadingImg[] =
{
	62, 16, 0,
	0xFF,0xFF,0x63,0x63,0x63,0x63,0x63,0x63,0x63,0x00,0x00,0xF8,0xF8,0x30,0x18,0x18,
	0x00,0x70,0xF8,0xD8,0xD8,0x98,0xB8,0x30,0x00,0x00,0xFF,0xFF,0xC0,0xE0,0xF0,0x18,
	0x08,0x00,0x18,0x78,0xE0,0x80,0x00,0x80,0xE0,0x78,0x18,0x03,0x03,0x03,0x03,0xFF,
	0xFF,0x03,0x03,0x03,0x03,0x00,0x18,0x38,0xF0,0xC0,0xF0,0x38,0x18,0x00,
	
	0x0F,0x0F,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x00,0x00,0x0F,0x0F,0x00,0x00,0x00,
	0x00,0x06,0x0E,0x0C,0x0D,0x0D,0x0F,0x07,0x00,0x00,0x0F,0x0F,0x01,0x00,0x03,0x0F,
	0x0C,0x00,0x00,0x60,0x63,0x7F,0x3E,0x0F,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,
	0x0F,0x00,0x00,0x00,0x00,0x00,0x0C,0x0E,0x07,0x01,0x07,0x0E,0x0C,0x00
} ;


extern uint16_t Battery ;
extern uint16_t UsbV ;
extern uint16_t Rtime ;
extern uint8_t JustLoadedModel ;

extern SemaphoreHandle_t SpiMutex ;

void menuBattery(uint8_t event)
{
	TITLE(PSTR(STR_BATTERY)) ;
	static MState2 mstate2 ;
	event = mstate2.check( event, e_battery, menuTabStat, DIM(menuTabStat), 0, 0, 0 ) ;

  switch(event)
  {
  	case EVT_KEY_BREAK(KEY_MENU):
//      g_timeMain = 0;
    break;
//    case EVT_KEY_LONG(KEY_MENU):
//      audioDefevent(AU_MENUS) ;
//    	killEvents(event) ;
//    break;
  	
//		case EVT_ENTRY :
//extern void convertFont() ;
//		 convertFont() ;
//    break;

  }

	lcd_puts_Pleft( 2*FH, PSTR(STR_Battery));
//		putsVolts( 13*FW, 2*FH, g_vbat100mV, 0 ) ;
	lcd_outdezAtt( 127, 2*FH, Battery, PREC2 ) ;
	lcd_puts_Pleft( 3*FH, "USB V" ) ;
	lcd_outdezAtt( 127, 3*FH, UsbV, PREC2 ) ;

//	disp_datetime( 5*FH ) ;

//extern uint32_t Master_frequency ;
// 	lcd_outdezAtt( 5*FW, 7*FH, Master_frequency/1000000, 0 ) ;
}

void setTextBuffer( uint8_t *source, uint32_t number )
{
	uint32_t i ;
	uint32_t j = 0 ;
	uint32_t k = 0 ;
	uint8_t *p = source ;
 	memset( TextControl.TextMenuBuffer,' ', 16*21 ) ;
	for ( i= 0 ; i < number ; i += 1 )
	{
		uint8_t c = *p++ ;
		if ( c == 10 )
		{
			j = 0 ;
			k += 21 ;
			TextControl.TextLines += 1 ;
			if ( k >= 16*21 )
			{
				break ;
			}
		}
		if ( c < ' ' )
		{
			continue ;
		}
		if ( j >= 21 )
		{
			j = 0 ;
			k += 21 ;
			if ( k >= 16*21 )
			{
				break ;
			}
		}
		TextControl.TextMenuBuffer[k+j] = c ;
		j += 1 ;
	}
	TextControl.TextLines = k / 21 ;
	if ( j )
	{
		TextControl.TextLines += 1 ;
	}
}

void writeModelNotes()
{
	uint8_t filename[50] ;
	int32_t result ;
  int32_t written ;
	uint8_t *p = TextControl.TextMenuStore ;
	uint8_t *q ;
	uint32_t i ;
	uint32_t j ;
	setModelFilename( filename, g_eeGeneral.currModel+1, FILE_TYPE_TEXT ) ;

	for ( i = 0 ; i < 16 ; i += 1 )
	{
		q = &TextControl.TextMenuBuffer[i*21+20] ;
		j = 21 ;
		do
		{
			if ( *q != ' ' )
			{
				break ;
			}
			q -= 1 ;
			j -= 1 ;
		} while ( j ) ;
		q = &TextControl.TextMenuBuffer[i*21] ;
		while ( j )
		{
			*p++ = *q++ ;
			j -= 1 ;
		}
		*p++ = '\n' ;
	}
	j = p - TextControl.TextMenuStore ;	// # bytes to write
  result = lfs_file_open( &Lfs, &TextControl.TextFile, (char *)filename, LFS_O_RDWR | LFS_O_CREAT) ;
	if ( result == 0 )
	{
    written = lfs_file_write( &Lfs, &TextControl.TextFile, TextControl.TextMenuStore, j ) ;
		lfs_file_close(&Lfs, &TextControl.TextFile ) ;
	}
}

uint32_t readModelNotes()
{
	uint8_t filename[50] ;
	int32_t result = 0 ;
	int32_t nread ;

	setModelFilename( filename, g_eeGeneral.currModel+1, FILE_TYPE_TEXT ) ;
	result = lfs_file_open( &Lfs, &TextControl.TextFile, (char *)filename, LFS_O_RDWR ) ;

	TextControl.TextLines = 0 ;
	if ( result == 0 )
	{
		
		nread = lfs_file_read( &Lfs, &TextControl.TextFile, TextControl.TextMenuStore, 16*21+32 ) ;

		if ( nread >= 0 )
		{
			setTextBuffer( TextControl.TextMenuStore, nread ) ;
		}
		else
		{
			result = -1 ;
		}
		lfs_file_close(&Lfs, &TextControl.TextFile ) ;
	}
	return result ;
}

void displayNotes( uint8_t y, uint8_t lines )
{
	uint32_t i ;
	uint32_t j ;
	for ( i = 0 ; i < lines*21 ; y += FH, i += 21 )
	{
		j = TextControl.TextOffset * 21 + i ;
		lcd_putsnAtt( 0, y, (const char *)&TextControl.TextMenuBuffer[j], 21, 0 ) ;
	}
}


void menuEditNotes(uint8_t event)
{
	int32_t result ;
	TITLE( XPSTR("Edit Notes") ) ;
	static MState2 mstate2 ;
	event = mstate2.check_columns( event, 15 ) ;
  uint8_t t_pgOfs ;
  uint8_t y ;
//  uint8_t k = 0;
  int8_t  sub    = mstate2.m_posVert ;

  t_pgOfs = evalOffset(sub);

	if ( event == EVT_ENTRY )
	{
		// read the notes here
		result = readModelNotes() ;
		if ( result != 0 )
		{
			setTextBuffer( (uint8_t *)"", 0 ) ;
		}
		Alpha.AlphaChanged = 0 ;
	}
  TextControl.TextOffset = t_pgOfs ;
	y = sub - t_pgOfs + 1 ;
	displayNotes( FH, (SCREEN_LINES - 1) ) ;
	lcd_char_inverse( 0, y*FH, 126, 0 ) ;
  lcd_puts_Pleft( 0, XPSTR("\014Line")) ;
  lcd_outdez( 18*FW, 0, sub+1 ) ;
	if ( checkForMenuEncoderBreak( event ) )
	{
		Alpha.AlphaLength = 21 ;
		Alpha.PalphaText = &TextControl.TextMenuBuffer[sub*21] ;
		Alpha.PalphaHeading = (uint8_t *)"Note text" ;
		s_editMode = 0 ;
    killEvents(event) ;
		pushMenu(menuAlpha) ;
		Alpha.AlphaChanged = 1 ;
	}
	if ( (event == EVT_KEY_BREAK(KEY_EXIT) ) || (event == EVT_KEY_LONG(KEY_EXIT) ) )
	{
		if ( Alpha.AlphaChanged )
		{
			writeModelNotes() ;		
		}
	}
}

void menuText(uint8_t event)
{
//	uint8_t filename[50] ;
	int32_t result ;
//	UINT nread ;

	if ( event == EVT_ENTRY )
	{
//	 if ( SharedMemory.TextControl.TextHelp )
//	 {
//	 		SharedMemory.TextControl.HelpTextPage = 0 ;
//			uint8_t *p = (uint8_t *) HelpText0 ;
//			setTextBuffer( p, strlen( ( char *)p ) ) ;
//	 }
//	 else
	 {
 		result = readModelNotes() ;
//		setModelFilename( filename, g_eeGeneral.currModel+1, FILE_TYPE_TEXT ) ;
//  	result = f_open( &SharedMemory.TextControl.TextFile, (TCHAR *)filename, FA_READ ) ;
//		SharedMemory.TextControl.TextLines = 0 ;
//		if ( result == FR_OK )
//		{
//			result = f_read( &SharedMemory.TextControl.TextFile, SharedMemory.TextControl.TextMenuStore, 16*21+32, &nread ) ;
//			if ( result == FR_OK )
//			{
//				setTextBuffer( SharedMemory.TextControl.TextMenuStore, nread ) ;
//			}
//		}
//		f_close( &SharedMemory.TextControl.TextFile ) ;
		if ( result != 0 )
		{
		 	memset( TextControl.TextMenuBuffer,' ', 16*21 ) ;
  		strncpy( (char *)&TextControl.TextMenuBuffer[63+3], XPSTR("No Notes Found"), 14 );
			TextControl.TextLines = 8 ;
		}
	 }
		TextControl.TextOffset = 0 ;
	}
  if ( event == EVT_KEY_BREAK(KEY_EXIT) )
	{
		killEvents(event) ;
    popMenu(false) ;
	}
	if ( ( event == EVT_KEY_FIRST(KEY_UP) ) || ( event== EVT_KEY_REPT(KEY_UP) ) )
	{
		if ( TextControl.TextOffset )
		{
			TextControl.TextOffset -= 1 ;
		}
	}
	if ( ( event == EVT_KEY_FIRST(KEY_DOWN) ) || ( event== EVT_KEY_REPT(KEY_DOWN) ) )
	{
		if ( TextControl.TextLines > 8 )
		{
			if ( TextControl.TextOffset < TextControl.TextLines - 8 )
			{
				TextControl.TextOffset += 1 ;
			}
		}
	}
//	if ( TextControl.TextHelp )
//	{
//		if ( ( event == EVT_KEY_FIRST(KEY_RIGHT) ) || ( event== EVT_KEY_REPT(KEY_RIGHT) ) )
//		{
//			TextControl.HelpTextPage += 1 ;
//			if ( TextControl.HelpTextPage >= NUM_HELP_PAGES )
//			{
//				TextControl.HelpTextPage = 0 ;
//			}
////	 		memset( SharedMemory.TextControl.TextMenuBuffer,' ', 16*21 ) ;
//			uint8_t *p = (uint8_t *) ( TextControl.HelpTextPage ? HelpText1 : HelpText0 ) ;
//			setTextBuffer( p, strlen( ( char *)p ) ) ;
//		}
//		if ( ( event == EVT_KEY_FIRST(KEY_LEFT) ) || ( event== EVT_KEY_REPT(KEY_LEFT) ) )
//		{
//			if ( TextControl.HelpTextPage == 0 )
//			{
//				TextControl.HelpTextPage = NUM_HELP_PAGES - 1 ;
//			}
//			else
//			{
//				TextControl.HelpTextPage -= 1 ;
//			}
////	 		memset( TextControl.TextMenuBuffer,' ', 16*21 ) ;
//			uint8_t *p = (uint8_t *) ( TextControl.HelpTextPage ? HelpText1 : HelpText0 ) ;
//			setTextBuffer( p, strlen( ( char *)p ) ) ;
//		}
//	}
	// display buffer
	displayNotes( 0, 8 ) ;

//	uint32_t i ;
//	uint32_t j ;
//	uint8_t y = 0 ;
//	for ( i = 0 ; i < 8*21 ; y += FH, i += 21 )
//	{
//		j = TextControl.TextOffset * 21 + i ;
//		lcd_putsnAtt( 0, y, (const char *)&SharedMemory.TextControl.TextMenuBuffer[j], 21, 0 ) ;
//	}
}

extern int16_t RawSticks[] ;

void setStickCenter(uint32_t toSubTrims ) // copy state of 3 primary to subtrim
{
	uint8_t thisPhase ;
  int16_t zero_chans512_before[NUM_SKYCHNOUT];
  int16_t zero_chans512_after[NUM_SKYCHNOUT];

	thisPhase = getFlightPhase() ;
	CurrentPhase = thisPhase ;

	if ( ( !g_model.instaTrimToTrims) || toSubTrims )
	{
  	perOut(zero_chans512_before,NO_TRAINER | NO_INPUT | FADE_FIRST | FADE_LAST); // do output loop - zero input channels
  	perOut(zero_chans512_after,NO_TRAINER | FADE_FIRST | FADE_LAST); // do output loop - zero input channels

  	for(uint8_t i=0; i<NUM_SKYCHNOUT ; i++)
  	{
			LimitData *pld = &g_model.limitData[i] ;
  	  int16_t v = pld->offset;
  	  v += pld->revert ?
  	                (zero_chans512_before[i] - zero_chans512_after[i]) :
  	                (zero_chans512_after[i] - zero_chans512_before[i]);
  	  pld->offset = max(min(v,(int16_t)1000),(int16_t)-1000); // make sure the offset doesn't go haywire
  	}
	}

  for(uint8_t i=0; i<4; i++)
	{
		if(!IS_THROTTLE(i))
		{
			
			if ( ( g_model.instaTrimToTrims) && !toSubTrims )
			{
				int16_t v = RawSticks[i] / 2 ;
				int16_t original_trim = getTrimValue(thisPhase, i);
				if ( ( v > 10 ) || ( v < -10 ) )
				{
					v = max(min( original_trim + v,125),-125) ;
					setTrimValue(thisPhase, i, v ) ;
				}
			}
			else
			{
				int16_t original_trim = getTrimValue(thisPhase, i);
      	for (uint8_t phase=0; phase<MAX_MODES; phase +=  1)
				{
      	  int16_t trim = getRawTrimValue(phase, i);
      	  if (trim <= TRIM_EXTENDED_MAX)
					{
      	    setTrimValue(phase, i, trim - original_trim);
					}
				}
			}
		}
	}
//  STORE_MODELVARS_TRIM;
  audioDefevent(AU_WARNING2);
}



void menuStatistic(uint8_t event)
{
	TITLE(PSTR(STR_STAT)) ;
	static MState2 mstate2 ;
	event = mstate2.check( event, e_stat1, menuTabStat, DIM(menuTabStat), 0, 0, 0 ) ;

  lcd_puts_Pleft( FH*0, XPSTR("\016TOT\037\001TME\016TSW\037\001STK\016ST%"));

  putsTime(    6*FW, FH*1, s_timer[0].s_timeCumAbs, 0, 0);
  putsTime(   11*FW, FH*1, s_timer[0].s_timeCumSw,      0, 0);

  putsTime(    6*FW, FH*2, s_timer[0].s_timeCumThr, 0, 0);
  putsTime(   11*FW, FH*2, s_timer[0].s_timeCum16ThrP/16, 0, 0);

extern uint16_t s_timeCumTot ;		// Total tx on time (secs)
  putsTime(   11*FW, FH*0, s_timeCumTot, 0, 0) ;


//extern uint16_t HeartbeatCount ;
//extern uint16_t HeartbeatTime ;

//	lcd_outhex4( 0, 7*FH, HeartbeatCount ) ;
//	lcd_outhex4( 30, 7*FH, HeartbeatTime ) ;

//  uint16_t traceRd = s_traceCnt>MAXTRACE ? s_traceWr : 0;
//  uint8_t x=5;
//  uint8_t y=60;
  
//	pushPlotType( PLOT_BLACK ) ;
//	lcd_hline(x-3,y,120+3+3);
//  lcd_vline(x,y-32,32+3);

//  for(uint8_t i=0; i<120; i+=6)
//  {
//    lcd_vline(x+i+6,y-1,3);
//  }
//  for(uint8_t i=1; i<=120; i++)
//  {
//    lcd_vline(x+i,y-s_traceBuf[traceRd],s_traceBuf[traceRd]);
//    traceRd++;
//    if(traceRd>=MAXTRACE) traceRd=0;
//    if(traceRd==s_traceWr) break;
//  }
//	popPlotType() ;
}


#define STAT2_OFF_0		0


void menuStatistic2(uint8_t event)
{
	TITLE(PSTR(STR_STAT2)) ;
	static MState2 mstate2 ;
	event = mstate2.check( event, e_stat2, menuTabStat, DIM(menuTabStat), 0, 0, 0 ) ;

  switch(event)
  {
    case EVT_KEY_FIRST(KEY_MENU):
//      g_timeMain = 0;
//      audioDefevent(AU_MENUS) ;
    break;
    case EVT_KEY_LONG(KEY_MENU):
//			g_eeGeneral.totalElapsedTime = 0 ;
    break;
  }

	lcd_puts_Pleft( FH, "Refresh Time" ) ;
	lcd_outdez( 127, FH, Rtime ) ;

	lcd_puts_Pleft( 2*FH, "EEPROM Id" ) ;
	lcd_outhex2( 11*FW, 2*FH, GdId[0] ) ;
	lcd_outhex2( 14*FW, 2*FH, GdId[1] ) ;
	lcd_outhex2( 17*FW, 2*FH, GdId[2] ) ;

extern uint32_t micros() ;
	lcd_putsAtt( 0, 3*FH, "Adc read time", 0 ) ;
extern uint16_t AdcReadTime ;
	lcd_outdez( 15*FW, 3*FH, AdcReadTime ) ;

extern uint16_t TelSent ;
	lcd_puts_Pleft( 6*FH, XPSTR("Access Sent") ) ;
  lcd_outhex4( 13*FW, 6*FH, TelSent ) ;

extern uint16_t TelRxCount ;
	lcd_puts_Pleft( 7*FH, XPSTR("TelRxCount") ) ;
  lcd_outhex4( 13*FW, 7*FH, TelRxCount ) ;



//  lcd_puts_Pleft( 1*FH, XPSTR("On Time")) ;
//  lcd_putcAtt( 11*FW+3+STAT2_OFF_0, 1*FH, ':', 0 ) ;
//	div_t qr ;
//	qr = div( g_eeGeneral.totalElapsedTime, 60 ) ;
//  putsTime( 9*FW+STAT2_OFF_0, FH*1, qr.quot, 0, 0 ) ;
//  lcd_outdezNAtt( 14*FW+STAT2_OFF_0, 1*FH, qr.rem, LEADING0, 2 ) ;

//  lcd_puts_Pleft( 2*FH, XPSTR("tmain          ms"));
//  lcd_outdezAtt(14*FW+STAT2_OFF_0 , 2*FH, (g_timeMain)/20 ,PREC2);

//#if defined(PCBLEM1)
//	lcd_puts_Pleft( 3*FH, XPSTR("Refresh time(uS)"));
//extern uint16_t RefreshTime ;
//  lcd_outdezAtt(20*FW+STAT2_OFF_0 , 3*FH, RefreshTime, 0 ) ;
//#endif

extern uint16_t PulsesRate ;
extern uint32_t MixerRate ;
//#if defined(PCBSKY) || defined(PCB9XT) || defined(PCBX12D) || defined(PCBX10) || defined(PCBLEM1)
	lcd_puts_Pleft( 4*FH, XPSTR("Mixer Rate"));
//#endif
  lcd_outdezAtt(17*FW, 4*FH, MixerRate, 0 ) ;
	lcd_puts_Pleft( 5*FH, XPSTR("Pulses Rate"));
  lcd_outdezAtt(17*FW, 5*FH, PulsesRate, 0 ) ;

//#if defined(PCBX9D) || defined(PCBX12D) || defined(PCBX10)
//  lcd_puts_Pleft( 3*FH, XPSTR("trefresh       ms"));
//  lcd_outdezAtt(14*FW+STAT2_OFF_0 , 3*FH, (g_timeRfsh)/20 ,PREC2);
//  lcd_puts_Pleft( 4*FH, XPSTR("tmixer         ms"));
//  lcd_outdezAtt(14*FW+STAT2_OFF_0 , 4*FH, (g_timeMixer)/20 ,PREC2);

////	lcd_puts_Pleft( 1*FH, XPSTR("ttimer1        us"));
////  lcd_outdezAtt(14*FW , 1*FH, (g_timePXX)/2 ,0);
//#endif

////#ifdef PCBX12D
////static uint16_t tt ;
////static uint16_t ct ;
////static uint16_t mt ;
////static uint16_t count ;
////extern uint16_t TestTime ;
////extern uint16_t ClearTime ;
////extern uint16_t MenuTime ;
////if ( ++count > 25 )
////{
////	tt = TestTime ;
////	ct = ClearTime ;
////	mt = MenuTime ;
////}
////	lcd_outhex4( 134, 2*FH, TestTime ) ; lcd_outdezAtt( 190, 2*FH, tt/2, 0 ) ;
////	lcd_outhex4( 134, 3*FH, ClearTime ) ; lcd_outdezAtt( 190, 3*FH, ct/2, 0 ) ;
////	lcd_outhex4( 134, 4*FH, MenuTime ) ; lcd_outdezAtt( 190, 4*FH, mt/2, 0 ) ;
////#endif

//#if defined(PCBSKY) || defined(PCB9XT) || defined(PCBX12D) || defined(PCBX10)
//  lcd_puts_Pleft( 3*FH, XPSTR("tBgRead        ms"));
//  lcd_outdezAtt(14*FW+STAT2_OFF_0 , 3*FH, (g_timeBgRead)/20 ,PREC2);
//#endif
  
//extern uint8_t AudioVoiceCountUnderruns ;
//	lcd_puts_Pleft( 5*FH, XPSTR("Voice underruns"));
//  lcd_outdezAtt( 20*FW+STAT2_OFF_0, 5*FH, AudioVoiceCountUnderruns, 0 ) ;

//  lcd_puts_P( 3*FW+STAT2_OFF_0,  6*FH, PSTR(STR_MENU_REFRESH));

////extern uint32_t IdleCount ;
  
////	lcd_outhex4( 0,  7*FH, IdleCount >> 16 ) ;
////  lcd_outhex4( 30, 7*FH, IdleCount ) ;

//extern uint32_t IdlePercent ;
//  lcd_puts_Pleft( 7*FH, XPSTR("Idle time\016%"));
//  lcd_outdezAtt( 14*FW-1+STAT2_OFF_0, 7*FH, IdlePercent ,PREC2);
////  lcd_outhex4( 75, 7*FH, IdlePercent ) ;
}

void menuBoot(uint8_t event)
{
	TITLE(PSTR(STR_BOOT_REASON)) ;
	static MState2 mstate2 ;
	event = mstate2.check( event, e_Boot, menuTabStat, DIM(menuTabStat), 0, 0, 0 ) ;

extern uint32_t ResetReason ;
	lcd_outhex8( 0, 6*FH, ResetReason ) ;
}


uint8_t ScriptDirNeeded ;

void menuScript(uint8_t event)
{
	static MState2 mstate2 ;
	struct fileControl *fc = &FileControl ;
	uint32_t i ;
#ifndef BASIC
	uint32_t j ;
#endif
	 
	uint8_t saveEvent = event ;

  TITLE( "SCRIPT" ) ;

	event = mstate2.check_columns( event, 0 ) ;
	event = saveEvent ;
#ifndef BASIC
	i = availableMemory() ;
	j = i / 10000 ;
	i %= 10000 ;
	lcd_outdezAtt( 15*FW, 0, i, LEADING0 ) ;
	lcd_outdezAtt( 15*FW-FWNUM*4, 0, j, 0 ) ;
#endif

	if ( ( event == EVT_ENTRY ) || ScriptDirNeeded )
	{
		ScriptDirNeeded = 0 ;
		WatchdogTimeout = 300 ;		// 3 seconds
#ifdef LUA
		setupFileNames( (char *)"/SCRIPTS", fc, g_model.basic_lua ? (char *)"LUA" : (char *)"BAS" ) ;
#else
		setupFileNames( (char *)"/SCRIPTS", fc, (char *)"BAS" ) ;
#endif
	}
	
//	if ( ( event == EVT_KEY_BREAK(BTN_RE) ) || (event == EVT_KEY_BREAK(KEY_MENU)) )
//	{
//		if ( savedRotaryState == ROTARY_MENU_LR )
//		{
//			event = 0 ;
//		}
//	}
	
//	switch(event)
//	{
//    case EVT_ENTRY:
//			WatchdogTimeout = 200 ;		// 2 seconds
//#ifdef LUA
//			setupFileNames( (TCHAR *)"/SCRIPTS", fc, (char *)"LUA" ) ;
//#else
//			setupFileNames( (TCHAR *)"/SCRIPTS", fc, (char *)"BAS" ) ;
//#endif
//    break ;
////		case EVT_KEY_LONG(KEY_EXIT):
////	    killEvents(event) ;
////			popMenu(false) ;
////    break ;
////#if defined(PCBX7) || defined(REV9E)
////		case EVT_KEY_BREAK(BTN_RE):
////			event = 0 ;
////    break ;
////#endif
//		case EVT_KEY_BREAK(BTN_RE) :
//		case EVT_KEY_BREAK(KEY_MENU) :
//			if ( savedRotaryState == ROTARY_MENU_LR )
//			{
//				event = 0 ;
//			}
			
//    break ;
			
//	}

	i = fileList( event, &FileControl ) ;
	if ( i == 1 )	// Select
	{
		char ScriptFilename[60] ;
		cpystr( cpystr( (uint8_t *)ScriptFilename, (uint8_t *)"/SCRIPTS/" ), (uint8_t *)FileList.Filenames[fc->vpos] ) ;
		WatchdogTimeout = 300 ;		// 3 seconds
#ifdef LUA
		if ( g_model.basic_lua )
		{
			luaExec(ScriptFilename) ;
//			RotaryState = ROTARY_MENU_UD ;
		}
		else
		{
			if ( loadBasic( ScriptFilename, BASIC_LOAD_ALONE ) == 0 )
			{
				// Didn't load
				basicLoadModelScripts() ;
			}
			RotaryState = ROTARY_MENU_UD ;
		}
#else
		if ( loadBasic( ScriptFilename, BASIC_LOAD_ALONE ) == 0 )
		{
			// Didn't load
//			basicLoadModelScripts() ;
		}
//		RotaryState = ROTARY_MENU_UD ;
//		basicExec(ScriptFilename) ;
#endif
	}
//	else if ( i == 2 )	// EXIT
//	{
//    killEvents(event) ;
////    popMenu() ;
//	}
//#ifdef BASIC


//extern uint8_t BasicLoadedType ;

//	lcd_outhex4( 0, 6*FH, BasicLoadedType ) ;
	
//extern uint8_t LoadingIndex ;
	
////extern uint8_t BasicErrorText[] ;
////	lcd_puts_Pleft( 5*FH, (char *)BasicErrorText ) ;

//#endif
}







uint16_t TelCounts[2] ;


void menuDebug(uint8_t event)
{
	uint8_t x ;
	uint16_t v ;
	TITLE(PSTR("DEBUG")) ;
	static MState2 mstate2 ;
	event = mstate2.check( event, e_debug, menuTabStat, DIM(menuTabStat), 0, 0, 0 ) ;

	lcd_outhex4(0, 7*FH, TelCounts[0] ) ;
	lcd_outhex4(6*FW, 7*FH, TelCounts[1] ) ;
	  


}

void checkXyCurve()
{
	uint32_t k ;
	for ( k = 0 ; k < MAX_CURVEXY ; k += 1 )
	{
		if ( g_model.curvexy[k][9] == 0 )
		{
			uint32_t i ;
			int8_t j = -100 ;
			for ( i = 9 ; i < 18 ; j += 25, i += 1 )
			{
				g_model.curvexy[k][i] = j ;
			}
		}
		
	}
}

static uint32_t popupDisplay( const char *list, uint16_t mask, uint8_t width )
{
	uint32_t entries = 0 ;
	uint8_t y = 0 ;

	while ( mask )
	{
		if ( mask & 1 )
		{
			lcd_putsn_P( 3*FW, y, "                ", width ) ;
			lcd_puts_P( 4*FW, y, (const char *)(list) ) ;
			entries += 1 ;
			y += FH ;
		}
		mask >>= 1 ;
		while ( *list )
		{
			list += 1 ;			
		}		
		list += 1 ;			
	}
	if ( y >= 7*FH )
	{
		y -= 1 ;
	}
	lcd_rect( 3*FW, 0, width*FW, y+1 ) ;
	lcd_char_inverse( 4*FW, (PopupData.PopupIdx)*FH, (width-2)*FW, 0 ) ;

	return entries ;
}

static uint8_t popupProcess( uint8_t event, uint8_t max )
{
	int8_t popidxud = 0 ; // qRotary() ;
	uint8_t popidx = PopupData.PopupIdx ;

	if ( PopupData.PopupTimer )
	{
		if ( BLINK_ON_PHASE )
		{
			if ( --PopupData.PopupTimer == 0 )
			{
				// Timeout popup
				PopupData.PopupActive = 0 ;
				return POPUP_EXIT ;
			}
		}
	}
	else
	{
		PopupData.PopupTimer = 51 ;
  }

	switch(event)
	{
    case EVT_KEY_BREAK(KEY_MENU) :
			PopupData.PopupActive = 0 ;
			PopupData.PopupTimer = 0 ;
		return POPUP_SELECT ;
    
		case EVT_KEY_FIRST(KEY_UP) :
			popidxud = -1 ;
		break ;
    
		case EVT_KEY_FIRST(KEY_DOWN) :
			popidxud = 1 ;
		break ;
    
//		case EVT_KEY_LONG(BTN_RE) :
//			if ( g_eeGeneral.disableBtnLong )
//			{
//				break ;
//			}		
		case EVT_KEY_BREAK(KEY_EXIT) :
			killEvents( event ) ;
			PopupData.PopupActive = 0 ;
			PopupData.PopupTimer = 0 ;
		return POPUP_EXIT ;
	}

	if (popidxud > 0)
	{
		if ( popidx < max )
		{
			popidx += 1 ;
		}
		else
		{
			popidx = 0 ;			
		}
	}
	else if (popidxud < 0)
	{		
		if ( popidx )
		{
			popidx -= 1 ;
		}
		else
		{
			popidx = max ;
		}
	}

	if (popidxud )
	{
		if ( PopupData.PopupTimer )
		{
			PopupData.PopupTimer = 51 ;
		}
	}	

	PopupData.PopupIdx = popidx ;
	return POPUP_NONE ;
}

static uint8_t popTranslate( uint8_t popidx, uint16_t mask )
{
	uint8_t position ;
	popidx += 1 ;
	for ( position = 0 ; position < 16 ; position += 1 )
	{
		if ( mask & 1 )
		{
			if ( --popidx == 0)
			{
				break ;
			}
		}
		mask >>= 1 ;
	}
	return position ;
}

uint32_t doPopup( const char *list, uint16_t mask, uint8_t width, uint8_t event )
{
	uint32_t count = popupDisplay( list, mask, width ) ;
	uint32_t popaction = popupProcess( event, count - 1 ) ;
	uint32_t popidx = PopupData.PopupIdx ;
	PopupData.PopupSel = popTranslate( popidx, mask ) ;
	return popaction ;
}

void actionMainPopup( uint8_t event )
{
	uint16_t mask = 0x143F ;

  if(PopupData.PopupActive == 1)
	{
		mask = 0xE3C0 ;
	}
  else if(PopupData.PopupActive == 3)
	{
		mask = 0x183F ;
	}
		
	uint8_t popaction = doPopup( STR_MAIN_POPUP, mask, 16, event ) ;
																 
//	UseLastSubmenuIndex = 0 ;
  if ( popaction == POPUP_SELECT )
	{
		uint8_t popidx = PopupData.PopupSel ;
		if ( popidx == 0 )	// Model Select
		{
      pushMenu(menuModelSelect) ;
		}
		else if( popidx == 1 )	// Edit Model
		{
//			RotaryState = ROTARY_MENU_UD ;
	  	pushMenu(menuModelIndex) ;
		}
//		else if( popidx == 2 )	// Last Menu
//		{
////			UseLastSubmenuIndex = 1 ;
////      pushMenu(lastPopMenu());
//		}
		else if ( popidx == 3 )	// Radio Setup
		{
      pushMenu(menuRadioIndex) ;
		}
		else if( popidx == 4 )	// Statistics
		{
////			RotaryState = ROTARY_MENU_LR ;
	  	pushMenu(menuBattery) ;
		}
		else if( popidx == 5 )	// Notes
		{
			TextControl.TextHelp = 0 ;
	  	pushMenu(menuText) ;
		}
		else if( popidx == 6 )	// Zero Alt.
		{
			resetTelemetry( TEL_ITEM_RESET_ALT ) ;
		}
		else if( popidx == 7 )	// A1 Offset
		{
      if ( g_model.frsky.channels[0].units == 3 )		// Current (A)
			{
				resetTelemetry( TEL_ITEM_RESET_A1OFF ) ;
			}
		}
		else if( popidx == 8 )	// A2 Offset
		{
      if ( g_model.frsky.channels[1].units == 3 )		// Current (A)
			{
				resetTelemetry( TEL_ITEM_RESET_A2OFF ) ;
			}
		}
		else if( popidx == 9 )	// GPS reset
		{
			resetTelemetry( TEL_ITEM_RESET_GPS ) ;
		}
//		else if( popidx == 10 )	// Help
//		{
////			SharedMemory.TextControl.TextHelp = 1 ;
////	  	pushMenu(menuProcText) ;
//		}
//		else if( popidx == 11 )	// Main Display
//		{
////			g_model.mview = 0 ;
//		}
		else if( popidx == 12 )	// Run Script
		{
////				SharedMemory.TextControl.TextHelp = 1 ;
//			RotaryState = ROTARY_MENU_UD ;
	  	pushMenu(menuScript) ;
		}
		else if( popidx == 13 )	// Reset Telemetry
		{
      resetTelemetry( TEL_ITEM_RESET_ALL ) ;
		}
		else if( ( popidx == 14 )	|| ( popidx == 15 ) )// Reset Telemetry
		{
      resetTimern( popidx - 14 ) ;
		}
	}
}

//uint8_t LastEvent ;

void displayTimer( uint8_t x, uint8_t y, uint8_t timer, uint8_t att )
{
	struct t_timer *tptr = &s_timer[timer] ;
  att |= (tptr->s_timerState==TMR_BEEPING ? BLINK : 0);
  putsTime( x, y, tptr->s_timerVal, att, att ) ;
}

void menuDeleteDupModel(uint8_t event)
{
	uint8_t action ;
  lcd_putsnAtt(1,2*FH, (char *)ModelNames[DupSub],sizeof(g_model.name),/*BSS*/0);
  lcd_putc(sizeof(g_model.name)*FW+FW,2*FH,'?');
	action = yesNoMenuExit( event, DupIfNonzero ? PSTR(STR_DUP_MODEL) : PSTR(STR_DELETE_MODEL) ) ;

	switch(action)
	{
    case YN_YES :
      if ( DupIfNonzero )
      {
        message(PSTR(STR_DUPLICATING));
        if(eeDuplicateModel(DupSub))
        {
          audioDefevent(AU_MENUS);
          DupIfNonzero = 2 ;		// sel_editMode = false;
        }
        else audioDefevent(AU_WARNING1);
      }
      else
      {
				ee32_delete_model( DupSub-1 ) ;
      }
    break;
		
		case YN_NO :
//      pushMenu(menuProcModelSelect);
    break;
  }
}

void displayCustomPage( uint32_t page )
{
	lcd_putsnAtt(0, 0, g_model.name, sizeof(g_model.name), INVERS) ;
extern uint16_t g_vbat10mV ;
  uint8_t att = g_vbat10mV/10 < g_eeGeneral.vBatWarn ? BLINK : 0;
	putsVolts( 14*FW, 0, g_vbat10mV/10, att) ;
	displayTimer( 18*FW+5, 0, 0, 0 ) ;
	
  for (uint32_t i = 0 ; i < 6 ; i += 1 )
	{
		uint32_t j ;
		j = g_model.customDisplayIndex[page][i] ;
		if ( j )
		{
			uint32_t index = j-1 ;
			uint32_t x = (i&1)?65:1 ;
			uint32_t y = (i&0x0E)*FH+2*FH ;
			uint8_t style = TELEM_LABEL|TELEM_UNIT|TELEM_UNIT_LEFT|TELEM_VALUE_RIGHT ;
//	    if ( index == TEL_ITEM_T1 )
//			{
//				if ( ( g_model.telemetryProtocol == TELEMETRY_ARDUCOPTER ) || ( g_model.telemetryProtocol == TELEMETRY_ARDUPLANE ) || ( g_model.telemetryProtocol == TELEMETRY_MAVLINK ) )
//				{
//					style = TELEM_ARDUX_NAME ;
//				}	
//			}
			putsTelemetryChannel( x, y, index, get_telemetry_value(index),
									 DBLSIZE|CONDENSED, style ) ;
		}
	}

	lcd_puts_Pleft( 7*FH, "RSSI" ) ;
	lcd_hbar( 30, 57, 43, 6, get_telemetry_value(TEL_ITEM_RSSI) ) ;
	lcd_vline( 63, 8, 48 ) ;
}


void menuProc0(uint8_t event)
{
  static uint8_t trimSwLock ;
	uint32_t i ;
	switch(event)
	{
	  case EVT_KEY_LONG(KEY_MENU) :		// Nav Popup
			PopupData.PopupActive = 2 ;
			PopupData.PopupIdx = 0 ;
     	killEvents(event) ;
			event = 0 ;
    break ;

	  case EVT_KEY_BREAK(KEY_MENU) :
			if ( PopupData.PopupActive == 0 )
			{
				if ( ++MainDisplayIndex > 3 )
				{
					MainDisplayIndex = 0 ;
				}
			}
    break ;
	  
		case EVT_KEY_BREAK(KEY_EXIT) :
			if ( PopupData.PopupActive == 0 )
			{
				PopupData.PopupActive = 1 ;
				PopupData.PopupIdx = 0 ;
     		killEvents(event) ;
				event = 0 ;
			}
    break ;
	}

	{
		uint8_t tsw ;
		tsw = getSwitch00(g_model.trimSw) ;
		if( tsw && !trimSwLock)
		{
			setStickCenter(0) ;
		}
		trimSwLock = tsw ;
	}

	switch ( MainDisplayIndex )
	{
		case 0 :
		{
			lcd_img( 33, 0, (uint8_t *)HeadingImg, 0, 0 ) ;
			lcd_putsnAtt( 3*FW, 2*FH+2, g_model.name, MODEL_NAME_LEN, BOLD ) ; //DBLSIZE ) ;

			uint8_t x = FW*2 ;
			displayTimer( x+14*FW-1, FH*2, 0, DBLSIZE|CONDENSED ) ;

extern uint16_t g_vbat10mV ;

  		uint8_t att = g_vbat10mV/10 < g_eeGeneral.vBatWarn ? BLINK | PREC2 : PREC2 ;

			lcd_outdezAtt( 113, 0*FH, g_vbat10mV, att ) ;
			lcd_putc( 114, 0*FH, 'v' ) ;

			doMainScreenGraphics() ;
		extern void displayTrims() ;
			displayTrims() ;

			i = getFlightPhase() ;

			if ( i )
			{
				if (g_model.phaseData[i-1].name[0] != ' ' )
				{
					lcd_putsnAtt( 64-3*FW, 4*FH, g_model.phaseData[i-1].name, 6, /*BSS*/ 0 ) ;
				}
				else
				{
  				lcd_putc( 64-3*FW, 4*FH, 'F' ) ;
  				lcd_putc( 64-2*FW, 4*FH, '0'+ i ) ;
				}
			}
			else
			{
				lcd_putc( 64-3*FW, 4*FH, 'F' ) ;
				lcd_putc( 64-2*FW, 4*FH, '0' ) ;
			}
			lcd_rect( 64-3*FW-1, 4*FH-1, 6*FW+2, 9 ) ;   // put a bounding box
		}
		break ;
		case 1 :
		{	
			lcd_img( 33, 0, (uint8_t *)HeadingImg, 0, 0 ) ;
			lcd_putsnAtt( 3*FW, 2*FH+2, g_model.name, MODEL_NAME_LEN, BOLD ) ; //DBLSIZE ) ;

			uint8_t x = FW*2 ;
			displayTimer( x+14*FW-1, FH*2, 0, DBLSIZE|CONDENSED ) ;

		extern uint16_t g_vbat10mV ;
			lcd_outdezAtt( 113, 0*FH, g_vbat10mV, PREC2 ) ;
			lcd_putc( 114, 0*FH, 'v' ) ;

//			lcd_puts_P( 0, 2*FH, "RSSI" ) ;
//			lcd_outdezAtt( 9*FW, 2*FH, TelemetryData[FR_RXRSI_COPY], DBLSIZE ) ;
	    
			for( i = 0 ; i < 8 ; i += 1 )
	    {
      	uint32_t x0, y0 ;
				uint32_t chan = i ; // 8 * io_subview + i ;
	      int16_t val = g_chans512[chan] ;
				x0 = i<4 ? 128/4+2 : 128*3/4-2 ;
				y0 = 38+(i%4)*5 ;
extern void singleBar( uint8_t x0, uint8_t y0, int16_t val ) ;
				singleBar( x0, y0, val ) ;
			}
extern void displayTrims() ;
			displayTrims() ;
		}
		break ;
		case 2 :
			displayCustomPage( 0 ) ;
//			lcd_img( 33, 0, (uint8_t *)HeadingImg, 0, 0 ) ;
//			lcd_puts_P( 2*FW, 7*FH, "P2" ) ;
		break ;
		case 3 :
			displayCustomPage( 1 ) ;
//			lcd_img( 33, 0, (uint8_t *)HeadingImg, 0, 0 ) ;
//			lcd_puts_P( 2*FW, 7*FH, "P3" ) ;
		break ;
	}
	
	if ( PopupData.PopupActive )
	{
		actionMainPopup( event ) ;
	}
	 
}

uint16_t SelectNewModel ;

uint32_t checkRssi(uint8_t event)
{
  if(g_eeGeneral.disableRxCheck)
	{
		return RSSI_POWER_OFF ;
	}
	if ( get_telemetry_value(TEL_ITEM_RSSI) == 0 )
	{
		return RSSI_POWER_OFF ;
	}

	if ( event == EVT_KEY_BREAK(KEY_EXIT) )
	{
		return RSSI_POWER_OFF ;
	}

// Now display warning
	clearDisplay() ;
extern uint8_t HandImage[] ;
	lcd_img( 1, 0, HandImage,0,0 ) ;
  lcd_putsAtt(36, 0*FH,XPSTR("Receiver"),DBLSIZE|CONDENSED) ;
  lcd_putsAtt(36, 2*FH,PSTR(STR_WARNING),DBLSIZE|CONDENSED) ;
	lcd_puts_P(0 ,5*FH, XPSTR("Rx still powered") ) ;
	lcd_puts_P(0 ,7*FH, PSTR(STR_PRESS_KEY_SKIP) ) ;
	refreshDisplay() ;

	return RSSI_STAY_ON ;
}

static uint8_t NewModel ;

void menuModelSelect(uint8_t event)
{
  static uint8_t sel_editMode ;
  static MState2 mstate2 ;
	if ( SelectNewModel )
	{
		if ( checkRssi( event ) == RSSI_POWER_OFF )
		{
			// Now can change models
//			stopMusic() ;
			WatchdogTimeout = 300 ;		// 3 seconds
//				stopMusic() ;
			eeDirty(EE_MODEL) ;
			ee32WaitFinished() ;
			
			NewModel = SelectNewModel & 0x00FF ;
			g_eeGeneral.currModel = NewModel ;
//			pausePulses() ;
			
			ee32LoadModel(g_eeGeneral.currModel) ; //load default values
			xSemaphoreGive( SpiMutex ) ;
			JustLoadedModel = 2 ;
			VoiceCheckFlag100mS |= 2 ;// Set switch current states
			processSwitches() ;	// Guarantee unused switches are cleared
//			telemetry_init( decodeTelemetryType( g_model.telemetryProtocol ) ) ;
////#ifdef LUA
////				luaLoadModelScripts() ;
////#endif
////#ifdef BASIC
////				basicLoadModelScripts() ;
////#endif
			eeDirty(EE_GENERAL) ;			
			
			if ( ( SelectNewModel >> 8 ) == 2 )
			{
				chainMenu(menuModelIndex) ;
			}
			else
			{
	      popMenu(true) ;
			}
			SelectNewModel = 0 ;
		}
		else
		{
			return ;
		}
	}
  TITLE(PSTR(STR_MODELSEL)) ;

  uint8_t subOld  = mstate2.m_posVert ;
	
	if ( !PopupData.PopupActive )
	{
//		RotaryState = ROTARY_MENU_UD ;
		event = mstate2.check_columns( event, MAX_MODELS-1 ) ;
	}

  uint8_t sub = mstate2.m_posVert ;
  if ( DupIfNonzero == 2 )
  {
      sel_editMode = false ;
      DupIfNonzero = 0 ;
  }
  
	if(sub-s_pgOfs < 1)
	{
		s_pgOfs = max(0,sub-1) ;
	}
  else if(sub-s_pgOfs > (SCREEN_LINES-4) )
	{
		s_pgOfs = min(MAX_MODELS-(SCREEN_LINES-2), sub-(SCREEN_LINES-4)) ;
	}
  for( uint32_t i = 0 ; i < SCREEN_LINES-2 ; i += 1 )
	{
    uint16_t y = (i+2)*FH ;
    uint32_t k = i + s_pgOfs ;
    lcd_outdezNAtt(  3*FW, y, k+1, ((sub==k) ? INVERS : 0) + LEADING0, 2) ;
    if(k==g_eeGeneral.currModel)
		{
			lcd_putc(1,  y,'*') ;
		}
    lcd_putsn_P(  4*FW, y, (char *)ModelNames[k+1], sizeof(g_model.name) ) ;
		
		if ( (sub==k) && ( sel_editMode ) )
		{
			lcd_rect( 0, y-1, 125, 9 ) ;
		}
  }

	if ( PopupData.PopupActive )
	{
		uint16_t mask ;
		if ( g_eeGeneral.currModel == mstate2.m_posVert )
		{
			mask = 0x219 ;
		}
		else
		{
			mask = ( eeModelExists( mstate2.m_posVert ) == 0 ) ?  0x16 :  0x003E ;
		}

		uint8_t popaction = doPopup( PSTR(STR_MODEL_POPUP), mask, 10, event ) ;
		
  	if ( popaction == POPUP_SELECT )
		{
			uint8_t popidx = PopupData.PopupSel ;
			if ( popidx == 0 )	// edit
			{
//				RotaryState = ROTARY_MENU_LR ;
				chainMenu(menuModelIndex) ;
			}
			else if ( ( popidx == 1 ) || ( popidx == 2 ) )	// select or SEL/EDIT
			{

//uint32_t checkRssi(uint32_t swappingModels) ;
//				checkRssi(1) ;
				
				SelectNewModel = mstate2.m_posVert | ( popidx << 8 ) ;
				return ;
				
//				WatchdogTimeout = 300 ;		// 3 seconds
////				stopMusic() ;
////				pausePulses() ;
//				eeDirty(EE_MODEL) ;
//				ee32WaitFinished() ;

//				g_eeGeneral.currModel = mstate2.m_posVert ;
////				chainMenu(menuSwapModel) ;
        
////				while( xSemaphoreTake( SpiMutex, pdMS_TO_TICKS( 2 ) ) != pdTRUE )
////				{
////					// wait
////				}
//				ee32LoadModel(g_eeGeneral.currModel) ; //load default values
//				xSemaphoreGive( SpiMutex ) ;
//				g_model.Module[INTERNAL_MODULE].protocol = PROTO_OFF ;
////				eeLoadModel( g_eeGeneral.currModel ) ;
////				loadModelImage() ;

////				protocolsToModules() ;
//				JustLoadedModel = 2 ;
////  			checkTHR();
////				checkCustom() ;
////				checkSwitches() ;
////				checkMultiPower() ;
//// 				perOut( g_chans512, NO_DELAY_SLOW | FADE_FIRST | FADE_LAST ) ;
				
////				SportStreamingStarted = 0 ;

////				speakModelVoice() ;
////        resetTimers();
//				VoiceCheckFlag100mS |= 2 ;// Set switch current states
//				processSwitches() ;	// Guarantee unused switches are cleared
////				telemetry_init( decodeTelemetryType( g_model.telemetryProtocol ) ) ;
////				resumePulses() ;
////#ifdef LUA
////				luaLoadModelScripts() ;
////#endif
////#ifdef BASIC
////				basicLoadModelScripts() ;
////#endif
////        STORE_GENERALVARS;
//				if ( ( PopupData.PopupActive == 2 ) || ( popidx == 2 ) )
//				{
//					chainMenu(menuModelIndex) ;
//				}
//				else
//				{
//	        popMenu(true) ;
//				}
			}
			else if ( popidx == 5 )		// Delete
			{
       	killEvents(event);
       	DupIfNonzero = 0 ;
				DupSub = sub + 1 ;
       	pushMenu(menuDeleteDupModel);
			}
			else if( popidx == 3 )	// copy
			{
				{
 	        DupIfNonzero = 1 ;
 	        DupSub = sub + 1 ;
 	        pushMenu(menuDeleteDupModel);//menuProcExpoAll);
				}
			}
//			else if( popidx == 6 )	// backup
//			{
//				WatchdogTimeout = 300 ;		// 3 seconds
//				BackResult = ee32BackupModel( mstate2.m_posVert+1 ) ;
//				AlertType = MESS_TYPE ;
//				AlertMessage = BackResult ;
//			}
			else if( popidx == 7 )	// restore
			{
	        
					popMenu(true) ;


//				RestoreIndex = mstate2.m_posVert+1 ;
//       	pushMenu( menuProcRestore ) ;				
			}
//			else if( popidx == 8 )	// replace
//			{
//				WatchdogTimeout = 300 ;		// 3 seconds
//				BackResult = ee32BackupModel( mstate2.m_posVert+1 ) ;
//				AlertType = MESS_TYPE ;
//				AlertMessage = BackResult ;
//				RestoreIndex = mstate2.m_posVert+1 ;
//       	pushMenu( menuProcRestore ) ;				
//			}
			else if( popidx == 9 )	// Notes
			{
       	pushMenu( menuEditNotes ) ;
			}
			else if( popidx == 4 ) // Move = 4
			{
 	    	sel_editMode = true ;
			}
		}
	}
	else
	{
  	switch(event)
  	{
//  	//case  EVT_KEY_FIRST(KEY_MENU):
	  	case  EVT_KEY_FIRST(KEY_EXIT):
  	    if(sel_editMode)
				{
 	        sel_editMode = false;
  	    }
      break ;

//	  	case  EVT_KEY_FIRST(KEY_LEFT):
//  		case  EVT_KEY_FIRST(KEY_RIGHT):
//  	    if(g_eeGeneral.currModel != mstate2.m_posVert)
//  	    {
//          killEvents(event);
//					PopupData.PopupIdx = 0 ;
//					PopupData.PopupActive = 2 ;
//  	    }
//				else
//				{
//					RotaryState = ROTARY_MENU_LR ;
//		      if(event==EVT_KEY_FIRST(KEY_LEFT))  chainMenu(menuProcModelIndex);//{killEvents(event);popMenu(true);}

//		      if(event==EVT_KEY_FIRST(KEY_RIGHT)) chainMenu(menuProcModelIndex);
//				}
// 	    break;
  		
			case  EVT_KEY_FIRST(KEY_MENU) :
//			case  EVT_KEY_BREAK(BTN_RE) :
				if(sel_editMode)
				{
  	    	sel_editMode = false ;
				}
				else
				{
          killEvents(event);
					PopupData.PopupIdx = 0 ;
					PopupData.PopupActive = 1 ;
				}	 
  	    s_editMode = 0 ;
  	  break;
  	
//			case  EVT_KEY_LONG(BTN_RE) :
//			if ( g_eeGeneral.disableBtnLong )
//			{
//				break ;
//			}		
			case  EVT_KEY_LONG(KEY_EXIT):  // make sure exit long exits to main
  	  	killEvents(event);
  	    popMenu(true);
      break;

  		case EVT_ENTRY:
  	    sel_editMode = false;
				PopupData.PopupActive = 0 ;
  	    mstate2.m_posVert = g_eeGeneral.currModel ;
    	break;
  	}
	}

  if(sel_editMode && subOld!=sub)
	{
		ee32SwapModels( subOld+1, sub+1 ) ;

		if ( sub == g_eeGeneral.currModel )
		{
			g_eeGeneral.currModel = subOld ;
//  	  STORE_GENERALVARS ;     //eeWriteGeneral();
		}
		else if ( subOld == g_eeGeneral.currModel )
		{
			g_eeGeneral.currModel = sub ;
//  	  STORE_GENERALVARS ;     //eeWriteGeneral();
		}
  }
}






