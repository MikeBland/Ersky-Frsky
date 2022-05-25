/****************************************************************************
*  erskytx_neo.ino
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

//#define CONFIG_SUPPORT_STATIC_ALLOCATION	1

#include <Arduino.h>
#include <U8x8lib.h>
#include <Wire.h>
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#include <BleGamepad.h>
#include "erskyTx.h"
#include "myeeprom.h"
#include "menus.h"
#include "lcd.h"
#include "logicIo.h"
#include "file.h"
#include "en.h"
#include "pxx2.h"
#include "frskywifi.h"
#include "audio.h"
#include "sound.h"
#include "telemetry.h"
#include "basic.h"

#define BASIC	1

#include "freertos/semphr.h"
#include <esp32-hal-timer.h>

#include <SPIFFS.h>

#include <driver/dac.h>

#include "handlbm.h"
#include "LITTLEFS.h"

#define SPI_CLOCK 9
#define SPI_DATA 10
#define SPI_CS 27
#define SPI_DC 20
#define SPI_RESET 5

#define AW1_ADDRESS		0x5B
#define AW2_ADDRESS		0x59
#define RTC_ADDRESS		0x51

#define RSSI_POWER_OFF	1
#define RSSI_STAY_ON		0

uint32_t readAws() ;
void initI2C() ;
void setI2C400kHz() ;
uint8_t readI2CByte( uint8_t address, uint8_t index ) ;
uint32_t writeI2CByte( uint8_t address, uint8_t index, uint8_t byte ) ;
//uint32_t readI2Cmulti( uint8_t address, uint8_t index, uint8_t count, uint8_t *buffer ) ;
//uint32_t writeI2Cmulti( uint8_t address, uint8_t index, uint8_t count, uint8_t *buffer ) ;
void runWifi() ;
void setupPulses(uint32_t module) ;
int32_t pollSportFifo() ;
void telemetryRecieveByte( uint8_t byte, uint32_t module ) ;
uint8_t menuPressed() ;
extern uint8_t LongMenuTimer ;

void startJoystick() ;
void handleJoystick() ;

// Temporary -> mixer
extern uint8_t	CurrentPhase ;
void timer(int16_t throttle_val) ;
uint16_t WatchdogTimeout ;
// Temporary -> from telemetry.cpp
uint16_t TelRxCount ;
uint16_t AccessSent ;
// Temporary -> pulses
uint8_t PxxSerial[2][50] ;
uint8_t *PtrSerialPxx[2] ;
volatile uint8_t *PxxTxPtr ;
volatile uint8_t PxxTxCount ;
volatile uint8_t *PxxTxPtr_x ;
volatile uint8_t PxxTxCount_x ;
// Temporary -> pulses/Timers
uint8_t BindRangeFlag[2] = { 0, 0 } ;

uint32_t PlaySquareWave ;

uint32_t UsedMenu ;
uint32_t UsedMixer ;
uint32_t Used10ms ;
uint8_t ScriptActive ;
uint8_t AlertType ;
const char *AlertMessage ;

uint32_t Used1secMenu ;
uint32_t Used1secMixer ;
uint32_t Used1sec10ms ;
int16_t g_ppmIns[16] ;

static uint8_t powerIsOn = 1 ;
uint32_t PoweringOff ;
uint32_t PowerOffTimeout ;

uint8_t ppmInValid = 0 ;
uint8_t Activated ;
uint8_t MaintenanceMode ;

#define INT_RF_ON		0x01
#define CHARGE_SET	0x08

#define EXT_RF_ON		0x40

#define LCD_RESET_PIN		0x80

#define LEDR_PIN				0x02
#define LEDG_PIN				0x04
#define LEDB_PIN				0x08

uint8_t Aw1Outputs[2] ;
uint8_t Aw2Outputs[2] ;

uint8_t Aw1HighRequired ;
uint8_t Aw2LowRequired ;

uint16_t g_LightOffCounter ;

uint8_t JustLoadedModel ;
uint8_t BTjoystickActive ;
uint8_t BTjoystickStarted ;
uint32_t checkRssi(uint8_t event) ;
//uint8_t ModelWarningsActive ;

const uint8_t bchout_ar[] = {
															0x1B, 0x1E, 0x27, 0x2D, 0x36, 0x39,
															0x4B, 0x4E, 0x63, 0x6C, 0x72, 0x78,
                              0x87, 0x8D, 0x93, 0x9C, 0xB1, 0xB4,
                              0xC6, 0xC9, 0xD2, 0xD8, 0xE1, 0xE4		} ;


// Pin connections
// AW1:
// P0.0 ENTER
// P0.1 MENU
// P0.2 EXIT
// P0.3 CHGSTA
// P0.4 TRIM DOWN
// P0.5 TRIM RIGHT
// P0.6 TRIM LEFT
// P0.7 TRIM UP
// P1.3 ICHGSET
// P1.0 RFPWRON

// Heartbeat
volatile uint16_t HeartbeatTime ;

#define SW_STACK_SIZE	6

uint8_t Last_switch[NUM_SKYCSW] ;
uint8_t Now_switch[NUM_SKYCSW] ;
int16_t CsTimer[NUM_SKYCSW] ;

struct t_NvsControl
{
	uint8_t nvs_state ;
	uint8_t nvs_delay ;
	int16_t nvs_timer ;
	int16_t nvs_last_value ;
} NvsControl[NUM_VOICE_ALARMS + NUM_GLOBAL_VOICE_ALARMS] ;


int8_t SwitchStack[SW_STACK_SIZE] ;

uint8_t CheckTimer = 2 ;
uint8_t VoiceTimer = 10 ;		// Units of 10 mS
uint8_t VoiceCheckFlag100mS = 0 ;
uint32_t OneSecPreCount ;
volatile uint32_t OneSecFlag ;

//uint8_t EepromStatus ;
uint8_t TempBuffer[256] ;
uint8_t Tb[12] ;

EE_X20General g_eeGeneral ;
X20ModelData g_model ;

uint16_t AnalogData[6] ;
int16_t CalibratedStick[4] ;
extern int8_t phyStick[] ;

U8X8_ST7567_ENH_DG128064_4W_HW_SPI u8x8(/* cs=*/ SPI_CS, /* dc=*/ SPI_DC, /* reset=*/ U8X8_PIN_NONE); 

uint32_t PowerStart ;
uint32_t PowerByUsb ;
uint32_t Now ;
uint16_t Tenms ;

uint32_t PreScale ;
uint32_t NotFirstTime ;

uint32_t TempPageIndex ;

uint16_t g_vbat10mV ;
uint16_t Battery ;
uint16_t BattAverage ;
uint8_t BattCount ;
uint16_t UsbV ;
uint16_t AdcReadTime ;

uint32_t ResetReason ;

MenuFuncP g_menuStack[10];
uint8_t  g_menuStackPtr = 0;
uint8_t  EnterMenu = 0 ;
uint32_t SwitchesStates = 0 ;

uint8_t SystemOptions ;

uint8_t MaxSwitchIndex = MAX_SKYDRSWITCH ;		// For ON and OFF

t_time Time ;

const char stickScramble[] =
{
  0, 1, 2, 3,
  0, 2, 1, 3,
  3, 1, 2, 0,
  3, 2, 1, 0
} ;

extern void menuUpdate( uint8_t event ) ;
extern void pollMrx() ;
int16_t m_to_ft( int16_t metres ) ;


hw_timer_t *PTg0T0 ;
hw_timer_t *PTg0T1 ;	// 2Mhz_timer

SemaphoreHandle_t SpiMutex = NULL ;
StaticSemaphore_t SpiMutexBuffer ;

BleGamepad bleGamepad("FrskyNeo", "Joystick", 100);

void checkAw1() ;
void checkAw2() ;

void intRfOn()
{
	Aw1HighRequired &= ~INT_RF_ON ;
}

uint32_t isIntRfOn()
{
	return (Aw1HighRequired & INT_RF_ON) ? 0 : 1 ;
}

void intRfOff()
{
	Aw1HighRequired |= INT_RF_ON ;
}

void extRfOn()
{
	Aw2LowRequired |= EXT_RF_ON ;
}

uint32_t isExtRfOn()
{
	return (Aw2LowRequired & EXT_RF_ON) ? 1 : 0 ;
}

void extRfOff()
{
	Aw2LowRequired &= ~EXT_RF_ON ;
}

void chargeHigh()
{
	Aw1HighRequired |= CHARGE_SET ;
}

void chargeLow()
{
	Aw1HighRequired &= ~CHARGE_SET ;
}

static void startTg0T1()
{
	PTg0T1 = timerBegin( 1, 49, true) ;
//	timerAlarmWrite( PTg0T0, 4999, true) ;
//	timerAttachInterrupt( PTg0T0, timer0Interrupt, true) ;
//	timerAlarmEnable( PTg0T0 ) ;
	timerStart( PTg0T1 ) ;
}

void start_2Mhz_timer()
{
	startTg0T1() ;
}

uint16_t getTmr2MHz()
{
//	hw_timer_reg_t *pTimer ;

//	pTimer = (hw_timer_reg_t *)DR_REG_TIMERGROUP0_BASE ;
//	pTimer.update = 0 ;
//	return pTimer.cnt_low ;
	return timerRead(PTg0T1) ;
}

void softPowerOff()
{
	GPIO.out1_w1tc.val = 1 ;
}

void softPowerOn()
{
	GPIO.out1_w1ts.val = 1 ;
}

uint32_t readPowerSwitch()
{
	return GPIO.in1.val & 2 ;
}

const uint8_t csTypeTable[] =
{ CS_VOFS, CS_VOFS, CS_VOFS, CS_VOFS, CS_VBOOL, CS_VBOOL, CS_VBOOL,
 CS_VCOMP, CS_VCOMP, CS_VCOMP, CS_VCOMP, CS_VBOOL, CS_VBOOL, CS_TIMER, 
 CS_TIMER, CS_TMONO, CS_TMONO, CS_VOFS, CS_U16, CS_VCOMP, CS_VOFS, CS_2VAL
} ;

uint8_t CS_STATE( uint8_t x)
{
	return csTypeTable[x-1] ;
}

uint8_t IS_EXPO_THROTTLE( uint8_t x )
{
	if ( g_model.thrExpo )
	{
		return IS_THROTTLE( x ) ;
	}
	return 0 ;
}

uint8_t switchMapTable[100] ;
uint8_t switchUnMapTable[100] ;

void createSwitchMapping()
{
	uint8_t *p = switchMapTable ;
	
	*p++ = 0 ;
	*p++ = HSW_SA0 ;
	*p++ = HSW_SA1 ;
	*p++ = HSW_SA2 ;
	
	*p++ = HSW_SB0 ;
	*p++ = HSW_SB1 ;
	*p++ = HSW_SB2 ;

	*p++ = HSW_SC0 ;
	*p++ = HSW_SC1 ;
	*p++ = HSW_SC2 ;
	
	*p++ = HSW_SD0 ;
	*p++ = HSW_SD1 ;
	*p++ = HSW_SD2 ;
	 
	*p++ = HSW_Ttrmup ;
	*p++ = HSW_Ttrmdn ;
	*p++ = HSW_Rtrmup ;
	*p++ = HSW_Rtrmdn ;
	*p++ = HSW_Atrmup ;
	*p++ = HSW_Atrmdn ;
	*p++ = HSW_Etrmup ;
	*p++ = HSW_Etrmdn ;
	 
	for ( uint32_t i = 10 ; i <=33 ; i += 1  )
	{
		*p++ = i ;	// Custom switches
	}
	*p = MAX_SKYDRSWITCH ;
	MaxSwitchIndex = p - switchMapTable ;
	*++p = MAX_SKYDRSWITCH+1 ;
	*++p = MAX_SKYDRSWITCH+2 ;

	for ( uint32_t i = 0 ; i <= (uint32_t)MaxSwitchIndex+2 ; i += 1  )
	{
		switchUnMapTable[switchMapTable[i]] = i ;
	}
}

int8_t switchUnMap( int8_t x )
{
	uint8_t sign = 0 ;
	if ( x < 0 )
	{
		sign = 1 ;
		x = -x ;
	}
	x = switchUnMapTable[x] ;
	if ( sign )
	{
		x = -x ;
	}
	return x ;
}

int8_t switchMap( int8_t x )
{
	uint8_t sign = 0 ;
	if ( x < 0 )
	{
		sign = 1 ;
		x = -x ;
	}
	x = switchMapTable[x] ;
	if ( sign )
	{
		x = -x ;
	}
	return x ;
}




int16_t scaleAnalog( int16_t v, uint8_t channel ) ;

//int16_t scaleAnalog( int16_t v, uint8_t channel )
//{
//	int16_t mid ;
//	int16_t neg ;
//	int16_t pos ;

//	mid = g_eeGeneral.calibMid[channel] ;
//  pos = g_eeGeneral.calibSpanPos[channel] ;
//  neg = g_eeGeneral.calibSpanNeg[channel] ;

//	v -= mid ;

//	v  =  v * (int32_t)RESX /  (max((int16_t)100,(v>0 ? pos : neg ) ) ) ;
	
//	if(v <= -RESX) v = -RESX;
//	if(v >=  RESX) v =  RESX;
	
//	return v ;
//}



void readAnalog()
{
//	static uint8_t notFirstTimeAdc = 0 ;
//	if ( notFirstTimeAdc )
//	{
//		AnalogData[0] = 4095 - __analogRawRead( 2 ) ;
//		AnalogData[1] = 4095 - __analogRawRead( 3 ) ;
//		AnalogData[2] = __analogRawRead( 1 ) ;
//		AnalogData[3] = __analogRawRead( 0 ) ;
//		AnalogData[4] = __analogRawRead( 6 ) ;
//		AnalogData[5] = __analogRawRead( 7 ) ;
//	}
//	else
	{
		AnalogData[0] = 4095 - analogRead( 38 ) ;
		AnalogData[1] = 4095 - analogRead( 39 ) ;
		AnalogData[2] = analogRead( 37 ) ;
		AnalogData[3] = analogRead( 36 ) ;
		AnalogData[4] = analogRead( 34 ) ;
		AnalogData[5] = analogRead( 35 ) ;
//		notFirstTimeAdc = 1 ;
	}
}

void DO_RSQUARE( uint16_t x, uint16_t y, uint16_t w )
{
  lcd_vline(x-w/2,y-w/2+1,w-2);
  lcd_hline(x-w/2+1,y+w/2,w-2);
  lcd_vline(x+w/2,y-w/2+1,w-2);
  lcd_hline(x-w/2+1,y-w/2,w-2);

}

void DO_SQUARE( uint16_t x, uint16_t y, uint16_t w )
{
  lcd_vline(x-w/2,y-w/2,w);
  lcd_hline(x-w/2+1,y+w/2,w-2);
  lcd_vline(x+w/2,y-w/2,w);
  lcd_hline(x-w/2+1,y-w/2,w-2);
}

void DO_CROSS( uint16_t xx, uint16_t yy, uint16_t ww )
{
  lcd_vline(xx,yy-ww/2,ww) ;
  lcd_hline(xx-ww/2,yy,ww) ;

}


#define BOX_WIDTH     23
#define BAR_HEIGHT    (BOX_WIDTH-1l)
#define MARKER_WIDTH  5
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define BOX_LIMIT     (BOX_WIDTH-MARKER_WIDTH)
//#define LBOX_CENTERX  (  SCREEN_WIDTH/4 + 10-1)
//#define RBOX_CENTERX  (3*SCREEN_WIDTH/4 - 10)
#define LBOX_CENTERX  (  SCREEN_WIDTH/4 )
#define RBOX_CENTERX  (3*SCREEN_WIDTH/4 )
#define BOX_CENTERY  (SCREEN_HEIGHT-9-BOX_WIDTH/2-1)

void telltale( uint16_t centrex, int8_t xval, int8_t yval )
{
  DO_SQUARE( centrex, BOX_CENTERY, BOX_WIDTH ) ;
  DO_CROSS( centrex, BOX_CENTERY, 3 ) ;
	DO_RSQUARE( centrex +( xval*BOX_LIMIT/(2*RESX/16)), BOX_CENTERY-( yval*BOX_LIMIT/(2*RESX/16)), MARKER_WIDTH ) ;
}

void doMainScreenGraphics()
{
	int8_t *cs = phyStick ;
	pushPlotType( PLOT_BLACK ) ;
	telltale( LBOX_CENTERX, cs[0], cs[1] ) ;
	telltale( RBOX_CENTERX, cs[3], cs[2] ) ;
	popPlotType() ;
}

uint8_t modeFixValue( uint8_t value )
{
	return stickScramble[g_eeGeneral.stickMode*4+value]+1 ;
}

void displayTrims()
//void displayTrims( uint32_t small )
{
 	uint32_t i ;
  for( i=0 ; i<4 ; i++ )
  {
#define TL 27
#define TDIV 4
  	//                        LH LV RV RH
  	static uint8_t x[4]    = {128*1/4+2, 4, 128-4, 128*3/4-2};
  	register uint8_t xm, ym ;
		xm = modeFixValue( i ) ;
    xm = x[xm-1] ;

		register int16_t valt = getTrimValue( CurrentPhase, i ) ;
		uint8_t centre = (valt == 0) ;
    int8_t val = max((int8_t)-(TL+1),min((int8_t)(TL+1),(int8_t)(valt/TDIV)));
    if( (i == 1) || ( i == 2 ))
		{
  	  ym=31;
  	  lcd_vline(xm,   ym-TL, TL*2);

      if((i == 1) || !(g_model.thrTrim))
			{
        lcd_vline(xm-1, ym-1,  3);
        lcd_vline(xm+1, ym-1,  3);
 	    }
 	    ym -= val;
  	}
		else
		{
  	  ym=59;
  	  lcd_hline(xm-TL,ym,    TL*2);
  	  lcd_hline(xm-1, ym-1,  3);
  	  lcd_hline(xm-1, ym+1,  3);
  	  xm += val;
  	}
  	DO_RSQUARE(xm,ym,7) ;
		if ( centre )
		{
			pushPlotType( PLOT_BLACK ) ;
      DO_SQUARE(xm,ym,5) ;
			popPlotType() ;
		}
	}
}

void setupPulsesAccess( uint32_t module ) ;
void setupPulsesXjtLite( uint32_t module ) ;
uint8_t InternalModuleActive ;
uint8_t ExternalModuleActive ;

#define PULSES_STACK_SIZE 1400
StaticTask_t PulsesTaskBuffer ;
StackType_t PulsesStack[ PULSES_STACK_SIZE ] ;
TaskHandle_t PulsesHandle = NULL ;

#define MIXER_STACK_SIZE 1400

StaticTask_t MixerTaskBuffer ;
StackType_t MixerStack[ MIXER_STACK_SIZE ] ;
TaskHandle_t MixerHandle = NULL ;

#define DEBUG_STACK_SIZE 1800

StaticTask_t DebugTaskBuffer ;
StackType_t DebugStack[ DEBUG_STACK_SIZE ] ;
TaskHandle_t DebugHandle = NULL ;

#define VOICE_STACK_SIZE	1400

StaticTask_t VoiceTaskBuffer ;
StackType_t VoiceStack[ VOICE_STACK_SIZE ] ;
TaskHandle_t VoiceHandle = NULL ;

//#define EEPROM_STACK_SIZE	1200

//StaticTask_t EepromTaskBuffer ;
//StackType_t EepromStack[ EEPROM_STACK_SIZE ] ;
//TaskHandle_t EepromHandle = NULL ;


uint16_t PulsesRate ;
uint32_t PulsesCountForRate ;


uint16_t PulsesDuration[3] ;


void IRAM_ATTR pulsesTask(void * parameter)
{
	TickType_t xLastWakeTime ;
	uint16_t lastHeartbeat ;
	uint16_t x ;
	uint32_t updateRate ;
//	uint16_t lastSent ;
	uint32_t thread_notification ;

	while ( Activated == 0 )
	{
   	vTaskDelay( pdMS_TO_TICKS( 50 ) ) ;
	}
//	xLastWakeTime = xTaskGetTickCount() ;
//	lastHeartbeat = HeartbeatCount ;
	
	ulTaskNotifyTake( pdTRUE, 100 ) ;

  for(;;)
  { // infinite loop
	
		updateRate = 11 ;
		if ( ExternalModuleActive )
		{
			if ( g_model.Module[1].protocol == PROTO_XFIRE )
			{
				updateRate = 4 ;
			}
		}
		thread_notification = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(updateRate) ) ;
		x = getTmr2MHz() ;
		PulsesDuration[2] = x - HeartbeatTime ;

		if ( InternalModuleActive )
		{
			if ( thread_notification )
			{
//				lastHeartbeat = HeartbeatCount ;
				PulsesCountForRate += 1 ;
				if ( JustLoadedModel == 0 )
				{
					setupPulsesAccess( 0 ) ;
				}
//				lastSent = 0 ;
				x = getTmr2MHz() - x ;
				if ( x > PulsesDuration[0] )
				{
					PulsesDuration[0] = x ;
				}
			}
			else
			{
//				if ( ++lastSent > 10 )
				{
					PulsesCountForRate += 1 ;
					if ( JustLoadedModel == 0 )
					{
						setupPulsesAccess( 0 ) ;
					}
//					lastSent = 0 ;
					x = getTmr2MHz() - x ;
					if ( x > PulsesDuration[1] )
					{
						PulsesDuration[1] = x ;
					}
				}
//				else
//				{
//					x = getTmr2MHz() - x ;
//					if ( x > PulsesDuration[2] )
//					{
//						PulsesDuration[2] = x ;
//					}
//				}
			}
		}
	
		
		if ( ExternalModuleActive )
		{
			if ( g_model.Module[1].protocol == PROTO_PXX )
			{
				if ( thread_notification )
				{
	//				lastHeartbeat = HeartbeatCount ;
					PulsesCountForRate += 1 ;
					if ( JustLoadedModel == 0 )
					{
						setupPulses( 1 ) ;
					}
	//				lastSent = 0 ;
					x = getTmr2MHz() - x ;
					if ( x > PulsesDuration[0] )
					{
						PulsesDuration[0] = x ;
					}
				}
				else
				{
					PulsesCountForRate += 1 ;
					if ( JustLoadedModel == 0 )
					{
						setupPulses( 1 ) ;
					}
					x = getTmr2MHz() - x ;
					if ( x > PulsesDuration[1] )
					{
						PulsesDuration[1] = x ;
					}
				}
			}
			else if ( g_model.Module[1].protocol == PROTO_XFIRE )
			{
				if ( JustLoadedModel == 0 )
				{
					setupPulses( 1 ) ;
				}
			}
		}
	}
}


//void eepromTask(void * parameter)
//{
//	TickType_t xLastWakeTime ;
//	xLastWakeTime = xTaskGetTickCount() ;
//  for(;;)
//	{
//		ee32_process() ;
//		vTaskDelayUntil( &xLastWakeTime, 10 );
//	}
//}

void mixerTask(void * parameter)
{
	TickType_t xLastWakeTime ;
	
	while ( Activated == 0 )
	{
   	vTaskDelay( pdMS_TO_TICKS( 50 ) ) ;
	}
	xLastWakeTime = xTaskGetTickCount() ;

	 
  for(;;)
  { // infinite loop

void runMixer() ;
		runMixer() ;

		// Run every 3ms
		vTaskDelayUntil( &xLastWakeTime, 3 );
//    vTaskDelay( pdMS_TO_TICKS( 4 ) ) ;
	}
}


// Test a timer
//uint32_t T0Count ;

//void timer0Interrupt()
//{
//	T0Count += 1 ;
//}



//uint32_t readTg0T0()
//{
//	hw_timer_reg_t *pTg0T0 ;
  
//	pTg0T0 = (hw_timer_reg_t *)DR_REG_TIMERGROUP0_BASE ;
//	pTg0T0.update = 0 ;
//	return pTg0T0.cnt_low ;
//}


// Debug Task
void handle_serial(void* pdata) ;

void IRAM_ATTR heartbeatHandler()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE ;
	
	HeartbeatTime = getTmr2MHz() ;
//	HeartbeatCount += 1 ;

	vTaskNotifyGiveFromISR( PulsesHandle, &xHigherPriorityTaskWoken ) ;
	portYIELD_FROM_ISR() ;
}

// RTC_CNTL_RESET_STATE_REG (0x3FF48034)
// bit 0x10 is watchdog (RWDT) for PROCPU

uint32_t SpiffsBegun ;
//uint32_t FileSize ;

void setup(void)
{
	uint32_t pinFunction ;

	ResetReason = *((uint32_t*)0x3FF48034 ) ;	// RTC_CNTL_RESET_STATE_REG

//	uint32_t intlev ;
//	intlev = XTOS_SET_INTLEVEL(15) ;

	start_2Mhz_timer() ;	//	startTg0T1() ;

//	XTOS_RESTORE_JUST_INTLEVEL(intlev) ;

  Wire.begin(-1,-1,400000) ;

//  aw_init() ;

	writeI2CByte( AW2_ADDRESS, 0x7F, 0 ) ;	// Reset Aw
	writeI2CByte( AW1_ADDRESS, 0x7F, 0 ) ;	// Reset Aw

	writeI2CByte( AW2_ADDRESS, 0x11, 0x10 ) ;	// Port0 pushpull Aw
	writeI2CByte( AW1_ADDRESS, 0x11, 0x10 ) ;	// Port0 pushpull Aw

	writeI2CByte( AW2_ADDRESS, 4, 0x3F ) ;
	writeI2CByte( AW2_ADDRESS, 5, 0xF0 ) ;
	
	writeI2CByte( AW1_ADDRESS, 4, 0xFF ) ;
	writeI2CByte( AW1_ADDRESS, 5, 0xA0 ) ;

  pinMode(SPI_CS, OUTPUT);
  pinMode(SPI_DATA, OUTPUT);
  pinMode(SPI_RESET, OUTPUT);
  pinMode(SPI_CLOCK, OUTPUT);
  pinMode(SPI_DC, OUTPUT);
	 
//	lcd_reset() ;
	Aw2Outputs[0] |= LCD_RESET_PIN ;
	writeI2CByte( AW2_ADDRESS, 2, Aw2Outputs[0] ) ;

	Aw2Outputs[0] &= ~LCD_RESET_PIN ;
	writeI2CByte( AW2_ADDRESS, 2, Aw2Outputs[0] ) ;

	Aw2Outputs[0] |= LCD_RESET_PIN ;
	writeI2CByte( AW2_ADDRESS, 2, Aw2Outputs[0] ) ;

  SPI.begin(SPI_CLOCK,-1,SPI_DATA);

  u8x8.begin();

	Aw2Outputs[1] = 0xFF & ~LEDB_PIN ;
	writeI2CByte( AW2_ADDRESS, 3, Aw2Outputs[1] ) ;

	readI2CByte( AW2_ADDRESS, 1 ) ;		// Seems to be needed to allow 16 bit reads to then work
	readI2CByte( AW1_ADDRESS, 1 ) ;		// Seems to be needed to allow 16 bit reads to then work

	Aw1Outputs[1] = readI2CByte( AW1_ADDRESS, 3 ) ;
	Aw1Outputs[1] = 0x5F ;
	writeI2CByte( AW1_ADDRESS, 3, Aw1Outputs[1] ) ;
	Aw1HighRequired = Aw1Outputs[1] ;
	Aw2LowRequired = Aw2Outputs[0] ;

	writeI2CByte( RTC_ADDRESS, 0, 0 ) ;	// Clock
	writeI2CByte( RTC_ADDRESS, 1, 0 ) ;

	clearDisplay() ;
	lcd_putsAtt( 3*FW, 3*FH, "POWER UP 1", DBLSIZE ) ;
	refreshDisplay() ;

//	GPIO.enable_w1ts = ( (uint32_t)1 << 19 ) ;
//	GPIO.func_out_sel_cfg[19].val = 256 + 1024 ; 
//	GPIO.out_w1ts = ( (uint32_t)1 << 19 ) ;

//	FUN_DRV_S is 10
//  MCU_SEL_S is 12
//  pinFunction |= ((uint32_t)2 << FUN_DRV_S);//what are the drivers?
//  pinFunction |= ((uint32_t)2 << MCU_SEL_S);
//  pinFunction |= FUN_IE ;	//input enable but required for output as well?
	// esp32_gpioMux[19].reg is 0x74
//	ESP_REG(DR_REG_IO_MUX_BASE + esp32_gpioMux[19].reg) = pinFunction ;

//	backlightFixed() ;

	initBacklight() ;

	pinMode( 33, INPUT ) ;		// Power switch
	pinMode( 32, OUTPUT ) ;		// Soft power on

	// Haptic
	pinMode( 5, OUTPUT ) ;
//	digitalWrite( 5, 0 ) ;
	GPIO.out_w1tc = 0x20 ;

	// Configure pin 14 (SPort pin) as debug output
//	pinMode( 14, OUTPUT ) ;
//	pinFunction = GPIO.func_out_sel_cfg[14].val ;
//	pinFunction &= 0xFFFFFE00 ;
//	pinFunction |= 14 ;	// U0TXD
//	GPIO.func_out_sel_cfg[14].val = pinFunction ;

	g_menuStack[0] = menuProc0 ;

  for (int i = 0; i < 4 ; i += 1 )
	{
		g_eeGeneral.calibMid[i] = 0x0800 ; // 0x400 ;
		g_eeGeneral.calibSpanPos[i] = 0x0800 ; // 0x300 ;
		g_eeGeneral.calibSpanNeg[i] = 0x0600 ; // 0x300 ;
  }


	createSwitchMapping() ;

	SpiMutex = xSemaphoreCreateMutexStatic( &SpiMutexBuffer ) ;

	startSpiMemory() ;
	LITTLEFS.begin();
//	EepromStatus = eeprom_read_status() ;

//	lcd_putcAtt( 0, 0, 'A', 0 ) ;
//	refreshDisplay() ;
	
	//delay(20) ;


	AwBits = readAws() ;
	if ( ~AwBits & 0x40000 )	// Enter pressed
	{
		// Maintenance mode
		MaintenanceMode = 1 ;
		softPowerOn() ;
		powerIsOn = 0 ;
		PowerStart = readPowerSwitch() ;
		g_menuStack[0] = menuUpdate ;
//		EnterMenu = EVT_ENTRY ;
		intRfOff() ;	// Request power off
		checkAw1() ;	// Send request to external logic
		checkAw2() ;	// Send request to external logic
		Serial.begin( 115200 ) ;
//		Serial.end() ;
//		Serial.begin( 115200, SERIAL_8N1, 14, 3 ) ;
	}
	else
	{
  	MixerHandle = xTaskCreateStatic( mixerTask,    // Function that should be called
  	  "Mix",   // Name of the task (for debugging)
  	  MIXER_STACK_SIZE,		// Stack size
  	  NULL,            	// Parameter to pass
  	  5,               	// Task priority (high)
  	  MixerStack,         	// Static stack
			&MixerTaskBuffer			// Task data structure
  	) ;
		
		PulsesHandle = xTaskCreateStatic( pulsesTask,    // Function that should be called
  	  "PulseI",   // Name of the task (for debugging)
  	  PULSES_STACK_SIZE,		// Stack size
  	  NULL,            	// Parameter to pass
  	  6,               	// Task priority (very high)
  	  PulsesStack,         	// Static stack
			&PulsesTaskBuffer			// Task data structure
  	) ;


extern void voice_task(void* pdata) ;

		VoiceHandle = xTaskCreateStatic( voice_task,    // Function that should be called
  	  "Voice",   // Name of the task (for debugging)
  	  VOICE_STACK_SIZE,		// Stack size
  	  NULL,            	// Parameter to pass
  	  6,               	// Task priority (very high)
  	  VoiceStack,         	// Static stack
			&VoiceTaskBuffer			// Task data structure
  	) ;

		Serial.begin( 115200 ) ;

	}

  DebugHandle = xTaskCreateStatic( handle_serial,    // Function that should be called
    "Dbg",   // Name of the task (for debugging)
    DEBUG_STACK_SIZE,   // Stack size
    NULL,            		// Parameter to pass
    1,               		// Task priority (low)
		DebugStack,
		&DebugTaskBuffer		// Task data structure
  ) ;

#define RXDAccess 8
#define TXDAccess 7

	Serial2.begin( 450000, SERIAL_8N1, RXDAccess, TXDAccess, true ) ;		// Invert serial signals

// For SPort try Serial3.begin, then Serial3.end
// Inspect config, set Tx pin to correct pin
// Check Tx is enabled and write a byte to the x fifo



//	Serial1.begin( 115200, SERIAL_8N1, 14, 14 ) ;

//	lcd_putcAtt( 6, 0, 'B', 0 ) ;
//	refreshDisplay() ;

	 
	bool result = SPIFFS.begin() ;

	SpiffsBegun = result ;

//	lcd_putcAtt( 12, 0, 'C', 0 ) ;
//	refreshDisplay() ;

//  File f = SPIFFS.open("/BG-ISRM_1.0.1_20211013.frk", "r");
//	if ( f )
//	{
//		FileSize = f.size() ;
//	}
//	f.close() ;

	Now = getTmr2MHz() ;
	clearDisplay() ;
	lcd_putsAtt( 3*FW, 3*FH, "SETUP", DBLSIZE ) ;
	refreshDisplay() ;
}

void speakModelVoice()
{
	if ( g_model.modelVoice == -1 )
	{
		putNamedVoiceQueue( g_model.modelVname, VLOC_MNAMES ) ;
	}
//	else
//	{
//		putVoiceQueue( ( g_model.modelVoice + 260 ) | VLOC_NUMUSER  ) ;
//	}
}

void refreshDisplay()
{
	u8x8.drawTile( 0, 0, 16, &DisplayBuf[0] ) ;
	u8x8.drawTile( 0, 1, 16, &DisplayBuf[128] ) ;
	u8x8.drawTile( 0, 2, 16, &DisplayBuf[256] ) ;
	u8x8.drawTile( 0, 3, 16, &DisplayBuf[384] ) ;
	u8x8.drawTile( 0, 4, 16, &DisplayBuf[512] ) ;
	u8x8.drawTile( 0, 5, 16, &DisplayBuf[640] ) ;
	u8x8.drawTile( 0, 6, 16, &DisplayBuf[768] ) ;
	u8x8.drawTile( 0, 7, 16, &DisplayBuf[896] ) ;
}



#define STARTUP_TIME		15

static uint16_t tgtime ;		// 1 sec
static uint16_t dtimer ;

void voiceMinutes( int16_t value )
{
	voice_numeric( value, 0, (abs(value) == 1) ? SV_MINUTE : SV_MINUTES ) ;
}

void doVoiceAlarmSource( VoiceAlarmData *pvad )
{
	if ( pvad->source )
	{
		// SORT OTHER values here
		if ( pvad->source >= NUM_SKYXCHNRAW )
		{
			voice_telem_item( pvad->source - NUM_SKYXCHNRAW - 1 ) ;
		}
		else
		{
			int16_t value ;
			value = getValue( pvad->source - 1 ) ;
			voice_numeric( value, 0, 0 ) ;
		}
	}
}



static void processVoiceAlarms()
{
	uint32_t i ;
	uint32_t curent_state ;
//	uint8_t flushSwitch ;
	VoiceAlarmData *pvad = &g_model.vad[0] ;
	i = 0 ;
	if ( VoiceCheckFlag100mS & 4 )
	{
		i = NUM_VOICE_ALARMS ;
	}
//	flushSwitch = getSwitch00( g_model.voiceFlushSwitch ) ;
//	if ( ( VoiceCheckFlag100mS & 2 ) == 0 )
//	{
//		if ( flushSwitch && ( LastVoiceFlushSwitch == 0 ) )
//		{
//			flushVoiceQueue() ;			
//		}
//	}
//	LastVoiceFlushSwitch = flushSwitch ;
  for ( ; i < NUM_VOICE_ALARMS + NUM_GLOBAL_VOICE_ALARMS ; i += 1 )
	{
		struct t_NvsControl *pc = &NvsControl[i] ;
		uint32_t play = 0 ;
		uint32_t functionTrue = 0 ;
		curent_state = 0 ;
		int16_t ltimer = pc->nvs_timer ;
	 	if ( i == NUM_VOICE_ALARMS )
		{
			pvad = &g_eeGeneral.gvad[0] ;
		}
		if ( pvad->func )		// Configured
		{
  		int16_t x ;
			int16_t y = pvad->offset ;
			x = getValue( pvad->source - 1 ) ;
  		switch (pvad->func)
			{
				case 1 :
					x = x > y ;
				break ;
				case 2 :
					x = x < y ;
				break ;
				case 3 :
					x = abs(x) > y ;
				break ;
				case 4 :
					x = abs(x) < y ;
				break ;
				case 5 :
				{
					if ( isAgvar( pvad->source ) )
					{
						x *= 10 ;
						y *= 10 ;
					}
    			x = abs(x-y) < 32 ;
				}
				break ;
				case 6 :
					x = x == y ;
				break ;
				case 7 :
					x = (x & y) != 0 ;
				break ;
				case 8 :
				{	
  				int16_t z ;
					z = x - pc->nvs_last_value ;
					z = abs(z) ;
					if ( z > y )
					{
						pc->nvs_last_value = x ;
						x = 1 ;
					}
					else
					{
						x = 0 ;
					}
				}
				break ;
				case 9 :
					if ( y )
					{
						x = (x % y) == 0 ;
					}
					else
					{
						x = 0 ;
					}
				break ;
			}
			functionTrue = x ;
// Start of invalid telemetry detection
//					if ( pvad->source > ( CHOUT_BASE - NUM_SKYCHNOUT ) )
//					{ // Telemetry item
//						if ( !telemItemValid( pvad->source - 1 - CHOUT_BASE - NUM_SKYCHNOUT ) )
//						{
//							x = 0 ;	// Treat as OFF
//						}
//					}
// End of invalid telemetry detection
			if ( pvad->swtch )
			{
				if ( pvad->swtch == MAX_SKYDRSWITCH + 1 )
				{
					if ( getFlightPhase() == 0 )
					{
						x = 0 ;
					}
				}
				else if ( getSwitch00( pvad->swtch ) == 0 )
				{
					x = 0 ;
				}
			}
			if ( x == 0 )
			{
				ltimer = 0 ;
			}
			else
			{
				play = 1 ;
			}
		}
		else // No function
		{
			if ( pvad->swtch )
			{
				if ( pvad->swtch == MAX_SKYDRSWITCH + 1 )
				{
					curent_state = getFlightPhase() ? 1 : 0 ;
				}
				else
				{
					curent_state = getSwitch00( pvad->swtch ) ;
				}
				if ( curent_state == 0 )
				{
					ltimer = -1 ;
				}
			}
			else// No switch, no function
			{ // Check for source with numeric rate
				if ( pvad->rate >= 4 )	// A time
				{
					if ( pvad->vsource )
					{
						play = 1 ;
					}
				}
			}
		}
		play |= curent_state ;
		
		if ( ( VoiceCheckFlag100mS & 2 ) == 0 )
		{
		 if ( pvad->rate == 3 )	// All
		 {
		 		uint32_t pos ;
				pos = 1 ;
				if ( pvad->func && ( functionTrue == 0 ) )
				{
					pos = 0 ;
				}
		 		if ( pos )
				{
					if ( pvad->swtch == MAX_SKYDRSWITCH + 1 )
					{
						pos = getFlightPhase() ;
					}
					else
					{
						pos = switchPosition( pvad->swtch ) ;
					}
					uint32_t state = pc->nvs_state ;
					play = 0 ;
					if ( state != pos )
					{
						if ( state > 0x80 )
						{
							if ( --state == 0x80 )
							{
								state = pos ;
								ltimer = 0 ;
								play = pos + 1 ;
							}
						}
						else
						{
							state = 0x83 ;
						}
						pc->nvs_state = state ;
					}
				}
				else
				{
					pc->nvs_state = 0x40 ;
				}
		 }
		 else
		 {
			if ( play == 1 )
			{
				if ( pc->nvs_state == 0 )
				{ // just turned ON
					if ( ( pvad->rate == 0 ) || ( pvad->rate == 2 ) )
					{ // ON
						if ( pvad->delay )
						{
							pc->nvs_delay = pvad->delay + 1 ;
						}
						ltimer = 0 ;
					}
				}
				else
				{ // just turned OFF
					if ( pvad->rate == 1 )
					{
						if ( pvad->func == 8 )	// |d|>val
						{
							if ( pvad->delay )
							{
								pc->nvs_delay = pvad->delay + 1 ;
								play = 0 ;
							}
						}
					}
				}
				pc->nvs_state = 1 ;
				if ( ( pvad->rate == 1 ) )
				{
					play = 0 ;
				}
				if ( pc->nvs_delay )
				{
					if ( --pc->nvs_delay )
					{
						play = 0 ;
					}
				}
			}
			else
			{
				if ( ( pvad->func == 8 ) && ( pc->nvs_delay ) )	// |d|>val
				{
					play = 0 ;
					if ( --pc->nvs_delay == 0 )
					{
						play = 1 ;
					}
				}
				else
				{
					pc->nvs_delay = 0 ;
					if ( pc->nvs_state == 1 )
					{
						if ( ( pvad->rate == 1 ) || ( pvad->rate == 2 ) )
						{
							ltimer = 0 ;
							play = 1 ;
							if ( pvad->rate == 2 )
							{
								play = 2 ;
							}
						}
					}
				}
				pc->nvs_state = 0 ;
			}
			if ( pvad->rate == 33 )
			{
				play = 0 ;
				ltimer = -1 ;
			}
		 }
		}
		else //( ( VoiceCheckFlag100mS & 2 ) != 0 )
		{
		 	uint32_t pos ;
			if ( pvad->func == 8 )	// |d|>val
			{
				pc->nvs_last_value = getValue( pvad->source - 1 ) ;
			}
			if ( pvad->rate == 3 )
			{
				if ( pvad->swtch == MAX_SKYDRSWITCH + 1 )
				{
					pos = getFlightPhase() ;
				}
				else
				{
					pos = switchPosition( pvad->swtch ) ;
				}
			}
			else
			{
				pos = play ;
			}
			pc->nvs_state = pos ;
			play = 0 ;
			if ( pvad->rate == 33 )	// ONCE
			{
	 			if ( i >= NUM_VOICE_ALARMS )
				{	// Global alert
					if ( VoiceCheckFlag100mS & 4 )
					{
						play = 1 ;
					}
				}
				else
				{
					play = 1 ;
				}
			}
			ltimer = -1 ;
		}

		if ( pvad->mute )
		{
			if ( pvad->source > ( CHOUT_BASE + NUM_SKYCHNOUT ) )
			{ // Telemetry item
//				if ( !telemItemValid( pvad->source - 1 - CHOUT_BASE - NUM_SKYCHNOUT ) )
//				{
//					play = 0 ;	// Mute it
//				}
			}
		}

		if ( play )
		{
			if ( ltimer < 0 )
			{
				if ( pvad->rate >= 4 )	// A time or ONCE
				{
					ltimer = 0 ;
				}
			}
			if ( ltimer == 0 )
			{
				if ( pvad->vsource == 1 )
				{
					doVoiceAlarmSource( pvad ) ;
				}
				if ( pvad->fnameType == 0 )	// None
				{
					// Nothing!
				}
				else if ( ( pvad->fnameType == 1 ) || ( pvad->fnameType == 4 ) )	// Name
				{
					char name[10] ;
					char *p ;
					p = (char *)ncpystr( (uint8_t *)name, pvad->file.name, 8 ) ;
					if ( name[0] && ( name[0] != ' ' ) )
					{
						if ( play >= 2 )
						{
							while ( *(p-1) == ' ' )
							{
								p -= 1 ;
							}
							*(p-1) += ( play - 1 ) ;
						}
						if ( pvad->fnameType == 4 )
						{
							putNamedVoiceQueue( name, VLOC_SYSTEM ) ;
						}
						else
						{
							putUserVoice( name, 0 ) ;
						}
					}
				}
				else if ( pvad->fnameType == 2 )	// Number
				{
					uint16_t value = pvad->file.vfile ;
					if ( value > 507 )
					{
//						value = calc_scaler( value-508, 0, 0 ) ;
					}
					else if ( value > 500 )
					{
						value = g_model.gvars[value-501].gvar ;
					}
					putVoiceQueue( ( value + ( play - 1 ) ) | VLOC_NUMUSER ) ;
				}
				else
				{ // Audio
					int16_t index ;
					index = pvad->file.vfile ;
					if ( index == 16 )
					{
						index = AU_HAPTIC4 ;
					}

					audio.event( index, 0, 1 ) ;
				}
				if ( pvad->vsource == 2 )
				{
					doVoiceAlarmSource( pvad ) ;
				}
        if ( pvad->haptic )
				{
					audioDefevent( (pvad->haptic > 1) ? ( ( pvad->haptic == 3 ) ? AU_HAPTIC3 : AU_HAPTIC2 ) : AU_HAPTIC1 ) ;
				}
				if ( ( pvad->rate < 4 ) || ( pvad->rate > 32 ) )	// Not a time
				{
					ltimer = -1 ;
				}
				else
				{
					ltimer = 1 ;
				}
			}
			else if ( ltimer > 0 )
			{
				ltimer += 1 ;
				if ( ltimer > ( (pvad->rate-2) * 10 ) )
				{
					ltimer = 0 ;
				}
			}
		}
		pvad += 1 ;
		pc->nvs_timer = ltimer ;
	}
	
//	if ( MuteTimer )
//	{
//		MuteTimer -= 1 ;
//	}
}

void processVarioTones()
// Vario
{

	static uint8_t varioRepeatRate = 0 ;
	static uint8_t sounded = 0 ;

	if ( g_model.varioData.varioSource ) // Vario enabled
	{
		if ( getSwitch00( g_model.varioData.swtch ) )
		{
			uint8_t new_rate = 0 ;
			if ( varioRepeatRate )
			{
				varioRepeatRate -= 1 ;
			}
			if ( varioRepeatRate == 0 )
			{
				sounded = 0 ;
			}
			int16_t vspd ;
			if ( g_model.varioData.varioSource == 1 )
			{
				vspd = TelemetryData[FR_VSPD] ;

				if ( g_model.varioData.param > 1 )
				{
					vspd /= g_model.varioData.param ;
				}
			}
			else if ( g_model.varioData.varioSource == 2 )
			{
				vspd = TelemetryData[FR_A2_COPY] - 128 ;
				if ( ( vspd < 3 ) && ( vspd > -3 ) )
				{
					vspd = 0 ;							
				}
				vspd *= g_model.varioData.param ;
			}
			else
			{
				// A Scaler
				vspd = calc_scaler( g_model.varioData.varioSource-3, 0, 0 ) ;
				if ( g_model.varioData.param > 1 )
				{
					vspd /= g_model.varioData.param ;
				}
			}
			if ( vspd )
			{
				if ( vspd < 0 )
				{
					vspd = -vspd ;
					if (!g_model.varioData.sinkTones )
					{
						if ( vspd > 25 )		// OpenXsensor
						{
							if ( sounded != 2 )
							{
								sounded = 2 ;
								varioRepeatRate = 0 ;
  	  	     		audio.event( AU_VARIO_DOWN, vspd/25 ) ;
							}
						}
					}
				}
				else
				{
					if ( vspd > 25 )			// OpenXsensor
					{
						if ( sounded != 1 )
						{
							sounded = 1 ;
							varioRepeatRate = 0 ;
		  	      audio.event( AU_VARIO_UP, vspd/25 ) ;
						}
					}
				}
				if ( vspd < 75 )
				{
					new_rate = 8 ;
				}
				else if ( vspd < 100 )
				{
					new_rate = 7 ;
				}
				else if ( vspd < 125 )
				{
					new_rate = 6 ;
				}
				else if ( vspd < 150 )
				{
					new_rate = 5 ;
				}
				else if ( vspd < 175 )
				{
					new_rate = 4 ;
				}
				else if ( vspd < 200 )
				{
					new_rate = 3 ;
				}
				else
				{
					new_rate = 2 ;
				}
			}
			else
			{
				if (g_model.varioData.sinkTones )
				{
					if ( sounded == 0 )
					{
						new_rate = 20 ;
						sounded = 3 ;
						varioRepeatRate = 0 ;
  	    	  audio.event( AU_VARIO_UP, 0 ) ;
					}
				}
			}
			if ( varioRepeatRate == 0 )
			{
				varioRepeatRate = new_rate ;
			}
		}
	}
}	





uint16_t get_tmr10ms()
{
	return Tenms ;
}

uint32_t powerStartup()
{
	static uint16_t tgtime ;		// 1 sec
	static uint16_t dtimer ;
	uint16_t m ;

	m = micros() ;
	readAnalog() ;
	AdcReadTime = micros() - m ;


	UsbV = ( 498 * (uint32_t)AnalogData[5] ) / 4096 ;
	if ( UsbV > 380 )	// USB plugged in
	{
		if ( !( ( readPowerSwitch() ) == 0 ) )
		{
			clearDisplay() ;
			lcd_putsAtt( 2*FW, 3*FH, "HOLD POWER SWITCH", 0 ) ;
			refreshDisplay() ;
			tgtime = Tenms ;
			m = UsbV > 380 ? 0 : LEDG_PIN ;
			if ( (Aw2Outputs[1] & LEDG_PIN) != m )
			{
				Aw2Outputs[1] = (Aw2Outputs[1] & ~LEDG_PIN) | m ;
				writeI2CByte( AW2_ADDRESS, 3, Aw2Outputs[1] ) ;
			}
			return 0 ;
		}
		
//		PowerByUsb = 1 ;
//		return 2 ;
	}

//	if ( ( ResetReason & 0x10 ) == 0 ) // Not watchdog
	{
//	  tgtime = Tenms ;		// 1 sec

//		dtimer = tgtime ;
		if ( (uint16_t)(Tenms - tgtime ) < STARTUP_TIME * 10 )
		{
			uint32_t switchValue ;

			switchValue = ( ( readPowerSwitch() ) == 0 ) ;	// Power switch on
//			wdt_reset() ;
			uint16_t now = Tenms ;
			if ( now != dtimer )
			{
				dtimer = now ;
				if ( ( dtimer & 3) == 0 )
				{
					clearDisplay() ;
					lcd_putsAtt( 3*FW, 3*FH, "STARTING", DBLSIZE ) ;

					pushPlotType( PLOT_BLACK ) ;
					lcd_hbar( 14, 46, 100, 7, (dtimer - tgtime) * 1000 / (98*STARTUP_TIME) ) ;
					popPlotType() ;
					refreshDisplay() ;
				}
			}
			
			if ( !switchValue )
			{
				// Don't power on
				softPowerOff() ;  		// Only turn power off if necessary
				for(;;)
				{
				}
			}
		}
		else
		{
			return 1 ;
		}
	}
//	else
//	{
//		return 1 ;
//	}
	return 0 ;
}	


uint32_t testPowerOff( uint8_t event )
{
 	static uint16_t tgtime = 0 ;

	if ( ( readPowerSwitch() ) == 0 )	// Power switch on
	{
		if ( powerIsOn == 1 )
		{
			powerIsOn = 2 ;
			tgtime = Tenms ;
		}
		else
		{
			if ( powerIsOn == 2 )
			{
//				stopMenus = NO_MENU ;
				uint8_t dtimer = Tenms - tgtime ;
				dtimer = ( 80 - dtimer ) * 100 / 80 ;
				if ( (uint16_t)(Tenms - tgtime ) > 80 )
				{
					if ( checkRssi( event ) == RSSI_POWER_OFF )
					{
						powerIsOn = 3 ;
					}
				}
				else
				{
					clearDisplay() ;
					lcd_putsAtt( 3*FW, 3*FH, "STOPPING", DBLSIZE ) ;
					pushPlotType( PLOT_BLACK ) ;
					lcd_hbar( 14, 46, 100, 7, dtimer ) ;
					popPlotType() ;
					refreshDisplay() ;
				}
			}
		}
	}
	else
	{
		powerIsOn = 1 ;
	}
//	if ( PowerByUsb )
//	{
//		if ( UsbV < 100 )
//		{
//			powerIsOn = 3 ;
//		}
//	}

	if ( powerIsOn >= 3 )
	{
		PoweringOff = 1 ;
		if ( powerIsOn == 3 )
		{
			PowerOffTimeout = 100 ;
			ee32StoreGeneral() ;
		 	powerIsOn = 4 ;
		}
	}
	return ( powerIsOn > 1 ) ? 1 : 0 ;	
}

int16_t convertTelemConstant( int8_t channel, int8_t value)
{
  int16_t result ;

	channel = TelemIndex[channel] ;
	result = value + 125 ;
	if ( ( channel <= V_GVAR7 ) && ( channel >= V_GVAR1 ) )
	{
		return value ;
	}

  switch (channel)
	{
    case V_RTC :
      result *= 12 ;
    break;
    case MODELTIME :
      result *= 20 ;
    break ;
    case RUNTIME :
      result *= 3 ;
    break ;
    case TIMER1 :
    case TIMER2 :
      result *= 10 ;
    break;
    case FR_ALT_BARO:
    case TELEM_GPS_ALT:
			if ( result > 63 )
			{
      	result *= 2 ;
      	result -= 64 ;
			}
			if ( result > 192 )
			{
      	result *= 2 ;
      	result -= 192 ;
			}
			if ( result > 448 )
			{
      	result *= 2 ;
      	result -= 488 ;
			}
			result *= 10 ;		// Allow for decimal place
      if ( g_model.FrSkyImperial )
      {
        // m to ft *105/32
        value = m_to_ft( result ) ;
      }
    break;
    case FR_RPM:
      result *= 100;
    break;
    case FR_TEMP1:
    case FR_TEMP2:
      result -= 30;
    break;
    case FR_A1_MAH:
    case FR_A2_MAH:
		case FR_AMP_MAH :
		case FR_RBOX_B1_CAP :
		case FR_RBOX_B2_CAP :
      result *= 50;
    break;

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
		case FR_CELL_MIN:
		case FR_SBEC_VOLT :
      result *= 2;
		break ;
		case FR_CELLS_TOT :
		case FR_VOLTS :
		case FR_CELLS_TOTAL1 :
		case FR_CELLS_TOTAL2 :
      result *= 2;
		break ;
		case FR_RBOX_B1_V :
		case FR_RBOX_B2_V :
      result *= 4 ;
		break ;
		case FR_RBOX_B1_A :
		case FR_RBOX_B2_A :
		case FR_SBEC_CURRENT :
      result *= 20;
		break ;
    case FR_WATT:
      result *= 8 ;
    break;
		case FR_VSPD :
			result = value * 10 ;
		break ;
		case FMODE :
			result = value ;
		break ;
		
		default :
			result = 0 ;
		break ;
  }

  return result ;
}

int16_t get_telemetry_value( int8_t channel )
{
//#ifndef TELEMETRY_LOST
	if (telemItemValid( channel ) == 0 )
	{
		return 0 ;
	}
//#endif	
	
	
	channel = TelemIndex[channel] ;
//	if ( ( channel <= V_SC8 ) && ( channel >= V_SC1 ) )	// A Scaler
//	{
//		return calc_scaler(channel-V_SC1, 0, 0 ) ;
//	}
//	if ( ( channel <= V_GVAR7 ) && ( channel >= V_GVAR1 ) )	// A GVAR
//	{
//		return g_model.gvars[channel-V_GVAR1].gvar ;
//	}
  switch (channel)
	{
    case RUNTIME :
    return g_eeGeneral.totalElapsedTime / 60 ;

    case MODELTIME :
    return g_model.totalTime / 60 ;

    case TIMER1 :
    case TIMER2 :
    return s_timer[channel+2].s_timerVal ;
    
    case BATTERY :
    return g_vbat10mV/10 ;

    case FR_ALT_BARO :
		return TelemetryData[channel] + AltOffset ;

//    case TMOK :
//		return TmOK ;
    
		case V_RTC :
		return Time.hour * 60 + Time.minute ;
    
		case FR_WATT :
		return (uint32_t)TelemetryData[FR_VOLTS] * (uint32_t)TelemetryData[FR_CURRENT] / (uint32_t)100 ;
    
		case FMODE :
		return getFlightPhase() ;
    
		default :
		return TelemetryData[channel] ;
  }
}


int16_t getValue(uint8_t i)
{
  if(i<4) return CalibratedStick[i];//-512..512
	
//	if ( i >= EXTRA_POTS_START-1 )
//	{
//		if ( i >= EXTRA_POTS_START-1+8 )
//		{
//			return get_telemetry_value( i-CHOUT_BASE-NUM_SKYCHNOUT ) ;
//		}
//  	if( i >= EXTRA_PPM_START )
//		{
//			if ( i < EXTRA_PPM_START + NUM_EXTRA_PPM )
//			{
//				return g_ppmIns[ i + NUM_PPM - EXTRA_PPM_START ] ;
//			}
//			else
//			{
//				return ex_chans[i+NUM_SKYCHNOUT-EXTRA_CHANNELS_START] ;
//			}
//		}
//		return calibratedStick[i-EXTRA_POTS_START+8] ;
//	}
//  if(i<PPM_BASE) return 0 ;
//	else if(i<CHOUT_BASE)
//	{
//		int16_t x ;
//		x = g_ppmIns[i-PPM_BASE] ;
//		if(i<PPM_BASE+4)
//		{
//			x -= g_eeGeneral.trainerProfile[g_model.trainerProfile].channel[i-PPM_BASE].calib ;
//		}
//		return x*2;
//	}
//	else if(i<CHOUT_BASE+NUM_SKYCHNOUT) return ex_chans[i-CHOUT_BASE];
//  else if(i<CHOUT_BASE+NUM_SKYCHNOUT+NUM_TELEM_ITEMS)
//	{
//		return get_telemetry_value( i-CHOUT_BASE-NUM_SKYCHNOUT ) ;
//	}
  return 0 ;
}


int32_t isAgvar(uint8_t value)
{
	if ( value >= 70 )
	{
		if ( value <= 76 )
		{
			return 1 ;
		}
	}
	return 0 ;
}

void processSwitchTimer( uint32_t i )
{
  X20CSwData &cs = g_model.customSw[i];
//  uint8_t cstate = CS_STATE(cs.func);

//  if(cstate == CS_TIMER)
//	{
		int16_t y ;
		y = CsTimer[i] ;
		if ( y == 0 )
		{
			int8_t z ;
			z = cs.v1 ;
			if ( z >= 0 )
			{
				z = -z-1 ;
				y = z * 50 ;
			}
			else
			{
				y = z * 5 ;
			}
		}
		else if ( y < 0 )
		{
			if ( ++y == 0 )
			{
				int8_t z ;
				z = cs.v2 ;
				if ( z >= 0 )
				{
					z += 1 ;
					y = z * 50 - 1 ;
				}
				else
				{
					y = -(z*5)-1 ;
				}
			}
		}
		else  // if ( CsTimer[i] > 0 )
		{
			y -= 1 ;
		}

		int8_t x = cs.andsw ;
		if ( x )
		{
		  if (getSwitch00( x) == 0 )
			{
				Last_switch[i] = 0 ;
				if ( cs.func == CS_NTIME )
				{
					int8_t z ;
					z = cs.v1 ;
					if ( z >= 0 )
					{
						z = -z-1 ;
						y = z * 50 ;					
					}
					else
					{
						y = z * 5 ;
					}
				}
				else
				{
					y = -1 ;
				}
			}
			else
			{
				Last_switch[i] = 2 ;
			}
		}
		CsTimer[i] = y ;
//	}
}



void processSwitches()
{
	uint32_t cs_index ;
	for ( cs_index = 0 ; cs_index < NUM_SKYCSW ; cs_index += 1 )
	{
  	X20CSwData &cs = g_model.customSw[cs_index] ;
  	uint8_t ret_value = false ;

  	if( cs.func )
		{
  		int8_t a = cs.v1 ;
  		int8_t b = cs.v2 ;
  		int16_t x = 0 ;
  		int16_t y = 0 ;
  		uint8_t s = CS_STATE( cs.func ) ;

  		if ( (s == CS_VOFS) || (s == CS_2VAL) )
  		{
  		  x = getValue(cs.v1u-1);
    		if ( cs.v1u > CHOUT_BASE+NUM_SKYCHNOUT )
				{
  		    y = convertTelemConstant( cs.v1u-CHOUT_BASE-NUM_SKYCHNOUT-1, cs.v2 ) ;
				}
  		  else
				{
  		  	y = calc100toRESX(cs.v2);
				}
  		}
  		else if(s == CS_VCOMP)
  		{
 		    x = getValue(cs.v1u-1);
 		    y = getValue(cs.v2u-1);
  		}

  		switch (cs.func)
			{
	  		case (CS_VPOS):
  		    ret_value = (x>y);
  	    break;
  			case (CS_VNEG):
  		    ret_value = (x<y) ;
  	    break;
  			case (CS_APOS):
	  	    ret_value = (abs(x)>y) ;
  		  break;
	  		case (CS_ANEG):
  		    ret_value = (abs(x)<y) ;
  	    break;
				case CS_VEQUAL :
  		    ret_value = (x == y) ;
  	    break;
				case CS_EXEQUAL:
					if ( isAgvar( cs.v1 ) )
					{
						x *= 10 ;
						y *= 10 ;
					}
  		  	ret_value = abs(x-y) < 32 ;
  			break;
	
				case CS_VXEQUAL:
					if ( isAgvar( cs.v1 ) || isAgvar( cs.v2 ) )
					{
						x *= 10 ;
						y *= 10 ;
					}
  			  ret_value = abs(x-y) < 32 ;
  			break;
		
  			case (CS_AND):
  			case (CS_OR):
  			case (CS_XOR):
  			{
  			  bool res1 = getSwitch(a,0,0) ;
  			  bool res2 = getSwitch(b,0,0) ;
  			  if ( cs.func == CS_AND )
  			  {
  			    ret_value = res1 && res2 ;
  			  }
  			  else if ( cs.func == CS_OR )
  			  {
  			    ret_value = res1 || res2 ;
  			  }
  			  else  // CS_XOR
  			  {
  			    ret_value = res1 ^ res2 ;
  			  }
  			}
  			break;

	  		case (CS_EQUAL):
  		    ret_value = (x==y);
  	    break;
  			case (CS_NEQUAL):
  		    ret_value = (x!=y);
  	    break;
  			case (CS_GREATER):
  		    ret_value = (x>y);
  		   break;
	  		case (CS_LESS):
  		    ret_value = (x<y);
  	    break;
	  		case (CS_NTIME):
					processSwitchTimer( cs_index ) ;
					ret_value = CsTimer[cs_index] >= 0 ;
  			break ;
				case (CS_TIME):
				{	
					processSwitchTimer( cs_index ) ;
  			  ret_value = CsTimer[cs_index] >= 0 ;
					int8_t x = cs.andsw ;
					if ( x )
					{
					  if (getSwitch00( x ) )
						{
							if ( ( Last_switch[cs_index] & 2 ) == 0 )
							{ // Triggering
								ret_value = 1 ;
							}	
						}
					}
				}
  			break ;
  			case (CS_MONO):
  			case (CS_RMONO):
				{
					if ( VoiceCheckFlag100mS & 2 )
					{
						// Resetting, retrigger any monostables
						Last_switch[cs_index] &= ~2 ;
					}
					int8_t andSwOn = 1 ;
					if ( ( cs.func == CS_RMONO ) )
					{
						andSwOn = cs.andsw ;
						if ( andSwOn )
						{
							andSwOn = getSwitch00( andSwOn) ;
						}
						else
						{
							andSwOn = 1 ;
						}
					}
					
				  if (getSwitch00( cs.v1) )
					{
						if ( ( Last_switch[cs_index] & 2 ) == 0 )
						{
							// Trigger monostable
							uint8_t trigger = 1 ;
							if ( ( cs.func == CS_RMONO ) )
							{
								if ( ! andSwOn )
								{
									trigger = 0 ;
								}
							}
							if ( trigger )
							{
								Last_switch[cs_index] = 3 ;
								int16_t x ;
								x = cs.v2 * 5 ;
								if ( x < 0 )
								{
									x = -x ;
								}
								else
								{
									x += 5 ;
									x *= 10 ;
								}
								CsTimer[cs_index] = x ;							
							}
						}
					}
					else
					{
						Last_switch[cs_index] &= ~2 ;
					}
					int16_t y ;
					y = CsTimer[cs_index] ;
					if ( Now_switch[cs_index] < 2 )	// not delayed
					{
						if ( y )
						{
							if ( ( cs.func == CS_RMONO ) )
							{
								if ( ! andSwOn )
								{
									y = 1 ;
								}	
							}
							if ( --y == 0 )
							{
								Last_switch[cs_index] &= ~1 ;
							}
							CsTimer[cs_index] = y ;
						}
					}
 			  	ret_value = CsTimer[cs_index] > 0 ;
				}
  			break ;
  
				case (CS_LATCH) :
		  		if (getSwitch00( cs.v1) )
					{
						Last_switch[cs_index] = 1 ;
					}
					else
					{
					  if (getSwitch00( cs.v2) )
						{
							Last_switch[cs_index] = 0 ;
						}
					}
  			  ret_value = Last_switch[cs_index] & 1 ;
  			break ;
  			
				case (CS_FLIP) :
		  		if (getSwitch00( cs.v1) )
					{
						if ( ( Last_switch[cs_index] & 2 ) == 0 )
						{
							// Clock it!
					    if (getSwitch00( cs.v2) )
							{
								Last_switch[cs_index] = 3 ;
							}
							else
							{
								Last_switch[cs_index] = 2 ;
							}
						}
					}
					else
					{
						Last_switch[cs_index] &= ~2 ;
					}
  			  ret_value = Last_switch[cs_index] & 1 ;
  			break ;
  			case (CS_BIT_AND) :
				{	
  			  x = getValue(cs.v1u-1);
					y = cs.v2u ;
					y |= cs.bitAndV3 << 8 ;
  			  ret_value = ( x & y ) != 0 ;
				}
  			break ;
				case CS_RANGE :
				{
					int16_t z ;
    			if ( cs.v1u > CHOUT_BASE+NUM_SKYCHNOUT )
					{
  		  	  z = convertTelemConstant( cs.v1u-CHOUT_BASE-NUM_SKYCHNOUT-1, (int8_t)cs.bitAndV3 ) ;
					}
  		  	else
					{
  		  		z = calc100toRESX((int8_t)cs.bitAndV3) ;
					}
  		    ret_value = (x >= y) && (x <= z) ;
				}			 
  			break ;
  			default:
  		    ret_value = false;
 		    break;
  		}

			
			
			int8_t z = cs.andsw ;
			if ( z )
			{
				switch ( cs.exfunc )
				{
					case 0 :
  		    	ret_value &= getSwitch( z, 0, 0 ) ;
					break ;
					case 1 :
  		    	ret_value |= getSwitch( z, 0, 0 ) ;
					break ;
					case 2 :
  		    	ret_value ^= getSwitch( z, 0, 0 ) ;
					break ;
				}
			}
			
//			if ( ret_value )
//			{
//				int8_t x = getAndSwitch( cs ) ;
//				if ( x )
//				{
//  		    ret_value = getSwitch( x, 0, 0 ) ;
//				}
//			}
			
			if ( ( cs.func < CS_LATCH ) || ( cs.func > CS_RMONO ) )
			{
				Last_switch[cs_index] = ret_value ;
			}
			if ( Now_switch[cs_index] == 0 )	// was off
			{
				if ( ret_value )
				{
					if ( cs.switchDelay )
					{
						ret_value = cs.switchDelay * 10 ;
					}
				}
			}
			else
			{
				if ( Now_switch[cs_index] > 1 )	// delayed
				{
					if ( ret_value )
					{
						uint8_t temp = Now_switch[cs_index] - 2 ;
						if ( temp )
						{
							ret_value = temp ;
						}
					}
				}
			}
			Now_switch[cs_index] = ret_value ;
		}
		else // no function
		{
			if ( VoiceCheckFlag100mS & 2 )
			{
				Now_switch[cs_index] = 0 ;
			}
		}
	}
}


uint32_t Level = 1 ;
//uint16_t Inputs1 ;

uint16_t readInputs() ;
void ledOn() ;
void ledOff() ;

//uint8_t readAwReg( uint8_t reg ) ;
//uint8_t readAw1Reg( uint8_t reg ) ;
//void writeAwReg( uint8_t reg, uint8_t value ) ;
//uint16_t readAw1Data() ;
uint16_t Rtime ;

// RTC_CNTL_CLK_CONF_REG (0x3FF48070)

uint32_t AwBits ;
uint32_t TenMsFlag ;
uint32_t Cleared ;

//uint16_t AwTime ;

uint8_t throttleReversed()
{
	return g_model.throttleReversed ^	g_eeGeneral.throttleReversed ;
}

bool getSwitch00( int8_t swtch )
{
	return getSwitch( swtch, 0, 0 ) ;
}

bool getSwitch(int8_t swtch, bool nc, uint8_t level)
{
  bool ret_value ;
  uint8_t cs_index ;
  uint8_t aswitch ;
  
	aswitch = abs(swtch) ;
 	SwitchStack[level] = aswitch ;
	
//	cs_index = aswitch-(MAX_SKYDRSWITCH-NUM_SKYCSW);
	cs_index = aswitch-(CSW_INDEX) ;

	{
		int32_t index ;
		for ( index = level - 1 ; index >= 0 ; index -= 1 )
		{
			if ( SwitchStack[index] == aswitch )
			{ // Recursion on this switch taking place
    		ret_value = Last_switch[cs_index] & 1 ;
		    return swtch>0 ? ret_value : !ret_value ;
			}
		}
	}
  if ( level > SW_STACK_SIZE - 1 )
  {
    ret_value = Last_switch[cs_index] & 1 ;
    return swtch>0 ? ret_value : !ret_value ;
  }

	if ( swtch == 0 )
	{
    return nc ;
	}
	else if ( swtch == MAX_SKYDRSWITCH )
	{
    return true ;
	}
	else if ( swtch == -MAX_SKYDRSWITCH )
	{
    return false ;
	}

	if ( abs(swtch) > MAX_SKYDRSWITCH )
	{
		uint8_t value = hwKeyState( abs(swtch) ) ;
		if ( swtch > 0 )
		{
			return value ;
		}
		else
		{
			return ! value ;
		}
	}

  uint8_t dir = swtch>0 ;
  if(abs(swtch)<(MAX_SKYDRSWITCH-NUM_SKYCSW))
	{
    if(!dir) return ! keyState((enum EnumKeys)(SW_BASE-swtch-1));
    return            keyState((enum EnumKeys)(SW_BASE+swtch-1));
  }

	ret_value = Now_switch[cs_index] & 1 ;
	 
	return swtch>0 ? ret_value : !ret_value ;
}


uint32_t getFlightPhase()
{
	uint32_t i ;
  for ( i = 0 ; i < MAX_MODES ; i += 1 )
	{
    PhaseData *phase = &g_model.phaseData[i];
    if ( phase->swtch )
		{
    	if ( getSwitch00( phase->swtch ) )
			{
    		if ( phase->swtch2 )
				{
					if ( getSwitch00( phase->swtch2 ) )
					{
						return i + 1 ;
					}
				}
				else
				{
					return i + 1 ;
				}
    	}
		}
		else
		{
    	if ( phase->swtch2 && getSwitch00( phase->swtch2 ) )
			{
    	  return i + 1 ;
    	}
		}
  }
  return 0 ;
}



int16_t getRawTrimValue( uint8_t phase, uint8_t idx )
{
	if ( phase )
	{
		return g_model.phaseData[phase-1].trim[idx] ;
	}	
	else
	{
		return g_model.trim[idx] ;
	}
}

uint32_t getTrimFlightPhase( uint8_t phase, uint8_t idx )
{
  for ( uint32_t i=0 ; i<MAX_MODES ; i += 1 )
	{
    if (phase == 0) return 0;
    int16_t trim = getRawTrimValue( phase, idx ) ;
//    if ( ( trim <= TRIM_EXTENDED_MAX ) || ( trim > TRIM_EXTENDED_MAX + MAX_MODES + 1 ) )
    if ( trim <= TRIM_EXTENDED_MAX )
		{
			return phase ;
		}
    uint32_t result = trim-TRIM_EXTENDED_MAX-1 ;
    if (result >= phase)
		{
			result += 1 ;
		}
    phase = result;
  }
  return 0 ;
}



int16_t getTrimValue( uint8_t phase, uint8_t idx )
{
  return getRawTrimValue( getTrimFlightPhase( phase, idx ), idx ) ;
}

void setTrimValue(uint8_t phase, uint8_t idx, int16_t trim)
{
	if ( phase )
	{
		phase = getTrimFlightPhase( phase, idx ) ;
	}
	if ( phase )
	{
    if(trim < -125 || trim > 125)
//    if(trim < -500 || trim > 500)
		{
			trim = ( trim > 0 ) ? 125 : -125 ;
//			trim = ( trim > 0 ) ? 500 : -500 ; For later addition
		}
		
//		int16_t value ;
//		value = g_model.phaseData[phase-1].trim[idx] ;
//  	if ( value > TRIM_EXTENDED_MAX + MAX_MODES + 1 )
//		{
//			trim += 2000 ;
//		}
  	g_model.phaseData[phase-1].trim[idx] = trim ;
	}
	else
	{
    if(trim < -125 || trim > 125)
		{
			trim = ( trim > 0 ) ? 125 : -125 ;
		}	
		g_model.trim[idx] = trim ;
	}
	eeDirty(EE_MODEL|EE_TRIM) ;
}

const uint8_t trimMap[] ={ 6,7,4,5,0,1,2,3 } ;

static uint8_t checkTrim(uint8_t event)
{
  int8_t  k = (event & EVT_KEY_MASK) - TRM_BASE ;
  int8_t  s = g_model.trimInc ;

  if ( ScriptActive )
	{
  	return event ;
	}

	if ( ~AwBits & 0x40000 )	// Enter/Shift pressed
	{
		k += 4 ;
	}

	if ( s == 4 )
	{
		s = 8 ;			  // 1=>1  2=>2  3=>4  4=>8
	}
	else
	{
		if ( s == 3 )
		{
			s = 4 ;			  // 1=>1  2=>2  3=>4  4=>8
		}
	}

	// 0,1 -> 6,7
	// 2,3 ->4,5

	if ( k < 8 )
	{
		k = trimMap[k] ;
	}

  if( (k>=0) && (k<8) )
	{
		if ( !IS_KEY_BREAK(event)) // && (event & _MSK_KEY_REPT))
  	{
//			TrimBits |= (1 << k ) ;
			
  	  //LH_DWN LH_UP LV_DWN LV_UP RV_DWN RV_UP RH_DWN RH_UP
  	  uint8_t idx = k/2;
		
	// SORT idx for stickmode if FIX_MODE on
//			idx = stickScramble[g_eeGeneral.stickMode*4+idx] ;

	// Now sort for the stick mode
	// Mode 1 RETA  Swap 1 and 2 from no swap
	// Mode 2 RTEA  No swap from Swap 1 and 2 
	// Mode 3 AETR  Swap 1 and 2, and 0 and 3 from Swap 0 and 3
	// Mode 4 ATER  Swap 0 and 3 from Swap 1 and 2, and 0 and 3

	// So modes 1,2 swap 1 and 2
	//    modes 3,4 swap 1 and 2

//  	  uint8_t t = g_eeGeneral.stickMode ^ 1 ;
//			idx = stickScramble[t*4+idx] ;

			// This line does the required operation
			idx = stickScramble[1*4+idx] ;
			
// Crosstrim not useful due to the hardware trim switches
//			uint8_t ct = g_eeGeneral.crosstrim + ( g_eeGeneral.xcrosstrim << 1 ) ;
//			if ( ct )
//			{
//				idx = 3 - idx ;			
//			}
//      if ( ct == 2 ) // Vintage style crosstrim
//      {
//        if (idx == 0 || idx == 3)       // swap back LH/RH trims
//          idx = 3 - idx;
//      }
			
//			if ( TrimInUse[idx] )
			{
				uint32_t phaseNo = getTrimFlightPhase( CurrentPhase, idx ) ;
  	  	int16_t tm = getTrimValue( phaseNo, idx ) ;
  	  	int8_t  v = (s==0) ? (abs(tm)/4)+1 : s;
  	  	bool thrChan = (2 == idx) ;
				bool thro = (thrChan && (g_model.thrTrim));
  	  	if(thro) v = 2; // if throttle trim and trim trottle then step=2

//				if ( GvarSource[idx] )
//				{
//					v = 1 ;
//				}

  	  	if(thrChan && throttleReversed()) v = -v;  // throttle reversed = trim reversed
  	  	int16_t x = (k&1) ? tm + v : tm - v;   // positive = k&1

  	  	if(((x==0)  ||  ((x>=0) != (tm>=0))) && (!thro) && (tm!=0))
				{
					setTrimValue( phaseNo, idx, 0 ) ;
  	  	  killEvents(event);
  	  	  audioDefevent(AU_TRIM_MIDDLE) ;
  	  	}
				else if(x>-125 && x<125)
				{
					setTrimValue( phaseNo, idx, x ) ;
					if(x <= 125 && x >= -125)
					{
						audio.event(AU_TRIM_MOVE,(abs(x)/4)+60) ;
					}	
  	  	}
  	  	else
  	  	{
					setTrimValue( phaseNo, idx, (x>0) ? 125 : -125 ) ;
					if(x <= 125 && x >= -125)
					{
						audio.event(AU_TRIM_MOVE,(-abs(x)/4)+60) ;
					}	
  	  	}
			}
  	  return 0 ;
  	}
//		else
//		{
//			TrimBits &= ~(1 << k ) ;
//		}
	}
  return event ;
}

void accessRecieveByte( uint16_t data, uint32_t module ) ;

t_time *ClockWriteData ;
uint8_t ClockWriteRequest ;


void rtc_settime( t_time *ptr )
{
	ClockWriteData = ptr ;
	ClockWriteRequest = 1 ;
}

void readClock() ;
void writeClock( t_time *ptr ) ;

void doTenms()
{
	uint16_t m ;
	int16_t byte ;
	uint32_t localAwBits ;
	uint32_t localAwBits2 ;

//	AwBits = ( readAw1Reg(0) << 16 ) | t ;
//	m = micros() ;
	localAwBits = readAws() ;

extern void txmit( uint8_t c ) ;
extern void crlf() ;
extern void p8hex( uint32_t value ) ;

	if ( (localAwBits & 0x7FFFFFFF) != AwBits )
	{
		localAwBits2 = readAws() ;
		if ( localAwBits != localAwBits2 )
		{
			localAwBits = 0 ;
		}
	}

	if ( localAwBits )	// If read OK
	{
		AwBits = localAwBits & 0x7FFFFFFF ;	// Update
	}
//	AwTime = micros() - m ;
	buttons10ms() ;
	checkAw1() ;
	checkAw2() ;	// Send request to external logic
	ee32_process() ;

	if ( --CheckTimer == 0 )
	{
		CheckTimer = 2 ;
		processSwitches() ;	// Every 20mS
	}
	
	if (--VoiceTimer == 0 )
	{
		VoiceTimer = 10 ;		// Restart timer
		VoiceCheckFlag100mS |= 1 ;	// Flag time to check alarms
	
		if ( ClockWriteRequest )
		{
			ClockWriteRequest = 0 ;
			writeClock( ClockWriteData ) ;
		}
		else
		{
	  	readClock() ;
		}
	}

	if ( ++OneSecPreCount >= 100 )
	{
		OneSecPreCount = 0 ;
//		OneSecond += 1 ;
		OneSecFlag = 1 ;
//		ReadDateFlag = 1 ;
	}

	if ( g_model.Module[INTERNAL_MODULE].protocol == PROTO_OFF )
	{
		intRfOff() ;
		InternalModuleActive = 0 ;
	}
	else
	{
		if ( isIntRfOn() == 0 )
		{
			intRfOn() ;
			startAccess( INTERNAL_MODULE ) ;
			InternalModuleActive = 1 ;
		}
//		setupPulsesAccess( 0 ) ;
	}

	if ( ( g_model.Module[EXTERNAL_MODULE].protocol == PROTO_OFF ) || (JustLoadedModel) )
	{
		extRfOff() ;
		ExternalModuleActive = 0 ;
	}
	else
	{
		if ( isExtRfOn() == 0 )
		{
			extRfOn() ;
			
			startAccess( EXTERNAL_MODULE ) ;
			
			ExternalModuleActive = 1 ;
		}
//		setupPulsesAccess( 0 ) ;
	}

	while ( Serial2.available() )
	{
		accessRecieveByte( Serial2.read(), 1 ) ;
	}
	
	while ( (byte = pollSportFifo() ) != -1 )
	{
		telemetryRecieveByte( byte, 1 ) ;
	}

	if ( VoiceCheckFlag100mS & 1 )
	{
		static uint32_t delayTimer = 0 ;
		processVoiceAlarms() ;
		processVarioTones() ;
		VoiceCheckFlag100mS = 0 ;
   	for ( uint32_t i = 0 ; i < TELEMETRYDATALENGTH ; i += 1 )
		{
			if (TelemetryDataValid[i] )
			{
				TelemetryDataValid[i] -= 1 ;
			}
		}
		// RSSI checks
		if ( delayTimer )
		{
			delayTimer -= 1 ;
		}
		uint8_t redAlert = 0 ;
		static uint8_t redCounter ;
		static uint8_t orangeCounter ;
		uint8_t rssiValue = TelemetryData[FR_RXRSI_COPY] ;

		if ( FrskyStreaming )
		{
			int8_t offset ;
			if ( g_model.enRssiRed == 0 )
			{
				offset = 42 ;	// rssiOffsetValue( 1 ) ;

				if ( rssiValue && rssiValue < g_model.rssiRed + offset )
				{
					// Alarm
					redAlert = 1 ;
					orangeCounter += 1 ;
					if ( ++redCounter > 3 )
					{
						if ( delayTimer == 0 )
						{
							putSystemVoice( SV_RSSICRIT, V_RSSI_CRITICAL ) ;
							delayTimer = 40 ;	// 4 seconds
						}
						redCounter = 0 ;
					}
				}
				else
				{
					redCounter = 0 ;
				}
			}
			if ( ( redAlert == 0 ) && ( g_model.enRssiOrange == 0 ) )
			{
				offset = 45 ;	// rssiOffsetValue( 0 ) ;
				if ( rssiValue && rssiValue < g_model.rssiOrange + offset )
				{
					// Alarm
					if ( ++orangeCounter > 3 )
					{
						if ( delayTimer == 0 )
						{
							putSystemVoice( SV_RSSI_LOW, V_RSSI_WARN ) ;
							delayTimer = 40 ;	// 4 seconds
						}
						orangeCounter = 0 ;
					}
				}
				else
				{
					orangeCounter = 0 ;
				}
			}
		}
	}
}

extern void checkLeavingFilelist() ;

void leavingMenu()
{
	checkLeavingFilelist() ;
//	if ( SharedMemory.TextControl.TextFileOpen )
//	{
//		f_close( &SharedMemory.TextControl.TextFile ) ;
//		SharedMemory.TextControl.TextFileOpen = 0 ;
//	}
}

MenuFuncP lastPopMenu()
{
  return  g_menuStack[g_menuStackPtr+1];
}

void popMenu(bool uppermost)
{
	leavingMenu() ;
  if(g_menuStackPtr>0 || uppermost)
	{
    g_menuStackPtr = uppermost ? 0 : g_menuStackPtr-1;
 		EnterMenu = EVT_ENTRY_UP ;
  }
	if ( uppermost )
	{
		killEvents(EVT_KEY_LONG(KEY_EXIT)) ;
	}
}

void chainMenu(MenuFuncP newMenu)
{
	leavingMenu() ;
  g_menuStack[g_menuStackPtr] = newMenu;
	EnterMenu = EVT_ENTRY ;
}

void pushMenu(MenuFuncP newMenu)
{
	leavingMenu() ;
	EnterMenu = EVT_ENTRY ;
  g_menuStack[++g_menuStackPtr] = newMenu ;
}

uint8_t *cpystr( uint8_t *dest, uint8_t *source )
{
  while ( (*dest++ = *source++) )
    ;
  return dest - 1 ;
}

#ifdef WIFI
extern uint8_t WifiActive ;
#endif

void maintenance()
{
	uint8_t event ;
	
	pollMrx() ;
#ifdef WIFI
	if ( WifiActive )
	{
		runWifi() ;
	}
#endif
	 
	if ( TenMsFlag )
	{
		TenMsFlag = 0 ;
		AwBits = readAws() ;
		buttons10ms() ;
		PreScale += 1 ;
	}
	
	if ( PreScale >= 2 )
	{
		clearDisplay() ;
		
		PreScale = 0 ;
		event = getEvent() ;
		if ( EnterMenu )
		{
			event = EnterMenu ;
			EnterMenu = 0 ;
		}
		g_menuStack[g_menuStackPtr](event) ;

		refreshDisplay() ;
	}
	
	if ( ( readPowerSwitch() ) == 0 )	// Power switch on
	{
		if ( powerIsOn == 1 )
		{
			powerIsOn = 2 ;
			tgtime = Tenms ;
		}
	}
	else
	{
		powerIsOn = 1 ;
	}
	if ( powerIsOn >= 2 )
	{
		clearDisplay() ;
		lcd_putsAtt( 5*FW, 3*FH, "Powering off", 0 ) ;
		refreshDisplay() ;
		softPowerOff() ;
		for(;;)
		{
			// wait for everything to stop
		}
	}	
}

void alertMessages( const char * s, const char * t )
{
  lcd_putsAtt(64-5*FW,0*FH,PSTR(STR_ALERT),DBLSIZE);
  lcd_puts_P(0,4*FH,s);
  lcd_puts_P(0,5*FH,t);
	lcd_puts_P(0, 6*FH, PSTR(STR_PRESS_KEY_SKIP) ) ;
}

extern uint16_t S_anaFilt[] ;
void getADC_osmp() ;

uint8_t checkThrottlePosition()
{
//  uint8_t thrchn=(2-(g_eeGeneral.stickMode&1));//stickMode=0123 -> thr=2121
//	int16_t v = CalibratedStick[2] ;
	getADC_osmp() ;
  uint8_t thrchn=(2-(g_eeGeneral.stickMode&1));//stickMode=0123 -> thr=2121
	int16_t v = scaleAnalog( S_anaFilt[thrchn], thrchn ) ;
	if ( g_model.throttleIdle )
	{
		if ( abs( v ) < THRCHK_DEADBAND )
		{
			return 1 ;
		}
	}
	else
	{
  	if(v <= -RESX + THRCHK_DEADBAND )
  	{
  		return 1 ;
  	}
	}
	return 0 ;
}

int32_t readControl( uint8_t channel )
{
	int32_t value ;
	value = CalibratedStick[channel] ;
	value *= 100 ;
	value /= 1024 ;
	return value ;
}


void endModelChecks()
{
	JustLoadedModel = 0 ;
	popMenu( 0 ) ;
	VoiceCheckFlag100mS |= 6 ;// Set switch current states (global)
	processVoiceAlarms() ;
	VoiceCheckFlag100mS = 0 ;
	speakModelVoice() ;
	VoiceCheckFlag100mS |= 2 ;// Set switch current states
	processSwitches() ;	// Guarantee unused switches are cleared
}


void checkCustom( uint8_t event )
{
	CustomCheckData *pdata ;
	pdata = &g_model.customCheck ;
	int32_t value ;
	uint32_t timer ;
	uint8_t idx = pdata->source - 1 ;

	if ( pdata->source == 0 )
	{
		endModelChecks() ;
		return ;
	}
//	if ( idx < 4 )
//	{
//		idx = stickScramble[g_eeGeneral.stickMode*4+idx] ;
//	}
	
//#ifndef SIMU
//  getADC_osmp() ;
//#endif
	
	value = readControl( idx ) ;
	if ( ( value >= pdata->min ) && ( value <= pdata->max ) )
	{
		endModelChecks() ;
		return ;
	}

//  clearKeyEvents();

  timer = 0 ;
	
//    getADC_single();
//		check_backlight() ;

	value = readControl( idx ) ;
	if ( ( value >= pdata->min ) && ( value <= pdata->max ) )
	{
		if ( ++timer > 19 )
		{
			endModelChecks() ;
			return ;
		}
	}
	else
	{
		timer = 0 ;
	}
	if ( event == EVT_ENTRY )
	{
		putSystemVoice( SV_CUSTOM_WARN, V_CUSTOM_WARN ) ;
	}
	alertMessages( XPSTR("Custom Check"), XPSTR("Set Control") ) ;
	putsChnRaw( 9*FW, 2*FH, pdata->source, 0 ) ;
	lcd_outdezAtt( 5*FW, 3*FH, pdata->min, 0) ;
	lcd_outdezAtt( 11*FW, 3*FH, value, 0) ;
	lcd_outdezAtt( 17*FW, 3*FH, pdata->max, 0) ;

	if ( event == EVT_KEY_BREAK(KEY_EXIT) )
	{
		endModelChecks() ;
		return ;
	}	 
}

void checkSwitches( uint8_t event )
{
	uint32_t i ;
	uint32_t k ;
	uint16_t states = g_model.modelswitchWarningStates ;
	
	if ( states	& 1 )
	{
		chainMenu(checkCustom) ;
		return ;
	}
	
	states >>= 1 ;
	states &= ~g_model.modelswitchWarningDisables ;
	getMovedSwitch() ;
	k = SwitchesStates & ~g_model.modelswitchWarningDisables ;

	if ( (states & 0xFF) != (k & 0xFF ) )
	{
		uint16_t j ;

	  lcd_img( 1, 0, HandImage, 0, 0 ) ;
		lcd_putsAtt( 32, 0*FH, XPSTR("SWITCH"), DBLSIZE) ;
		lcd_putsAtt( 32, 2*FH, PSTR(STR_WARNING), DBLSIZE) ;

		if ( event == EVT_ENTRY )
		{
			putSystemVoice( SV_SW_WARN, V_SW_WARN ) ;
		}

		j = 3 ;
		for ( i = 0 ; i < 4 ; i += 1 )
		{
			if ( ( g_model.modelswitchWarningDisables & j ) == 0 )
			{
				uint8_t attr = 0 ;
				if ( (states & j) != (k & j ) )
				{
 		  		attr = INVERS ;
					
				}
				// Display switch
	  		lcd_putcAtt( 3*FW+i*(2*FW+2), 5*FH, 'A'+i, attr ) ;
				lcd_putcAtt( 4*FW+i*(2*FW+2), 5*FH, PSTR(HW_SWITCHARROW_STR)[(states & j) >> (i*2)], attr ) ;
			}
			j <<= 2 ;
		}

		if ( event == EVT_KEY_BREAK(KEY_EXIT) )
		{
			chainMenu(checkCustom) ;
			return ;
		}	 
	}
	else
	{
		chainMenu(checkCustom) ;
		return ;
	}
}


void checkThrottle( uint8_t event )
{
	static uint16_t timer ;
  if(g_eeGeneral.disableThrottleWarning)
	{
//		popMenu( 0 ) ;
		chainMenu(checkSwitches) ;
		return ;
	}
  if(g_model.disableThrottleCheck)
	{
//		popMenu( 0 ) ;
		chainMenu(checkSwitches) ;
		return ;
	}

//  getADC_single() ;   // if thr is down - do not display warning at all

	if ( checkThrottlePosition() )
	{
		chainMenu(checkSwitches) ;
		return ;
	}

  lcd_img( 1, 0, HandImage, 0, 0 ) ;
  lcd_putsAtt(36, 0*FH, XPSTR("THROTTLE"),DBLSIZE|CONDENSED) ;
  lcd_putsAtt(36, 2*FH, PSTR(STR_WARNING),DBLSIZE|CONDENSED) ;
	lcd_puts_P(0, 5*FH, PSTR(STR_THR_NOT_IDLE) ) ;
	lcd_puts_P(0, 6*FH, PSTR(STR_RST_THROTTLE) ) ;
	lcd_puts_P(0, 7*FH, PSTR(STR_PRESS_KEY_SKIP) ) ;

	if ( event == EVT_ENTRY )
	{
		timer = 0 ;
	}

	if ( ++timer == 10 )
	{
		putSystemVoice( SV_TH_WARN, V_THR_WARN ) ;
	}
	 
	if ( event == EVT_KEY_BREAK(KEY_EXIT) )
	{
//		popMenu( 0 ) ;
		chainMenu(checkSwitches) ;
		return ;
	}	 
}




void loop(void)
{
	uint32_t time ;
	uint8_t event ;

	time = getTmr2MHz() ;
	if ( time - Now >= 20000 )
	{
		Now += 20000 ;
		if ( time - Now >= 50000 )
		{
			Now = time ;	// Startup problem
		}
		TenMsFlag = 1 ;
		Tenms += 1 ;
//		MixTick10ms = 1 ;
	}

//	time = millis() ;
//	if ( time - Now >= 10 )
//	{
//		Now += 10 ;
//		TenMsFlag = 1 ;
//		Tenms += 1 ;
//	}

	if ( MaintenanceMode )
	{
		maintenance() ;
		return ;
	}



	if ( NotFirstTime )
	{
		if ( TenMsFlag )
		{
			TenMsFlag = 0 ;
  		g_blinkTmr10ms += 1 ;
			PreScale += 1 ;
			doTenms() ;
			sound_5ms() ;

			AUDIO_HEARTBEAT() ;

			{
				uint16_t a = 0 ;
				uint16_t b = 0 ;
//				int32_t x ;
    
				uint16_t lightoffctr ;
				lightoffctr = g_LightOffCounter ;
				if(lightoffctr) lightoffctr -= 1 ;
//				x = (EncoderCount >> 1) - LastRotaryValue ;
				if( event )
				{
					a = g_eeGeneral.lightAutoOff*500 ; // on keypress turn the light on 5*100
					InacCounter = 0 ;
				}
				if(InactivityMonitor) b = g_eeGeneral.lightOnStickMove*500 ;
				if(a>lightoffctr) lightoffctr = a ;
				if(b>lightoffctr) lightoffctr = b ;
				g_LightOffCounter = lightoffctr ;
  			InactivityMonitor = 0 ; //reset this flag
			}
			check_backlight() ;

//			readAnalog() ;

//			for( uint32_t i = 0 ; i < 4 ; i += 1 ) // calc Sticks
//			{
//				int16_t v = AnalogData[i] ; // anaIn( i ) ;
//				v = scaleAnalog( v, i ) ;

//  		 	phyStick[i] = v >> 4 ;
//				index = stickScramble[stickIndex+i] ;
//	 		  CalibratedStick[i] = v ; //for show in expo
//			}

//void runMixer() ;
//			runMixer() ;

  		int16_t val ;
  		val = CalibratedStick[3-1];

	// ***** Handle sticky throttle here
//			if ( ThrottleStickyOn )
//			{
//				val = -RESX ;
//			}
  	  timer(val);


			if ( ( g_menuStackPtr == 0 ) && (PopupData.PopupActive == 0) )
			{
				event = peekEvent() ;
  			int8_t  k = (event & EVT_KEY_MASK) - TRM_BASE ;
  			if ( ( k >= 0 ) && ( k < 4 ) )
				{
					event = getEvent() ;
					checkTrim( event ) ;
				}
			}

			if ( ( event == 0 ) || ( event == EVT_KEY_REPT(KEY_MENU) ) )
			{
				uint8_t timer = LongMenuTimer ;
				if ( menuPressed() )
				{
					if ( timer < 255 )
					{
						timer += 1 ;
					}
				}
				else
				{
					timer = 0 ;
				}
				if ( timer == 200 )
				{
					event = EVT_TOGGLE_GVAR ;
					timer = 255 ;
				}
				LongMenuTimer = timer ;
			}

			if ( PowerStart == 0 )
			{
				if ( readPowerSwitch() )
				{
					PowerStart = 1 ;
				}
			}
		}


		if ( PreScale < 5 )
		{
			if ( Cleared == 0 )
			{
				clearDisplay() ;
				Cleared = 1 ;
			}
		}
		
		if ( ( PreScale >= 5 ) && ( Cleared ) )
		{
			uint32_t stopping ;
			PreScale = 0 ;
			event = peekEvent() ;
			stopping = ( PowerStart == 0 ) ? 0 : testPowerOff( event ) ;
			
			if ( stopping )
			{
				event = getEvent() ;
			}

			if ( PoweringOff )
			{
//				PowerSwitchTime = DWT->CYCCNT ;
				clearDisplay() ;
				Cleared = 1 ;
				lcd_putsAtt( 5*FW, 3*FH, "Powering off", 0 ) ;
				refreshDisplay() ;
//				// Save EEPROM data
//extern void eeSaveAll() ;
//				eeSaveAll() ;

//				while ( DWT->CYCCNT - PowerSwitchTime < 480000 * 1500 )	// 1.5S
//				{
//					wdt_reset() ;
//				}
				if ( ee32_check_finished() || --PowerOffTimeout == 0 )
				{
					softPowerOff() ;
					for(;;)
					{
						// wait for everything to stop
					}
				}
				return ;
			}

			if ( stopping == 0 )
			{
				uint16_t m ;
//				uint32_t q = AwBits >> 8 ;
//				q &= 0x00FF ;
//				q >>= 4 ;
//				q &= 3 ;
//				if ( g_menuStackPtr )
//				{
//					q = 0 ;
//				}


//				lcd_outhex8( 0, 7*FH, AwBits ) ;
//				lcd_outhex2( 100, 7*FH, readAwByte( 0x59, 5 ) ) ;
				
//				m = ( 930 * ((uint32_t)AnalogData[4] + 180) ) / 4096 ;
				m = (AnalogData[4] * 903 ) >> 11 ;
 				Battery = m + m*g_eeGeneral.vBatCalib/512 ;
				BattAverage += Battery ;
				if (++BattCount >= 16)
				{
					g_vbat10mV = BattAverage / 16 ;
					BattAverage = 0 ;
					BattCount = 0 ;

        	static uint8_t s_batCheck ;
        	s_batCheck += 16 ;
        	if( s_batCheck < 16 )
					{
						if( (g_vbat10mV/10<g_eeGeneral.vBatWarn) && (g_vbat10mV>490) )
						{
							voiceSystemNameNumberAudio( SV_TXBATLOW, V_BATTERY_LOW, AU_TX_BATTERY_LOW ) ;
//        	    if (g_eeGeneral.flashBeep) g_LightOffCounter = FLASH_DURATION;
						}
					}
				}
				UsbV = ( 498 * (uint32_t)AnalogData[5] ) / 4096 ;
				m = UsbV > 380 ? 0 : LEDG_PIN ;
				if ( (Aw2Outputs[1] & LEDG_PIN) != m )
				{
					Aw2Outputs[1] = (Aw2Outputs[1] & ~LEDG_PIN) | m ;
					writeI2CByte( AW2_ADDRESS, 3, Aw2Outputs[1] ) ;
				}
				
//				lcd_outhex8( 100-24, 7*FH, readAws() ) ;

				if ( JustLoadedModel == 2 )
				{
					pushMenu( checkThrottle ) ;
					JustLoadedModel = 1 ;
				}
				event = getEvent() ;
				if ( EnterMenu )
				{
					event = EnterMenu ;
					EnterMenu = 0 ;
				}
#if defined(LUA) || defined(BASIC)
				uint32_t refreshNeeded = 0 ;
				{
extern uint32_t TotalExecTime ;
					uint16_t execTime = getTmr2MHz() ;
					refreshNeeded = basicTask( event, SCRIPT_LCD_OK	| SCRIPT_STANDALONE ) ;
					execTime = (uint16_t)(getTmr2MHz() - execTime) ;
					TotalExecTime += execTime ;
					if ( refreshNeeded == 3 )
					{
						refreshNeeded = 0 ;
//						if ( sd_card_ready() )
						{
							event = 0 ;
						}
						// standalone finished so:
//					basicLoadModelScripts() ;
					}
//					else
//					{
//	  				if ( !refreshNeeded )
//						{
//    					refreshNeeded = basicTask( PopupData.PopupActive ? 0 : evt, SCRIPT_TELEMETRY ) ;
//							if ( refreshNeeded )
//							{
//								telemetryScriptRunning = 1 ;
//							}
//	  				}
//					} 	
				}
				if ( refreshNeeded )
				{
					ScriptActive = 1 ;
					if ( PopupData.PopupActive )
					{
						actionMainPopup( event ) ;
					}
//					else
//					{
//						if ( telemetryScriptRunning )
//						{
//							if ( ( evt==EVT_KEY_LONG(KEY_MENU)) || ( evt==EVT_KEY_LONG(BTN_RE) ) )
//							{
//								PopupData.PopupActive = 3 ;
//								PopupData.PopupIdx = 0 ;
//    		  			killEvents(evt) ;
//								evt = 0 ;
//								Tevent = 0 ;
//							}
//						}
//					}
				}
		  	else
#endif
				{
//					if ( ! PoweringOff )
//					{

					if ( BTjoystickActive )
					{
  					if (bleGamepad.isConnected() )
						{
							handleJoystick() ;						
						}
					}
						
						ScriptActive = 0 ;
						g_menuStack[g_menuStackPtr](event) ;
#if defined(LUA) || defined(BASIC)
						refreshNeeded = 4 ;
#endif
//					}
//					else
//					{
//						refreshNeeded = 2 ;
//					}
				}
#if defined(LUA) || defined(BASIC)
				if ( ( refreshNeeded == 2 ) || ( ( refreshNeeded == 4 ) ) )
#endif
				{
					m = micros() ;
					refreshDisplay() ;
					m = micros() - m ;
					Rtime = m ;
					Cleared = 0 ;
				}
	  	}
		}
//		else
//		{
//			if ( Cleared == 0 )
//			{
//			}
//		}
	}
	else
	{
		
		if ( powerStartup() )
		{
			NotFirstTime = 1 ;
			Now = millis() ;
			TenMsFlag = 0 ;

//	clearDisplay() ;
//	lcd_putsAtt( 3*FW, 3*FH, "POWERUP3", DBLSIZE ) ;
//	refreshDisplay() ;
			// Set power on
			softPowerOn() ;
			PowerStart = readPowerSwitch() ;
			init_eeprom() ;

			eeReadAll() ;
			setBacklightBrightness( g_eeGeneral.bright ) ;		
			g_model.Module[INTERNAL_MODULE].protocol = PROTO_OFF ;
			JustLoadedModel = 2 ;
//			VoiceCheckFlag100mS |= 6 ;// Set switch current states (global)
			Activated = 1 ;

			if ( g_eeGeneral.welcomeType == 0 )
			{
				if(!g_eeGeneral.disableSplashScreen)
  		  {
					voiceSystemNameNumberAudio( SV_WELCOME, V_HELLO, AU_TADA ) ;
  		  }
			}
			else if ( g_eeGeneral.welcomeType == 2 )
			{
				putNamedVoiceQueue( (char *)g_eeGeneral.welcomeFileName, VLOC_USER ) ;
			}

			g_vbat10mV = ( 930 * (uint32_t)AnalogData[4] ) / 4096 ;


//			if ( MaintenanceMode == 0 )
//			{
				if ( g_model.Module[1].protocol == PROTO_PXX )
				{
			  	pinMode( 4, INPUT ) ;
					attachInterrupt( 4, heartbeatHandler, CHANGE ) ;
				}
//			}


//			intRfOn() ;

		}
	}
}




uint16_t s_timeCumTot;		// Total tx on time (secs)
static uint16_t s_time;
static uint8_t s_cnt;

void resetTimern( uint32_t timer )
{
  struct t_timer *tptr = &s_timer[timer] ;
	tptr->s_timerState = TMR_OFF; //is changed to RUNNING dep from mode
  tptr->s_timeCumThr=0;
  tptr->s_timeCumSw=0;
  tptr->s_timeCum16ThrP=0;
	tptr->s_sum = 0 ;
	tptr->last_tmr = g_model.timer[timer].tmrVal ;
	tptr->s_timerVal = ( g_model.timer[timer].tmrDir ) ? 0 : tptr->last_tmr ;
  tptr->s_timeCumAbs = 0 ;
}

// Timer triggers:
// OFF - disabled
// ABS AND no switch - always running
// THs AND no switch - throttle stick
// TH% AND no switch - throttle %
// cx% AND no switch - channel %
// ABS AND switch/switchm - controlled by switch
// THs AND switch/switchm - both throttle stick AND switch
// TH% AND switch/switchm - both throttle % AND switch
// cx% AND switch/switchm - both channel % AND switch

// 1. Test modeA, if OFF, timer OFF
// 2. Test modeB, if no switch, or switch is ON, timer may run, else OFF
// 3. Test modeA, if ABS timer is ON
// 4. Test modeA, if THs, timer ON if throttle not 0
// 4. Test modeA, if Th% or cx%, timer on as %

void timer(int16_t throttle_val)
{
	
	int16_t val ;
	uint8_t timer ;
	int8_t tma ;
  int16_t tmb ;
  uint16_t tv ;
  
  s_cnt++;			// Number of times val added in
	for( timer = 0 ; timer < 2 ; timer += 1 )
	{
		struct t_timer *ptimer = &s_timer[timer] ;
		uint8_t resetting = 0 ;
		tmb = g_model.timer[timer].timerRstSw ;
		if ( tmb )
		{
    	if(tmb>=(256))	 // toggeled switch
			{
    	  uint8_t swPos = getSwitch00( tmb-(256) ) ;
				if ( swPos != ptimer->lastResetSwPos )
				{
					ptimer->lastResetSwPos = swPos ;
					if ( swPos )	// Now on
					{
						resetting = 1 ;
					}
				}
			}
			else
			{
				if ( getSwitch00( tmb) )
				{
					resetting = 1 ;
				}
			}
		}
		if ( resetting )
		{
			if ( timer == 0 )
			{
				resetTimern(0) ;
//				resetTimer1() ;
			}
			else
			{
				resetTimern(1) ;
//				resetTimer2() ;
			}
		}
		
		tma = g_model.timer[timer].tmrModeA ;
    tmb = g_model.timer[timer].tmrModeB ;
//		if ( tmb < -HSW_MAX )
//		{
//			tmb += 256 ;
//		}

// code for cx%
		val = throttle_val ;
   	if(tma>=TMR_VAROFS) // Cxx%
		{
 	    val = g_chans512[tma-TMR_VAROFS] ;
		}		

		val = ( val + RESX ) / (RESX/16) ;

		if ( tma != TMRMODE_NONE )		// Timer is not off
		{ // We have a triggerA so timer is running 
    	if(tmb>=256)	 // toggeled switch
			{
    	  if(!(ptimer->sw_toggled | ptimer->s_sum | s_cnt | s_time | ptimer->lastSwPos)) ptimer->lastSwPos = 0 ;  // if initializing then init the lastSwPos
    	  uint8_t swPos = getSwitch00( tmb-(256) ) ;
//				char xxxx[2] ;
//				xxxx[1] = 0 ;
//				xxxx[0] = swPos + '0' ;
//				Serial.print(xxxx) ;
    	  if(swPos && !ptimer->lastSwPos)  ptimer->sw_toggled = !ptimer->sw_toggled;  //if switch is flipped first time -> change counter state
    	  ptimer->lastSwPos = swPos;
    	}
    	else
			{
				if ( tmb )
				{
    	  	ptimer->sw_toggled = getSwitch00( tmb ); //normal switch
				}
				else
				{
					ptimer->sw_toggled = 1 ;	// No trigger B so use as active
				}
			}
		}

		if ( ( ptimer->sw_toggled == 0 ) || resetting )
		{
			val = 0 ;
		}

    ptimer->s_sum += val ;   // Add val in
    if( ( (uint16_t)( get_tmr10ms()-s_time) ) < 100 )		// BEWARE of 32 bit processor extending 16 bit values
		{
			if ( timer == 0 )
			{
				continue ; //1 sec
			}
			else
			{
				return ;
			}
		}
    val     = ptimer->s_sum/s_cnt;   // Average of val over last 100mS
    ptimer->s_sum  -= val*s_cnt;     //rest (remainder not added in)

	  ptimer->s_timeCumAbs += 1;
		if ( timer == 0 )
		{
    	s_timeCumTot += 1;
			g_eeGeneral.totalElapsedTime += 1 ;
			g_model.totalTime += 1 ;
		}
		else
		{
	    s_cnt   = 0;    // ready for next 100mS
			s_time += 100;  // 100*10mS passed
		}
    if(val) ptimer->s_timeCumThr       += 1;
		if ( !resetting )
		{
    	if(ptimer->sw_toggled) ptimer->s_timeCumSw += 1;
		}
    ptimer->s_timeCum16ThrP            += val>>1;	// val/2

    tv = ptimer->s_timerVal = g_model.timer[timer].tmrVal ;
    if(tma == TMRMODE_NONE)
		{
			ptimer->s_timerState = TMR_OFF;
		}
    else
		{
			if ( tma==TMRMODE_ABS )
			{
				if ( tmb == 0 ) ptimer->s_timerVal -= ptimer->s_timeCumAbs ;
	    	else ptimer->s_timerVal -= ptimer->s_timeCumSw ; //switch
			}
	    else if(tma<TMR_VAROFS-1) ptimer->s_timerVal -= ptimer->s_timeCumThr;	// stick
		  else ptimer->s_timerVal -= ptimer->s_timeCum16ThrP/16 ; // stick% or Cx%
		}   
		 
    switch(ptimer->s_timerState)
    {
    case TMR_OFF:
        if(tma != TMRMODE_NONE) ptimer->s_timerState=TMR_RUNNING;
        break;
    case TMR_RUNNING:
        if(ptimer->s_timerVal<0 && tv) ptimer->s_timerState=TMR_BEEPING;
        break;
    case TMR_BEEPING:
        if(ptimer->s_timerVal <= -MAX_ALERT_TIME)   ptimer->s_timerState=TMR_STOPPED;
        if(tv == 0)       ptimer->s_timerState=TMR_RUNNING;
        break;
    case TMR_STOPPED:
        break;
    }

//		if ( timer == 0 )
//		{

  	  if(ptimer->last_tmr == 0)
			{
				if (ptimer->s_timerVal == -1)
				{
					if ( timer == 0 )
					{
						if ( g_model.timer[timer].autoReset )
						{
							resetTimern(0) ;
						}
					}
					else
					{
						if ( g_model.timer[timer].autoReset )
						{
							resetTimern(1) ;
						}
					}
				}
			}

  	  if(ptimer->last_tmr != ptimer->s_timerVal)  //beep only if seconds advance
    	{
    		ptimer->last_tmr = ptimer->s_timerVal;
        if(ptimer->s_timerState==TMR_RUNNING)
        {
					uint8_t audioControl ;
					uint8_t hapticControl = 0 ;
					if ( timer == 0 )
					{
						audioControl = g_eeGeneral.preBeep | g_model.timer[timer].timerCdown ;
						if ( audioControl )
						{
							if ( g_model.timer[timer].timerHaptic & 2 )
							{
								hapticControl = 1 ;
							}
						}
					}
					else
					{
						audioControl = g_model.timer[timer].timerCdown ;
						if ( audioControl )
						{
							if ( g_model.timer[timer].timerHaptic & 2 )
							{
								hapticControl = 1 ;
							}
						}
					}
            if(audioControl && g_model.timer[timer].tmrVal) // beep when 30, 15, 10, 5,4,3,2,1 seconds remaining
            {
              	if(ptimer->s_timerVal==30) {audioNamedVoiceDefevent( AU_TIMER_30, SV_30SECOND );if ( hapticControl ){audioDefevent(AU_HAPTIC1);}}
              	if(ptimer->s_timerVal==20) {audioNamedVoiceDefevent( AU_TIMER_20, SV_20SECOND );if ( hapticControl ){audioDefevent(AU_HAPTIC1);}}
                if(ptimer->s_timerVal==10) {audioNamedVoiceDefevent( AU_TIMER_10, SV_10SECOND );if ( hapticControl ){audioDefevent(AU_HAPTIC1);}}
                if(ptimer->s_timerVal<= 5)
								{
									if(ptimer->s_timerVal>= 0)
									{
										audioVoiceDefevent(AU_TIMER_LT3, ptimer->s_timerVal | VLOC_NUMSYS ) ;
										if ( hapticControl ){audioDefevent(AU_HAPTIC1);}
									}
								}
								if(g_eeGeneral.flashBeep && (ptimer->s_timerVal==30 || ptimer->s_timerVal==20 || ptimer->s_timerVal==10 || ptimer->s_timerVal<=3))
                    g_LightOffCounter = FLASH_DURATION ;
            }
						div_t mins ;
						mins = div( g_model.timer[timer].tmrDir ? g_model.timer[timer].tmrVal- ptimer->s_timerVal : ptimer->s_timerVal, 60 ) ;
					hapticControl = 0 ;
					if ( timer == 0 )
					{
						audioControl = g_eeGeneral.minuteBeep | g_model.timer[timer].timerMbeep ;
						if ( audioControl )
						{
							if ( g_model.timer[timer].timerHaptic & 1 )
							{
								hapticControl = 1 ;
							}
						}
					}
					else
					{
						audioControl = g_model.timer[timer].timerMbeep ;
						if ( audioControl )
						{
							if ( g_model.timer[timer].timerHaptic & 1 )
							{
								hapticControl = 1 ;
							}
						}
					}
            if( audioControl && ((mins.rem)==0)) //short beep every minute
            {
							if ( mins.quot )
							{
								voiceMinutes( mins.quot ) ;
								if ( hapticControl )
								{
									audioDefevent(AU_HAPTIC1);
								}
							}
              if(g_eeGeneral.flashBeep) g_LightOffCounter = FLASH_DURATION;
            }
        }
        else if(ptimer->s_timerState==TMR_BEEPING)
        {
					if ( ( timer == 0 ) && g_eeGeneral.preBeep )
					{
            audioDefevent(AU_TIMER_LT3);
            if(g_eeGeneral.flashBeep) g_LightOffCounter = FLASH_DURATION;
					}
        }
    	}
//		}
    if( g_model.timer[timer].tmrDir) ptimer->s_timerVal = tv-ptimer->s_timerVal; //if counting backwards - display backwards
	}
}

int32_t getMovedSwitch()
{
	uint8_t skipping = 0 ;
  int8_t result = 0 ;

	static uint16_t s_last_time = 0 ;
	uint16_t time = get_tmr10ms() ;
  if ( (uint16_t)(time - s_last_time) > 10)
	{
		skipping = 1 ;
	}
  s_last_time = time ;

  for (uint32_t i = 0 ; i < 4 ; i += 1 )
	{
    uint16_t mask = (0x03 << (i*2)) ;
    uint8_t prev = (SwitchesStates & mask) >> (i*2) ;
		uint8_t next = switchPosition( i ) ;

    if (prev != next)
		{
      SwitchesStates = (SwitchesStates & (~mask)) | (next << (i*2));
      if (i<5)
        result = 1+(3*i)+next;
      else if (i==5)
			{
        result = -(1+(3*5)) ;
				if (next!=0) result = -result ;
			}
      else if (i==6)
        result = 1+(3*5)+1+next;
      else
			{
        result = -(1+(3*5)+1+3) ;
				if (next!=0) result = -result ;
			}
    }
  }

  if ( skipping )
	{
    result = 0 ;
	}
  return result ;
}

//#include "stamp-erskyTx.h"

#define SVN_VERS "erskyTx-r1"
#define MOD_VERS "Neo"

const char Stamps[] = "VERS: " SVN_VERS "\037"\
"DATE: " __DATE__ "\037"\
"TIME: " __TIME__ "\037"\
" MOD: " MOD_VERS ;

void menuVersion(uint8_t event)
{
	TITLE(PSTR(STR_Version)) ;
	static MState2 mstate2 ;
	mstate2.check_columns(event, 1-1) ;
  
	lcd_puts_Pleft( 2*FH,Stamps ) ;
		
}

extern void backlightOff() ;
extern void backlightOn() ;

#define BACKLIGHT_ON backlightOn()
#define BACKLIGHT_OFF backlightOff()

void check_backlight()
{
	int8_t sw = g_model.mlightSw ;
	if ( !sw )
	{
		sw = g_eeGeneral.lightSw ;
	}
  if(getSwitch00(sw) || g_LightOffCounter)
	{
		BACKLIGHT_ON ;
	}
  else
	{
    BACKLIGHT_OFF ;
	}
}


void startJoystick()
{
	if ( !BTjoystickStarted)
	{
		BTjoystickStarted = 1 ;
  	bleGamepad.begin() ;
//  	bleGamepad.setAutoReport(false) ; // to disable auto reporting, and then use bleGamepad.sendReport(); as needed
	}
}

void handleJoystick()
{
	// add in switches
	
  if( getSwitch00(HSW_SA0) )
	{
		bleGamepad.press(BUTTON_1) ;
	}
	else
	{
		bleGamepad.release(BUTTON_1) ;
	}
  if( getSwitch00(HSW_SA2) )
	{
		bleGamepad.press(BUTTON_2) ;
	}
	else
	{
		bleGamepad.release(BUTTON_2) ;
	}
 	if( getSwitch00(HSW_SB0) )
	{
		bleGamepad.press(BUTTON_3) ;
	}
	else
	{
		bleGamepad.release(BUTTON_3) ;
	}
  if( getSwitch00(HSW_SC0) )
	{
		bleGamepad.press(BUTTON_4) ;
	}
	else
	{
		bleGamepad.release(BUTTON_4) ;
	}
  if( getSwitch00(HSW_SC2) )
	{
		bleGamepad.press(BUTTON_5) ;
	}
	else
	{
		bleGamepad.release(BUTTON_5) ;
	}
 	if( getSwitch00(HSW_SD0) )
	{
		bleGamepad.press(BUTTON_6) ;
	}
	else
	{
		bleGamepad.release(BUTTON_6) ;
	}

	bleGamepad.setLeftThumb( g_chans512[0] * 31, (-g_chans512[1]) * 31 ) ;
	bleGamepad.setRightThumb( g_chans512[2] * 31, g_chans512[3] * 31 ) ;
	bleGamepad.setTriggers( g_chans512[4] * 31, g_chans512[5] * 31 ) ;
	bleGamepad.setSliders( g_chans512[6] * 31, g_chans512[7] * 31 ) ;

//  bleGamepad.setBatteryLevel(g_vbat10mV/10) ;
	bleGamepad.sendReport() ;
}

