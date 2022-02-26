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

#ifndef erskytx_h
#define erskytx_h

#include <stdint.h>

#include "lfs.h"

typedef void (*MenuFuncP)(uint8_t event) ;

#define RESX    (1<<10) // 1024
#define RESXu   1024u
#define RESXul  1024ul
#define RESXl   1024l
#define RESKul  100ul

#define DR_HIGH   0
#define DR_MID    1
#define DR_LOW    2
#define DR_EXPO   0
#define DR_WEIGHT 1
#define DR_RIGHT  0
#define DR_LEFT   1
#define DR_BOTH   2
#define DR_DRSW1  99
#define DR_DRSW2  98

#define NO_TRAINER 0x01
#define NO_INPUT   0x02
#define FADE_FIRST	0x20
#define FADE_LAST		0x40
#define NO_DELAY_SLOW	0x80

extern uint8_t SystemOptions ;

#define NUM_SKYCSW  24 //number of custom switches

#define CSW_INDEX	10	// Index of first custom switch

#define MOVE_VOICE		1

#define HSW_SC0				4
#define HSW_SC1				5
#define HSW_SC2				6

#define HSW_SB0				45	// Skip some values because of safety switch values
#define HSW_SB1				46
#define HSW_SB2				47
#define HSW_SA0				51
#define HSW_SA1				52
#define HSW_SA2				53
#define HSW_SD0				54
#define HSW_SD1				55
#define HSW_SD2				56

#define HSW_Ttrmup			44
#define HSW_Ttrmdn			43
#define HSW_Rtrmup			42
#define HSW_Rtrmdn			41
#define HSW_Atrmup			40
#define HSW_Atrmdn			39
#define HSW_Etrmup			38
#define HSW_Etrmdn			37

#define HSW_MAX				57


#define HSW_FM0					100
#define HSW_FM1					101
#define HSW_FM2					102
#define HSW_FM3					103
#define HSW_FM4					104
#define HSW_FM5					105
#define HSW_FM6					106

#define NUM_MIX_PHY_SWITCHES	4

#define MIX_GVAR (PPM_BASE+NUM_PPM+NUM_SKYCHNOUT+NUM_MIX_PHY_SWITCHES+NUM_SKYCSW)


#define MAX_SKYDRSWITCH 34

//(1+4+1+NUM_SKYCSW+8)

#define HSW_OFFSET ( HSW_SB0 - ( 9 + NUM_SKYCSW + 1 ) )

#define DIM(arr) (sizeof((arr))/sizeof((arr)[0]))

extern uint8_t MaxSwitchIndex ;		// For ON and OFF


#define INACTIVITY_THRESHOLD 200
#define THRCHK_DEADBAND 31


#define IS_THROTTLE( x ) ( x== 2 )
uint8_t IS_EXPO_THROTTLE( uint8_t x ) ;


#define EE_GENERAL 1
#define EE_MODEL   2
#define EE_TRIM    4           // Store model because of trim
#define INCDEC_SWITCH   0x08
#define NO_MENU_ONLY_EDIT   0x80

// Bits in SystemOptions
#define SYS_OPT_HARDWARE_EDIT	1
#define SYS_OPT_MUTE					2

#define TRIM_EXTENDED_MAX	500

#define NUM_TELEM_ITEMS 82
#define NUM_SKYXCHNRAW (CHOUT_BASE+NUM_SKYCHNOUT) // NUMCH + P1P2P3+ AIL/RUD/ELE/THR + MAX/FULL + CYC1/CYC2/CYC3

#define MIX_MAX   10
#define MIX_FULL  11 
#define MIX_CYC1  12
#define MIX_CYC2  13
#define MIX_CYC3  14

#define NUM_PPM     16
#define NUM_SKYCHNOUT  32
#define PPM_BASE    MIX_CYC3
#define CHOUT_BASE  (PPM_BASE+NUM_PPM)
#define PHY_BASE	  (CHOUT_BASE+NUM_SKYCHNOUT)

#define CURVE_BASE 7


#define CS_OFF       0
#define CS_VPOS      1  //v>offset
#define CS_VNEG      2  //v<offset
#define CS_APOS      3  //|v|>offset
#define CS_ANEG      4  //|v|<offset
#define CS_AND       5
#define CS_OR        6
#define CS_XOR       7
#define CS_EQUAL     8
#define CS_NEQUAL    9
#define CS_GREATER   10
#define CS_LESS      11
#define CS_LATCH		 12
#define CS_FLIP			 13
#define CS_TIME	     14
#define CS_NTIME     15
#define CS_MONO		   16	// Monostable
#define CS_RMONO	   17	// Monostable with reset
#define CS_EXEQUAL   18	// V~=offset
#define CS_BIT_AND   19
#define CS_VXEQUAL   20	// V1~=V2
#define CS_VEQUAL    21  //v == offset
#define CS_RANGE		 22  //a<=v<=b
#define CS_MAXF      22  //max function

#define CS_VOFS       0
#define CS_VBOOL      1
#define CS_VCOMP      2
#define CS_TIMER			3
#define CS_TMONO      4
#define CS_U16	      5
#define CS_2VAL	      6

#define SWASH_TYPE_120   1
#define SWASH_TYPE_120X  2
#define SWASH_TYPE_140   3
#define SWASH_TYPE_90    4
#define SWASH_TYPE_NUM   4

#define MIXSRC_RUD   1
#define MIXSRC_ELE   2
#define MIXSRC_THR   3
#define MIXSRC_AIL   4


#define MIXSRC_MAX   10
#define MIXSRC_FULL  11
#define MIXSRC_CYC1  12
#define MIXSRC_CYC2  13
#define MIXSRC_CYC3  14

#define INTERNAL_MODULE 0
#define EXTERNAL_MODULE 1

#define TMR_VAROFS  4

#define TMRMODE_NONE     0
#define TMRMODE_ABS      1
#define TMRMODE_THR      2
#define TMRMODE_THR_REL  3
#define MAX_ALERT_TIME   20

#define PROTO_PPM        0
#define PROTO_PXX        1
//#define PROTO_DSM2       2
//#define PROTO_MULTI      3
#define PROTO_XFIRE	     4
#define PROTO_ACCESS     5
//#define PROT_MAX         3
#define PROTO_OFF		     15

#define PXX_BIND			     0x01
#define PXX_RANGE_CHECK		 0x20

//#define PROT_STR_LEN      6
//#ifdef ASSAN
//#define DSM2_STR "\011LP4/LP5  DSM2only DSM2/DSMX9XR-DSM  "
//#else
//#define DSM2_STR "\011LP4/LP5  DSM2only DSM2/DSMX9XR-DSM  "
//#endif
//#define DSM2_STR_LEN   9
//#define LPXDSM2          0
//#define DSM2only         1
//#define DSM2_DSMX        2
//#define DSM_9XR		       3

// Failsafe values
#define FAILSAFE_NOT_SET		0
#define FAILSAFE_RX					1
#define FAILSAFE_CUSTOM			2
#define FAILSAFE_HOLD				3
#define FAILSAFE_NO_PULSES	4

extern uint16_t FailsafeCounter[2] ;

#define FLASH_DURATION 50

#define TMR_OFF     0
#define TMR_RUNNING 1
#define TMR_BEEPING 2
#define TMR_STOPPED 3

extern uint8_t ScriptFlags ;
// Bitfields in ScriptFlags
#define	SCRIPT_LCD_OK					1
#define	SCRIPT_STANDALONE			2
#define	SCRIPT_TELEMETRY			4
#define	SCRIPT_RESUME					8
#define	SCRIPT_BACKGROUND			16
#define	SCRIPT_FRSKY					32
#define	SCRIPT_ROTARY					64

struct t_timer
{
	uint16_t s_sum ;
	uint8_t lastSwPos ;
	uint8_t sw_toggled ;
	uint16_t s_timeCumSw ;  //laufzeit in 1/16 sec
	uint8_t  s_timerState ;
	uint8_t lastResetSwPos;
	uint16_t s_timeCumThr ;  //gewichtete laufzeit in 1/16 sec
	uint16_t s_timeCum16ThrP ; //gewichtete laufzeit in 1/16 sec
	int16_t  s_timerVal ;
	int16_t last_tmr ;
	uint16_t s_timeCumAbs;  //laufzeit in 1/16 sec
} ;

typedef struct
{ 
	unsigned char second;   //enter the current time, date, month, and year
	unsigned char minute;
	unsigned char hour;                                     
	unsigned char date;       
	unsigned char month;
	unsigned int year;      
 } t_time ;

extern t_time Time ;

extern struct t_timer s_timer[] ;

struct t_text
{
	lfs_file_t TextFile ;
	uint8_t TextMenuBuffer[16*21] ;
	uint8_t TextMenuStore[16*21+32] ;	// Allow for CRLF
	uint8_t TextLines ;
	uint8_t TextOffset ;
	uint8_t TextHelp ;
	uint8_t HelpTextPage ;
	uint8_t TextFileOpen ;
} ;


struct t_SportTx
{
	uint8_t *ptr ;
	uint8_t index ;
	uint8_t data[16] ;
} ;

struct t_accessSportTx
{
	uint8_t *ptr ;
	uint8_t index ;
	uint16_t module_destination ;
	uint8_t data[16] ;
} ;

struct t_XfireTx
{
	uint16_t count ;
	uint8_t command ;
	uint8_t data[64] ;
} ;

struct t_telemetryTx
{
	volatile uint16_t sportCount ;
	volatile uint8_t sportBusy ;
	union
	{
		struct t_SportTx SportTx ;
		struct t_XfireTx XfireTx ;
		struct t_accessSportTx AccessSportTx ;
	} ;
} ;


inline int32_t calc100toRESX(register int8_t x)
{
  return ((int32_t)x*1311)>>7 ;
}

inline int16_t calc1000toRESX( register int32_t x)  // improve calc time by Pat MacKenzie
{
    register int32_t y = x>>5;
    x+=y;
    y=y>>2;
    x-=y;
    return x+(y>>2);
    //  return x + x/32 - x/128 + x/512;
}

#define	ALERT_TYPE	0
#define MESS_TYPE		1

extern const char *AlertMessage ;
extern uint8_t AlertType ;

extern const char stickScramble[] ;
uint8_t modeFixValue( uint8_t value ) ;
extern int16_t CalibratedStick[] ;

extern const uint8_t bchout_ar[] ;

extern uint32_t getFlightPhase( void ) ; 
extern int16_t getRawTrimValue( uint8_t phase, uint8_t idx ) ;
extern int16_t getTrimValue( uint8_t phase, uint8_t idx ) ;
extern void setTrimValue(uint8_t phase, uint8_t idx, int16_t trim) ;

extern bool getSwitch00( int8_t swtch ) ;
extern bool getSwitch(int8_t swtch, bool nc, uint8_t level = 0 ) ;

extern int16_t g_ppmIns[] ;
extern uint8_t ppmInState ; //0=unsync 1..8= wait for value i-1
extern uint8_t ppmInValid ;

extern uint16_t AnalogData[] ;
extern int16_t ex_chans[] ;

extern uint8_t StepSize ;

extern const int8_t TelemIndex[] ;
extern const uint8_t TelemValid[] ;

//template<class t> inline t min(t a, t b){ return a<b?a:b; }
//template<class t> inline t max(t a, t b){ return a>b?a:b; }
template<class t> inline t limit(t mi, t x, t ma){ return min(max(mi,x),ma); }

extern uint16_t get_tmr10ms() ;

int8_t checkIncDecSwitch( int8_t i_val, int8_t i_min, int8_t i_max, uint8_t i_flags);
int8_t checkIncDec_hm( int8_t i_val, int8_t i_min, int8_t i_max) ;
int8_t checkIncDec_hg( int8_t i_val, int8_t i_min, int8_t i_max);
int8_t checkIncDec_hg0( int8_t i_val, int8_t i_max);


#define CHECK_INCDEC_H_MODELVAR( var, min, max)    var = checkIncDec_hm(var,min,max)
#define CHECK_INCDEC_H_GENVAR_0( var, max)     var = checkIncDec_hg0( var, max )
#define CHECK_INCDEC_MODELSWITCH( var, min, max)  var = checkIncDecSwitch(var,min,max,EE_MODEL|INCDEC_SWITCH)
#define CHECK_INCDEC_GENERALSWITCH( event, var, min, max)  var = checkIncDecSwitch(var,min,max,EE_GENERAL|INCDEC_SWITCH)

extern int16_t g_chans512[NUM_SKYCHNOUT] ;

void popMenu(bool uppermost=false) ;

void pushMenu(MenuFuncP newMenu) ;
void chainMenu(MenuFuncP newMenu) ;
void popMenu(bool uppermost) ;
MenuFuncP lastPopMenu() ;

int8_t switchUnMap( int8_t x ) ;
int8_t switchMap( int8_t x ) ;

extern uint8_t *cpystr( uint8_t *dest, uint8_t *source ) ;
extern uint8_t *ncpystr( uint8_t *dest, uint8_t *source, uint8_t count ) ;

int16_t calc_scaler( uint8_t index, uint16_t *unit, uint8_t *num_decimals) ;

extern void readAnalog() ;

uint8_t throttleReversed( void ) ;

void eeDirty(uint8_t msk) ;


#endif

