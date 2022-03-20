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

#ifndef eeprom_h
#define eeprom_h

#include <stdint.h>

#define MAX_MODELS  60

#define GENERAL_OWNER_NAME_LEN 10
#define MODEL_NAME_LEN         10
#define VOICE_NAME_SIZE					8

#define NUM_GLOBAL_VOICE_ALARMS 8

#define MAX_SKYMIXERS  64

#define MAX_CURVE5  8
#define MAX_CURVE9  8
#define MAX_CURVEXY 2

//#define MDVERS      12
#define MDX20VERS   4

#define NUM_VOICE_ALARMS	36
#define NUM_GLOBAL_VOICE_ALARMS 8
#define NUMBER_EXTRA_IDS				4
#define NUM_GVAR_ADJUST		20
#define NUM_SKYCHNOUT  32

#define MUSIC_NAME_LENGTH		14
#define MUSIC_DIR_LENGTH		8
#define PLAYLIST_COUNT			16

#define MAX_GVARS 7

#define MAX_MODES		6
#define NUM_SCALERS	8


typedef struct t_TrainerMix
{
  int8_t  swtch ;
  int8_t  studWeight ;
  uint8_t srcChn:4 ; //0-15 = ch1-16
  uint8_t mode:2;   //off,add-mode,subst-mode
} TrainerMix ;

typedef struct t_TrainerData
{
  int16_t        calib[4] ;
  TrainerMix     mix[4] ;
} TrainerData ;

typedef struct t_voiceAlarm
{
  uint8_t source ;
	uint8_t func;
  int8_t  swtch ;
	uint8_t rate ;
	uint8_t fnameType:3 ;
	uint8_t haptic:2 ;
	uint8_t vsource:2 ;
	uint8_t mute:1 ;
	uint8_t delay ;
  int16_t  offset ;		//offset
	union
	{
		int16_t vfile ;
		uint8_t name[8] ;
	} file ;
} VoiceAlarmData ;

typedef struct t_TrainerChannel
{
  int16_t calib ;
  uint8_t srcChn:3 ; //0-7 = ch1-8
  uint8_t source:3 ;	// Only used on index 0, invert option on channel 1
  uint8_t mode:2;   //off,add-mode,subst-mode
  int8_t  swtch ;
  int8_t  studWeight ;
} TrainerChannel ; //

typedef struct t_TrainerProfile
{
	TrainerChannel channel[4] ;
	uint8_t profileName[6] ;
} TrainerProfile ; //



typedef struct t_EEGeneral
{
  int16_t   calibMid[9] ;
  int16_t   calibSpanNeg[9] ;
  int16_t   calibSpanPos[9] ;
  uint16_t  chkSum ;
  uint8_t   myVers ;
  uint8_t   currModel ;
  uint8_t   contrast ;
  uint8_t   vBatWarn ;
  int8_t    vBatCalib ;
  int8_t    lightSw ;
  uint8_t   view ;
  uint8_t   stickMode ;
  char      ownerName[GENERAL_OWNER_NAME_LEN] ;
  uint8_t   templateSetup;  //RETA order according to chout_ar array 
	uint8_t		bt_baudrate ;
	uint8_t 	stickDeadband[4] ;
	uint8_t		btName[15] ;				// For the HC06 module
	uint8_t		trainerSource ;
	uint8_t		stickReverse ;
	uint8_t		language ;
  int8_t    inactivityTimer ;
  uint8_t   filterInput ;
  uint8_t   lightAutoOff ;
  uint8_t   lightOnStickMove;
	uint8_t		volume ;
	uint8_t 	bright ;			// backlight (red for 9Xt)
  uint8_t	hapticStrength;

  
	uint8_t   disableThrottleWarning:1 ;
  uint8_t   disableSwitchWarning:1 ;
  uint8_t   disableMemoryWarning:1 ;
  uint8_t   beeperVal:3 ;
  uint8_t   reserveWarning:1 ; // could be: uint8_t calibrateWarning:1;
  uint8_t   disableAlarmWarning:1 ;

  uint8_t   throttleReversed:1;
  uint8_t   minuteBeep:1;
  uint8_t   preBeep:1;
  uint8_t   flashBeep:1;
  uint8_t   disableSplashScreen:1;
  uint8_t   disablePotScroll:1;
  uint8_t   stickScroll:1 ;
  uint8_t   frskyinternalalarm:1;

	uint8_t   crosstrim:1 ;
	uint8_t   hapticMinRun:6 ;
	uint8_t   xcrosstrim:1 ;

//	uint8_t altSwitchNames:1 ;    // use alternative switch names (ARUNI)

	TrainerProfile trainerProfile[4] ;
	uint8_t CurrentTrainerProfile ;

	uint8_t		customStickNames[16] ;

	VoiceAlarmData gvad[NUM_GLOBAL_VOICE_ALARMS] ;

	uint8_t disableBtnLong:1 ;
	uint8_t		forceMenuEdit:1 ;

	uint16_t backgroundColour ;		// For Horus
	uint16_t textColour ;		// For Horus

	uint8_t gpsFormat:1 ;
	uint8_t altMixMenu:1 ;
  uint8_t hideNameOnSplash:1;
  uint8_t softwareVolume:1;
	uint8_t disableRxCheck:1 ;
	uint8_t musicLoop:1 ;	// Unused
	uint8_t musicType:1 ;	// Unused

  uint8_t sixPos ;
  uint8_t speakerPitch ;
	uint8_t musicVoiceFileName[MUSIC_NAME_LENGTH+2] ;
	uint8_t playListIndex ;
	int8_t inactivityVolume ;
	
	uint16_t totalElapsedTime ;
	uint16_t sparetotalElapsedTime ;	// In case we need 32 bits
	int16_t rtcCal ;		// Unused
	
	uint8_t radioRegistrationID[8] ;
  int8_t  screenShotSw ;	// Unused
	uint8_t welcomeType ;
	uint8_t welcomeFileName[8] ;

	uint8_t	forExpansion[20] ;	// Allows for extra items not yet handled
	 
} EE_X20General ;

typedef struct t_X20_ExpoData
{
  int8_t  expo[3][2][2];
  int8_t  drSw1;
  int8_t  drSw2;
} X20ExpoData ;

typedef struct t_LimitData
{
  int8_t  min ;
  int8_t  max ;
  bool    revert ;
  int16_t offset ;
} LimitData ;

#define MLTPX_ADD  0
#define MLTPX_MUL  1
#define MLTPX_REP  2

typedef struct t_X20_MixData
{
  uint8_t destCh ;            //        1..NUM_CHNOUT
  uint8_t srcRaw ;            //
  int16_t weight ;
  int16_t sOffset ;
  int8_t  swtch ;
  int8_t	curve ;             //0=symmetrisch 1=no neg 2=no pos
  uint8_t delayUp ;
  uint8_t delayDown ;
  uint8_t speedUp ;         // Servogeschwindigkeit aus Tabelle (10ms Cycle)
  uint8_t speedDown ;       // 0 nichts
  uint8_t carryTrim:1 ;
  uint8_t mltpx:2 ;           // multiplex method 0=+ 1=* 2=replace
  uint8_t mixWarn:2 ;         // mixer warning
  uint8_t disableExpoDr:1 ;
  uint8_t differential:2 ;	// Curve, Diff, Expo
	uint16_t modeControl ;
	uint8_t	switchSource ;
} X20MixData ;


typedef struct t_PhaseData
{
  int16_t trim[4];     // -500..500 => trim value, 501 => use trim of phase 0, 502, 503, 504 => use trim of phases 1|2|3|4 instead
  int8_t swtch;       // swtch of phase[0] is not used
  char name[6];
  uint8_t fadeIn:4;
  uint8_t fadeOut:4;
  int8_t swtch2 ;       // swtch of phase[0] is not used
	uint8_t spare ;		// Future expansion
} PhaseData ;

struct t_hiResBox
{
	uint8_t item ;
	uint8_t type ;
	uint16_t colour ;
	uint16_t bgColour ;
} ;

#define HIRES_OPT_TRIMS		0x01
#define HIRES_OPT_BORDERS	0x02

struct t_hiResDisplay
{
	uint8_t layout ;
	uint8_t options ;
	struct t_hiResBox boxes[6] ;
} ;

typedef struct te_CSwData
{ // Custom Switches data
	union
	{
  	int8_t  v1 ; //input
		uint8_t v1u ;
	} ;
	union
	{
  	int8_t  v2 ; 		//offset
		uint8_t v2u ;
	} ;
//	uint8_t func ;
	uint8_t func:5 ;
	uint8_t exfunc:3 ;
	int8_t andsw;
	uint8_t bitAndV3 ;
	uint8_t switchDelay ;
} X20CSwData ;

typedef struct te_SafetySwData // Safety Switches data
{
	int8_t  swtch ;
	uint8_t mode:2 ;
	int8_t source:6 ;
  int8_t  val ;
	uint8_t tune ;
} X20SafetySwData ;

typedef struct t_TimerMode
{
	uint8_t tmrModeA ;          // timer trigger source -> off, abs, stk, stk%, cx%
  int16_t tmrModeB ;          // timer trigger source -> !sw, none, sw, m_sw
  uint16_t tmrVal ;
	int16_t timerRstSw ;
  uint8_t tmrDir:1 ;						// Timer direction
	uint8_t timerCdown:1 ;
	uint8_t timerMbeep:1 ;
	uint8_t timerHaptic:2 ;
  uint8_t autoReset:1 ;
} X20TimerMode ;

// Scale a value
typedef struct t_scale
{
  uint8_t source ;
	int16_t offset ;
	uint16_t mult ;
	uint16_t div ;
	uint8_t unit ;
	uint8_t neg:1 ;
	uint8_t precision:2 ;
	uint8_t offsetLast:1 ;
	uint8_t exFunction:4 ;
	uint8_t name[4] ;
	uint8_t mod ;
	uint8_t dest ;
	uint8_t exSource ;
	uint8_t spare[3] ;
} ScaleData ;

typedef struct t_gvar
{
	int8_t gvar ;
	uint8_t gvsource ;
	int8_t gvswitch ;
} GvarData ;

typedef struct t_gvarAdjust
{
	uint8_t function:4 ;
	uint8_t gvarIndex:4 ;
	int8_t swtch ;
	int8_t switch_value ;
} GvarAdjust ;


struct t_module
{
	uint8_t protocol:4 ;
  uint8_t country:2 ;
	uint8_t ppmOpenDrain:1 ;
	uint8_t pulsePol:1 ;
  uint8_t channels ;
	uint8_t startChannel ;
  uint8_t	sub_protocol ;
	uint8_t	pxxRxNum ;
 	int8_t ppmDelay ;
  int8_t ppmFrameLength ;   //0=22.5  (10msec-30msec) 0.5msec increments
	int8_t option_protocol ;
	uint8_t failsafeMode:3 ;
	uint8_t notfailsafeRepeat:1 ;
	uint8_t r9mPower:2 ;
	uint8_t pxxDoubleRate:1 ;
	uint8_t pxxHeartbeatPB14:1 ;
	int8_t failsafe[16] ;
	uint8_t externalAntenna:1 ;
	uint8_t r9MflexMode:2 ;		// 0 - OFF, 1 - 915MHz, 2 - 868MHz
	uint8_t highChannels:1 ;
	uint8_t disableTelemetry:1 ;
	uint8_t exsub_protocol:2 ;
	uint8_t multiDisableTelemetry:1 ;
	int8_t ppmChannels ;
	uint8_t sparex ;
} ;

struct t_access
{
	uint8_t numChannels ;
	uint8_t type:2 ;
	uint8_t unusedType:6 ;
	uint8_t unused[7] ;
	uint8_t receiverName[3][8] ; // PXX2_LEN_RX_NAME
} ;

typedef struct t_extraId
{
	uint16_t id ;
	uint8_t dest ;
	uint8_t spare ;
} ExtraId ;

typedef struct te_FrSkyChannelData
{
  uint16_t   lratio ;               // 0.0 means not used, 0.1V steps EG. 6.6 Volts = 66. 25.1V = 251, etc.
//  uint8_t   gain ;                // 
  uint16_t   ratio3_4 ;               // 0.0 means not used, 0.1V steps EG. 6.6 Volts = 66. 25.1V = 251, etc.
//  uint8_t   unused_alarms_level ;
  uint8_t   units3_4 ;      			// 0=volts, 1=raw, 2=volts*2, 3=Amps
//  uint8_t   units ;               // 0=volts, 1=raw, 2=volts*2, 3=Amps
} SKYFrSkyChannelData ;

typedef struct te_FrSkyData
{
  SKYFrSkyChannelData channels[2] ;
} SKYFrSkyData ;

typedef struct t_Vario
{
  uint8_t varioSource ;
  int8_t  swtch ;
  uint8_t sinkTones:1 ;
  uint8_t spare:1 ;
  uint8_t param:6 ;
  int8_t baseFrequency ;
  int8_t offsetFrequency  ;
  uint8_t volume ;
} VarioData ;	

typedef struct t_customCheck
{
  uint8_t source ;
	int8_t  min ;
	int8_t  max ;
} CustomCheckData ;

typedef struct t_music
{
	int8_t musicStartSwitch ;
	int8_t musicPauseSwitch ;
	int8_t musicPrevSwitch ;
	int8_t musicNextSwitch ;
} MusicData ;


typedef struct t_X20_ModelDataV4
{
  char      name[MODEL_NAME_LEN];             // 10 must be first for eeLoadModelName
  int8_t  	modelVoice ;			// Index to model name voice (260+value)
  uint8_t   telemetryRxInvert:1 ;	// was tmrDir, now use tmrVal>0 => count down
  uint8_t   traineron:1;  		// 0 disable trainer, 1 allow trainer
  uint8_t   autoBtConnect:1 ;
  uint8_t   FrSkyUsrProto:1 ; // Protocol in FrSky User Data, 0=FrSky Hub, 1=WS HowHigh
  uint8_t   FrSkyGpsAlt:1 ;		// Use Gps Altitude as main altitude reading
  uint8_t   FrSkyImperial:1 ; // Convert FrSky values to imperial units
  uint8_t   spare0:2;
	uint8_t 	modelVersion ;
  uint8_t   not_protocol:4 ;
  uint8_t   not_country:2 ;
  uint8_t   spare1:2 ;
  int8_t    not_ppmNCH;
  uint8_t   thrTrim:1;            // Enable Throttle Trim
	uint8_t   not_numBlades:2;					// RPM scaling, now elsewhere as uint8_t
	uint8_t   extendedTrims:1;			// Only applies to phases
  uint8_t   thrExpo:1;            // Enable Throttle Expo
	uint8_t   frskyComPort:1 ;
	uint8_t		DsmTelemetry:1 ;
	uint8_t   useCustomStickNames:1 ;
  int8_t    trimInc;          // Trim Increments
  int8_t    not_ppmDelay;
  int8_t    trimSw;
  uint8_t   beepANACenter;    // 1<<0->A1.. 1<<6->A7, 1<<7->P4
  uint8_t   pulsePol:1;
  uint8_t   extendedLimits:1;
  uint8_t   swashInvertELE:1;
  uint8_t   swashInvertAIL:1;
  uint8_t   swashInvertCOL:1;
  uint8_t   swashType:3;
  uint8_t   swashCollectiveSource;
  uint8_t   swashRingValue;
  int8_t    not_ppmFrameLength;   //0=22.5  (10msec-30msec) 0.5msec increments
	uint8_t		rxVratio ;
  X20MixData   mixData[MAX_SKYMIXERS];
//  LimitData limitData[NUM_SKYCHNOUT];
  X20ExpoData  expoData[4];
  int16_t trim[4];
  int8_t  curves5[MAX_CURVE5][5];
  int8_t  curves9[MAX_CURVE9][9];
  int8_t  curvexy[MAX_CURVEXY][18] ;
  int8_t	curve6[6] ;
	char modelVname[VOICE_NAME_SIZE] ;
	char modelImageName[VOICE_NAME_SIZE+2] ;
	ScaleData Scalers[NUM_SCALERS] ;

	PhaseData phaseData[MAX_MODES] ;

	VoiceAlarmData vad[NUM_VOICE_ALARMS] ;

  LimitData limitData[NUM_SKYCHNOUT];
  X20SafetySwData safetySw[NUM_SKYCHNOUT];

	X20TimerMode timer[2] ;
  X20CSwData   customSw[NUM_SKYCSW] ;
	
	uint8_t throttleSource:3 ;
	uint8_t throttleIdle:1 ;
  uint8_t throttleReversed:1;
	uint8_t disableThrottleCheck:1 ;
	uint8_t basic_lua:1 ;
	uint8_t instaTrimToTrims:1 ;
	GvarData gvars[MAX_GVARS] ;
	uint8_t trainerProfile ;
	uint8_t throttleIdleScale ;
	uint8_t sub_trim_limit ;
	struct t_module Module[2] ;
	int8_t accessFailsafe[2][8] ;
	struct t_access Access[2] ;
	uint8_t telemetryTimeout ;
	uint8_t	customTelemetryNames[40] ;
	uint32_t totalTime ;
  SKYFrSkyData frsky ;
	GvarAdjust gvarAdjuster[NUM_GVAR_ADJUST] ;
	uint8_t voiceFlushSwitch ;
	int8_t enRssiOrange:2 ;
	int8_t rssiOrange:6 ;
	uint8_t	enRssiRed:2 ;
	int8_t rssiRed:6 ;
	uint8_t customDisplayIndex[2][6] ;
	ExtraId extraId[NUMBER_EXTRA_IDS] ;
	uint8_t extraSensors ;
	VarioData varioData ;
 	CustomCheckData customCheck ;
	
	int8_t logSwitch ;                     		// Unused
	uint8_t logRate:4 ;                    		// Unused
	uint8_t logNew:1 ;                     		// Unused
	uint32_t LogDisable[4] ;	// Up to 128 sensors etc. Unused
	uint8_t backgroundScript[8] ;
  int8_t mlightSw ;

	uint8_t anaVolume ;	// analog volume control Unused

	struct t_hiResDisplay hiresDisplay[2] ;		// Unused
	struct t_hiResDisplay xhiresDisplay[4] ;	// Unused
	int8_t cellScalers[12] ;
	uint8_t   numBlades ;					// RPM scaling
  int8_t    dsmMode ;
	uint8_t		dsmAasRssi:1 ;
  uint8_t   dsmVario:3 ;
	MusicData musicData ;                  		// Unused
	uint8_t trainerStartChannel ;          		// Unused
	uint16_t modelswitchWarningDisables ;
  uint16_t  modelswitchWarningStates ;
  uint16_t xmodelswitchWarningStates ;	// Enough bits for Taranis X9E
	uint8_t currentSource ;
	uint8_t 	FASoffset ;			// 0.0 to 1.5

} X20ModelData ;

extern X20ModelData g_model ;









#endif

