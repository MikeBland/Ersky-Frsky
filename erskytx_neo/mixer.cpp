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
#include <string.h>
#include "erskyTx.h"
#include "myeeprom.h"
//#include "adcDriver.h"
#include "mixer.h"
#include "logicIo.h"
#include "audio.h"
#include "menus.h"


// Temp
//#include "stm32h7xx.h"
extern uint32_t UsedMenu ;
extern uint32_t UsedMixer ;
extern uint32_t Used10ms ;

uint8_t  InactivityMonitor = 0 ;

extern uint32_t Used1secMenu ;
extern uint32_t Used1secMixer ;
extern uint32_t Used1sec10ms ;


#define OSMP_SAMPLES	4
#define OSMP_TOTAL		16384
#define OSMP_ROUNDUP	0
#define OSMP_SHIFT		2


extern EE_X20General g_eeGeneral ;

#define IS_THROTTLE( x ) ( x== 2 )

uint16_t S_anaFilt[ANALOG_DATA_SIZE] ;				// Analog inputs after filtering

uint32_t IdlePercent ;

uint8_t CalcScaleNest = 0 ;

int16_t getValue(uint8_t i) ;


const uint8_t switchIndex[4] = { HSW_SA0, HSW_SB0, HSW_SC0, HSW_SD0 } ;

int16_t g_chans512[NUM_SKYCHNOUT] ;

//int16_t  anas[NUM_SKYXCHNRAW+1+MAX_GVARS+1] ;		// To allow for 3POS and THIS and 8 extra PPM inputs
int16_t  anas[115] ;		// To allow for 3POS and THIS and 8 extra PPM inputs
int32_t  chans[NUM_SKYCHNOUT] = {0} ;

uint16_t  bpanaCenter = 0;
uint16_t sDelay[MAX_SKYMIXERS] = {0};
int32_t act[MAX_SKYMIXERS] = {0} ;
uint8_t swOn[MAX_SKYMIXERS] = {0} ;
uint8_t	CurrentPhase = 0 ;
int16_t RawSticks[4] ;
uint8_t TrimInUse[4] = { 1, 1, 1, 1 } ;
uint8_t ThrottleStickyOn = 0 ;



extern volatile uint32_t OneSecFlag ;
//extern uint32_t IdleCount ;
uint32_t MixerDebugCount ;
uint32_t MixerRate ;
extern uint16_t PulsesRate ;
extern uint32_t PulsesCountForRate ;

uint16_t LastTim7 ;
uint16_t Tim7Rate ;
uint32_t Tim7Total ;

volatile uint8_t MixTick10ms ;

int8_t phyStick[4] ;

void runMixer( void ) ;
void perOut(int16_t *chanOut, uint8_t att ) ;

uint16_t isqrt32(uint32_t n)
{
  uint16_t c = 0x8000;
  uint16_t g = 0x8000;

  for(;;)
	{
    if((uint32_t)g*g > n)
		{
      g ^= c ;
		}
    c >>= 1 ;
    if(c == 0)
		{
      return g ;
		}
    g |= c ;
  }
}

int8_t REG(int8_t x, int8_t min, int8_t max)
{
  int8_t result = x;
  if (x >= 126 || x <= -126) {
    x = (uint8_t)x - 126;
    result = g_model.gvars[x].gvar ;
    if (result < min) {
      g_model.gvars[x].gvar = result = min;
    }
    if (result > max) {
      g_model.gvars[x].gvar = result = max;
    }
  }
  return result;
}


int8_t REG100_100(int8_t x)
{
	return REG( x, -100, 100 ) ;
}

uint32_t InacCounter = 0 ;
uint16_t InacSum = 0;

static void inactivityCheck()
{
  if(s_noHi) s_noHi--;
  uint16_t tsum = 0;
  for(uint8_t i=0;i<4;i++) tsum += anas[i] ;
  if(abs(int16_t(tsum-InacSum))>INACTIVITY_THRESHOLD)
	{
		InacSum = tsum ;
		InactivityMonitor = 1 ;  // reset in perMain
		InacCounter = 0 ;
  }
  if( g_eeGeneral.inactivityTimer + 10 )
	{
    InacCounter += 1 ;
  	if( InacCounter >(((uint32_t)g_eeGeneral.inactivityTimer+10)*(100*60)))
    if( ( InacCounter & 0x1FF ) == 1 )
		{
			SystemOptions &= ~SYS_OPT_MUTE ;						
			putVoiceQueue( VOLUME_MASK + g_eeGeneral.inactivityVolume + ( NUM_VOL_LEVELS-3 ) ) ;
			voiceSystemNameNumberAudio( SV_INACTV, V_INACTIVE, AU_INACTIVITY ) ;
			putVoiceQueue( 0xFFFF ) ;
    }
  }
}



bool getSwitchDr( int8_t swtch )
{
	uint8_t aswitch = abs(swtch) ;
	if ( ( aswitch <= HSW_FM6 ) && ( aswitch >= HSW_FM0 ) )
	{
		aswitch -= HSW_FM0 ;
		aswitch = getFlightPhase() == aswitch ;
		return (swtch < 0) ? !aswitch : aswitch ;
	}
	else
	{
		return getSwitch00( swtch ) ;
	}
}

int16_t calc_scaler( uint8_t index, uint16_t *unit, uint8_t *num_decimals)
{
	int32_t value ;
	int32_t exValue ;
	uint8_t lnest ;
	ScaleData *pscaler ;
	
	lnest = CalcScaleNest ;
	if ( lnest > 5 )
	{
		return 0 ;
	}
	CalcScaleNest = lnest + 1 ;
	// process
	pscaler = &g_model.Scalers[index] ;
	if ( pscaler->source )
	{
		value = getValue( pscaler->source - 1 ) ;
//		if ( ( pscaler->source == NUM_SKYXCHNRAW+1 ) || ( pscaler->source == NUM_SKYXCHNRAW+2 ) )
//		{
//			value = scale_telem_value( value, pscaler->source - NUM_SKYXCHNRAW-1, NULL ) ;
//		}
	}
	else
	{
		value = 0 ;
	}
	if ( !pscaler->offsetLast )
	{
		value += pscaler->offset ;
	}
	value *= pscaler->mult+1 ;
	value /= pscaler->div+1 ;
	if ( pscaler->mod )
	{
		value %= pscaler->mod + 1 ;
	}
	if ( pscaler->exSource )
	{
		exValue = getValue( pscaler->exSource - 1 ) ;
//		if ( ( pscaler->exSource == NUM_SKYXCHNRAW+1 ) || ( pscaler->exSource == NUM_SKYXCHNRAW+2 ) )
//		{
//			exValue = scale_telem_value( exValue, pscaler->exSource - NUM_SKYXCHNRAW-1, NULL ) ;
//		}
		if ( pscaler->exFunction )
		{
			switch ( pscaler->exFunction )
			{
				case 1 :	// Add
					value += exValue ;
				break ;
				case 2 :	// Subtract
					value -= exValue ;
				break ;
				case 3 :	// Multiply
					value *= exValue ;
				break ;
				case 4 :	// Divide
					if ( exValue )
					{
						value /= exValue ;
					}
				break ;
				case 5 :	// Mod
					if ( exValue )
					{
						value %= exValue ;
					}
				break ;
				case 6 :	// Min
					if ( exValue )
					{
						if ( exValue < value )
						{
							value = exValue ;
						}
					}
				break ;
			}
		}
	}
	CalcScaleNest = lnest ;
	if ( pscaler->offsetLast )
	{
		value += pscaler->offset ;
	}
	if ( pscaler->neg )
	{
		value = -value ;
	}
	// Limit to an int16_t
	if ( value > 32767 )
	{
		value = 32767 ;
	}
	if ( value < -32768 )
	{
		value = -32768 ;
	}
	if ( unit )
	{
		*unit = pscaler->unit ;
	}
	if ( num_decimals )
	{
		*num_decimals = pscaler->precision ;
	}
	if ( pscaler->dest )
	{
//		store_telemetry_scaler( pscaler->dest, value ) ;
	}

	return value ;
}


uint16_t expou(uint16_t x, uint16_t k)
{
    // k*x*x*x + (1-k)*x
    return ((unsigned long)x*x*x/0x10000*k/(RESXul*RESXul/0x10000) + (RESKul-k)*x+RESKul/2)/RESKul;
}
// expo-funktion:
// ---------------
// kmplot
// f(x,k)=exp(ln(x)*k/10) ;P[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
// f(x,k)=x*x*x*k/10 + x*(1-k/10) ;P[0,1,2,3,4,5,6,7,8,9,10]
// f(x,k)=x*x*k/10 + x*(1-k/10) ;P[0,1,2,3,4,5,6,7,8,9,10]
// f(x,k)=1+(x-1)*(x-1)*(x-1)*k/10 + (x-1)*(1-k/10) ;P[0,1,2,3,4,5,6,7,8,9,10]

int16_t expo(int16_t x, int16_t k)
{
    if(k == 0) return x;
    int32_t   y;
    bool    neg =  x < 0;
    if(neg)   x = -x;
    if(k<0){
        y = RESXu-expou(RESXu-x,-k);
    }else{
        y = expou(x,k);
    }
    return neg? -y:y;
}


uint8_t get_dr_state(uint8_t x)
{
	X20ExpoData *ped ;

	ped = &g_model.expoData[x] ;
	
 	return (!getSwitchDr( ped->drSw1) ? DR_HIGH :
    !getSwitchDr( ped->drSw2) ? DR_MID : DR_LOW) ;
}

int16_t intpol(int16_t x, uint8_t idx) // -100, -75, -50, -25, 0 ,25 ,50, 75, 100
{
#define D9 (RESX * 2 / 8)
#define D5 (RESX * 2 / 4)
#define D6 (RESX * 2 / 5)
    uint32_t cv9 = idx >= MAX_CURVE5;
		int8_t *crv ;
		
		if ( idx >= MAX_CURVE5 + MAX_CURVE9 )
		{
			if ( idx == MAX_CURVE5 + MAX_CURVE9 + MAX_CURVEXY )
			{
				crv = g_model.curve6 ;
				cv9 = 3 ;
			}
			else
			{
				crv = g_model.curvexy[idx - (MAX_CURVE5 + MAX_CURVE9)] ;
				cv9 = 2 ;
			}
		}
		else
		{
    	crv = cv9 ? g_model.curves9[idx-MAX_CURVE5] : g_model.curves5[idx];
		}
    int16_t erg ;

    x+=RESXu;
    if(x < 0)
		{
      erg = (int16_t)crv[0] * (RESX/4);
    }
		else if(x >= (RESX*2))
		{
			if ( cv9 == 3 )
			{
      	erg = (int16_t)crv[5] * (RESX/4);
			}
			else
			{
      	erg = (int16_t)crv[(cv9 ? 8 : 4)] * (RESX/4);
			}
    }
		else
		{
			int16_t deltax ;
			div_t qr ;
			if ( cv9 == 2 ) // xy curve
			{
		    int16_t a = 0 ;
				int16_t b ;
				int16_t c ;
				uint32_t i ;

				// handle end points
  	    c = RESX + calc100toRESX(crv[17]) ;
				if ((uint16_t)x>c)
				{
					return calc100toRESX(crv[8]) ;
				}
  	    b = RESX + calc100toRESX(crv[9]) ;
				if ((uint16_t)x<b)
				{
					return calc100toRESX(crv[0]) ;
				}

				for ( i = 0 ; i < 8 ; i += 1 )
				{
	        a = b ;
  	      b = (i==7 ? c : RESX + calc100toRESX(crv[i+10]));
    	    if ((uint16_t)x<=b) break;
				}
				qr.quot = i ;
				qr.rem = x - a ;
				deltax = b - a ;
			}
			else
			{
        if(cv9 == 3)
				{
					qr = div( x, D6 ) ;
					deltax = D6 ;
				}
				else if ( cv9 )
				{
					qr = div( x, D9 ) ;
					deltax = D9 ;
        }
				else
				{
					qr = div( x, D5 ) ;
					deltax = D5 ;
        }
  	  }
			int32_t y1 = (int16_t)crv[qr.quot] * (RESX/4) ;
			int32_t deltay = (int16_t)crv[qr.quot+1] * (RESX/4) - y1 ;
			if ( deltax )
			{
				erg = y1 + ( qr.rem ) * deltay / deltax ;
			}
		}
    return erg / 25; // 100*D5/RESX;
}

int16_t calcExpo( uint8_t channel, int16_t value )
{
  uint8_t expoDrOn = get_dr_state(channel);
  uint8_t stkDir = value > 0 ? DR_RIGHT : DR_LEFT ;

  if(IS_THROTTLE(channel) && g_model.thrExpo)
	{
    value  = 2*expo((value+RESX)/2,REG100_100(g_model.expoData[channel].expo[expoDrOn][DR_EXPO][DR_RIGHT])) ;
    stkDir = DR_RIGHT ;
  }
  else
    value  = expo(value,REG100_100(g_model.expoData[channel].expo[expoDrOn][DR_EXPO][stkDir])) ;

  value = (int32_t)value * (REG(g_model.expoData[channel].expo[expoDrOn][DR_WEIGHT][stkDir]+100, 0, 100))/100 ;
  if (IS_THROTTLE(channel) && g_model.thrExpo) value -= RESX;
	return value ;
}

//__attribute__((section(".itcm_text")))

void mixer_loop(void* pdata)
{
	for(;;)
	{
// 		uint32_t dtimer = DWT->CYCCNT ;
		runMixer() ;
//		dtimer = DWT->CYCCNT - dtimer ;
//		UsedMixer += dtimer ;
		
    vTaskDelay( pdMS_TO_TICKS( 5 ) ) ;
	}
}

//__attribute__((section(".itcm_text")))

int16_t scaleAnalog( int16_t v, uint8_t channel )
{
	int16_t mid ;
	int16_t neg ;
	int16_t pos ;

	mid = g_eeGeneral.calibMid[channel] ;
  pos = g_eeGeneral.calibSpanPos[channel] ;
  neg = g_eeGeneral.calibSpanNeg[channel] ;

	v -= mid ;

	v  =  v * (int32_t)RESX /  (max((int16_t)100,(v>0 ? pos : neg ) ) ) ;
	
	if(v <= -RESX) v = -RESX;
	if(v >=  RESX) v =  RESX;
	
	return v ;
}

void getADC_osmp()
{
	register uint32_t x ;
	register uint32_t y ;
	uint32_t numAnalog = ANALOG_DATA_SIZE ;

	uint16_t temp[ANALOG_DATA_SIZE] ;
//	static uint16_t next_ana[ANALOG_DATA_SIZE] ;

	for( x = 0 ; x < numAnalog ; x += 1 )
	{
		temp[x] = 0 ;
	}
	for( y = 0 ; y < OSMP_SAMPLES ; y += 1 )
	{
		readAnalog() ;
//		read_adc() ;
		
		for( x = 0 ; x < numAnalog ; x += 1 )
		{
			temp[x] += AnalogData[x] ;
		}
	}
	for( x = 0 ; x < ANALOG_DATA_SIZE ; x += 1 )
	{
		uint16_t y = (temp[x] + OSMP_ROUNDUP) >> OSMP_SHIFT ;
		
//		uint16_t z = S_anaFilt[x] ;
//		uint16_t w = next_ana[x] ;
		
//		int16_t diff = abs( (int16_t) y - z ) ;

//		next_ana[x] = y ;
//		if ( diff < 10 )
//		{
//			if ( y > z )
//			{
//				if ( w > z )
//				{
//					y = z + 1 ;
//				}
//				else
//				{
//					y = z ;
//				}
//			}
//			else if ( y < z )
//			{
//				if ( w < z )
//				{
//					y = z - 1 ;
//				}
//				else
//				{
//					y = z ;
//				}
//			}
//		}
		S_anaFilt[x] = y ;
	}
}


//__attribute__((section(".itcm_text")))

//	MixerCount += 1 ;		
//	uint16_t t1 = getTmr2MHz() ;
//	perOutPhase(g_chans512, 0);
//	t1 = getTmr2MHz() - t1 ;
//	g_timeMixer = t1 ;





void runMixer()
{
//	readAnalog() ;
	getADC_osmp() ;
	
	MixerDebugCount += 1 ;
	perOut( g_chans512, 0 | FADE_FIRST ) ;
}

void perOut(int16_t *chanOut, uint8_t att )
{
	int16_t trimA[4] ;
  uint16_t anaCenter = 0 ;
	int16_t trainerThrottleValue = 0 ;
	uint8_t trainerThrottleValid = 0 ;
	
	uint32_t x ;
//static uint32_t saveIdleCount ;

//	x = TIM7->CNT ;
//	Tim7Total += (x - LastTim7) & 0x0000FFFF ;
//	LastTim7 = x ;

	if ( OneSecFlag )
	{
		MixerRate = MixerDebugCount ;
		MixerDebugCount = 0 ;
		PulsesRate = PulsesCountForRate ;
		PulsesCountForRate = 0 ;
		OneSecFlag = 0 ;

//		x = ( IdleCount - saveIdleCount ) ;
//		x /= 200 ;
//		saveIdleCount = IdleCount ;
//		if ( x > 9999 )
//		{
//			x = 9999 ;
//		}
//		IdlePercent = x ;

		Tim7Rate = Tim7Total / 2000 ;
		Tim7Total = 0 ;

		Used1secMenu = UsedMenu ;
		Used1secMixer   = UsedMixer ;
		Used1sec10ms    =  Used10ms ;

		UsedMenu = 0 ;
		UsedMixer = 0 ;
		Used10ms = 0 ;

	}

  uint8_t ele_stick, ail_stick ;
  ele_stick = 1 ; //ELE_STICK ;
  ail_stick = 3 ; //AIL_STICK ;


	uint8_t stickIndex = g_eeGeneral.stickMode*4 ;

	for( uint32_t i = 0 ; i < 4 ; i += 1 ) // calc Sticks
	{
		int16_t v = S_anaFilt[i] ; // anaIn( i ) ;
		v = scaleAnalog( v, i ) ;

		uint8_t index = i ;
		if ( i < 4 )
		{
    	phyStick[i] = v >> 4 ;
			index = stickScramble[stickIndex+i] ;
		}
    CalibratedStick[index] = v ; //for show in expo

		// Filter beep centre
		{
			int8_t t = v/16 ;
			uint16_t mask = 1 << index ;
			if ( t < 0 )
			{
				t = -t ;		//abs(t)
			}
			if ( t <= 1 )
			{
        anaCenter |= ( t == 0 ) ? mask : bpanaCenter & mask ;
			}
		}

    if(i<4)
		{ //only do this for sticks
      //===========Trainer mode================
      if (!(att&NO_TRAINER) && g_model.traineron)
			{
				TrainerChannel *tChan = &g_eeGeneral.trainerProfile[g_model.trainerProfile].channel[index] ;
        if (tChan->mode && getSwitch00(tChan->swtch))
				{
					if ( ppmInValid )
					{
            uint8_t chStud = tChan->srcChn ;
            int32_t vStud  = (g_ppmIns[chStud] - tChan->calib) /* *2 */ ;
            vStud *= tChan->studWeight ;
            vStud /= 50 ;
            switch ((uint8_t)tChan->mode)
						{
              case 1: v += vStud;   break; // add-mode
              case 2: v  = vStud;   break; // subst-mode
            }
						if ( index == 2 )
						{
							trainerThrottleValue = vStud ;
							trainerThrottleValid = 1 ;
						}												 
					}
				}
      }
     	
			
			
			
			
			
			if ( att & FADE_FIRST )
			{
    		RawSticks[index] = v; //set values for mixer
			}
			v = calcExpo( index, v ) ;
		
      trimA[i] = getTrimValue( CurrentPhase, i )*2 ; //    if throttle trim -> trim low end
		}
		if ( att & FADE_FIRST )
		{
			if ( i < 9 )
			{
        anas[index] = v ; //set values for mixer
			}
		}
		if(att&NO_INPUT)
		{ //zero input for setStickCenter()
		  if ( i < 4 )
			{
    	  if(!IS_THROTTLE(index))
				{
					if ( ( v > (RESX/100 ) ) || ( v < -(RESX/100) ) )
					{
				    anas[index] = 0; //set values for mixer
					}
          trimA[index] = 0;
      	}
        anas[i+PPM_BASE] = 0;
      }
    }

	}
	//    if throttle trim -> trim low end
  if(g_model.thrTrim)
	{
		int8_t ttrim ;
		ttrim = getTrimValue( CurrentPhase, 2 ) ;
		if(throttleReversed())
		{
			ttrim = -ttrim ;
		}
		int16_t tmp = calc100toRESX( 100 - g_model.throttleIdleScale ) * 2 ;	// 0 to 2 * RESX
		if ( ( anas[2] + RESX) > tmp )
		{
			trimA[2] = 0 ;
		}
		else
		{
			tmp = ( tmp - ( anas[2] + RESX ) ) * RESX / tmp ;
	    trimA[2] = ((int32_t)ttrim+125) * tmp / (RESX) ;
		}
	}
	if ( att & FADE_FIRST )
	{

    //===========BEEP CENTER================
    anaCenter &= g_model.beepANACenter;
//    if(((bpanaCenter ^ anaCenter) & anaCenter)) audioDefevent(AU_POT_STICK_MIDDLE);
    bpanaCenter = anaCenter;
	 
		// Set up anas[] array
    anas[MIX_MAX-1]  = RESX ;     // MAX
    anas[MIX_FULL-1] = RESX ;     // FULL

    for(uint8_t i=0;i<4;i++) anas[i+PPM_BASE] = (g_ppmIns[i] - g_eeGeneral.trainerProfile[g_model.trainerProfile].channel[i].calib)*2; //add ppm channels
    for(uint8_t i=4;i<NUM_PPM;i++)    anas[i+PPM_BASE]   = g_ppmIns[i]*2; //add ppm channels
    for(uint8_t i=0;i<NUM_SKYCHNOUT;i++) anas[i+CHOUT_BASE] = chans[i]; //other mixes previous outputs
    for(uint8_t i=0;i<MAX_GVARS;i++) anas[i+MIX_GVAR] = g_model.gvars[i].gvar * 1024 / 100 ;
  
		int16_t heliEle = anas[ele_stick] ;
		int16_t heliAil = anas[ail_stick] ;
	
    //===========Swash Ring================
    if(g_model.swashRingValue)
    {
      uint32_t v = ((int32_t)heliEle*heliEle + (int32_t)heliAil*heliAil);
		  int16_t tmp = calc100toRESX(g_model.swashRingValue) ;
      uint32_t q ;
      q = (int32_t)tmp * tmp ;
      if(v>q)
      {
        uint16_t d = isqrt32(v);
        heliEle = (int32_t)heliEle*tmp/((int32_t)d) ;
        heliAil = (int32_t)heliAil*tmp/((int32_t)d) ;
      }
    }

#define REZ_SWASH_X(x)  ((x) - (x)/8 - (x)/128 - (x)/512)   //  1024*sin(60) ~= 886
#define REZ_SWASH_Y(x)  ((x))   //  1024 => 1024
	
    if(g_model.swashType)
    {
      int16_t vp = 0 ;
      int16_t vr = 0 ;

      if( !(att & NO_INPUT) )  //zero input for setStickCenter()
			{
	      vp = heliEle+trimA[ele_stick];
  	    vr = heliAil+trimA[ail_stick];
				TrimInUse[ele_stick] |= 1 ;
				TrimInUse[ail_stick] |= 1 ;
			}

      int16_t vc = 0;
      if(g_model.swashCollectiveSource)
			{
        vc = anas[g_model.swashCollectiveSource-1];
			}

      if(g_model.swashInvertELE) vp = -vp;
      if(g_model.swashInvertAIL) vr = -vr;
      if(g_model.swashInvertCOL) vc = -vc;

      switch (( uint8_t)g_model.swashType)
      {
      case (SWASH_TYPE_120):
          vp = REZ_SWASH_Y(vp);
          vr = REZ_SWASH_X(vr);
          anas[MIX_CYC1-1] = vc - vp;
          anas[MIX_CYC2-1] = vc + vp/2 + vr;
          anas[MIX_CYC3-1] = vc + vp/2 - vr;
          break;
      case (SWASH_TYPE_120X):
          vp = REZ_SWASH_X(vp);
          vr = REZ_SWASH_Y(vr);
          anas[MIX_CYC1-1] = vc - vr;
          anas[MIX_CYC2-1] = vc + vr/2 + vp;
          anas[MIX_CYC3-1] = vc + vr/2 - vp;
          break;
      case (SWASH_TYPE_140):
          vp = REZ_SWASH_Y(vp);
          vr = REZ_SWASH_Y(vr);
          anas[MIX_CYC1-1] = vc - vp;
          anas[MIX_CYC2-1] = vc + vp + vr;
          anas[MIX_CYC3-1] = vc + vp - vr;
          break;
      case (SWASH_TYPE_90):
          vp = REZ_SWASH_Y(vp);
          vr = REZ_SWASH_Y(vr);
          anas[MIX_CYC1-1] = vc - vp;
          anas[MIX_CYC2-1] = vc + vr;
          anas[MIX_CYC3-1] = vc - vr;
          break;
      default:
          break;
      }
	  }

    if(MixTick10ms)
		{
			inactivityCheck() ;
//			trace(); //trace thr 0..32  (/32)
		}

	
	}
	
	
	memset(chans,0,sizeof(chans)) ;        // All outputs to 0
	
  uint8_t mixWarning = 0;
  //========== MIXER LOOP ===============

	uint32_t i ;
  for( i=0;i<MAX_SKYMIXERS;i++)
	{
    X20MixData *md = &g_model.mixData[i] ;
	 
			// need to check for GVAR
			int16_t lweight = md->weight ;
			int16_t mixweight = lweight ;
			int16_t loffset = md->sOffset ;
			int16_t mixoffset = loffset ;

      if((md->destCh==0) || (md->destCh>NUM_SKYCHNOUT))
			{
				break ;
			}
	 
      //Notice 0 = NC switch means not used -> always on line
      int16_t v  = 0;
      uint8_t swTog;
      uint8_t swon = swOn[i] ;

			bool t_switch = getSwitch(md->swtch,1) ;
      if (md->swtch && (md->srcRaw > PPM_BASE) && (md->srcRaw <= PPM_BASE+NUM_PPM) && (ppmInValid == 0) )
			{
				// then treat switch as false ???				
				t_switch = 0 ;
			}	
//        if (md->swtch && (md->srcRaw > MIX_3POS+MAX_GVARS + NUM_SCALERS ) && (ppmInValid == 0) )
//				{ // Extra PPM inputs (9-16)
//					if (md->srcRaw <= MIX_3POS+MAX_GVARS + NUM_SCALERS + NUM_EXTRA_PPM )
//					{
//						// then treat switch as false ???				
//						t_switch = 0 ;
//					}
//				}
      
      if ( t_switch )
			{
				if ( md->modeControl & ( 1 << CurrentPhase ) )
				{
					t_switch = 0 ;
				}
			}

      uint8_t k = md->srcRaw ;

#define DEL_MULT 256

      if(!t_switch)
      { // switch on?  if no switch selected => on
        swTog = swon ;
        swOn[i] = swon = false ;
        if (k == MIX_GVAR+MAX_GVARS+1)
				{
					act[i] = chans[md->destCh-1] * DEL_MULT / 100 ;	// "THIS"
				}
        if( k !=MIX_MAX && k !=MIX_FULL)
				{
					continue ;// if not MAX or FULL - next loop
				}
        if(md->mltpx==MLTPX_REP)
				{
					continue; // if switch is off and REPLACE then off
				}
        v = ( k == MIX_FULL ? -RESX : 0); // switch is off and it is either MAX=0 or FULL=-512
      }
      else
			{
        swTog = !swon ;
        swon = true ;
        k -= 1 ;
				if ( k < MIX_GVAR+MAX_GVARS+1 )
				{
					v = anas[k] ; //Switch is on. MAX=FULL=512 or value.
				}
				if ( k < 4 )
				{
					if ( md->disableExpoDr )
					{
 		      	v = RawSticks[k]; //Switch is on. MAX=FULL=512 or value.
					}
				}
				if ( ( k >= PHY_BASE ) && ( k < PHY_BASE + NUM_MIX_PHY_SWITCHES + NUM_SKYCSW ) )
				{
					uint32_t index = k - PHY_BASE ;
					if ( index >= NUM_MIX_PHY_SWITCHES )
					{
						v = getSwitch( CSW_INDEX + index - NUM_MIX_PHY_SWITCHES, 0, 0 ) ;
						v = v ? 1024 : -1024 ;
					}
					else
					{
						uint32_t /*EnumKeys*/ sw = switchIndex[index] ;
//						if ( ( index == 5) || ( index == 7) )
//						{ // 2-POS switch
//     					v = hwKeyState(sw) ? 1024 : -1024 ;
//						}
//						else if ( index == NUM_MIX_PHY_SWITCHES - 1 )
//						{
//							v = ((int32_t)switchPosition( HSW_Ele6pos0 ) * 2048 - 5120)/5 ;
//						}
//						else
						{ // 3-POS switch
     					v = hwKeyState(sw) ? -1024 : (hwKeyState(sw+1) ? 0 : 1024) ;
						}
					}
				}
				if ( k >= CHOUT_BASE )
        {
					if(k<CHOUT_BASE+NUM_SKYCHNOUT)
					{
						if ( md->disableExpoDr )
						{
							v = g_chans512[k-CHOUT_BASE] ;
						}
						else
						{
            	if(k<CHOUT_BASE+md->destCh-1)
							{
								v = chans[k-CHOUT_BASE] / 100 ; // if we've already calculated the value - take it instead // anas[i+CHOUT_BASE] = chans[i]
							}
							else
							{
								v = ex_chans[k-CHOUT_BASE] ;
							}
						}
					}
				}


				if ( k == MIX_GVAR+MAX_GVARS )
				{
					v = chans[md->destCh-1] / 100 ;	// "THIS"
				}	
				if ( ( k > MIX_GVAR+MAX_GVARS ) && ( k <= MIX_GVAR+MAX_GVARS+1 + NUM_SCALERS ) )
				{
					 v = calc_scaler( k - (MIX_GVAR+MAX_GVARS+1), 0, 0 ) ;
        }
				
				if(md->mixWarn) mixWarning |= 1<<(md->mixWarn-1); // Mix warning

			}

       swOn[i] = swon ;

      //========== INPUT OFFSET ===============
//      if ( md->lateOffset == 0 )
//      {
//				if(mixoffset) v += calc100toRESX( mixoffset	) ;
//      }

      //========== DELAY and SLOW ===============
			if ( ( att & NO_DELAY_SLOW ) == 0 )
			{
        if (md->speedUp || md->speedDown || md->delayUp || md->delayDown)  // there are delay values
        {

					int16_t my_delay = sDelay[i] ;
					int32_t tact = act[i] ;
#if DEL_MULT == 256
					int16_t diff = v-(tact>>8) ;
#else
          int16_t diff = v-tact/DEL_MULT;
#endif
					if ( ( diff > 10 ) || ( diff < -10 ) )
					{
						if ( my_delay == 0 )
						{
       				if (md->delayUp || md->delayDown)  // there are delay values
							{								
								swTog = 1 ;
							}
						}
					}
					else
					{
						my_delay = 0 ;							
					}

          if(swTog)
					{
            //need to know which "v" will give "anas".
            //curves(v)*weight/100 -> anas
            // v * weight / 100 = anas => anas*100/weight = v
            if(md->mltpx==MLTPX_REP)
            {
              tact = (int32_t)anas[md->destCh-1+CHOUT_BASE]*DEL_MULT * 100;
              if(mixweight) tact /= mixweight ;
            }
            diff = v-tact/DEL_MULT;
            if(diff) my_delay = (diff<0 ? md->delayUp :  md->delayDown) * 10 ;
          }

          if(my_delay > 0)
					{ // perform delay
            if(MixTick10ms)
            {
              my_delay -= 1 ;
						}
            if ( my_delay != 0)
           	{ // At end of delay, use new V and diff
#if DEL_MULT == 256
             	v = tact >> 8 ;	   // Stay in old position until delay over
#else
              v = tact/DEL_MULT;   // Stay in old position until delay over
#endif
              diff = 0;
 	          }
						else
						{
							my_delay = -1 ;
						}
          }

					sDelay[i] = my_delay ;

          if(diff && (md->speedUp || md->speedDown))
					{
            //rate = steps/sec => 32*1024/100*md->speedUp/Down
            //act[i] += diff>0 ? (32768)/((int16_t)100*md->speedUp) : -(32768)/((int16_t)100*md->speedDown);
            //-100..100 => 32768 ->  100*83886/256 = 32768,   For MAX we divide by 2 since it's asymmetrical
            if(MixTick10ms)
						{
              int32_t rate = (int32_t)DEL_MULT*2048*100;
              if(mixweight) rate /= abs(mixweight);
							int16_t speed ;
              if ( diff>0 )
							{
								speed = md->speedUp ;
							}
							else
							{
								rate = -rate ;											
								speed = md->speedDown ;
							}
							tact = (speed) ? tact+(rate)/((int16_t)10*speed) : (int32_t)v*DEL_MULT ;

            }
						{
#if DEL_MULT == 256
							int32_t tmp = tact>>8 ;
#else
							int32_t tmp = tact/DEL_MULT ;
#endif
             	if(((diff>0) && (v<tmp)) || ((diff<0) && (v>tmp))) tact=(int32_t)v*DEL_MULT; //deal with overflow
            }
#if DEL_MULT == 256
            v = tact >> 8 ;
#else
            v = tact/DEL_MULT;
#endif
          }
          else if (diff)
          {
            tact=(int32_t)v*DEL_MULT;
          }
					act[i] = tact ;
        }
			}
			else
			{
				act[i] = (int32_t)v*DEL_MULT ;
			}

      //========== CURVES ===============
			if ( md->differential )
			{
      //========== DIFFERENTIAL or expo =========
				if ( md->differential == 2 )	// expo
				{
     			v = expo( v, md->curve ) ;
				}
				else
				{
					int8_t curveParam = REG100_100( md->curve ) ;
     			if (curveParam > 0 && v < 0)
					{
     			  v = (v * (100 - curveParam)) / 100 ;
					}
     			else if (curveParam < 0 && v > 0)
					{
     			  v = (v * (100 + curveParam)) / 100 ;
					}
				}
			}
			else
			{
        switch(md->curve)
				{
	        case 0:
        	break ;
	        case 1:
        		if(md->srcRaw == MIX_FULL) //FUL
        		{
       		    if( v<0 )
							{
								v=-RESX ;   //x|x>0
							}
        		  else
							{
								v=-RESX+2*v ;
							}
        		}
						else
						{
        		  if( v<0 )
							{
								v=0 ;   //x|x>0
							}
        		}
        	break;
        	case 2:
        		if(md->srcRaw == MIX_FULL) //FUL
        		{
        		    if( v>0 ) v=RESX;   //x|x<0
        		    else      v=RESX+2*v;
        		}else{
        		    if( v>0 ) v=0;   //x|x<0
        		}
        	break;
	        case 3:       // x|abs(x)
  	      		v = abs(v);
    	    	break;
      	  case 4:       //f|f>0
        		v = v>0 ? RESX : 0;
        	break;
	        case 5:       //f|f<0
  	      		v = v<0 ? -RESX : 0;
    	    	break;
      	  case 6:       //f|abs(f)
   		  	  v = v>0 ? RESX : -RESX;
 		    	break;
     			default: //c1..c16
						{
							int8_t idx = md->curve ;
							if ( idx < 0 )
							{
								v = -v ;
								idx = 6 - idx ;								
							}
        			v = intpol(v, idx - 7);
						}
				}
			}

      //========== TRIM ===============
      if((md->carryTrim==0) && (md->srcRaw>0) && (md->srcRaw<=4))
			{
				int32_t trim = trimA[md->srcRaw-1] ;
				v += trim ;  //  0 = Trim ON  =  Default
				TrimInUse[md->srcRaw-1] |= 1 ;
			}

			act[i] = (int32_t)v*DEL_MULT ;

//        //========== MULTIPLEX ===============
      int32_t dv = (int32_t)v*mixweight ;
				
      //========== lateOffset ===============
//			if ( md->lateOffset )
//      {
			if(mixoffset) dv += calc100toRESX( mixoffset ) * 100 ;
//      }

			int32_t *ptr ;			// Save calculating address several times
			ptr = &chans[md->destCh-1] ;


      switch((uint8_t)md->mltpx)
			{
	      case MLTPX_REP:
        	*ptr = dv;
       	break ;
  	    case MLTPX_MUL:
					dv /= 100 ;
					dv *= *ptr ;
        	dv /= RESXl ;
        	*ptr = dv ;
       	break ;
    	  default:  // MLTPX_ADD
        *ptr += dv ; //Mixer output add up to the line (dv + (dv>0 ? 100/2 : -100/2))/(100);
       	break ;
      }

		}

		ThrottleStickyOn = 0 ;
    //========== LIMITS ===============
    for(uint32_t i=0;i<NUM_SKYCHNOUT;i++)
		{
//        // chans[i] holds data from mixer.   chans[i] = v*weight => 1024*100
//        // later we multiply by the limit (up to 100) and then we need to normalize
//        // at the end chans[i] = chans[i]/100 =>  -1024..1024
//        // interpolate value with min/max so we get smooth motion from center to stop
//        // this limits based on v original values and min=-1024, max=1024  RESX=1024

        int32_t q = chans[i] ;// + (int32_t)g_model.limitData[i].offset*100; // offset before limit

    	  chans[i] = q / 100 ; // chans back to -1024..1024
        
				ex_chans[i] = chans[i] ; //for getswitch

        LimitData *limit = &g_model.limitData[i] ;
				
				int16_t ofs = limit->offset;
				int16_t xofs = ofs ;
				if ( xofs > g_model.sub_trim_limit )
				{
					xofs = g_model.sub_trim_limit ;
				}
				else if ( xofs < -g_model.sub_trim_limit )
				{
					xofs = -g_model.sub_trim_limit ;
				}
        int16_t lim_p = 10*(limit->max+100) + xofs ;
        int16_t lim_n = 10*(limit->min-100) + xofs ; //multiply by 10 to get same range as ofs (-1000..1000)
				if ( lim_p > 1250 )
				{
					lim_p = 1250 ;
				}
				if ( lim_n < -1250 )
				{
					lim_n = -1250 ;
				}
        if(ofs>lim_p) ofs = lim_p;
        if(ofs<lim_n) ofs = lim_n;

        if(q) q = (q>0) ?
                    q*((int32_t)lim_p-ofs)/100000 :
                    -q*((int32_t)lim_n-ofs)/100000 ; //div by 100000 -> output = -1024..1024

        q += calc1000toRESX(ofs);
        lim_p = calc1000toRESX(lim_p);
        lim_n = calc1000toRESX(lim_n);
        if(q>lim_p) q = lim_p;
        if(q<lim_n) q = lim_n;
        if(limit->revert) q=-q;// finally do the reverse.

				{
					uint8_t numSafety = NUM_SKYCHNOUT ;
					if ( i < numSafety )
					{
        		if(g_model.safetySw[i].swtch)  //if safety sw available for channel check and replace val if needed
						{
//							if ( ( g_model.safetySw[i].mode != 1 ) && ( g_model.safetySw[i].mode != 2 ) )	// And not used as an alarm
							{
								static uint32_t sticky = 0 ;
								uint8_t applySafety = 0 ;
								int8_t sSwitch = g_model.safetySw[i].swtch ;
								
								if(getSwitch00( sSwitch))
								{
									applySafety = 1 ;
								}

								if ( g_model.safetySw[i].mode == 1 )
								{
									int8_t thr = g_model.safetySw[i].source ;
									uint32_t rev_thr = 0 ;
									if ( thr == 0 )
									{
										thr = 2 ;
									}
									else
									{
										if ( thr > 0 )
										{
											thr += 3 ;
										}
										else
										{
											rev_thr = 1 ;
											thr = -thr + 3 ;
										}
									}	
									// Special case, sticky throttle
									if( applySafety )
									{
										sticky &= ~(1<<i) ;
									}
									else
									{
										uint32_t throttleOK = 0 ;
										if ( g_model.throttleIdle )
										{
											if ( abs( CalibratedStick[thr] ) < 20 )
											{
												throttleOK = 1 ;
											}
										}
										else
										{
											if ( rev_thr )
											{
  											if(CalibratedStick[thr] > 1004)
  											{
													throttleOK = 1 ;
  											}
											}
											else
											{
  											if(CalibratedStick[thr] < -1004)
  											{
													throttleOK = 1 ;
  											}
											}
										}
										
										if ( throttleOK )
										{
											if ( trainerThrottleValid )
											{
												if ( trainerThrottleValue < -1004 )
												{
													sticky |= (1<<i) ;
												}
											}	
											else
											{
												sticky |= (1<<i) ;
											}
										}
									}
									if ( ( sticky & (1<<i) ) == 0 )
									{
										applySafety = 1 ;
									}
									ThrottleStickyOn = applySafety ;
								}
								if ( applySafety )
								{
									q = calc100toRESX(g_model.safetySw[i].val) ;
									q += (q>=0) ? g_model.safetySw[i].tune : -g_model.safetySw[i].tune ;
								}
							}
						}
					}
				}

//				if ( i < 5 )
				{
					chanOut[i] = q ; //copy consistent word to int-level
				}
//				g_chans512[i] = q / 100 ; //copy consistent word to int-level
		}
	MixTick10ms = 0 ;
}

/*
Source offsets:
0-3 sticks
4-8 pots/sliders
9 half
10 full
11-13 CYC1-3
14-29 PPM1-16
30-61 Channels
62-70 Physical switches
71-94 Logical switches
95-101 GVARS
102 THIS
103-110 Scalers
111-114 Trims
*/

