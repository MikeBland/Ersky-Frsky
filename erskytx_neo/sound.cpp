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
#include "myeeprom.h"
#include "audio.h"
#include "sound.h"

#include <esp32-hal-timer.h>
#include <driver/dac.h>

#define __disable_irq() XTOS_SET_INTLEVEL(15)
#define __enable_irq() XTOS_SET_INTLEVEL(0)

#define	TONE_MODE_2		1

void start_sound( void ) ;
void start_dactimer( void ) ;
void init_dac( void ) ;
extern "C" void DAC_IRQHandler( void ) ;

void end_sound( void ) ;
void tone_start( register uint32_t time ) ;
void tone_stop( void ) ;
void setVolume( register uint8_t volume ) ;
//void audioDefevent( uint8_t e ) ;

void startULPSound() ;

extern uint32_t Master_frequency ;
extern uint8_t CurrentVolume ;
extern EE_X20General g_eeGeneral ;

struct t_sound_globals Sound_g ;

struct t_VoiceBuffer VoiceBuffer[3] ;

#define SOUND_NONE	0
#define SOUND_TONE	1
#define SOUND_VOICE	2
#define SOUND_STOP	3

uint8_t CurrentVolume ;
uint8_t HoldVolume ;

//uint8_t ActualVolume ;
//uint8_t TargetVolume ;
//int8_t VolumeChanging ;
//uint8_t VolumeDelay ;

struct t_VoiceBuffer *PtrVoiceBuffer[3] ;
volatile uint8_t VoiceCount ;
uint8_t SoundType ;
volatile uint8_t DacIdle ;
volatile uint8_t DacStart ;

volatile uint32_t SoundCount ;
volatile uint8_t *SoundBuffer ;
uint32_t LastBufferIndex ;

extern hw_timer_t *PTg0T0 ;
	 
// Sound routines

void start_sound()
{
	init_dac() ;
	CurrentVolume = g_eeGeneral.volume ;
	setVolume( CurrentVolume ) ;

//	startULPSound() ;

//	timerStart( PTg0T0 ) ;
}

void IRAM_ATTR timer0Interrupt()
{
	uint32_t *p = (uint32_t *)0x3FF48484 ;
	uint32_t x ;
	if ( SoundCount )
	{
//		dac_output_voltage( DAC_CHANNEL_1, *SoundBuffer++ ) ;	// Value 0-255
		x = *p & 0xF807FFFF ;
		x |= *SoundBuffer++ << 19 ;
		*p = x ;

		SoundCount -= 1 ;
	}
	else
	{
		// reached end of voice buffer
		if ( Sound_g.VoiceActive == 1 )
		{
			PtrVoiceBuffer[0]->flags |= VF_SENT ;		// Flag sent
			PtrVoiceBuffer[0] = PtrVoiceBuffer[1] ;
			PtrVoiceBuffer[1] = PtrVoiceBuffer[2] ;
			VoiceCount -= 1 ;
			if ( VoiceCount == 0 )		// Run out of buffers
			{
				AudioVoiceUnderrun = 1 ;		// For debug
				Sound_g.VoiceActive = 2 ;
				timerStop( PTg0T0 ) ;
				DacIdle = 1 ;
				dac_output_voltage( DAC_CHANNEL_1, 128 ) ;
			}
			else
			{
				SoundBuffer = PtrVoiceBuffer[0]->datab ;
				SoundCount = PtrVoiceBuffer[0]->count ;
			}
		}
	}
}

void set_frequency( uint32_t frequency )
{
	if ( frequency == 32000 )
	{
		timerAlarmWrite( PTg0T0, 124, true) ;
	}
	else
	{
		timerAlarmWrite( PTg0T0, 249, true) ;
	}
}


extern void stop_sound()
{
}


// Start TIMER6 at 100000Hz, used for DAC trigger
void start_dactimer()
{
}


//#include <esp32/ulp.h>
//#include <soc/rtc.h>

//unsigned long samplingRate = 32000 ;
//const int opcodeCount = 17 ;
//const int dacTableStart1 = 2048 - 512 ;
////const int dacTableStart1 = 512 ;
////const int dacTableStart2 = dacTableStart1 - 512 ;
////const int totalSampleWords = 2048 - 512 - (opcodeCount + 1) ;
//const int totalSampleWords = 84 ;
//const int totalSamples = totalSampleWords * 2 ;
//const int indexAddress = opcodeCount ;
//const int bufferStart = indexAddress + 1 ;

//void startULPSound()
//{
	
//	// calibrate 8M/256 clock against XTAL, get 8M/256 clock period
////uint32_t rtc_8md256_period = rtc_clk_cal(RTC_CAL_8MD256, 100);
////uint32_t rtc_fast_freq_hz = 1000000ULL * (1 << RTC_CLK_CAL_FRACT) * 256 / rtc_8md256_period;
	
//	uint32_t i = 0 ;
//  //calculate the actual ULP clock
//  unsigned long rtc_8md256_period = rtc_clk_cal(RTC_CAL_8MD256, 100) ;
//  while ( rtc_8md256_period == 0 )
//	{
//    Serial.println("Retry Cal") ; 
//		rtc_8md256_period = rtc_clk_cal(RTC_CAL_8MD256, 1000) ;
//		if ( ++i > 20 )
//		{
//			return ;
//		}
//	}
//	unsigned long rtc_fast_freq_hz = 1000000ULL * (1 << RTC_CLK_CAL_FRACT) * 256 / rtc_8md256_period;

//  int retAddress1 = 13 ;

//  int loopCycles = 84 ;
//  Serial.print("Real RTC clock: ") ;
//  Serial.println(rtc_fast_freq_hz) ;
//  int dt = (rtc_fast_freq_hz / samplingRate) - loopCycles;
//  if(dt < 0)
//    Serial.println("Sampling rate too high") ; 
//  Serial.print("dt: ") ;
//  Serial.println(dt) ;
//const ulp_insn_t mono[] =
//{
//  //reset offset register
//  I_MOVI(R3, 0),
//  //delay to get the right sampling rate
//  I_DELAY(dt),                // 6 + dt
//  //reset sample index
//  I_MOVI(R0, 0),              // 6
//  //write the index back to memory for the main cpu
//  I_ST(R0, R3, indexAddress), // 8
//  //divide index by two since we store two samples in each dword
//  I_RSHI(R2, R0, 1),          // 6
//  //load the samples
//  I_LD(R1, R2, bufferStart),  // 8
//  //get if odd or even sample
//  I_ANDI(R2, R0, 1),          // 6
//  //multiply by 8
//  I_LSHI(R2, R2, 3),          // 6
//  //shift the bits to have the right sample in the lower 8 bits
//  I_RSHR(R1, R1, R2),         // 6
//  //mask the lower 8 bits
//  I_ANDI(R1, R1, 255),        // 6
//  //multiply by 2
//  I_LSHI(R1, R1, 1),          // 6
//  //add start position
//  I_ADDI(R1, R1, dacTableStart1),// 6
//  //jump to the dac opcode
//  I_BXR(R1),                  // 4
//  //here we get back from writing a sample
//  //increment the sample index
//  I_ADDI(R0, R0, 1),          // 6
//  //if reached end of the buffer, jump relative to index reset
//  I_BGE(-13, totalSamples),   // 4
//  //wait to get the right sample rate (2 cycles more to compensate the index reset)
//  I_DELAY((unsigned int)dt + 2),            // 8 + dt
//  //if not, jump absolute to where index is written to memory
//  I_BXI(3)	                  // 4
//// write io and jump back another 12 + 4
//} ;

//  size_t load_addr = 0 ;
//  size_t size = sizeof(mono)/sizeof(ulp_insn_t) ;
//  ulp_process_macros_and_load(load_addr, mono, &size) ;
////  this is how to get the opcodes
////  for(int i = 0; i < size; i++)
////    Serial.println(RTC_SLOW_MEM[i], HEX);
  
//  //create DAC opcode tables
//  for(int i = 0; i < 256; i++)
//  {
//    RTC_SLOW_MEM[dacTableStart1 + i * 2] = 0x1D4C0121 | (i << 10) ; //dac0
//    RTC_SLOW_MEM[dacTableStart1 + 1 + i * 2] = 0x80000000 + retAddress1 * 4 ;
//  }

//  //set all samples to 128 (silence)
//  for(int i = 0; i < totalSampleWords; i++)
//    RTC_SLOW_MEM[bufferStart + i] = 0x8080 ;

//  //start
//  RTC_SLOW_MEM[indexAddress] = 0 ;
//  ulp_run(0) ;
//}


uint16_t Padding ;

//uint32_t SampleIndex ;
//uint32_t SampleStart ;

//void IRAM_ATTR timer0Interrupt()
//{
//  uint32_t currentSample = (RTC_SLOW_MEM[indexAddress] & 0xffff) >> 1 ;
//	uint32_t i ;
	
////	int32_t j ;
////	uint32_t k ;

////	if ( SampleStart == 0 )
////	{
////		SampleStart = 1 ;
////		LastBufferIndex = currentSample + 10 ;
////		if ( LastBufferIndex >= totalSampleWords )
////		{
////			LastBufferIndex -= totalSampleWords ;
////		}
////	}
	
////	i = SampleIndex ;
////	k = 0 ;
	
////	while ( LastBufferIndex != currentSample )
////	{
////		j = XSine16kInt[i*2] ;
////		j /= 6 ;
////		j += 128 ;
////		j &= 0xFF ;
////		j |= j << 8 ;
////		RTC_SLOW_MEM[bufferStart + LastBufferIndex] = j ;
////		if ( ++LastBufferIndex >= totalSampleWords )
////		{
////			LastBufferIndex = 0 ;
////		}
////		if ( ++i >= 16 )
////		{
////			i = 0 ;
////		}
////		if ( ++k > 7 )
////		{
////			break ;
////		}
////	}
////	SampleIndex = i ;
////}

//// 	i = RTC_SLOW_MEM[bufferStart + LastBufferIndex] ;
////	i += 0xF0000000 ;
////	RTC_SLOW_MEM[bufferStart + LastBufferIndex] = i ;
////	SentCount += 1 ;
////	return ;

//	i = 0 ;
//	if ( DacStart )
//	{
//		DacStart = 0 ;
//		LastBufferIndex = currentSample + 10 ;
//		if ( LastBufferIndex >= totalSampleWords )
//		{
//			LastBufferIndex -= totalSampleWords ;
//		}
//	}
//	while ( LastBufferIndex != currentSample )
//	{
//		if ( SoundCount )
//		{
//			uint32_t x ;
//			x = *SoundBuffer++ ;
//			x |= x << 8 ;		// For 16 kHz set 2 samples
//    	RTC_SLOW_MEM[bufferStart + LastBufferIndex] = x ;
//			if ( ++LastBufferIndex >= totalSampleWords )
//			{
//				LastBufferIndex = 0 ;
//			}
//			SoundCount -= 1 ;
//		}
//		else
//		{
//			if ( Padding )
//			{
//	    	RTC_SLOW_MEM[bufferStart + LastBufferIndex] = 0x8080 ;
//				if ( ++LastBufferIndex >= totalSampleWords )
//				{
//					LastBufferIndex = 0 ;
//				}
//				if ( --Padding == 0 )
//				{
//					timerStop( PTg0T0 ) ;
//					return ;
//				}
//			}
//			// reached end of voice buffer
//			else if ( Sound_g.VoiceActive == 1 )
//			{
//				PtrVoiceBuffer[0]->flags |= VF_SENT ;		// Flag sent
//				PtrVoiceBuffer[0] = PtrVoiceBuffer[1] ;
//				PtrVoiceBuffer[1] = PtrVoiceBuffer[2] ;
//				VoiceCount -= 1 ;
//				if ( VoiceCount == 0 )		// Run out of buffers
//				{
//					AudioVoiceUnderrun = 1 ;		// For debug
//					Sound_g.VoiceActive = 2 ;
//					Padding = totalSampleWords ;
//					DacIdle = 1 ;
//				}
//				else
//				{
//					SoundBuffer = PtrVoiceBuffer[0]->datab ;
//					SoundCount = PtrVoiceBuffer[0]->count ;
//				}
//			}
//		}
//		if ( ++i > 16 )
//		{
//			break ;
//		}
//	}
//}



void init_dac()
{
//	Serial.println("InitDac") ;
	
	DacIdle = 1 ;

  pinMode( 25, ANALOG) ;
  pinMode( 26, ANALOG) ;

	dac_output_enable( DAC_CHANNEL_1) ;
	dac_output_voltage( DAC_CHANNEL_1, 128 ) ;	// Value 0-255

	dac_output_enable( DAC_CHANNEL_2) ;
	dac_output_voltage( DAC_CHANNEL_2, 255 ) ;	// Value 0-255
	
	// Keep DAC2 above about 23 to prevent AMP going into shutdown

	// Turn on time is 280mS, allow 300mS

	uint32_t i ;
	// delay to allow volume to settle
	for ( i = 0 ; i < 15 ; i += 1 )
	{
  	vTaskDelay( pdMS_TO_TICKS( 20 ) ) ;
	}

	PTg0T0 = timerBegin( 0, 20, true) ;
	timerAlarmWrite( PTg0T0, 249, true) ;
	timerAttachInterrupt( PTg0T0, timer0Interrupt, true) ;
	timerAlarmEnable( PTg0T0 ) ;
}

uint8_t AudioVoiceUnderrun ;
uint8_t AudioVoiceCountUnderruns ;


void end_sound()
{
}


// Called every 10mS

void sound_5ms()
{
	if ( Sound_g.Tone_ms_timer == 0 )
	{
		if ( Sound_g.VoiceRequest )
		{
			Sound_g.Sound_time = 0 ;						// Remove any pending tone requests
			
			if ( DacIdle )	// All sent
			{
				DacIdle = 0 ;
				DacStart = 1 ;
				// Now we can send the voice file
				Sound_g.VoiceRequest = 0 ;
				Sound_g.VoiceActive = 1 ;
				set_frequency( VoiceBuffer[0].frequency ? VoiceBuffer[0].frequency : 16000 ) ;

				SoundBuffer = VoiceBuffer[0].datab ;
				SoundCount = VoiceBuffer[0].count ;
				timerStart( PTg0T0 ) ;
			}
			return ;
		}
	}
}

uint16_t g_timeAppendVoice ;
uint16_t g_timeAppendMaxVoice ;
uint16_t g_timeAppendtime ;

void startVoice( uint32_t count )		// count of filled in buffers
{
	AudioVoiceUnderrun = 0 ;
	VoiceBuffer[0].flags &= ~VF_SENT ;
	PtrVoiceBuffer[0] = &VoiceBuffer[0] ;
	if ( count > 1 )
	{
		VoiceBuffer[1].flags &= ~VF_SENT ;
		PtrVoiceBuffer[1] = &VoiceBuffer[1] ;
	}
	if ( count > 2 )
	{
		VoiceBuffer[2].flags &= ~VF_SENT ;
		PtrVoiceBuffer[2] = &VoiceBuffer[2] ;
	}
	VoiceCount = count ;
	Sound_g.VoiceRequest = 1 ;

	g_timeAppendVoice = 0 ;
	g_timeAppendMaxVoice = 0 ;
	g_timeAppendtime = get_tmr10ms() ;

}


void endVoice()
{
	if ( Sound_g.VoiceActive == 2 )
	{
		Sound_g.VoiceActive = 0 ;
	}
}

void appendVoice( uint32_t index )		// index of next buffer
{
	VoiceBuffer[index].flags &= ~VF_SENT ;
	__disable_irq() ;
	PtrVoiceBuffer[VoiceCount++] = &VoiceBuffer[index] ;
	__enable_irq() ;
	if ( DacIdle )	// All sent
	{
		DacIdle = 0 ;
		Sound_g.VoiceActive = 1 ;
		SoundBuffer = VoiceBuffer[index].datab ;
		SoundCount = VoiceBuffer[index].count ;
		timerStart( PTg0T0 ) ;
	}
	uint16_t t10ms ;
	uint16_t now ;
	now = get_tmr10ms() ;
	t10ms = now - g_timeAppendtime ;
	g_timeAppendtime = now ;
	if ( t10ms > g_timeAppendMaxVoice )
	{
		g_timeAppendMaxVoice = t10ms ;
	}
	g_timeAppendVoice = t10ms ;
}

static const uint8_t Volume_scale[NUM_VOL_LEVELS] = 
{
//	 0,  3,  5,   8,   12,  17,  24,  33,  45,  57,  70,  85,
//	100, 118, 140, 163, 190, 207, 224, 239, 244, 248, 252, 255 	
	0, 100, 109, 118, 127, 136, 145, 154, 162, 170, 178, 186,
	194, 202, 210, 218, 226, 233, 239, 243, 247, 250, 253, 255 	
} ;

void setVolume( register uint8_t volume )
{
	if ( volume >= NUM_VOL_LEVELS )
	{
		volume = NUM_VOL_LEVELS - 1 ;		
	}
	CurrentVolume = volume ;
	volume = Volume_scale[volume] ;
	dac_output_voltage( DAC_CHANNEL_2, volume ) ;	// Value 0-255
}

void initHaptic()
{
}

void hapticOff()
{
	GPIO.out_w1tc = 0x20 ;
}

// pwmPercent 0-100
void hapticOn( uint32_t pwmPercent )
{
	GPIO.out_w1ts = 0x20 ;
}


