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
#include "pxx2.h"
//#include "telemetry.h"

#define ACCESS_IDLE			0
#define ACCESS_START		1
#define ACCESS_DATA			2
#define ACCESS_CRC1			3
#define ACCESS_CRC2			4

#define START_STOP      0x7E


uint16_t FailsafeCounter[2] ;
struct t_telemetryTx TelemetryTx ;

extern uint8_t PxxSerial[2][50] ;
extern uint8_t *PtrSerialPxx[2] ;
//extern struct t_telemetryTx TelemetryTx ;

uint16_t AccessCrc[2] ;

struct t_moduleSettings ModuleSettings[] ;
struct t_moduleControl ModuleControl[] ;

struct t_accessTelemetry
{
	uint16_t dataCrc ;
	uint16_t startTime ;
	uint8_t dataState ;
	uint8_t dataCount ;
	uint8_t dataReceived ;
	uint8_t AccessPacket[50] ;
} AccessTelemetry[2] ;

extern EE_X20General g_eeGeneral ;

void processSportData( uint8_t *packet, uint32_t receiver ) ;

void IRAM_ATTR pxx2AddByte( uint8_t byte, uint32_t module )
{
  AccessCrc[module] -= byte;
	*PtrSerialPxx[module]++ = byte ;
}


void IRAM_ATTR pxx2AddWord( uint32_t word, uint32_t module )
{
  pxx2AddByte(word, module ) ;
  pxx2AddByte(word >> 8, module ) ;
  pxx2AddByte(word >> 16, module ) ;
  pxx2AddByte(word >> 24, module ) ;
}

// type 0 for ACCESS, 1 for XJTLite
uint32_t IRAM_ATTR pxx2AddFlag0( uint32_t module )
{
	uint8_t byte ;
	struct t_module *pmodule = &g_model.Module[module] ;
	
	byte = pmodule->pxxRxNum & 0x3F ;

  if (pmodule->failsafeMode != FAILSAFE_NOT_SET && pmodule->failsafeMode != FAILSAFE_RX )
	{
    if ( FailsafeCounter[module]-- == 0 )
		{
      byte |= PXX2_CHANNELS_FLAG0_FAILSAFE;
			FailsafeCounter[module] = PXX2_FAILSAFE_RATE ;
		}
	}

	if (BindRangeFlag[module] & PXX_RANGE_CHECK)
	{
    byte |= PXX2_CHANNELS_FLAG0_RANGECHECK ;
  }
	pxx2AddByte	( byte, module ) ;
	return byte ;
}



void setupAccstBindFrame(uint8_t module)
{
	pxx2AddByte( PXX2_TYPE_C_MODULE, module ) ;
	pxx2AddByte( PXX2_TYPE_ID_BIND, module ) ;
	pxx2AddByte(0x01, module) ;
  for (uint32_t i=0; i<PXX2_LEN_RX_NAME; i++)
	{
    pxx2AddByte(0x00, module) ;
  }
	pxx2AddByte((g_model.Module[module].highChannels << 7) + (g_model.Module[module].disableTelemetry << 6), module) ;
  pxx2AddByte(g_model.Module[module].pxxRxNum, module) ;
}

void setupTelemetryFrame(uint8_t module)
{
	uint32_t i ;
	pxx2AddByte( PXX2_TYPE_C_MODULE, module ) ;
	pxx2AddByte( PXX2_TYPE_ID_TELEMETRY, module ) ;
	pxx2AddByte( ( TelemetryTx.AccessSportTx.module_destination >> 8) & 0x03 , module) ;
//	pxx2AddByte( TelemetryTx.AccessSportTx.index, module) ;
	for ( i = 0 ; i < 8 ; i += 1 )
	{
		pxx2AddByte( TelemetryTx.AccessSportTx.data[i], module ) ;
	}
	TelemetryTx.sportCount = 0 ;
}

void IRAM_ATTR addChannels( uint8_t module, uint8_t sendFailsafe, uint8_t firstChannel )
{
  uint16_t pulseValue = 0 ;
  uint16_t pulseValueLow = 0 ;

  for ( uint32_t i = 0 ; i < 8 ; i += 1 )
	{
    uint8_t channel = firstChannel + i ;
    if (sendFailsafe)
		{
      if (g_model.Module[module].failsafeMode == FAILSAFE_HOLD)
			{
        pulseValue = 2047;
      }
      else if (g_model.Module[module].failsafeMode == FAILSAFE_NO_PULSES)
			{
        pulseValue = 0;
      }
      else
			{
        int32_t failsafeValue ;
				if ( channel < 16 )
				{
					failsafeValue = g_model.Module[module].failsafe[channel] ;
				}
				else
				{
//					uint32_t index = module * 8 + channel - 16 ;
					failsafeValue = g_model.accessFailsafe[module][channel - 16] ;
				}
			 
//				if (failsafeValue == FAILSAFE_CHANNEL_HOLD)
//				{
//          pulseValue = 2047;
//        }
//        else if (failsafeValue == FAILSAFE_CHANNEL_NOPULSE)
//				{
//          pulseValue = 0;
//        }
//        else
				{
					failsafeValue = ( failsafeValue *3933 ) >> 9 ;
//					failsafeValue += 1024 ;					
          pulseValue = limit(1, ((int16_t)failsafeValue) + 1024, 2046);
        }
      }
    }
    else
		{
      int value = g_chans512[channel] ; // + 2*PPM_CH_CENTER(channel) - 2*PPM_CENTER;
      pulseValue = limit(1, (value * 3 / 4) + 1024, 2046);
    }

    if (i & 1)
		{
      pxx2AddByte(pulseValueLow, module); // Low byte of channel
      pxx2AddByte(((pulseValueLow >> 8) & 0x0F) | (pulseValue << 4), module);  // 4 bits each from 2 channels
      pxx2AddByte(pulseValue >> 4, module);  // High byte of channel
    }
    else
		{
      pulseValueLow = pulseValue ;
    }
  }
}

// type 0 for ACCESS, 1 for XJTLite
void IRAM_ATTR setupChannelsAccess( uint32_t module, uint32_t type )
{
	uint32_t flag0 ;
	uint8_t flag1 ;

	// For channels
	pxx2AddByte( PXX2_TYPE_C_MODULE, module ) ;
	pxx2AddByte( PXX2_TYPE_ID_CHANNELS, module ) ;

  // flag0 bit 7 range, bit 6 failsafe , rest - RxNum
////	uint8_t flag0 = addFlag0(module);
	flag0 = pxx2AddFlag0( module ) ;

  // flag1

	if ( type )
	{
		flag1 = ((g_model.Module[module].sub_protocol + 1) & 3) << 4 ;
	}
	else
	{	
		flag1 = g_model.Module[module].country << 4 ;
	}

	// For BG-ISRM
	// Bits 7-4 0 for FCC, 1 for EU, 5 for MIC-Japan
//	if ( g_eeGeneral.rfCountry == 2 )
//	{
//		flag1 = 0x50 ;
//	}
//	else
//	{
//		flag1 = g_eeGeneral.rfCountry << 4 ;
//	}
	
	pxx2AddByte( flag1, module ) ;

	addChannels( module, flag0 & PXX2_CHANNELS_FLAG0_FAILSAFE, g_model.Module[module].startChannel ) ;
	

	if ( g_model.Access[module].type || type )
	{
		if ( g_model.Module[module].channels == 0 )
		{
			addChannels( module, flag0 & PXX2_CHANNELS_FLAG0_FAILSAFE, g_model.Module[module].startChannel+8 ) ;
		}
	}
	else
	{
		if ( g_model.Access[module].numChannels > 0 )
		{
			addChannels( module, flag0 & PXX2_CHANNELS_FLAG0_FAILSAFE, g_model.Module[module].startChannel+8 ) ;
		}
		if ( g_model.Access[module].numChannels > 1 )
		{
			addChannels( module, flag0 & PXX2_CHANNELS_FLAG0_FAILSAFE, g_model.Module[module].startChannel+16 ) ;
		}
	}
//	if (size > 0)
//	{
		// update the frame LEN = frame length minus the 2 first bytes
//	}
}



void IRAM_ATTR setupPulsesXjtLite( uint32_t module )
{
	PtrSerialPxx[module] = PxxSerial[module] ;
	AccessCrc[module] = 0xFFFF ;
	*PtrSerialPxx[module]++ = 0x7E ;
	*PtrSerialPxx[module]++ = 0 ;		// Place for length


 	if (BindRangeFlag[module] & PXX_BIND)
	{
		setupAccstBindFrame( module ) ;
	}
	else
	{
		if ( TelemetryTx.sportCount )
		{
			setupTelemetryFrame( module ) ;
		}
		else
		{
			setupChannelsAccess( module, 1 ) ;
		}
	}
 	PxxSerial[module][1] = PtrSerialPxx[module] - PxxSerial[module] - 2;

	*PtrSerialPxx[module]++ = AccessCrc[module] >> 8 ;
	*PtrSerialPxx[module] = AccessCrc[module] ;

	if ( module )
	{
extern volatile uint8_t *PxxTxPtr_x ;
extern volatile uint8_t PxxTxCount_x ;
		PxxTxPtr_x = PxxSerial[EXTERNAL_MODULE] ;
		PxxTxCount_x = PxxSerial[module][1] + 4 ;
		uint32_t sent ;
		sent = Serial2.write( (uint8_t *)PxxTxPtr_x, (size_t)PxxTxCount_x ) ;
	}
	else
	{
extern volatile uint8_t *PxxTxPtr ;
extern volatile uint8_t PxxTxCount ;
		PxxTxPtr = PxxSerial[0] ;
		PxxTxCount = PxxSerial[module][1] + 4 ;
		uint32_t sent ;
		sent = Serial2.write( (uint8_t *)PxxTxPtr, (size_t)PxxTxCount ) ;
	}
}


void setupHardwareInfoFrame( uint32_t module )
{
  if ( ModuleControl[module].step >= -1 && ModuleControl[module].step < 3 ) //  PXX2_MAX_RECEIVERS_PER_MODULE)
	{
    if (ModuleControl[module].timeout == 0)
		{
			pxx2AddByte( PXX2_TYPE_C_MODULE, module ) ;
			pxx2AddByte( PXX2_TYPE_ID_HW_INFO, module ) ;
      
			pxx2AddByte(ModuleControl[module].step, module);
      ModuleControl[module].timeout = 20;
//      ModuleControl[module].step++;
    }
    else
		{
      ModuleControl[module].timeout -= 1 ;
      setupChannelsAccess(module,0) ;
    }
  }
  else
	{
    ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
    setupChannelsAccess(module,0);
  }
}

void pxx2AddRegId( uint32_t module )
{
	uint32_t i ;
	uint8_t *p ;
	p = g_eeGeneral.radioRegistrationID ;
  for ( i = 0 ; i < PXX2_LEN_REGISTRATION_ID ; i += 1 )
	{
    pxx2AddByte( *p++, module ) ;
	}
}

uint8_t CopyRegRxFrame[20] ;
void byteCopy( uint8_t *dest, uint8_t *src, uint32_t length ) ;

void setupRegisterFrame(uint8_t module)
{
//  if ( ModuleControl[module].registerStep == REGISTER_RX_NAME_RECEIVED )
//	{
//    ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
//    setupChannelsAccess(module,0);
//		return ;
//	}
	
	pxx2AddByte( PXX2_TYPE_C_MODULE, module ) ;
	pxx2AddByte( PXX2_TYPE_ID_REGISTER, module ) ;

  if (ModuleControl[module].registerStep == REGISTER_RX_NAME_SELECTED)
	{
		pxx2AddByte(0x01, module) ;
    for (uint8_t i=0; i<PXX2_LEN_RX_NAME; i++)
		{
      pxx2AddByte( ModuleControl[module].registerRxName[i], module) ;
    }
		pxx2AddRegId( module ) ;
    pxx2AddByte( ModuleControl[module].registerModuleIndex, module);
  }
  else
	{
    pxx2AddByte(0, module) ;
  }
  byteCopy( CopyRegRxFrame, PxxSerial[module], 14 ) ;
}

void setupResetFrame(uint8_t module)
{
	pxx2AddByte( PXX2_TYPE_C_MODULE, module ) ;
	pxx2AddByte( PXX2_TYPE_ID_RESET, module ) ;
	pxx2AddByte( ModuleControl[module].bindReceiverIndex, module ) ;
	pxx2AddByte( ModuleControl[module].resetType, module ) ;
}

void setupShareFrame(uint8_t module)
{
	pxx2AddByte( PXX2_TYPE_C_MODULE, module ) ;
	pxx2AddByte( PXX2_TYPE_ID_SHARE, module ) ;
	pxx2AddByte( ModuleControl[module].bindReceiverIndex, module ) ;
}

void setupBindFrame(uint8_t module)
{
  if ( ModuleControl[module].bindStep == BIND_WAIT)
	{
    if ( ( (uint16_t) (get_tmr10ms() - ModuleControl[module].bindWaitTimeout)) > 30 )
		{
      ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
      ModuleControl[module].bindStep = BIND_OK ;
//      POPUP_INFORMATION(STR_BIND_OK);
    }
    return ;
  }
  
	if ( ModuleControl[module].bindStep == BIND_OK)
	{
    setupChannelsAccess(module,0) ;
		return ;
	}

	pxx2AddByte( PXX2_TYPE_C_MODULE, module ) ;
	pxx2AddByte( PXX2_TYPE_ID_BIND, module ) ;

  if ( ModuleControl[module].bindStep == BIND_RX_NAME_SELECTED)
	{
    pxx2AddByte(0x01, module) ;
    for ( uint32_t i=0; i<PXX2_LEN_RX_NAME; i += 1)
		{
      pxx2AddByte( ModuleControl[module].bindReceiversNames[ModuleControl[module].bindReceiverNameIndex][i], module) ;
    }
    pxx2AddByte( ModuleControl[module].bindReceiverId, module); // RX_UID is the slot index (which is unique and never moved)
    pxx2AddByte(g_model.Module[module].pxxRxNum, module);
  }
  else
	{
    pxx2AddByte(0x00, module) ;
		pxx2AddRegId( module ) ;
  }
}

uint8_t SendingRxSettings ;


void setupReceiverSettingsFrame(uint8_t module)
{
  if ( ( (uint16_t) (get_tmr10ms() - ModuleControl[module].receiverSetupTimeout)) > 200 ) /*next try in 2s*/
	{
		pxx2AddByte( PXX2_TYPE_C_MODULE, module ) ;
		pxx2AddByte( PXX2_TYPE_ID_RX_SETTINGS, module ) ;
    uint8_t flag0 = ModuleControl[module].receiverSetupReceiverId ;
//    uint8_t flag0 = 0 ;
    if ( ModuleControl[module].rxtxSetupState == RECEIVER_SETTINGS_WRITE)
		{
      flag0 |= PXX2_RX_SETTINGS_FLAG0_WRITE ;
		}
    pxx2AddByte(flag0, module);
    uint8_t flag1 = 0;
    if (ModuleControl[module].receiverSetupTelemetryDisabled)
      flag1 |= PXX2_RX_SETTINGS_FLAG1_TELEMETRY_DISABLED ;
    if (ModuleControl[module].receiverSetupPwmRate)
      flag1 |= PXX2_RX_SETTINGS_FLAG1_FASTPWM ;
    if (ModuleControl[module].receiverSetupFPort)
      flag1 |= PXX2_RX_SETTINGS_FLAG1_FPORT ;
    if (ModuleControl[module].receiverSetupTelePower)
      flag1 |= PXX2_RX_SETTINGS_FLAG1_TELEMETRY_25MW ;
    if (ModuleControl[module].receiverSetupFPort2)
      flag1 |= PXX2_RX_SETTINGS_FLAG1_FPORT2 ;
    if (ModuleControl[module].receiverSetupCH56pwm)
      flag1 |= PXX2_RX_SETTINGS_FLAG1_ENABLE_PWM_CH5_CH6 ;
    pxx2AddByte(flag1, module);

//    uint8_t channelsCount = sentModuleChannels(module);
//    for (int i = 0; i < channelsCount ; i++)
    for (int i = 0; i < 8 ; i += 1 )
		{
      pxx2AddByte( ModuleControl[module].channelMapping[i], module) ;
    }
    ModuleControl[module].receiverSetupTimeout = get_tmr10ms() ;
		
		
		SendingRxSettings = 1 ;


  }
  else
	{
    setupChannelsAccess(module,0) ;
  }
}

void setupGetPowerFrame(uint8_t module)
{
  uint8_t flag0 = 0 ;
  uint8_t flag1 = 0 ;
  uint8_t power = 1 ;
	pxx2AddByte( PXX2_TYPE_C_MODULE, module ) ;
	pxx2AddByte( PXX2_TYPE_ID_TX_SETTINGS, module ) ;
	if ( ModuleControl[module].moduleRequest == 1 )
	{
		flag0 = 0x40 ;
		// Set flag1:
		// Bit 0 to 1 for telemetry
		flag1 = 1 ;
		if ( ModuleControl[module].moduleExtAerial )
		{
			flag1 |= 0x08 ;
		}
		
		
		
		power = ModuleControl[module].power ;

	}
	if ( g_model.Access[module].type)	// ACCST
	{
		flag1 |= g_model.Access[module].type << 1 ;
		flag1 |= (g_model.Module[module].highChannels << 7) |
				(g_model.Module[module].disableTelemetry << 6) ;
	}

  pxx2AddByte( flag0, module) ;
  pxx2AddByte( flag1, module) ;
  pxx2AddByte( power, module) ;
  ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
}


//void setupSpectrumAnalyser( uint8_t module )
//{
//	if ( ModuleControl[module].step )
//	{
//		return ;
//	}
//	ModuleControl[module].step = 1 ;
	
////  if (moduleSettings[module].counter > 1000) {
////    moduleSettings[module].counter = 1002;
////    return;
////  }

////  moduleSettings[module].counter = 1002;
		 
//	pxx2AddByte( PXX2_TYPE_C_POWER_METER, module ) ;
//	pxx2AddByte( PXX2_TYPE_ID_SPECTRUM, module ) ;

//	pxx2AddByte( 0, module ) ;

//  pxx2AddWord(SharedMemory.SpectrumAnalyser.freq, module ) ;
//  pxx2AddWord(SharedMemory.SpectrumAnalyser.span, module ) ;
//  pxx2AddWord(SharedMemory.SpectrumAnalyser.step, module ) ;
//}

//#ifndef PCBX10
//void setupPulsesXjtLite( uint32_t module )
//{
//	PtrSerialPxx[module] = PxxSerial[module] ;
//	AccessCrc[module] = 0xFFFF ;
//	*PtrSerialPxx[module]++ = 0x7E ;
//	*PtrSerialPxx[module]++ = 0 ;		// Place for length


// 	if (BindRangeFlag[module] & PXX_BIND)
//	{
//		setupAccstBindFrame( module ) ;
//	}
//	else
//	{
//		if ( TelemetryTx.sportCount )
//		{
//			setupTelemetryFrame( module ) ;
//		}
//		else
//		{
//			setupChannelsAccess( module, 1 ) ;
//		}
//	}
// 	PxxSerial[module][1] = PtrSerialPxx[module] - PxxSerial[module] - 2;

//	*PtrSerialPxx[module]++ = AccessCrc[module] >> 8 ;
//	*PtrSerialPxx[module] = AccessCrc[module] ;

//	if ( module )
//	{
//extern volatile uint8_t *PxxTxPtr_x ;
//extern volatile uint8_t PxxTxCount_x ;
//		PxxTxPtr_x = PxxSerial[EXTERNAL_MODULE] ;
//		PxxTxCount_x = PxxSerial[module][1] + 4 ;
//	}
//	else
//	{
//extern volatile uint8_t *PxxTxPtr ;
//extern volatile uint8_t PxxTxCount ;
//		PxxTxPtr = PxxSerial[0] ;
//		PxxTxCount = PxxSerial[module][1] + 4 ;
//	}
//}
//#endif

static uint8_t ModuleConfCount[2] ;
static uint8_t ModuleConfStatus[2] ;
static uint8_t LastProtocolType[2] ;
uint8_t ConfTimeout[2] ;

void startAccess( uint32_t module )
{
	ModuleConfStatus[module] = 0 ;
	ModuleConfCount[module] = 6 ;
	LastProtocolType[module] = g_model.Access[module].type ;
}

void IRAM_ATTR setupPulsesAccess( uint32_t module )
{
	PtrSerialPxx[module] = PxxSerial[module] ;
	AccessCrc[module] = 0xFFFF ;
	// Add header
	*PtrSerialPxx[module]++ = 0x7E ;
	*PtrSerialPxx[module]++ = 0 ;		// Place for length

	if ( g_model.Access[module].type != LastProtocolType[module] )
	{
		startAccess( module ) ;		
	}
	if ( ModuleConfStatus[module] < 2 )
	{
		if ( ModuleConfCount[module] )
		{
			ModuleConfCount[module] -= 1 ;
		}
		if ( ModuleConfCount[module] == 0 )
		{
			if ( ModuleConfStatus[module] == 0 )	// Still getting power etc.
			{
				if ( ++ConfTimeout[module] > 5 )
				{
					ModuleConfStatus[module] = 1 ;
					ConfTimeout[module] = 0 ;
					ModuleControl[module].power = 14 ;
				}
				ModuleControl[module].moduleRequest = 0 ;	// Get
				setupGetPowerFrame(module) ;
				ModuleConfCount[module] = 6 ;
			}
			else if ( ModuleConfStatus[module] == 1 )	// Got response, send SET
			{
				ModuleControl[module].moduleRequest = 1 ;	// Set
				setupGetPowerFrame(module) ;
				ModuleConfStatus[module] = 2 ;
			}
		}
		else
		{
			setupChannelsAccess( module, 0 ) ;
		}
	}
	else
	{
//	SendingRxSettings = 0 ;


//extern uint8_t RawLogging ;
//void rawLogByte( uint8_t byte ) ;

//	if ( TelemetryTx.sportCount )
//	{
//    setupTelemetryFrame(module) ;
//	}
//	else
//	{
  	switch (ModuleSettings[module].mode)
		{
  	  case MODULE_MODE_GET_HARDWARE_INFO:
  	    setupHardwareInfoFrame(module);
	//			if ( RawLogging )
	//			{
	//				rawLogByte( 0x5E ) ;
	//				rawLogByte( 0x01 ) ;
	//				DebugLog = 1 ;
	//			}
  	  break;
  	  case MODULE_MODE_RECEIVER_SETTINGS:
  	    setupReceiverSettingsFrame(module);
////extern uint8_t RawLogging ;
////void rawLogByte( uint8_t byte ) ;
////void rawLogChar( uint8_t byte ) ;
////				if ( RawLogging )
////				{
////					rawLogChar( '-' ) ;
////					rawLogByte( 0x02 ) ;
////					DebugLog = 1 ;
////				}
  	  break;
  	  case MODULE_MODE_REGISTER :
  	    setupRegisterFrame(module) ;
	//			if ( RawLogging )
	//			{
	//				rawLogByte( 0x5E ) ;
	//				rawLogByte( 0x03 ) ;
	//				DebugLog = 1 ;
	//			}
 	  break ;
  	  case MODULE_MODE_BIND:
				if ( g_model.Access[module].type)
				{
					setupAccstBindFrame( module ) ;
				}
				else
				{
  	    	setupBindFrame(module);
				}
//	//			if ( RawLogging )
//	//			{
//	//				rawLogByte( 0x5E ) ;
//	//				rawLogByte( 0x04 ) ;
//	//				DebugLog = 1 ;
//	//			}
  	  break;
			case MODULE_MODE_GETSET_TX :
				setupGetPowerFrame(module) ;
//	//			if ( RawLogging )
//	//			{
//	//				rawLogByte( 0x5E ) ;
//	//				rawLogByte( 0x05 ) ;
//	//				DebugLog = 1 ;
//	//			}
  	  break ;

			case MODULE_MODE_RESET :
				setupResetFrame( module ) ;
//	//			if ( RawLogging )
//	//			{
//	//				rawLogByte( 0x5E ) ;
//	//				rawLogByte( 0x06 ) ;
//	//				DebugLog = 1 ;
//	//			}
  	  break ;

			case MODULE_MODE_SHARE :
				setupShareFrame( module ) ;
  	  break ;
			
//	    case MODULE_MODE_SPECTRUM_ANALYSER:
//	      setupSpectrumAnalyser(module);
//	    break ;

  	  default:
	//			if ( RawLogging )
	//			{
	//				rawLogByte( 0x5E ) ;
	//				rawLogByte( 0x55 ) ;
	//				DebugLog = 1 ;
	//			}
				
				
//      if (outputTelemetryBuffer.isModuleDestination(module)) {
//				if ( TelemetryTx.sportCount && ( TelemetryTx.AccessSportTx.index == 0xFF ) && ( ( ( TelemetryTx.AccessSportTx.module_destination >> 10 ) & 0x03  ) == module ) )
//				{
//					setupTelemetryFrame( module ) ;
//				}
//				else
				{
					setupChannelsAccess( module, 0 ) ;
				}
  	  break;
  	}
	
//	}

//  if (moduleSettings[module].counter-- == 0)
//	{
//    moduleSettings[module].counter = 1000;
//  }
	}
 	PxxSerial[module][1] = PtrSerialPxx[module] - PxxSerial[module] - 2;

	*PtrSerialPxx[module]++ = AccessCrc[module] >> 8 ;
	*PtrSerialPxx[module] = AccessCrc[module] ;

//	if ( RawLogging )
//	{
//		if ( DebugLog )
//		{
//			uint8_t *p ;
//			uint32_t count ;
//			p = PxxSerial[module] ;
//			count = p[1] + 4 ;
//			while ( count-- )
//			{
//				rawLogByte( *p++ ) ;
//			}
//			DebugLog = 0 ;
//		}
//	}

//	if ( SendingRxSettings )
	{
//extern uint8_t RawLogging ;
//void rawLogByte( uint8_t byte ) ;
//void rawLogChar( uint8_t byte ) ;
//	if ( RawLogging )
//	{
//		uint8_t *packet ;
//		packet = PxxSerial[module] ;
//		rawLogChar( '(' ) ;
//		rawLogByte( packet[0] ) ;
//		rawLogByte( packet[1] ) ;
//		rawLogByte( packet[2] ) ;
//		rawLogByte( packet[3] ) ;
//		rawLogByte( packet[4] ) ;
//		rawLogByte( packet[5] ) ;
//		rawLogByte( packet[6] ) ;
//		rawLogByte( packet[7] ) ;
//		rawLogByte( packet[8] ) ;
//		rawLogByte( packet[9] ) ;
//		rawLogByte( packet[10] ) ;
//		rawLogByte( packet[11] ) ;
//		rawLogByte( packet[12] ) ;
//		rawLogByte( packet[13] ) ;
//		rawLogByte( packet[14] ) ;
//		rawLogByte( packet[15] ) ;
//		rawLogByte( packet[16] ) ;
//		rawLogByte( packet[17] ) ;
//		rawLogChar( ')' ) ;
//	}
		
	}

	if ( PxxSerial[module][1] )
	{
		if ( module )
		{

//#ifndef PCBX10
extern volatile uint8_t *PxxTxPtr_x ;
extern volatile uint8_t PxxTxCount_x ;
			PxxTxPtr_x = PxxSerial[EXTERNAL_MODULE] ;
			PxxTxCount_x = PxxSerial[EXTERNAL_MODULE][1] + 4 ;
//			EXTMODULE_USART->CR1 |= USART_CR1_TXEIE ;		// Enable this interrupt
			uint32_t sent ;
			sent = Serial2.write( (uint8_t *)PxxTxPtr_x, (size_t)PxxTxCount_x ) ;
extern uint16_t TelSent ;
			TelSent += sent ;
//#endif
		 
		}
		else
		{
extern volatile uint8_t *PxxTxPtr ;
extern volatile uint8_t PxxTxCount ;
			PxxTxPtr = PxxSerial[0] ;
			PxxTxCount = PxxSerial[module][1] + 4 ;
//			INTMODULE_USART->CR1 |= USART_CR1_TXEIE ;		// Enable this interrupt
			uint32_t sent ;
			sent = Serial2.write( (uint8_t *)PxxTxPtr, (size_t)PxxTxCount ) ;
extern uint16_t TelSent ;
			TelSent += sent ;
		}
	}
}


void byteCopy( uint8_t *dest, uint8_t *src, uint32_t length )
{
	uint32_t i ;
	for ( i = 0 ; i < length ; i += 1 )
	{
		*dest++ = *src++ ;
	}
}

uint32_t byteMatch( uint8_t *dest, uint8_t *src, uint32_t length )
{
	uint32_t i ;
	for ( i = 0 ; i < length ; i += 1 )
	{
		if ( *dest++ != *src++ )
		{
			return 0 ;
		}
	}
	return 1 ;
}




void processRegisterFrame(uint8_t module, uint8_t *frame)
{
  
	
	if ( ModuleSettings[module].mode != MODULE_MODE_REGISTER)
	{
    return ;
  }
  switch(frame[3])
	{
    case 0x00:
      if ( ModuleControl[module].registerStep == REGISTER_START)
			{
        // RX_NAME follows, we store it for the next step
        byteCopy( ModuleControl[module].registerRxName, &frame[4], PXX2_LEN_RX_NAME ) ;
				ModuleControl[module].registerModuleIndex = frame[12] ;
        ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
        ModuleControl[module].registerStep = REGISTER_RX_NAME_RECEIVED ;
      }
    break ;

    case 0x01:
      if ( ModuleControl[module].registerStep == REGISTER_RX_NAME_SELECTED)
			{
        // RX_NAME + PASSWORD follow, we check they are good
        if ( byteMatch( &frame[4], ModuleControl[module].registerRxName, PXX2_LEN_RX_NAME) &&
            byteMatch( &frame[12], g_eeGeneral.radioRegistrationID, PXX2_LEN_REGISTRATION_ID))
				{
          ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
          ModuleControl[module].registerStep = REGISTER_OK;
//          POPUP_INFORMATION(STR_REG_OK);
        }
      }
    break ;
  }
}

	void processBindFrame( uint8_t module, uint8_t *frame )
{
  if ( ModuleSettings[module].mode != MODULE_MODE_BIND )
	{
    return ;
  }


  switch(frame[3])
	{
    case 0 :
      if ( ModuleControl[module].bindStep == BIND_START)
			{
        bool found = false;
        for (uint32_t i=0; i<ModuleControl[module].bindReceiverCount ; i += 1 )
				{
          if (byteMatch(ModuleControl[module].bindReceiversNames[i], &frame[4], PXX2_LEN_RX_NAME))
					{
            found = true ;
            break ;
          }
        }
        if (!found && ModuleControl[module].bindReceiverCount < PXX2_MAX_RECEIVERS_PER_MODULE )
				{
          byteCopy(ModuleControl[module].bindReceiversNames[ModuleControl[module].bindReceiverCount++], &frame[4], PXX2_LEN_RX_NAME ) ;
        }
      }
		break ;

		case 1 :
      if ( ModuleControl[module].bindStep == BIND_RX_NAME_SELECTED)
			{
        if (byteMatch(ModuleControl[module].bindReceiversNames[ModuleControl[module].bindReceiverNameIndex], &frame[4], PXX2_LEN_RX_NAME))
				{
          byteCopy(g_model.Access[module].receiverName[ModuleControl[module].bindReceiverIndex], &frame[4], PXX2_LEN_RX_NAME) ;
					eeDirty(EE_MODEL) ;
          ModuleControl[module].bindStep = BIND_WAIT ;
					ModuleControl[module].bindWaitTimeout = get_tmr10ms() ;
//          ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
        }
      }
		break ;
	}
}

void processResetFrame( uint8_t module, uint8_t *frame )
{
  if ( ModuleSettings[module].mode != MODULE_MODE_RESET )
	{
    return ;
  }
	if ( frame[3] == ModuleControl[module].bindReceiverIndex )
	{
		ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
	}
}

//void processSpectrumAnalyserFrame( uint8_t module, uint8_t *frame)
//{
//  if (ModuleSettings[module].mode != MODULE_MODE_SPECTRUM_ANALYSER)
//	{
//    return ;
//  }

//  uint32_t * frequency = (uint32_t *)&frame[4];
//  int8_t * power = (int8_t *)&frame[8];

//  // center = 2440000000;  // 2440MHz
//  // span = 40000000;  // 40MHz
//  // left = 2440000000 - 20000000
//  // step = 10000

//  int32_t position = *frequency - (SharedMemory.SpectrumAnalyser.freq - SharedMemory.SpectrumAnalyser.span / 2);
//  int32_t x = (position * 128/*LCD_W*/ / 8) / (SharedMemory.SpectrumAnalyser.span / 8);
//  if (x < 128/*LCD_W*/)
//	{
//    SharedMemory.SpectrumAnalyser.bars[x] = 127 + *power ;
//  }
//}



// Packet contains count followed by actual data
void processAccessFrame( uint8_t *packet, uint32_t module )
{
	if ( packet[1] == PXX2_TYPE_C_MODULE )
	{
		switch ( packet[2] )
		{
			case PXX2_TYPE_ID_TELEMETRY :
				processSportData( &packet[4], packet[3] & 0x0F ) ;
			break ;
  	  case PXX2_TYPE_ID_REGISTER :
  	    processRegisterFrame( module, packet ) ;
			break ;
  	  case PXX2_TYPE_ID_HW_INFO :
				if ( packet[3] == 0xFF )
				{
					ModuleControl[module].hwVersion = ( packet[5] << 8 ) | packet[6] ;
					ModuleControl[module].swVersion = ( packet[7] << 8 ) | packet[8] ;
					ModuleControl[module].variant = packet[9] ;
					ModuleControl[module].moduleId = packet[4] ;
				}
				else
				{
					if ( packet[3] == ModuleControl[module].step )
					{
						ModuleControl[module].rxHwVersion = ( packet[5] << 8 ) | packet[6] ;
						ModuleControl[module].rxSwVersion = ( packet[7] << 8 ) | packet[8] ;
						ModuleControl[module].rxVariant = packet[9] ;
						ModuleControl[module].rxModuleId = packet[4] ;
					}
				}
  	    ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
			break ;

			case PXX2_TYPE_ID_BIND :
  	    processBindFrame( module, packet ) ;
			break ;
		
  	  case PXX2_TYPE_ID_TX_SETTINGS :
				ModuleControl[module].power = packet[5] ;
				ModuleControl[module].moduleExtAerial = packet[4] & 0x08 ? 1 : 0 ;
  	    ModuleSettings[module].mode = MODULE_MODE_NORMAL ;
				ModuleControl[module].rxtxSetupState = MODULE_SETTINGS_OK ;
				if ( ModuleConfStatus[module] == 0 )
				{
					ModuleConfStatus[module] = 1 ;
					ModuleConfCount[module] = 0 ;
				}
			break ;

			case PXX2_TYPE_ID_RX_SETTINGS :
				ModuleControl[module].receiverSetupTelemetryDisabled = packet[4] & PXX2_RX_SETTINGS_FLAG1_TELEMETRY_DISABLED ? 1 : 0 ; 
				ModuleControl[module].receiverSetupPwmRate = packet[4] & PXX2_RX_SETTINGS_FLAG1_FASTPWM ? 1 : 0 ;
				ModuleControl[module].receiverSetupTelePower = packet[4] & PXX2_RX_SETTINGS_FLAG1_TELEMETRY_25MW ? 1 : 0 ;
				ModuleControl[module].receiverSetupFPort = packet[4] & PXX2_RX_SETTINGS_FLAG1_FPORT ? 1 : 0 ;
				ModuleControl[module].receiverSetupFPort2 = packet[4] & PXX2_RX_SETTINGS_FLAG1_FPORT2 ? 1 : 0 ;
				ModuleControl[module].receiverSetupCH56pwm = packet[4] & PXX2_RX_SETTINGS_FLAG1_ENABLE_PWM_CH5_CH6 ? 1 : 0 ;
				for ( uint32_t i = 0 ; i < 8 ; i += 1 )
				{
					ModuleControl[module].channelMapping[i] = ( packet[0] > 4 + i ) ? packet[5+i] : 0xFF ;
				}
				ModuleControl[module].rxtxSetupState = RECEIVER_SETTINGS_OK ;
			break ;

  	  case PXX2_TYPE_ID_RESET :
  	    processResetFrame( module, packet ) ;
			break ;

			case PXX2_TYPE_ID_SHARE :
				if ( packet[3] < 3 )
				{
  	     	memset(g_model.Access[module].receiverName[packet[3]], 0, PXX2_LEN_RX_NAME ) ;
				}
			break ;
		}
	}
	else if ( packet[1] == PXX2_TYPE_C_POWER_METER )
	{
		switch ( packet[2] )
		{
			case PXX2_TYPE_ID_SPECTRUM :
//				processSpectrumAnalyserFrame( module, packet ) ;
			break ;
		
		}
	}

}


void accessRecieveByte( uint16_t data, uint32_t module )
{
	uint8_t byte ;
	struct t_accessTelemetry *at ;
	byte = data ;
	data &= 0xFF00 ;
extern uint16_t TelRxCount ;
	TelRxCount += 1 ;
//extern uint8_t RawLogging ;
//void rawLogByte( uint8_t byte ) ;
//	if ( RawLogging )
//	{
//		rawLogByte( byte ) ;
//		if ( byte == START_STOP )
//		{
//			rawLogByte( data >> 8 ) ;
//		}
//	}

	at = &AccessTelemetry[module] ;

//	if ( ( byte == START_STOP ) ) // && ( ( uint16_t)( data - at->startTime ) > 3000 ) )
//	{
//		at->dataState = ACCESS_START ;
//		at->dataCount = 0 ;
//		at->startTime = data ;
//	}
//	else
	{
 		switch (at->dataState) 
		{
			case ACCESS_IDLE :
				if ( byte == START_STOP )
				{
					at->dataState = ACCESS_START ;
					at->dataCount = 0 ;
					at->startTime = data ;
				}
			break ;

			case ACCESS_START :
				at->dataState = ACCESS_DATA ;
				at->dataCount = byte ;
				at->dataReceived = 1 ;
				at->AccessPacket[0] = byte ;
				at->dataCrc = 0xFFFF ;
				if ( byte > 48 )
				{
					at->dataState = ACCESS_IDLE ;
				}
			break ;

			case ACCESS_DATA :
				at->dataCrc -= byte ;
				at->AccessPacket[at->dataReceived++] = byte ;
				if ( at->dataReceived > at->dataCount )
				{
					at->dataState = ACCESS_CRC1 ;
				}
			break ;

			case ACCESS_CRC1 :
				at->dataCrc -= byte << 8 ;
				at->dataState = ACCESS_CRC2 ;
			break ;

			case ACCESS_CRC2 :
				at->dataCrc -= byte ;
				at->dataState = ACCESS_IDLE ;
//				if ( RawLogging )
//				{
//					rawLogByte( at->dataCrc >> 8 ) ;
//					rawLogByte( at->dataCrc ) ;
//				}
				if ( at->dataCrc == 0 )
				{
					processAccessFrame( at->AccessPacket, module ) ;
				}
			break ;

		}
	}
}


const char *moduleName( uint32_t index )
{
	if ( index > 15 )
	{
		index = 0 ;
	}
	return PXX2ModulesNames[index] ;
}


uint32_t sportPacketSend( uint8_t *pdata, uint16_t index )
{
	uint32_t i ;
	uint32_t j ;
	uint32_t crc ;
	uint32_t byte ;
	
	if ( pdata == 0 )	// Test for buffer available
	{
		return TelemetryTx.sportCount ? 0 : 1 ;
	}
	if ( TelemetryTx.sportCount )
	{
		return 0 ;	// Can't send, packet already queued
	}
	crc = 0 ;
	j = 0 ;
	for ( i = 0 ; i < 8 ; i += 1 )
	{
		byte = *pdata++ ;
		if ( i == 7 )
		{
			byte = 0xFF-crc ;
		}
		crc += byte ;
		crc += crc >> 8 ;
		crc &= 0x00FF ;
		if ( ( byte == 0x7E ) || ( byte == 0x7D ) )
		{
			TelemetryTx.SportTx.data[j++] = 0x7D ;
			byte &= ~0x20 ;
		}
		TelemetryTx.SportTx.data[j++] = byte ;
	}
	TelemetryTx.SportTx.ptr = TelemetryTx.SportTx.data ;
	TelemetryTx.SportTx.index = index ;
	TelemetryTx.sportCount = j ;
	return 1 ;
}





