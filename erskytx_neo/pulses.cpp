/****************************************************************************
*  Copyright (c) 2022 by Michael Blandford. All rights reserved.
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

//#include <stdint.h>
//#include <stdlib.h>
//#include <string.h>
#include <Arduino.h>

#include "erskyTx.h"
#include "myeeprom.h"

void heartbeatHandler() ;

uint8_t s_current_protocol[2] = { 255, 255 } ;

uint8_t setupPulsesXfire( uint32_t module ) ;
uint8_t PulsesPaused = 0 ;

void initSportUart() ;
void disableSportUart() ;
void disable_pxx( uint32_t module ) ;
void disable_xfire( uint32_t module ) ;
void init_pxx( uint32_t module ) ;
void init_xfire( uint32_t module ) ;
void init_no_pulses( uint32_t module ) ;
void setupPulsesXjtLite( uint32_t module ) ;


void disable_pxx( uint32_t module )
{
	
}

void disable_xfire( uint32_t module )
{
	disableSportUart() ;
}

void init_pxx( uint32_t module )
{
 	pinMode( 4, INPUT ) ;
	attachInterrupt( 4, heartbeatHandler, CHANGE ) ;
}

void init_xfire( uint32_t module )
{
	initSportUart() ;	
}

void init_no_pulses( uint32_t module )
{
	
}






void setupPulses(uint32_t module)
{
	uint32_t requiredprotocol ;
//  heartbeat |= HEART_TIMER_PULSES ;

	if ( module == 0 )
	{
		requiredprotocol = g_model.Module[0].protocol ;
		if ( PulsesPaused )
		{
			requiredprotocol = PROTO_OFF ;
		}
	}
	else
	{
		requiredprotocol = g_model.Module[1].protocol ;
		if ( PulsesPaused )
		{
			requiredprotocol = PROTO_OFF ;
		}
  	if ( s_current_protocol[1] != requiredprotocol )
		{
  	  switch( s_current_protocol[1] )
  	  {	// stop existing protocol hardware
//  	    case PROTO_PPM:
//					disable_ppm(EXTERNAL_MODULE) ;
//  	    break;
  	    case PROTO_PXX:
					disable_pxx(EXTERNAL_MODULE) ;
  	    break;
  	    case PROTO_OFF:
//					disable_no_pulses(EXTERNAL_MODULE) ;
  	    break;
  	    case PROTO_XFIRE:
					disable_xfire(EXTERNAL_MODULE) ;
  	    break;
  	  }
		
  		s_current_protocol[EXTERNAL_MODULE] = requiredprotocol ;
  	  switch(s_current_protocol[EXTERNAL_MODULE])
  	  { // Start new protocol hardware here
//  	    case PROTO_PPM:
//				  setupPulsesPpmAll(EXTERNAL_MODULE) ;
//					init_ppm(EXTERNAL_MODULE) ;
//  	    break;
  	    case PROTO_PXX:
					init_pxx( EXTERNAL_MODULE ) ;
  	    break;
		    case PROTO_OFF:
				default :
					init_no_pulses( EXTERNAL_MODULE ) ;
  	  	break;
  	    case PROTO_XFIRE:
					init_xfire( EXTERNAL_MODULE ) ;
  	  	break;
		  }
		}
		switch(requiredprotocol)
  	{
//		  case PROTO_PPM:
//  	    setupPulsesPpmAll(EXTERNAL_MODULE) ;		// Don't enable interrupts through here
//  	  break;
  		case PROTO_PXX:
  	    setupPulsesXjtLite( EXTERNAL_MODULE ) ;
  	  break;
  	  case PROTO_XFIRE:
				setupPulsesXfire( EXTERNAL_MODULE ) ;
  	  break;
		}
	}
}






