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
#include "erskyTx.h"
#include "myeeprom.h"
#include "logicIo.h"

extern EE_X20General g_eeGeneral ;

#define DIM(arr) (sizeof((arr))/sizeof((arr)[0]))

Key keys[NUM_BUTTONS+NUM_TRIMS] ;

static uint8_t s_evt[2] ;
static uint8_t s_evtCount ;
uint8_t Tevent ;

void putEvent( register uint8_t evt)
{
	if (s_evtCount == 0)
	{
  	s_evt[0] = evt ;
		s_evtCount = 1 ;
	}
	else
	{
  	s_evt[1] = evt ;
		s_evtCount = 2 ;
	}
}

uint8_t getEvent()
{
  uint8_t evt = s_evt[0] ;
	s_evt[0] = 0 ;
	Tevent = evt ;
	if ( s_evtCount )
	{
		s_evtCount -= 1 ;
		s_evt[0] = s_evt[1] ;
		s_evt[1] = 0 ;
	}
  return evt ;
}

uint8_t peekEvent()
{
	return s_evt[0] ;
}


uint32_t readButtons()
{
	uint32_t bits ;
	bits = (~AwBits & 0x70000) >> 16 ;
	return bits ;
}

uint32_t readTrims()
{
	uint32_t bits ;
	uint32_t xbits ;
	bits = (~AwBits & 0xF00000) >> 20 ;
	xbits = ( bits & 0x0A ) | ( ( bits >> 2 ) & 1 ) ;
	bits <<= 2 ;
	return (bits & 0x04) | xbits ;
}

void buttons10ms()
{
	uint32_t data ;
	uint32_t i ;
	
	data = readButtons() ;
  for( i = 0 ; i < NUM_BUTTONS ; i += 1 )
	{
		uint32_t value = data & (1<<i) ;
    keys[i].input(value,(EnumKeys)i) ;
	}
	data = readTrims() ;
  for( i = 0 ; i < NUM_TRIMS ; i += 1 )
	{
		uint32_t value = data & (1<<i) ;
    keys[i+TRM_BASE].input(value,(EnumKeys)(uint8_t)(i+TRM_BASE)) ;
	}
}

void Key::input(bool val, EnumKeys enuk)
{
  //  uint8_t old=m_vals;
  m_vals <<= 1;  if(val) m_vals |= 1 ; //portbit einschieben
  m_cnt += 1 ;

  if(m_state && m_vals==0)
	{  //gerade eben sprung auf 0
    if(m_state!=KSTATE_KILLED)
		{
      putEvent(EVT_KEY_BREAK(enuk)) ;
#ifdef KSTATE_RPTDELAY
      if(!( m_state == KSTATE_RPTDELAY && m_cnt<20))
			{
#else
      if(!( m_state == 16 && m_cnt<16))
			{
#endif
        m_dblcnt=0;
      }
        //      }
    }
    m_cnt = 0 ;
    m_state = KSTATE_OFF ;
  }
  switch(m_state)
	{
    case KSTATE_OFF:
      if(m_vals==FFVAL)
			{ //gerade eben sprung auf ff
        m_state = KSTATE_START ;
        if(m_cnt>20) m_dblcnt=0 ; //pause zu lang fuer double
        m_cnt = 0 ;
      }
			else
			{
				if( m_vals == 0 )
				{
	        if(m_cnt>30) m_dblcnt=0 ; //pause zu lang fuer double
				}
			}
      break ;
      //fallthrough
    case KSTATE_START:
      putEvent(EVT_KEY_FIRST(enuk)) ;
      m_dblcnt += 1 ;
#ifdef KSTATE_RPTDELAY
      m_state   = KSTATE_RPTDELAY ;
#else
      m_state   = 16 ;
#endif
      m_cnt     = 0 ;
      break;
#ifdef KSTATE_RPTDELAY
    case KSTATE_RPTDELAY: // gruvin: longer delay before first key repeat
      if(m_cnt == 38) putEvent(EVT_KEY_LONG(enuk)) ; // need to catch this inside RPTDELAY time
      if (m_cnt == 40)
			{
        m_state = 16 ;
        m_cnt = 0 ;
      }
      break ;
#endif
    case 16:
#ifndef KSTATE_RPTDELAY
      if(m_cnt == 38) putEvent(EVT_KEY_LONG(enuk)) ;
      //fallthrough
#endif
    case 8:
    case 4:
    case 2:
      if(m_cnt >= 48)
			{ //3 6 12 24 48 pulses in every 480ms
        m_state >>= 1 ;
        m_cnt     = 0 ;
      }
      //fallthrough
    case 1:
      if( (m_cnt & (m_state-1)) == 0)  putEvent(EVT_KEY_REPT(enuk)) ;
      break ;

    case KSTATE_PAUSE: //pause
      if(m_cnt >= 64)
			{
        m_state = 8 ;
        m_cnt   = 0 ;
      }
      break ;

    case KSTATE_KILLED: //killed
      break ;
  }
}

void pauseEvents(uint8_t event)
{
  event = event & EVT_KEY_MASK ;

	// Add in discarding events in the queue
	// Possibly only repeat events
  if(event < (int)DIM(keys))
	{
		keys[event].pauseEvents() ;
		if ( s_evtCount == 2 )
		{
			if ( ( s_evt[1] ) == ( event | _MSK_KEY_REPT ) )
			{
				s_evtCount = 1 ;
				s_evt[1] = 0 ;
			}
		}
		if ( s_evtCount )
		{
			if ( ( s_evt[0] ) == ( event | _MSK_KEY_REPT ) )
			{
				s_evtCount -= 1 ;
				s_evt[0] = s_evt[1] ;
				s_evt[1] = 0 ;
			}
		}
	}
}

void killEvents(uint8_t event)
{
  event = event & EVT_KEY_MASK ;
  if(event < (int)DIM(keys))
	{
		keys[event].killEvents() ;
		if ( s_evtCount == 2 )
		{
			if ( ( s_evt[1] & EVT_KEY_MASK ) == event )
			{
				s_evtCount = 1 ;
				s_evt[1] = 0 ;
			}
		}
		if ( s_evtCount )
		{
			if ( ( s_evt[0] & EVT_KEY_MASK ) == event )
			{
				s_evtCount -= 1 ;
				s_evt[0] = s_evt[1] ;
				s_evt[1] = 0 ;
			}
		}
	}
}

uint8_t getEventDbl(uint8_t event)
{
  event = event & EVT_KEY_MASK ;
  if(event < (int)DIM(keys))  return keys[event].getDbl() ;
  return 0;
}


uint32_t keyState(EnumKeys enuk)
{

  if (enuk < (int) DIM(keys)) return keys[enuk].state() ? 1 : 0;

	return hwKeyState( (uint8_t) enuk -SW_BASE + 1 ) ;
}

uint32_t hwKeyState( uint8_t key )
{
  uint32_t xxx = 0 ;
extern uint32_t AwBits ;
	
	uint32_t data = ~AwBits ;
	
	if( key > HSW_MAX )  return 0 ;

	if ( ( key >= HSW_Etrmdn ) && ( key <= HSW_Ttrmup ) )
	{
		uint8_t ct = g_eeGeneral.crosstrim + ( g_eeGeneral.xcrosstrim << 1 ) ;
		key -= HSW_Etrmdn ;		// 0 - 7
		if (key >= 2 && key <= 5)       // swap back LH/RH trims
		{
			if ( g_eeGeneral.stickMode & 2 )
			{
				key ^= 0x06 ;
			}
		}
		else
		{
			if ( g_eeGeneral.stickMode & 1 )
			{
				key ^= 0x06 ;
			}
		}
		if ( ct )
		{
			key ^= 0x06 ;
	 		if ( ct == 2 ) // Vintage style crosstrim
			{
	 			if (key >= 2 && key <= 5)       // swap back LH/RH trims
				{
					key ^= 0x06 ;
				}
			}
		}
		key += HSW_Etrmdn ;		// 0 - 7
	}

  switch ( key )
	{
    case HSW_SA2:
      xxx = data & SW_A_L ;
    break ;
    case HSW_SA1:
      xxx = ((data & SW_A_L) | (data & SW_A_H)) == 0 ;
    break ;
    case HSW_SA0:
      xxx = data & SW_A_H ;
    break ;

    case HSW_SB2:
      xxx = data & SW_B_L ;
    break ;
    case HSW_SB1:
      xxx = ((data & SW_B_L) | (data & SW_B_H)) == 0 ;
    break;
    case HSW_SB0:
      xxx = data & SW_B_H ;
    break ;

    case HSW_SC2:
      xxx = data & SW_C_L ;
    break;
    case HSW_SC1:
      xxx = ((data & SW_C_L) | (data & SW_C_H)) == 0 ;
    break;
    case HSW_SC0:
      xxx = data & SW_C_H ;
    break;

    case HSW_SD2:
      xxx = data & SW_D_L ;
    break;
    case HSW_SD1:
      xxx = ((data & SW_D_L) | (data & SW_D_H)) == 0 ;
    break;
    case HSW_SD0:
      xxx = data & SW_D_H ;
    break;
		 

		case HSW_Ttrmup :
			xxx = keyState( (EnumKeys) TRM_LV_UP ) ;
    break ;
	
		case HSW_Ttrmdn :
			xxx = keyState( (EnumKeys) TRM_LV_DWN ) ;
    break ;
	
		case HSW_Rtrmup :
			xxx = keyState( (EnumKeys) TRM_LH_UP ) ;
    break ;
	
		case HSW_Rtrmdn :
			xxx = keyState( (EnumKeys) TRM_LH_DWN ) ;
    break ;
	
		case HSW_Atrmup :
			xxx = keyState( (EnumKeys) TRM_LH_UP ) ;
    break ;
	
		case HSW_Atrmdn :
			xxx = keyState( (EnumKeys) TRM_LH_DWN ) ;
    break ;
	
		case HSW_Etrmup :
			xxx = keyState( (EnumKeys) TRM_LV_UP ) ;
    break ;
	
		case HSW_Etrmdn :
			xxx = keyState( (EnumKeys) TRM_LV_DWN ) ;
    break ;
		
//		case HSW_Pb1 :
//			xxx = readKeyUpgradeBit( 4 ) ;
//    break ;
			 
//		case HSW_Pb2 :
//			xxx = readKeyUpgradeBit( 5 ) ;
//    break ;
 
	}

	if ( xxx )
  {
    return 1 ;
  }
  return 0 ;
}

static const uint8_t SwitchIndices[] = {HSW_SA0,HSW_SB0,HSW_SC0,HSW_SD0} ;

uint32_t switchPosition( uint32_t swtch )
{
	if ( swtch < sizeof(SwitchIndices) )
	{
		swtch = SwitchIndices[swtch] ;
	}
	if ( hwKeyState( swtch ) )
	{
		return 0 ;
	}
	swtch += 1 ;
	if ( hwKeyState( swtch ) )
	{
		return 1 ;			
	}
	return 2 ;
}






