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




#define NUM_BUTTONS			3
#define NUM_TRIMS			4

#define _MSK_KEY_REPT    0x40
#define _MSK_KEY_DBL     0x10
#define IS_KEY_BREAK(key)  (((key)&0xf0)        ==  0x20)
#define EVT_KEY_BREAK(key) ((key)|                  0x20)
#define EVT_KEY_FIRST(key) ((key)|    _MSK_KEY_REPT|0x20)
#define EVT_KEY_REPT(key)  ((key)|    _MSK_KEY_REPT     )
#define EVT_KEY_LONG(key)  ((key)|0x80)
#define EVT_KEY_DBL(key)   ((key)|_MSK_KEY_DBL)
//#define EVT_KEY_DBL(key)   ((key)|0x10)
#define EVT_ENTRY               (0xff - _MSK_KEY_REPT)	// = BF
#define EVT_ENTRY_UP            (0xfe - _MSK_KEY_REPT)	// = BE
#define EVT_TOGGLE_GVAR         (0xfd - _MSK_KEY_REPT)	// = BD
//#define EVT_EXIT	              (0xfc - _MSK_KEY_REPT)	// = BC
#define EVT_KEY_MASK             0x0f

#define TRM_BASE TRM_LH_DWN

#define SW_A_L	0x00002000
#define SW_A_H  0x00001000
#define SW_B_L  0x00008000
#define SW_B_H  0x00004000
#define SW_C_L  0x00000004
#define SW_C_H  0x00000008
#define SW_D_L	0x00000001
#define SW_D_H  0x00000002

enum EnumKeys
{
    KEY_MENU  ,
    KEY_EXIT  ,
    KEY_ENTER ,
    TRM_LH_DWN, // 2
    TRM_LH_UP , // 4
    TRM_LV_DWN, // 1
    TRM_LV_UP ,	// 8
} ;

#define KEY_UP			TRM_LV_UP
#define KEY_DOWN		TRM_LV_DWN
#define KEY_LEFT		TRM_LH_DWN
#define KEY_RIGHT		TRM_LH_UP

#define SW_BASE      (TRM_LV_UP+1)

class Key
{
#define FILTERBITS      4
#define FFVAL          ((1<<FILTERBITS)-1)
#define KSTATE_OFF      0
#define KSTATE_RPTDELAY 95 // gruvin: longer dely before key repeating starts
  //#define KSTATE_SHORT   96
#define KSTATE_START   97
#define KSTATE_PAUSE   98
#define KSTATE_KILLED  99
  uint8_t m_vals:FILTERBITS;   // key debounce?  4 = 40ms
  uint8_t m_dblcnt:2;
  uint8_t m_cnt;
  uint8_t m_state;
public:
  void input(bool val, EnumKeys enuk);
  bool state()       { return m_vals==FFVAL;                }
	bool isKilled()    { return m_state == KSTATE_KILLED ;    }
  void pauseEvents() { m_state = KSTATE_PAUSE;  m_cnt   = 0;}
  void killEvents()  { m_state = KSTATE_KILLED; m_dblcnt=0; }
  uint8_t getDbl()   { return m_dblcnt;                     }
};

extern Key keys[NUM_BUTTONS+NUM_TRIMS] ;

extern uint32_t AwBits ;
extern uint8_t Tevent ;

uint8_t getEvent() ;
uint8_t peekEvent() ;
uint8_t getEventDbl(uint8_t event) ;
void pauseEvents(uint8_t event) ;
void killEvents(uint8_t event) ;
uint32_t readButtons() ;
uint32_t hwKeyState( uint8_t key ) ;
uint32_t switchPosition( uint32_t swtch ) ;

uint32_t keyState(EnumKeys enuk) ;
uint32_t hwKeyState( uint8_t key ) ;

void buttons10ms() ;


