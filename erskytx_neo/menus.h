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

#define ALPHA_NO_NAME		0x80
#define ALPHA_FILENAME	0x40
#define ALPHA_HEX				0x100

// GVAR flags
#define GVAR_100			0x100
#define GVAR_250			0x200

#define YN_NONE	0
#define YN_YES	1
#define YN_NO		2

#define PARAM_OFS   17*FW

#define POPUP_NONE			0
#define POPUP_SELECT		1
#define POPUP_EXIT			2

#define EDIT_DR_SWITCH_EDIT		0x01
#define EDIT_DR_SWITCH_MOMENT	0x02
#define EDIT_DR_SWITCH_FMODE	0x04

#define VOICE_FILE_TYPE_NAME	0
#define VOICE_FILE_TYPE_USER	1
#define VOICE_FILE_TYPE_MUSIC	2
#define VOICE_FILE_TYPE_SYSTEM	3

// Styles
#define TELEM_LABEL				0x01
#define TELEM_UNIT    		0x02
#define TELEM_UNIT_LEFT		0x04
#define TELEM_VALUE_RIGHT	0x08
#define TELEM_NOTIME_UNIT	0x10
#define TELEM_ARDUX_NAME	0x20
#define TELEM_CONSTANT		0x80

#define min( a, b ) ( a<b?a:b )
#define max( a, b ) ( a>b?a:b )

#define MODELTIME	-24
#define RUNTIME		-23
#define FMODE			-22
#define TMOK			-21

#define V_RTC			-20

#define V_SC1			-19
#define V_SC2			-18
#define V_SC3			-17
#define V_SC4			-16
#define V_SC5			-15
#define V_SC6			-14
#define V_SC7			-13
#define V_SC8			-12

#define FR_WATT		-11

#define V_GVAR1		-10
#define V_GVAR2		-9
#define V_GVAR3		-8
#define V_GVAR4		-7
#define V_GVAR5		-6
#define V_GVAR6		-5
#define V_GVAR7		-4

#define BATTERY		-3
#define TIMER1		-2
#define TIMER2		-1

#define TEL_ITEM_A1			0
#define TEL_ITEM_A2			1
#define TEL_ITEM_RSSI		2
#define TEL_ITEM_TSSI		3
#define TEL_ITEM_TIM1		4
#define TEL_ITEM_TIM2		5
#define TEL_ITEM_BALT		6
#define TEL_ITEM_GALT		7
#define TEL_ITEM_GSPD		8
#define TEL_ITEM_T1			9
#define TEL_ITEM_T2			10
#define TEL_ITEM_RPM		11
#define TEL_ITEM_FUEL		12
#define TEL_ITEM_MAH1		13
#define TEL_ITEM_MAH2		14
#define TEL_ITEM_CVLT		15
#define TEL_ITEM_BATT		16
#define TEL_ITEM_AMPS		17
#define TEL_ITEM_MAHC		18
#define TEL_ITEM_CTOT		19
#define TEL_ITEM_FASV		20
#define TEL_ITEM_ACCX		21
#define TEL_ITEM_ACCY		22
#define TEL_ITEM_ACCZ		23
#define TEL_ITEM_VSPD		24
#define TEL_ITEM_GVAR1	25
#define TEL_ITEM_GVAR2	26
#define TEL_ITEM_GVAR3	27
#define TEL_ITEM_GVAR4	28
#define TEL_ITEM_GVAR5	29
#define TEL_ITEM_GVAR6	30
#define TEL_ITEM_GVAR7	31
#define TEL_ITEM_FWATT	32
#define TEL_ITEM_RXV		33
#define TEL_ITEM_GHDG		34
#define TEL_ITEM_A3			35
#define TEL_ITEM_A4			36
#define TEL_ITEM_SC1		37
#define TEL_ITEM_SC2		38
#define TEL_ITEM_SC3		39
#define TEL_ITEM_SC4		40
#define TEL_ITEM_SC5		41
#define TEL_ITEM_SC6		42
#define TEL_ITEM_SC7		43
#define TEL_ITEM_SC8		44
#define TEL_ITEM_RTC		45
#define TEL_ITEM_TMOK		46
#define TEL_ITEM_ASPD		47
#define TEL_ITEM_CELL1	48
#define TEL_ITEM_CELL2	49
#define TEL_ITEM_CELL3	50
#define TEL_ITEM_CELL4	51
#define TEL_ITEM_CELL5	52
#define TEL_ITEM_CELL6	53
#define TEL_ITEM_RBV1		54
#define TEL_ITEM_RBA1		55
#define TEL_ITEM_RBV2		56
#define TEL_ITEM_RBA2		57
#define TEL_ITEM_RBM1		58
#define TEL_ITEM_RBM2		59
#define TEL_ITEM_RBSERVO	60
#define TEL_ITEM_RBSTATE	61
#define TEL_ITEM_CELL7	  62
#define TEL_ITEM_CELL8	  63
#define TEL_ITEM_CELL9	  64
#define TEL_ITEM_CELL10	  65
#define TEL_ITEM_CELL11	  66
#define TEL_ITEM_CELL12	  67
#define TEL_ITEM_CUST1	  68
#define TEL_ITEM_CUST2	  69
#define TEL_ITEM_CUST3	  70
#define TEL_ITEM_CUST4	  71
#define TEL_ITEM_CUST5	  72
#define TEL_ITEM_CUST6	  73
#define TEL_ITEM_CUST7	  74
#define TEL_ITEM_CUST8	  75

// TextHelp types
#define TEXT_TYPE_TELE_SOURCE		0
#define TEXT_TYPE_SW_SOURCE			1
#define TEXT_TYPE_MIX_SOURCE		2
#define TEXT_TYPE_SW_FUNCTION		3

#define TITLE(str)   lcd_putsAtt(0,0,str,INVERS)

struct t_popupData
{
	uint8_t PopupActive ;
	uint8_t	PopupIdx ;
	uint8_t	PopupSel ;
	uint8_t PopupTimer ;
} ;

struct t_alpha
{
	uint8_t AlphaIndex ;
	uint8_t lastSub ;
	uint8_t AlphaLength ;
	uint8_t AlphaChanged ;
	uint8_t *PalphaText ;
	uint8_t *PalphaHeading ;
	uint8_t AlphaHex ;
	uint8_t copyOfText[15] ;
} ;

struct fileControl
{
	uint8_t index ;
	uint32_t nameCount ;
	uint32_t vpos ;
	uint32_t hpos ;
	uint8_t ext[4] ;
} ;

struct t_filelist
{
	char Filenames[8][50] ;
	struct lfs_info info ;
	lfs_dir_t hdir ;
	uint8_t directoryOpen ;
} ;

extern struct t_popupData PopupData ;
extern uint8_t InverseBlink ;
extern uint8_t EditColumns ;
extern uint8_t IlinesCount ;

extern uint8_t TextIndex ;
extern uint8_t TextOption ;
extern uint8_t TextType ;
extern uint8_t TextResult ;
extern char LastItem[8] ;

extern uint8_t s_noHi ;


struct MState2
{
  uint8_t m_posVert;
//  uint8_t m_posHorz;
  void init(){m_posVert=0;};
  uint8_t check(uint8_t event, uint8_t curr, MenuFuncP *menuTab, uint8_t menuTabSize, uint8_t *subTab, uint8_t subTabMax, uint8_t maxrow);
	uint8_t check_columns( uint8_t event, uint8_t maxrow) ;
//  void check_simple(uint8_t event, uint8_t curr, MenuFuncP *menuTab, uint8_t menuTabSize, uint8_t maxrow);
//  void check_submenu_simple(uint8_t event, uint8_t maxrow);
};

extern uint16_t AnalogData[] ;

extern EE_X20General g_eeGeneral ;
extern int16_t CalibratedStick[] ;


extern void doMainScreenGraphics( void ) ;
extern void menuProc0(uint8_t event) ;

extern void actionMainPopup( uint8_t event ) ;

uint8_t yesNoMenuExit( uint8_t event, const char * s ) ;

uint8_t indexProcess( uint8_t event, MState2 *pmstate, uint8_t extra ) ;
uint32_t doPopup( const char *list, uint16_t mask, uint8_t width, uint8_t event ) ;
uint8_t evalOffset(int8_t sub) ;

void displayIndex( const char *strings[], uint8_t extra, uint8_t lines, uint8_t highlight ) ;
int16_t checkIncDec16( int16_t val, int16_t i_min, int16_t i_max, uint8_t i_flags) ;
void menu_lcd_onoff( uint8_t x,uint8_t y, uint8_t value, uint8_t mode ) ;
uint8_t checkIndexed( uint16_t y, const char *s, uint8_t value, uint8_t edit ) ;
uint8_t onoffItem( uint8_t value, uint16_t y, uint8_t condition ) ;
uint8_t offonItem( uint8_t value, uint16_t y, uint8_t condition ) ;
uint8_t onoffMenuItem( uint8_t value, uint16_t y, const char *s, uint8_t condition ) ;
uint8_t offonMenuItem( uint8_t value, uint16_t y, const char *s, uint8_t condition ) ;
void alphaEditName( uint16_t x, uint16_t y, uint8_t *name, uint8_t len, uint16_t type, uint8_t *heading ) ;
void validateName( uint8_t *text, uint32_t length ) ;

void putHwSwitchName(uint16_t x, uint16_t y, uint8_t z, uint8_t att) ;
void putsMomentDrSwitches(uint16_t x,uint16_t y,int16_t idx1,uint8_t att) ;
void putsDrSwitches( uint16_t x, uint16_t y, int16_t idx1, uint8_t att) ;
void putSwitchName(uint8_t x, uint8_t y, int8_t z, uint8_t att) ;
void putsChn( uint16_t x, uint16_t y, uint8_t idx1, uint8_t att) ;
void dispGvar( uint8_t x, uint8_t y, uint8_t gvar, uint8_t attr ) ;
void putsChnOpRaw( uint16_t x, uint16_t y, uint8_t source, uint8_t switchSource, uint8_t output, uint8_t attr ) ;
void putsChnRaw(uint8_t x,uint8_t y,uint8_t idx,uint8_t att) ;

uint8_t locateMappedItem( uint8_t value, uint8_t *options, uint32_t count ) ;
uint8_t checkOutOfOrder( uint8_t value, uint8_t *options, uint32_t count ) ;
void menuTextHelp(uint8_t event) ;
uint32_t checkForMenuEncoderBreak( uint8_t event ) ;
void putsAttIdxTelemItems( uint8_t x, uint8_t y, uint8_t index, uint8_t attr ) ;
void setLastTelemIdx( uint8_t idx ) ;
void setLastIdx( char *s, uint8_t idx ) ;
uint32_t putTxSwr( uint8_t x, uint8_t y, uint8_t attr ) ;
void copyFileName( char *dest, char *source, uint32_t size ) ;
void voice_telem_item( int8_t index ) ;
int16_t get_telemetry_value( int8_t channel ) ;
void voiceMinutes( int16_t value ) ;
int16_t scale_telem_value( int16_t val, uint8_t channel, uint8_t *dplaces ) ;
uint8_t telemItemValid( uint8_t index ) ;
uint8_t putsTelemetryChannel( uint16_t x, uint16_t y, int8_t channel, int16_t val, uint8_t att, uint8_t style ) ;

void checkXyCurve() ;


extern uint8_t Columns ;
extern uint8_t s_editMode ;
extern uint8_t s_editing ;
extern uint8_t g_posHorz ;
extern uint8_t EditType ;
extern uint8_t s_pgOfs ;
extern uint8_t SubmenuIndex ;
extern uint8_t SubMenuFromIndex ;




