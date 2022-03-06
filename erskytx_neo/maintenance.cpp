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
#include "erskyTx.h"
#include "myeeprom.h"
#include "menus.h"
#include "lcd.h"
#include "logicIo.h"
#include "frskywifi.h"

#include <string.h>

#include <SPIFFS.h>
#include "lfs.h"

extern lfs_t Lfs ;

#ifdef WIFI
#include <WiFi.h>
//#include <ArduinoOTA.h>
#endif

#define KEY_LEFT	TRM_LH_DWN
#define KEY_RIGHT	TRM_LH_UP

#define START_STOP      0x7E
#define BYTESTUFF       0x7D
#define STUFF_MASK      0x20

#define FRSKY_DATA_IDLE			0
#define FRSKY_DATA_START		1
#define FRSKY_DATA_IN_FRAME	2
#define FRSKY_DATA_XOR			3

#define FRSKY_SPORT_PACKET_SIZE		9

#define PRIM_REQ_POWERUP    (0)
#define PRIM_REQ_VERSION    (1)
#define PRIM_CMD_DOWNLOAD   (3)
#define PRIM_DATA_WORD      (4)
#define PRIM_DATA_EOF       (5)

#define PRIM_ACK_POWERUP    (0x80)
#define PRIM_ACK_VERSION    (0x81)
#define PRIM_REQ_DATA_ADDR  (0x82)
#define PRIM_END_DOWNLOAD   (0x83)
#define PRIM_DATA_CRC_ERR   (0x84)

uint8_t SportState ;
#define SPORT_IDLE				0
#define SPORT_START				1
#define SPORT_POWER_ON		2
#define SPORT_VERSION			3
#define SPORT_DATA_START	4
#define SPORT_DATA				5
#define SPORT_DATA_READ		6
#define SPORT_END					7
#define SPORT_FAIL				8
#define SPORT_COMPLETE		9

#ifdef WIFI
#define WIFI_OFF					0
#define WIFI_STARTING			1
#define WIFI_RUNNING			2
#define WIFI_STOPPING			3
#define WIFI_CONNECTING		4
#define WIFI_NEED_REBOOT	5
#endif

struct t_frskHeader
{
	uint8_t name[4] ;
	uint8_t headerVersion ;
	uint8_t firmwareMajor ;
	uint8_t firmwareMinor ;
	uint8_t firmwareRevision ;
	uint32_t size ;
	uint8_t productFamily ;
	uint8_t productId ;
	uint16_t crc ;
} ;

#define FAMILY_INTERNAL_MODULE				0x00
#define FAMILY_EXTERNAL_MODULE      	0x01
#define FAMILY_RECEIVER             	0x02
#define FAMILY_SENSOR               	0x03
#define FAMILY_BLUETOOTH_CHIP       	0x04
#define FAMILY_POWER_MANAGEMENT_UNIT	0x05
#define FAMILY_FLIGHT_CONTROLLER    	0x06

#define MODULE_NONE							0x00
#define MODULE_XJT              0x01
#define MODULE_ISRM             0x02
#define MODULE_ISRM_PRO         0x03
#define MODULE_ISRM_S           0x04
#define MODULE_R9M              0x05
#define MODULE_R9M Lite         0x06
#define MODULE_R9M Lite PRO     0x07
#define MODULE_ISRM_N           0x08
#define MODULE_ISRM_S_X9        0x09
#define MODULE_ISRM_S_X10E      0x0A
#define MODULE_XJT Lite         0x0B
#define MODULE_ISRM_S_X10S      0x0C
#define MODULE_ISRM_X9LiteS     0x0D
#define MODULE_ISRM_BG					0x0F


#define RX_NONE								0x00
#define RX_X8R                0x01
#define RX_RX8R               0x02
#define RX_RX8R_PRO           0x03
#define RX_RX6R               0x04
#define RX_RX4R               0x05
#define RX_G_RX8              0x06
#define RX_G_RX6              0x07
#define RX_X6R                0x08
#define RX_X4R                0x09
#define RX_X4R_SB             0x0A
#define RX_XSR                0x0B
#define RX_XSR_M              0x0C
#define RX_RXSR               0x0D
#define RX_S6R                0x0E
#define RX_S8R                0x0F
#define RX_XM                 0x10
#define RX_XMPLUS             0x11
#define RX_XMR                0x12
#define RX_R9                 0x13
#define RX_R9_SLIM            0x14
#define RX_R9_SLIMPLUS        0x15
#define RX_R9_MINI            0x16
#define RX_R9_MM              0x17
#define RX_R9_STAB_O          0x18
#define RX_R9_MINI_O          0x19
#define RX_R9_MM_O            0x1A
#define RX_R9_SLIM_O          0x1B
#define RX_Archer_X           0x1C
#define RX_R9MX               0x1D
#define RX_R9SX               0X1E


uint8_t *cpystr( uint8_t *dest, uint8_t *source ) ;
void checkAw1() ;
void intRfOn() ;
void intRfOff() ;
uint16_t getTmr2MHz() ;
uint16_t crc16_ccitt( uint8_t *buf, uint32_t len ) ;
uint32_t byteMatch( uint8_t *dest, uint8_t *src, uint32_t length ) ;
void readNetIds() ;
void writeNetIds() ;


static uint8_t SportTimer ;
static uint8_t MrxDataState = FRSKY_DATA_IDLE ;
uint8_t MrxNumBytes ;
uint8_t MaintenanceRxBuffer[20] ;   // Receive buffer. 9 bytes (full packet), worst case 18 bytes with byte-stuffing
uint8_t SportVersion[4] ;
uint8_t SportVerValid ;

uint32_t BytesFlashed ;
uint32_t FirmwareSize ;

extern uint8_t FileData[] ;
File UpdateFile ;
uint32_t XblockCount ;
uint32_t BlockCount ;

static uint8_t TxPacket[40] ;

uint32_t HandshakeAddress ;
uint32_t LastHandshakeAddress ;
uint8_t HandshakeRequest ;

#ifdef WIFI
uint8_t WifiActive ;
uint8_t WifiType ;
uint8_t WiFiMode ;

#define WIFI_MODE_WEB		0
#define WIFI_MODE_FTP		1
#endif

extern uint8_t AlphaEdited ;


void blankTxPacket()
{
	uint32_t i ;
	for ( i = 2 ; i < 36 ; i += 1 )
	{
		TxPacket[i] = 0 ;
	}
}

// Packet has leading 0x7E stripped
void maintenance_receive_packet( uint8_t *packet )
{
	uint32_t addr ;

	uint8_t crc = crc16_ccitt( packet+1, 8 ) ;
	if ( crc != 0 )
	{
		return ;
	}

	if( ( packet[0] == 0x5E) && ( packet[1]==0x50) )
	{
		SportTimer = 0 ;		// stop timer
		switch( packet[2] )
		{
			case PRIM_ACK_POWERUP :
				if ( SportState == SPORT_POWER_ON )
				{
					SportTimer = 2 ;
					SportState = SPORT_VERSION ;
				}
			break ;
        
			case PRIM_ACK_VERSION:
				if ( SportState == SPORT_VERSION )
				{
					SportTimer = 2 ;
					SportState = SPORT_DATA_START ;
					SportVersion[0] = packet[3] ;
					SportVersion[1] = packet[4] ;
					SportVersion[2] = packet[5] ;
					SportVersion[3] = packet[6] ;
					SportVerValid = 1 ;
 				}
			break ;

			case PRIM_REQ_DATA_ADDR :
				HandshakeAddress = packet[3] | (	packet[4] << 8 ) | ( packet[5] << 16 ) | ( packet[6] << 24 ) ;
				HandshakeRequest = 1 ;
//				SportState = SPORT_END ;
			break ;
//			{
//				uint32_t bcount ;
//				bcount = BlockInUse ? XblockCount : BlockCount ;
				
//				if ( BytesFlashed >= FirmwareSize )
//				{
//					// We have finished
//					blankTxPacket() ;
//					TxPacket[0] = 0x50 ;
//					TxPacket[1] = PRIM_DATA_EOF ;
//					SportTimer = 20 ;		// 200 mS
//					writePacket( TxPacket, 0xFF ) ;
//					SportState = SPORT_END ;
//				}
//				else
//				{				
//					if ( bcount )
//					{
//						uint32_t *ptr ;
//						bcount -= 4 ;
//						BytesFlashed += 4 ;
//						addr = *((uint32_t *)(&packet[3])) ;
//						TxPacket[0] = 0x50 ;
//						TxPacket[1] = PRIM_DATA_WORD ;
//        		TxPacket[6] = addr & 0x000000FF ;
//						addr = ( addr & 1023 ) >> 2 ;		// 32 bit word offset into buffer
//						ptr = ( uint32_t *) (BlockInUse ? ExtraFileData : FileData ) ;
//						ptr += addr ;
//						uint32_t *dptr = (uint32_t *)(&TxPacket[2]) ;
//        		*dptr = *ptr ;
//						SportTimer = 10 ;		// 100 mS
//						writePacket( TxPacket, 0xFF ) ;
//					}
//					if ( BlockInUse )
//					{
//						XblockCount = bcount ;
//					}
//					else
//					{
//						BlockCount = bcount ;
//					}
//					if ( bcount == 0 )
//					{
//						SportState = SPORT_DATA_READ ;
//						if ( BlockInUse )
//						{
//							BlockInUse = 0 ;						
//						}
//						else
//						{
//							BlockInUse = 1 ;
//						}
//					}
//				}
//			}
//			break ;

			case PRIM_END_DOWNLOAD :
					SportState = SPORT_COMPLETE ;
			break ;
				
			case PRIM_DATA_CRC_ERR :
					SportState = SPORT_FAIL ;
			break ;
		}
	}
	return ;
}

void maintenanceReceiveByte( uint8_t data )
{
  switch (MrxDataState) 
  {
    case FRSKY_DATA_IDLE:
      if (data == START_STOP)
      {
        MrxNumBytes = 0 ;
        MrxDataState = FRSKY_DATA_START ;
      }
		break ;

    case FRSKY_DATA_START:
      if (data == START_STOP)
			{
        MrxDataState = FRSKY_DATA_IN_FRAME ;
        MrxNumBytes = 0 ;
				break ; // possible 0x7E,0x7E doublet found.
			}
      MrxDataState = FRSKY_DATA_IN_FRAME ;
      if (MrxNumBytes < 19)
	      MaintenanceRxBuffer[MrxNumBytes++] = data ;
      break;

    case FRSKY_DATA_IN_FRAME :
      if (data == BYTESTUFF)
      { 
        MrxDataState = FRSKY_DATA_XOR ;
				break ;
      }
      if (MrxNumBytes < 19)
			{
	      MaintenanceRxBuffer[MrxNumBytes++] = data ;
			}
      break;

    case FRSKY_DATA_XOR :
      MrxDataState = FRSKY_DATA_IN_FRAME ;
      if (MrxNumBytes < 19)
        MaintenanceRxBuffer[MrxNumBytes++] = data ^ STUFF_MASK ;
		break ;

  } // switch
  if (MrxNumBytes >= FRSKY_SPORT_PACKET_SIZE)
	{
		maintenance_receive_packet(MaintenanceRxBuffer) ;
		MrxNumBytes = 0 ;
		MrxDataState = FRSKY_DATA_IDLE ;
	}
}

void pollMrx()
{
	int32_t byte ;
	while ( (byte = Serial2.read() ) != -1 )
	{
		maintenanceReceiveByte( byte ) ;
	}
}

static const uint16_t crc16tab[256]= {
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
	0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
	0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
	0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
	0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
	0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
	0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
	0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
	0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
	0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
	0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
	0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
	0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
	0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
	0x7E97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
	0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
	0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
	0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
	0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
	0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
	0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
	0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77E,0xc71d,0xd73c,
	0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
	0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
	0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
	0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
	0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
	0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
	0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
	0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
	0x6e17,0x7E36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

uint16_t crc16_ccitt( uint8_t *buf, uint32_t len )
{
	uint32_t counter ;
	uint16_t crc = 0 ;
	for( counter = 0 ; counter < len ; counter += 1)
	{
//		crc = (crc<<8) ^ SharedMemory.Mdata.pCrcTable[ ((crc>>8) ^ *buf++ )	&0x00FF] ;
//		crc = (crc<<8) ^ CRC16Table( ((crc>>8) ^ *buf++ )) ;
		crc = (crc<<8) ^ crc16tab[((crc>>8) ^ *buf++ ) & 0x00FF] ;
	}
	return crc ;
}


uint8_t FileNames[7][32] ;
uint32_t FileCount ;
uint8_t InternalFile[32] ;
static uint8_t TxPhyPacket[80] ;

//void fillNamesLfs()
//{
//	int result ;
//	lfs_dir_t my_dir ;
//	struct lfs_info my_info ;
//	uint32_t i ;

//	result = lfs_dir_open( &Lfs, &my_dir, "/TEXT" ) ;
	
//	for ( i = 0 ; i < 7 ; i += 1 )
//	{
//		result = lfs_dir_read( &Lfs, &my_dir, &my_info );
//		if ( result == 0 )
//	  {
//  	  // no more files in the folder
//  	  return ;
//		}
//		FileCount = i + 1 ;
//		cpystr( FileNames[i], (uint8_t *)my_info.name ) ;
//	}
//}


void fillNames()
{
	uint32_t i ;
	FileCount = 0 ;
	FileNames[0][0] = 0 ;
	File dir = SPIFFS.open("/");
	for ( i = 0 ; i < 7 ; i += 1 )
	{
	  File entry = dir.openNextFile() ;
  	if (! entry)
		{
  	  // no more files in the folder
  	  return ;
  	}
		FileCount = i + 1 ;
		cpystr( FileNames[i], (uint8_t *)entry.name() ) ;
	}
}

uint16_t DebugBytesSent ;

void writePacket( uint8_t *buffer, uint8_t phyId )
{
	uint8_t *ptr = TxPhyPacket ;
	uint32_t i ;

	*ptr++ = 0x7E ;
	*ptr++ = phyId ;
	buffer[7+28] = crc16_ccitt( buffer, 7+28 ) ;
	for ( i = 0 ; i < 8+28 ; i += 1 )
	{
    if(  !( ( buffer[i] == 0x7E) || ( buffer[i] ==0x7D ) ) )
    {
      *ptr++ = buffer[i] ;
    }
		else
    {
      *ptr++ = 0x7D ;
      *ptr++ = 0x20 ^ buffer[i] ;
    }
	}
	i = ptr - TxPhyPacket ;		// Length of buffer to send

	DebugBytesSent += Serial2.write( (uint8_t *)TxPhyPacket, (size_t)i ) ;
}

uint32_t FileSize ;
File SportUpdateFile ; 

void menuFlashInternal( uint8_t event )
{
	TITLE("Flashing") ;
	
	pollMrx() ;

	if ( event == EVT_ENTRY )
	{
		SportState = SPORT_START ;
  	SportUpdateFile = SPIFFS.open( (char *)InternalFile, "r") ;
		if ( SportUpdateFile )
		{
			FirmwareSize = SportUpdateFile.size() ;
		}
		else
		{
			SportState = SPORT_FAIL ;
		}
	}

	if ( SportTimer )
	{
		SportTimer -= 1 ;
	}
	switch ( SportState )
	{
		case SPORT_IDLE :
			pollMrx() ;
			lcd_puts_Pleft( 2*FH, "IDLE" ) ;
			SportTimer = 0 ;
		break ;
		
		case SPORT_START :
			pollMrx() ;
			lcd_puts_Pleft( 2*FH, "START" ) ;
			intRfOn() ;		// Request power on
			checkAw1() ;	// Send request to external logic
			SportTimer = 2 ;		// 20 mS
			SportState = SPORT_POWER_ON ;
		break ;
	
		case SPORT_POWER_ON :
			pollMrx() ;
			lcd_puts_Pleft( 2*FH, "POWER ON" ) ;
			if ( SportTimer == 0 )
			{
				blankTxPacket() ;
				TxPacket[0] = 0x50 ;
				TxPacket[1] = PRIM_REQ_POWERUP ;
				SportTimer = 3 ;		// 30 mS
				writePacket( TxPacket, 0xFF ) ;
			}
		break ;
	
		case SPORT_VERSION :
			lcd_puts_Pleft( 2*FH, "VERSION" ) ;
			pollMrx() ;
			if ( SportTimer == 0 )
			{
				blankTxPacket() ;
				TxPacket[0] = 0x50 ;
				TxPacket[1] = PRIM_REQ_VERSION ;
				SportTimer = 20 ;		// 200 mS
				writePacket( TxPacket, 0xFF ) ;
			}
		break ;
	
		case SPORT_DATA_START :
			lcd_puts_Pleft( 2*FH, "DATA START" ) ;
			pollMrx() ;
			blankTxPacket() ;
			TxPacket[0] = 0x50 ;
			TxPacket[1] = PRIM_CMD_DOWNLOAD ;
			BytesFlashed = 0 ;
			LastHandshakeAddress = 0 ; 
			SportTimer = 10 ;		// 200 mS
			SportState = SPORT_DATA ;
			writePacket( TxPacket, 0xFF ) ;
		break ;

		case SPORT_DATA :
		{
			uint32_t i ;
			uint16_t startTime ;
			startTime = getTmr2MHz() ;
			
			lcd_puts_Pleft( 2*FH, "Flashing" ) ;
			while ( (uint16_t)( getTmr2MHz() - startTime ) < 15000 )	// 7.5mS
			{
				pollMrx() ;
				if ( HandshakeRequest )
				{
					HandshakeRequest = 0 ;
					if ( HandshakeAddress == BytesFlashed )
					{
						LastHandshakeAddress = HandshakeAddress ;
						uint32_t amountRead ;
						amountRead = UpdateFile.readBytes( (char *)FileData, 32 ) ;
						if ( amountRead == 0 )
						{
							SportState = SPORT_END ;
							blankTxPacket() ;
							TxPacket[0] = 0x50 ;
							TxPacket[1] = PRIM_DATA_EOF ;
							writePacket( TxPacket, 0xFF ) ;
							SportTimer = 20 ;
							break ;
						}
						BytesFlashed += amountRead ;
						TxPacket[0] = 0x50 ;
						TxPacket[1] = PRIM_DATA_WORD ;
						for ( i = 0 ; i < 32 ; i += 1 )
						{
							TxPacket[i+2] = FileData[i] ;
						}
						TxPacket[34] = HandshakeAddress ;
						writePacket( TxPacket, 0xFF ) ;
					}
					else if ( HandshakeAddress == LastHandshakeAddress )
					{
						writePacket( TxPacket, 0xFF ) ;		// Resend last packet
					}
				}
			}
			
			lcd_hbar( 4, 5*FH, 101, 7, BytesFlashed * 100 / FirmwareSize ) ;
		}
		break ;
		 
		case SPORT_END :
			lcd_puts_Pleft( 2*FH, "DATA END" ) ;
			pollMrx() ;
			if ( --SportTimer == 0 )
			{
				blankTxPacket() ;
				TxPacket[0] = 0x50 ;
				TxPacket[1] = PRIM_DATA_EOF ;
				writePacket( TxPacket, 0xFF ) ;
				SportTimer = 20 ;
			}
			lcd_hbar( 4, 5*FH, 101, 7, BytesFlashed * 100 / FirmwareSize ) ;
		break ;

		case SPORT_COMPLETE :
			lcd_puts_Pleft( 2*FH, "Flashing OK" ) ;
		break ;

		case SPORT_FAIL :
			lcd_puts_Pleft( 2*FH, "Flashing Failed" ) ;
		break ;
	}

	if ( SportVerValid )
	{
		lcd_puts_Pleft( 7*FH, "Ver:" ) ;
		
		lcd_outhex2(30, 7*FH, SportVersion[0] ) ;
		lcd_outhex2(46, 7*FH, SportVersion[1] ) ;
		lcd_outhex2(62, 7*FH, SportVersion[2] ) ;
		lcd_outhex2(78, 7*FH, SportVersion[3] ) ;
	}
	 
	if ( event == EVT_KEY_LONG(KEY_EXIT) )
	{
		if ( SportState != SPORT_DATA )
		{		
			intRfOff() ;	// Request power off
			checkAw1() ;	// Send request to external logic
			UpdateFile.close() ;
			popMenu( true ) ;
		}
	}
}


void menuCheckInternal( uint8_t event )
{
	uint32_t i ;
	TITLE("Confirm File") ;
	static MState2 mstate2;
	mstate2.check_columns(event, 0 ) ;
	
	if ( event == EVT_KEY_BREAK(KEY_MENU) )
	{
		SportState = SPORT_IDLE ;
		UpdateFile = SPIFFS.open( (char *)InternalFile, "r" ) ;
		uint32_t length = strlen( (char *)InternalFile ) ;
		
		if ( byteMatch( &InternalFile[length-3], (uint8_t *)"frsk", 4 ) 
				 || byteMatch( &InternalFile[length-3], (uint8_t *)"FRSK", 4 ) )
		{
			UpdateFile.readBytes( (char *)FileData, 16 ) ;	// Skip .frsk header
			
		}
		pushMenu(menuFlashInternal) ;
	}

	lcd_puts_Pleft( 2*FH, "Flash Int. Mod from" ) ;
	lcd_puts_Pleft( 3*FH, (char *)InternalFile ) ;
}

void menuCheckDelete( uint8_t event )
{
	uint32_t i ;
	TITLE("Confirm File") ;
	static MState2 mstate2;
	mstate2.check_columns(event, 0 ) ;
	
	if ( event == EVT_KEY_BREAK(KEY_MENU) )
	{
		SPIFFS.remove((char *)InternalFile);
		popMenu(false) ;
	}
	lcd_puts_Pleft( 1*FH, "Confirm Delete File" ) ;
	lcd_puts_Pleft( 3*FH, (char *)InternalFile ) ;
}

uint8_t Hpos ;

void menuUpInternal( uint8_t event )
{
	uint32_t i ;
	TITLE("Choose File") ;
	static MState2 mstate2;
	mstate2.check_columns(event, FileCount-1) ;
	uint8_t y = FH ;
	uint8_t sub = mstate2.m_posVert ;
	if ( event == EVT_ENTRY )
	{
		fillNames() ;
		Hpos = 0 ;
	}
	if ( ( event == EVT_KEY_REPT(KEY_RIGHT)) || ( event == EVT_KEY_FIRST(KEY_RIGHT) ) )
	{
		Hpos += 1 ;
	}
	if ( ( event == EVT_KEY_REPT(KEY_LEFT)) || ( event == EVT_KEY_FIRST(KEY_LEFT) ) )
	{
		if ( Hpos )	Hpos -= 1 ;
	}

	if ( event == EVT_KEY_BREAK(KEY_MENU) )
	{
		cpystr( InternalFile, FileNames[sub] ) ;
		pushMenu(menuCheckInternal) ;
	}

	for ( i = 0 ; i < FileCount ; i += 1 )
	{
		lcd_putsn_P( 0, FH+FH*i, (char *) &FileNames[i][Hpos], 21 ) ;
		if ( sub == i )
		{
			lcd_char_inverse( 0, FH+FH*i, 21*FW, 0 ) ;
		}
	}
}

void menuFileDelete( uint8_t event )
{
	uint32_t i ;
	TITLE("Choose File") ;
	static MState2 mstate2;
	mstate2.check_columns(event, FileCount-1) ;
	uint8_t y = FH ;
	uint8_t sub = mstate2.m_posVert ;
	if ( ( event == EVT_ENTRY ) || ( event == EVT_ENTRY_UP ) )
	{
		fillNames() ;
		Hpos = 0 ;
	}
	if ( ( event == EVT_KEY_REPT(KEY_RIGHT)) || ( event == EVT_KEY_FIRST(KEY_RIGHT) ) )
	{
		Hpos += 1 ;
	}
	if ( ( event == EVT_KEY_REPT(KEY_LEFT)) || ( event == EVT_KEY_FIRST(KEY_LEFT) ) )
	{
		if ( Hpos )	Hpos -= 1 ;
	}

	if ( event == EVT_KEY_BREAK(KEY_MENU) )
	{
		cpystr( InternalFile, FileNames[sub] ) ;
		pushMenu(menuCheckDelete) ;
	}

	for ( i = 0 ; i < FileCount ; i += 1 )
	{
		lcd_putsn_P( 0, FH+FH*i, (char *) &FileNames[i][Hpos], 21 ) ;
		if ( sub == i )
		{
			lcd_char_inverse( 0, FH+FH*i, 21*FW, 0 ) ;
		}
	}
}

//lfs_file_t LfsHandle ;
//uint8_t LfsCopyBuffer[64] ;

//void menuFileLfs( uint8_t event )
//{
//	uint32_t length ;
//	char *p ;
//	File dir ;
//	File entry ;
//	int32_t result ;
//	uint32_t amountRead ;
//  int32_t written ;
//	TITLE("Moving Files") ;

//	dir = SPIFFS.open("/") ;
//	do
//	{
//  	entry = dir.openNextFile() ;
// 		if (! entry)
//		{
// 			// no more files in the folder
//			popMenu( true ) ;
// 			return ;
// 		}
//		p = ( char *)cpystr( (uint8_t *)FileNames[0], (uint8_t *)entry.name() ) ;
//		p -= 3 ;
//		if ( *p == 'w' )
//		{
//			if ( *(p+1) == 'a' )
//			{
//				if ( *(p+2) == 'v' )
//				{
//  				lcd_puts_P( 0, 2*FH, "Moving:" ) ;
//					lcd_putsn_P( 0, 4*FH, (char *) FileNames[0], 21 ) ;
//					refreshDisplay() ;
//					UpdateFile = SPIFFS.open( (char *)FileNames[0], "r" ) ;
//					FirmwareSize = UpdateFile.size() ;
//					result = lfs_file_open( &Lfs, &LfsHandle, (char *)FileNames[0], LFS_O_RDWR | LFS_O_CREAT) ;
//					while ( FirmwareSize )
//					{
//						amountRead = UpdateFile.readBytes( (char *)LfsCopyBuffer, 64 ) ;
//						if ( amountRead == 0 )
//						{
//							break ;
//						}
//						written = lfs_file_write( &Lfs, &LfsHandle, (char *)LfsCopyBuffer, amountRead ) ;
//						FirmwareSize -= amountRead ;
//					}
//					UpdateFile.close() ;
//					lfs_file_close(&Lfs, &LfsHandle ) ;
//					SPIFFS.remove((char *)FileNames[0]) ;
//					break ;
//				}
//			}
//		}
//	} while (1) ;
//}


const char* ssid     = "Frsky Neo" ;
const char* password = "12345678" ;

#ifdef WIFI

void frskyWifi() ;
void startFrskyWifi() ;

//WiFiServer server(80) ;
//String header ;
IPAddress IP ;
uint32_t result ;
uint8_t NetId[20] ;
uint8_t NetPass[20] ;

void requestRestart()
{
	WifiActive = WIFI_NEED_REBOOT	;
}

void runWifi()
{
	switch ( WifiActive )
	{
		case WIFI_STARTING :
			if ( WifiType )
			{
				Serial.println("Starting Station") ;
			  WiFi.mode(WIFI_STA) ;
//				Serial.print( (char *)NetId) ;
//				Serial.println( "<" ) ;
//				Serial.print( (char *)NetPass) ;
//				Serial.println( "<" ) ;
			  WiFi.begin((char *)NetId, (char *)NetPass) ;
				WifiActive = WIFI_CONNECTING ;
			}			 
			else
			{
	  		WiFi.mode(WIFI_AP) ;
			  result = WiFi.softAP(ssid, password) ;
//			WiFi.softAPConfig( FIP, FIP, NMask) ;
	  		IP = WiFi.softAPIP() ;
				startFrskyWifi() ;
				WifiActive = WIFI_RUNNING ;
			}
		break ;

		case WIFI_CONNECTING :
			if ( WiFi.status() == WL_CONNECTED)
			{
				Serial.println("Connected(1)") ;
	  		IP = WiFi.localIP() ;
//				Serial.println(IP) ;
				startFrskyWifi() ;
				WifiActive = WIFI_RUNNING ;
			}	
		break ;
		 
		case WIFI_RUNNING	:
//		  server.handleClient();
			frskyWifi() ;
		break ;
		
		case WIFI_STOPPING :
			WiFi.disconnect();
//			WiFi.mode(WIFI_OFF);
//			WiFi.forceSleepBegin();
		break ;
	}
}

void menuNetId( uint8_t event )
{
	TITLE("Net Config") ;
	static MState2 mstate2;
	mstate2.check_columns(event, 2-1) ;
  uint8_t sub = mstate2.m_posVert ;
	uint8_t subN = 0 ;

	if ( event == EVT_ENTRY )
	{
		readNetIds() ;
	}
	
 	lcd_puts_P( 0, 2*FH, "Net Id" ) ;
 	lcd_puts_P( 0, 4*FH, "Password" ) ;
	
	if ( sub == 0 )
	{
		lcd_char_inverse( 0, 2*FH, 6*FW, 0 ) ;
	}
	else
	{
		lcd_char_inverse( 0, 4*FH, 8*FW, 0 ) ;
	}

	uint8_t attr = (sub==subN ? INVERS : 0) | ALPHA_NO_NAME ;
	alphaEditName( 0, 3*FH, NetId, 20, attr, (uint8_t *)"Net Id" ) ;
	subN += 1 ;
	 
	attr = (sub==subN ? INVERS : 0) | ALPHA_NO_NAME ;
	alphaEditName( 0, 5*FH, NetPass, 20, attr, (uint8_t *)"Net Password" ) ;

	if ( AlphaEdited )
	{
		AlphaEdited = 0 ;
		writeNetIds() ;
	}
}

void menuWifi( uint8_t event )
{
	uint32_t i ;
	TITLE("WIFI") ;
	static MState2 mstate2 ;
	
	if ( WifiActive == WIFI_NEED_REBOOT )
	{
  	lcd_puts_P( 0, 2*FH, "Update complete" ) ;
  	lcd_puts_P( 0, 4*FH, "Power off and on" ) ;
		return ;
	}

	mstate2.check_columns(event, 1-1) ;

	if ( event == EVT_ENTRY )
	{
		readNetIds() ;
		WifiActive = WIFI_STARTING ;
	}

	if ( WifiActive )
	{
  	lcd_puts_P( 0, 2*FH, "IP:    .   .   ." ) ;
		
		lcd_outdezNAtt( 7*FW, 2*FH, IP[0], 0, 3 ) ;
		lcd_outdezNAtt( 11*FW, 2*FH, IP[1], 0, 3 ) ;
		lcd_outdezNAtt( 15*FW, 2*FH, IP[2], 0, 3 ) ;
		lcd_outdezNAtt( 19*FW, 2*FH, IP[3], 0, 3 ) ;
	}

	if ( WifiActive == WIFI_CONNECTING )
	{
  	lcd_puts_P( 0, 3*FH, " Connecting" ) ;
	}



extern uint8_t FrskyWifiState ;
	lcd_outdez( 1*FW, 4*FH, FrskyWifiState ) ;

extern uint8_t UploadState ;
extern uint32_t UploadFileLength ;
extern uint32_t BytesWritten ;
extern uint32_t TotalBytesWritten ;

	if ( UploadState )
	{
		lcd_hbar( 4, 6*FH, 101, 7, TotalBytesWritten * 100 / (UploadFileLength-350) ) ;
	}


//	lcd_outhex8( 20, 4*FH, UploadFileSize ) ;

//extern uint8_t RequestLength ;
//extern uint8_t Flushing ;
//extern char Request[] ;
//extern WiFiClient Client ;

//	lcd_outhex2( 0,  6*FH, RequestLength ) ;
//	lcd_outhex2( 20, 6*FH, Request[0] ) ;
//	lcd_outhex2( 40, 6*FH, Flushing ) ;
//	lcd_outhex8( 60, 6*FH, (uint32_t) Client ) ;

//extern uint8_t DebugFileData[] ;

//	lcd_outhex2( 0, 7*FH, DebugFileData[0] ) ;
//	lcd_outhex2( 20, 7*FH, DebugFileData[1] ) ;
//	lcd_outhex2( 40, 7*FH, DebugFileData[2] ) ;
//	lcd_outhex2( 60, 7*FH, DebugFileData[3] ) ;

//extern uint32_t BytesWritten ;
//	lcd_outdezNAtt( 126, 4*FH, BytesWritten, 0, 6 ) ;

//	uint8_t y = FH ;
//	uint8_t sub = mstate2.m_posVert ;
}
#endif

extern uint8_t s_editMode ;

void menuUpdate( uint8_t event )
{
	TITLE("Maintenance") ;
	static MState2 mstate2 ;
#ifdef WIFI
	mstate2.check_columns(event, 6-1) ;
#else
	mstate2.check_columns(event, 2-1) ;
#endif
  uint8_t sub = mstate2.m_posVert ;
	uint8_t subN = 0 ;
	uint8_t attr ;
	uint16_t y = FH ;

#ifdef WIFI
	if ( event == EVT_ENTRY_UP )
	{
		if ( WifiActive )
		{
			WifiActive = WIFI_STOPPING ;
		}
	}

#endif

//  lcd_puts_P( 0, y, "Update Internal" ) ;
//	if ( event == EVT_KEY_BREAK(KEY_MENU) )
//	{
//		s_editMode = 0 ;
//	  if(sub==subN)
//		{
//			pushMenu(menuUpInternal) ;
//		}
//	}
// 	y += FH ;
//	subN += 1 ;
#ifdef WIFI
  lcd_puts_P( 0, y, "Wifi (AP)" ) ;

	if ( event == EVT_KEY_BREAK(KEY_MENU) )
	{
		s_editMode = 0 ;
	  if(sub==subN)
		{
			WifiType = 0 ;
			WiFiMode = WIFI_MODE_WEB ;
			pushMenu(menuWifi) ;
		}
	}
 	y += FH ;
	subN += 1 ;
  lcd_puts_P( 0, y, "Net Id+Pass" ) ;
	if ( event == EVT_KEY_BREAK(KEY_MENU) )
	{
		s_editMode = 0 ;
	  if(sub==subN)
		{
			pushMenu(menuNetId) ;
		}
	}
 	y += FH ;
	subN += 1 ;
  lcd_puts_P( 0, y, "Wifi (Station)" ) ;

	if ( event == EVT_KEY_BREAK(KEY_MENU) )
	{
		s_editMode = 0 ;
	  if(sub==subN)
		{
			WifiType = 1 ;
			WiFiMode = WIFI_MODE_WEB ;
			pushMenu(menuWifi) ;
		}
	}
 	y += FH ;
	subN += 1 ;

  lcd_puts_P( 0, y, "WifiFTP (AP)" ) ;

	if ( event == EVT_KEY_BREAK(KEY_MENU) )
	{
		s_editMode = 0 ;
	  if(sub==subN)
		{
			WifiType = 0 ;
			WiFiMode = WIFI_MODE_FTP ;
			pushMenu(menuWifi) ;
		}
	}
 	y += FH ;
	subN += 1 ;
  lcd_puts_P( 0, y, "WifiFTP (Station)" ) ;

	if ( event == EVT_KEY_BREAK(KEY_MENU) )
	{
		s_editMode = 0 ;
	  if(sub==subN)
		{
			WifiType = 1 ;
			WiFiMode = WIFI_MODE_FTP ;
			pushMenu(menuWifi) ;
		}
	}
 	y += FH ;
	subN += 1 ;
#endif
	lcd_puts_P( 0, y, "Delete File" ) ;
	if ( event == EVT_KEY_BREAK(KEY_MENU) )
	{
		s_editMode = 0 ;
	  if(sub==subN)
		{
			pushMenu(menuFileDelete) ;
		}
	}
// 	y += FH ;
//	subN += 1 ;
//	lcd_puts_P( 0, y, "Update from SPIFFS" ) ;
//	if ( event == EVT_KEY_BREAK(KEY_MENU) )
//	{
//void updateFromSpiffs() ;
//		s_editMode = 0 ;
//	  if(sub==subN)
//		{
//			updateFromSpiffs() ;
//		}
//	}

// 	y += FH ;
//	subN += 1 ;
//	lcd_puts_P( 0, y, "Set LFS from SPIFFS" ) ;
//	if ( event == EVT_KEY_BREAK(KEY_MENU) )
//	{
//		s_editMode = 0 ;
//	  if(sub==subN)
//		{
//			pushMenu(menuFileLfs) ;
//		}
//	}

	lcd_char_inverse( 0, (sub+1)*FH, 18*FW, 0 ) ;
}

#ifdef WIFI
#include "wifi.h"
#endif



