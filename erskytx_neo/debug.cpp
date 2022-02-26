/****************************************************************************
*  Copyright (c) 2011 by Michael Blandford. All rights reserved.
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
****************************************************************************
*  History:
*
****************************************************************************/

#include <Arduino.h>

#include <stdint.h>
#include <stdlib.h>
//#include <ctype.h>
#include <string.h>

#include "lfs.h"

#include <esp32-hal-timer.h>
#include <driver/dac.h>


#define AU_LONG_TONE (35)
void audioDefevent(uint8_t e);



extern lfs_t Lfs ;
lfs_file_t LfsFile ;
extern struct lfs_config Cfg ;

extern uint8_t Activated ;

#define YMODEM 				0

//void uputs( register char *string ) ;
void crlf( void ) ;
void p8hex( uint32_t value ) ;
void p4hex( uint16_t value ) ;
void p2hex( unsigned char c ) ;
void hex_digit_send( unsigned char c ) ;

uint16_t getTmr2MHz() ;

#if YMODEM
#define PACKET_SEQNO_INDEX      (1)
#define PACKET_SEQNO_COMP_INDEX (2)

#define PACKET_HEADER           (3)
#define PACKET_TRAILER          (2)
#define PACKET_OVERHEAD         (PACKET_HEADER + PACKET_TRAILER)
#define PACKET_SIZE             (128)
#define PACKET_1K_SIZE          (1024)

#define FILE_NAME_LENGTH        (256)
#define FILE_SIZE_LENGTH        (16)

#define SOH                     (0x01)  /* start of 128-byte data packet */
#define STX                     (0x02)  /* start of 1024-byte data packet */
#define EOT                     (0x04)  /* end of transmission */
#define ACK                     (0x06)  /* acknowledge */
#define NAK                     (0x15)  /* negative acknowledge */
#define CA                      (0x18)  /* two of these in succession aborts transfer */
#define CRC16                   (0x43)  /* 'C' == 0x43, request 16-bit CRC */

#define ABORT1                  (0x41)  /* 'A' == 0x41, abort by user */
#define ABORT2                  (0x61)  /* 'a' == 0x61, abort by user */

#define NAK_TIMEOUT             (100)	// Units of 2mS 
#define PACKET_TIMEOUT          (25)		// Units of 2mS 
#define MAX_ERRORS              (5)
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int32_t Ymodem_Receive (uint8_t *p ) ;

//uint8_t file_name[FILE_NAME_LENGTH];
//uint32_t FlashDestination = ApplicationAddress; /* Flash user program offset */
//uint16_t PageSize = PAGE_SIZE;
//uint32_t EraseCounter = 0x0;
//uint32_t NbrOfPage = 0;
//FLASH_Status FLASHStatus = FLASH_COMPLETE;
uint32_t RamSource ;
extern uint8_t tab_1024[1024] ;
uint8_t Ymbuffer[10] ;
uint8_t file_name[FILE_NAME_LENGTH];

#endif

uint8_t Tbuffer[128] ;

uint8_t *cpystr( uint8_t *dest, uint8_t *source ) ;

//uint32_t read32_eeprom_data( uint32_t eeAddress, uint8_t *buffer, uint32_t size, uint32_t immediate ) ;

//extern "C"
//{

//void cuputs( register char *string )
//{
//	while ( *string )
//	{
//		Serial.write( *string++ ) ;		
//	}	
//}


//// Send a <cr><lf> combination to the serial port
//void ccrlf()
//{
//	Serial.write( 13 ) ;
//	Serial.write( 10 ) ;
//}

//// Send the 32 bit value to the RS232 port as 8 hex digits
//void cp8hex( uint32_t value )
//{
//	p4hex( value >> 16 ) ;
//	p4hex( value ) ;
//}


//}


#define BEEP_DEFAULT_FREQ  (70)

const int16_t Sine16kInt[32] =
{
	  0, 117, 230, 333, 424, 499, 554, 588,
	600, 588, 554, 499, 424, 333, 230, 117,
	   0,-117,-230,-333,-424,-499,-554,-588,
	-600,-588,-554,-499,-424,-333,-230,-117
} ;

//extern hw_timer_t *PTg0T0 ;


//uint8_t ToneBuffer[4096] ;
//extern uint16_t toneCount ;
//extern uint8_t toneFrequency ;

//void toneFill(uint8_t * buffer)
//{
//	uint32_t i = 0 ;
//	uint32_t y ;
//	uint32_t multiplier ;
//	int32_t value ;
	
//	toneCount = 0 ;
//	toneFrequency = BEEP_DEFAULT_FREQ ;

//	multiplier = 800 ;

//	while ( i < 4096 )
//	{
//		toneCount += toneFrequency ;
//		y = ( toneCount & 0x01F0) >> 4 ;
//		if ( toneFrequency )
//		{
//			value = (int32_t)Sine16kInt[y] ;
//			value *= multiplier ;
//			value += 2048 * 256 ;
//			value >>= 12 ;
//		}
//		else
//		{
//			value = 128 ;
//		}
//		*buffer++ = value ;
//		i += 1 ;
//	}
//}














void txmit( uint8_t c )
{
	Serial.write( c ) ;
}

void uputs( register char *string )
{
	while ( *string )
	{
		Serial.write( *string++ ) ;		
	}	
}


// Send a <cr><lf> combination to the serial port
void crlf()
{
	Serial.write( 13 ) ;
	Serial.write( 10 ) ;
}

// Send the 32 bit value to the RS232 port as 8 hex digits
void p8hex( uint32_t value )
{
	p4hex( value >> 16 ) ;
	p4hex( value ) ;
}

// Send the 16 bit value to the RS232 port as 4 hex digits
void p4hex( uint16_t value )
{
	p2hex( value >> 8 ) ;
	p2hex( value ) ;
}

// Send the 8 bit value to the RS232 port as 2 hex digits
void p2hex( unsigned char c )
{
//	asm("swap %c") ;
	hex_digit_send( c >> 4 ) ;
//	asm("swap %c") ;
	hex_digit_send( c ) ;
}

// Send a single 4 bit value to the RS232 port as a hex digit
void hex_digit_send( unsigned char c )
{
	c &= 0x0F ;
	if ( c > 9 )
	{
		c += 7 ;
	}
	c += '0' ;
	Serial.write( c ) ;
}


void disp_256( uint32_t address, uint32_t lines ) ;
extern void dispw_256( register uint32_t address, register uint32_t lines ) ;

uint32_t Mem_address ;
uint32_t Next_mem_address ;

uint8_t Memaddmode ;
uint8_t Nameaddmode ;

uint8_t FnameEntry[30] ;
uint8_t FnameEntryIndex ;

//uint32_t SoundCount ;
//uint8_t *SoundBuffer ;

//volatile uint16_t StopCount ;
//volatile uint16_t ZeroReached ;
//volatile uint16_t T0IntCount ;

//void IRAM_ATTR timer0Interrupt()
//{
//	uint32_t *p = (uint32_t *)0x3FF48484 ;
//	uint32_t x ;
//	if ( SoundCount )
//	{
////		dac_output_voltage( DAC_CHANNEL_1, *SoundBuffer++ ) ;	// Value 0-255
//		x = *p & 0xF807FFFF ;
//		x |= *SoundBuffer++ << 19 ;
//		*p = x ;
		
//		SoundCount -= 1 ;

//		if ( SoundCount == 0 )
//		{
//			ZeroReached += 1 ;
//		}
//	}
//	else
//	{
//		timerStop( PTg0T0 ) ;
//		StopCount += 1 ;
//	}
//	T0IntCount += 1 ;
//}

//uint32_t SportDebug[20] ;

void delay5uS() ;
void delay1uS() ;
void sportUartEnableInterrupt() ;
extern uint32_t SportIntCount ;
extern uint32_t SportBadIntCount ;

//#include "esp32-hal-uart.h"
//#include "esp32-hal.h"
//#include "esp_intr_alloc.h"
#define UART_INTR_SOURCE(u) ((u==0)?ETS_UART0_INTR_SOURCE:( (u==1)?ETS_UART1_INTR_SOURCE:((u==2)?ETS_UART2_INTR_SOURCE:0)))



void handle_serial(void* pdata)
{
	int16_t rxchar ;

	while ( Activated == 0 )
	{
   	vTaskDelay( pdMS_TO_TICKS( 100 ) ) ;
	}

	Serial.begin(115200) ;
extern uint32_t _rodata_start ;
	Next_mem_address = _rodata_start ;	// 0x3f400020

	for(;;)
	{
		while ( ( rxchar = Serial.read() ) == -1 )
		{
    	vTaskDelay( pdMS_TO_TICKS( 20 ) ) ;
		}
		// Got a char, what to do with it?

		if ( Memaddmode )
		{
			rxchar = toupper( rxchar ) ;
			if ( ( ( rxchar >= '0' ) && ( rxchar <= '9' ) ) || ( ( rxchar >= 'A' ) && ( rxchar <= 'F' ) ) )
			{
				txmit( rxchar ) ;
				rxchar -= '0' ;
				if ( rxchar > 9 )
				{
					rxchar -= 7 ;				
				}
				Mem_address <<= 4 ;
				Mem_address |= rxchar ;			
			}
			else if ( rxchar == 13 )
			{
				crlf() ;
				if ( Mem_address == 0 )
				{
					Mem_address = Next_mem_address ;
				}
				dispw_256( Mem_address, 4 ) ;
				Next_mem_address = Mem_address + 64 ;
				Memaddmode = 0 ;				
			}
			else if ( rxchar == 8 )
			{
				txmit( rxchar ) ;
				txmit( rxchar ) ;
				txmit( rxchar ) ;
				Mem_address >>= 4 ;			
			}
			else if ( rxchar == 27 )
			{
				crlf() ;
				Memaddmode = 0 ;				
			}
			continue ;		 
		}

		if ( Nameaddmode )
		{
			if ( rxchar >= ' ' )
			{
				txmit( rxchar ) ;
				if ( FnameEntryIndex < 28 )
				{
					FnameEntry[FnameEntryIndex++] = rxchar ;
				}
			}
			else
			{
				if ( rxchar == 8 )	// backspace
				{
					if ( FnameEntryIndex )
					{
						FnameEntryIndex -= 1 ;
						txmit( 8 ) ;
						txmit( ' ' ) ;
						txmit( 8 ) ;
					}
				}
				else if ( rxchar == 27 )
				{
					Nameaddmode = 0 ;
					crlf() ;
				}
				else if ( rxchar == 13 )
				{
					uint8_t *p ;
					Nameaddmode = 0 ;
					p = cpystr( Tbuffer, (uint8_t *)"/TEXT/" ) ;
					FnameEntry[FnameEntryIndex] = 0 ;
					cpystr( p, FnameEntry ) ;
					lfs_remove( &Lfs, (char *)Tbuffer ) ;
					crlf() ;
				}
			}
			continue ;		 
		}

		if ( rxchar == '?' )
		{
			Memaddmode = 1 ;
			Mem_address = 0 ;
			txmit( '>' ) ;
		}

		if ( rxchar == 'a' )
		{
			Serial.write( 'a' ) ;
extern void softPowerOff() ;
			softPowerOff() ;
		}

//		if ( rxchar == 'm' )
//		{
//			int32_t result ;

//			result = lfs_mkdir( &Lfs, "/TEXT" ) ;
//			Serial.println(result) ;
//			result = lfs_mkdir( &Lfs, "/voice" ) ;
//			Serial.println(result) ;
//			result = lfs_mkdir( &Lfs, "/voice/system" ) ;
//			Serial.println(result) ;
//			result = lfs_mkdir( &Lfs, "/voice/modelNames" ) ;
//			Serial.println(result) ;
//			result = lfs_mkdir( &Lfs, "/voice/user" ) ;
//			Serial.println(result) ;
//			result = lfs_mkdir( &Lfs, "/SCRIPTS" ) ;
//			Serial.println(result) ;
//		}

		if ( rxchar == 'h' )
		{
			int result ;
			lfs_dir_t my_dir ;
			struct lfs_info my_info ;
			result = lfs_dir_open( &Lfs, &my_dir, "/voice/system" ) ;
			if ( result == 0 )
			{
				do
				{
					result = lfs_dir_read( &Lfs, &my_dir, &my_info );
					if ( result > 0 )
					{
						Serial.print( my_info.name ) ;
						Serial.print( "  " ) ;
						Serial.print( my_info.type ) ;
						Serial.print( "  " ) ;
						Serial.println( my_info.size ) ;
					}
				} while ( result > 0 ) ;
				lfs_dir_close( &Lfs, &my_dir ) ;
			}
		}
		 
		if ( rxchar == 'i' )
		{
			int result ;
			lfs_dir_t my_dir ;
			struct lfs_info my_info ;
			result = lfs_dir_open( &Lfs, &my_dir, "/voice/user" ) ;
			if ( result == 0 )
			{
				do
				{
					result = lfs_dir_read( &Lfs, &my_dir, &my_info );
					if ( result > 0 )
					{
						Serial.print( my_info.name ) ;
						Serial.print( "  " ) ;
						Serial.print( my_info.type ) ;
						Serial.print( "  " ) ;
						Serial.println( my_info.size ) ;
					}
				} while ( result > 0 ) ;
				lfs_dir_close( &Lfs, &my_dir ) ;
			}
		}

		if ( rxchar == 'p' )
		{
			int result ;
			lfs_dir_t my_dir ;
			struct lfs_info my_info ;
			result = lfs_dir_open( &Lfs, &my_dir, "/TEXT" ) ;
			if ( result == 0 )
			{
				do
				{
					result = lfs_dir_read( &Lfs, &my_dir, &my_info );
					if ( result > 0 )
					{
						Serial.print( my_info.name ) ;
						Serial.print( "  " ) ;
						Serial.print( my_info.type ) ;
						Serial.print( "  " ) ;
						Serial.println( my_info.size ) ;
					}
				} while ( result > 0 ) ;
				lfs_dir_close( &Lfs, &my_dir ) ;
			}
		}
		
		if ( rxchar == 'r' )
		{
			int result ;
			lfs_dir_t my_dir ;
			struct lfs_info my_info ;
			result = lfs_dir_open( &Lfs, &my_dir, "/SCRIPTS" ) ;
			if ( result != 0 )
			{
				result = lfs_mkdir( &Lfs, "/SCRIPTS" ) ;
			}
			if ( result == 0 )
			{
				do
				{
					result = lfs_dir_read( &Lfs, &my_dir, &my_info );
					if ( result > 0 )
					{
						Serial.print( my_info.name ) ;
						Serial.print( "  " ) ;
						Serial.print( my_info.type ) ;
						Serial.print( "  " ) ;
						Serial.println( my_info.size ) ;
					}
				} while ( result > 0 ) ;
				lfs_dir_close( &Lfs, &my_dir ) ;
			}
		}

		if ( rxchar == 'd' )
		{
			uputs( "\r\nDelete file from /TEXT >" ) ;

			Nameaddmode = 1 ;
			FnameEntryIndex = 0 ;
		}

		if ( rxchar == 'q' )
		{
			int result ;
			lfs_dir_t my_dir ;
			struct lfs_info my_info ;
			result = lfs_dir_open( &Lfs, &my_dir, "/" ) ;
			if ( result == 0 )
			{
				do
				{
					result = lfs_dir_read( &Lfs, &my_dir, &my_info );
					if ( result > 0 )
					{
						Serial.print( my_info.name ) ;
						Serial.print( "  " ) ;
						Serial.print( my_info.type ) ;
						Serial.print( "  " ) ;
						Serial.println( my_info.size ) ;
					}
				} while ( result > 0 ) ;
				lfs_dir_close( &Lfs, &my_dir ) ;
			}
		}

		
		 
//		if ( rxchar == 'r' )
//		{
//			int result ;
//			lfs_file_t lfsHandle ;
//			uint8_t data[50] ;
//			int32_t nread ;
//			uint32_t i ;
			
//			result = lfs_file_open( &Lfs, &lfsHandle, (char *)"/NetId", LFS_O_RDWR ) ;
//			if ( result == 0 )
//			{
//				nread = lfs_file_read( &Lfs, &lfsHandle, data, 49 ) ;
//				lfs_file_close(&Lfs, &lfsHandle ) ;
//				for ( i = 0 ; i < nread ; i += 1 )
//				{
//					p2hex( data[i] ) ;				
//				}
//			}
//		}



// clock div reg = 00E0056C, E if frac, 56c is div = 1388 
// APB = 80000000
// uint32_t ClockDiv = (80000000*16)/baudrate ;
// #include "soc/uart_struct.h"
// #include "soc/soc.h"
// uart_dev_t *SPortUart = (uart_dev_t *)(DR_REG_UART1_BASE)
// SPortUart->clk_div.val = (ClockDiv>>4) | ( (ClockDiv & 0x0F) << 20 ) ;


//		if ( rxchar == 'u' )
//		{
//			Serial.write( 'u' ) ;
//// temporary
//#define RXDSPORT 14
//#define TXDSPORT 14

//			Serial1.begin( 57600, SERIAL_8N1, RXDSPORT, TXDSPORT, true ) ;		// Invert serial signals
//		}

//		if ( rxchar == 'v' )
//		{
//			Serial.write( 'v' ) ;
//			// send a char to serial3 fifo
//			Serial1.write( 'A' ) ;
//		}

//		if ( rxchar == 'w' )
//		{
//			Serial.write( 'w' ) ;
//			Serial1.end() ;
//			Serial.begin(115200) ;
//			sportUartEnableInterrupt() ;
//			uint32_t *p = (uint32_t *) 0x3FF50020 ;
//			*p = 0x0800001C ;
//			p = (uint32_t *) 0x3FF44568 ;		// Output assign to uart
//			*p = 0x0211 ;
////			p = (uint32_t *) 0x3FF44168 ;		// Input assign to uart
////			*p = 0x0211 ;
//		}
		
//		if ( rxchar == 'x' )
//		{
////extern hw_timer_t *PTg0T0 ;
////			PTg0T0 = timerBegin( 1, 80, false ) ;
////			timerAlarmWrite( PTg0T0, 4999, true) ;
////			timerAttachInterrupt( PTg0T0, timer0Interrupt, true) ;
////			timerAlarmEnable( PTg0T0 ) ;
//			Serial.write( 'x' ) ;
//			GPIO.enable_w1ts = ( (uint32_t)1 << 14 ) ;
//			uint32_t *p = (uint32_t *) 0x3FF44568 ;
//			*p = 0x0100 ;		// Pin is digital I/O
//			GPIO.out_w1ts = ( (uint32_t)1 << 14 ) ;
//			delay5uS() ;
//			GPIO.out_w1tc = ( (uint32_t)1 << 14 ) ;
//			p = (uint32_t *) 0x3FF44568 ;		// Output assign to uart
//			*p = 0x0211 ;
//			p = (uint32_t *) 0x3FF50000 ;
			
////        while(uart->dev->status.txfifo_cnt == 0x7F);
////        uart->dev->fifo.rw_byte = *data++;
//			*p = 'A' ;
//			uint16_t now ;
//			now = getTmr2MHz() ;
//			uint32_t i = 0 ;
//			delay5uS() ;
//			delay1uS() ;
//			*p = 'B' ;
//			delay1uS() ;
//			*p = 'C' ;
////			p8hex(p[7]) ;
//			delay1uS() ;
//			*p = 'D' ;
////			SportDebug[0] = p[7] ;
////			SportDebug[1] = p[1] ;
//			delay1uS() ;
////			SportDebug[2] = p[7] ;
////			SportDebug[3] = p[1] ;
//			*p = 'E' ;
////			SportDebug[4] = p[7] ;
////			SportDebug[5] = p[1] ;
//			delay1uS() ;
////			SportDebug[6] = p[7] ;
////			SportDebug[7] = p[1] ;
//			*p = 'F' ;
//			delay1uS() ;
//			*p = 'G' ;
//			delay1uS() ;
//			*p = 'H' ;
//			delay1uS() ;
//			*p = 'I' ;
//			delay1uS() ;
//			*p = 'J' ;
//			delay1uS() ;
//			*p = 'K' ;
//			delay1uS() ;
//			*p = 'L' ;
//			delay1uS() ;
//			*p = 'M' ;
//			p[4] = 0x4000 ;			// Clear TxDone Int
//			p[3] |= 0x4000 ;		// Enable TxDone Int

////			while ( ( (uint16_t)( getTmr2MHz() - now ) ) < 1043 )	// units 0.5uS
////			while ( ( (uint16_t)( getTmr2MHz() - now ) ) < 2086 )	// units 0.5uS
////			{
////				// wait
////			}
////			SportDebug[8] = (uint16_t)( getTmr2MHz() - now ) ;
////			p = (uint32_t *) 0x3FF44568 ;
////			*p = 0x0100 ;
////			GPIO.out_w1ts = ( (uint32_t)1 << 14 ) ;
////			delay5uS() ;
////			GPIO.out_w1tc = ( (uint32_t)1 << 14 ) ;
////			GPIO.enable_w1tc = ( (uint32_t)1 << 14 ) ;

////			crlf() ;
////			p8hex(SportDebug[0]) ;
////			Serial.write( ' ' ) ;
////			p8hex(SportDebug[1]) ;
////			crlf() ;
////			p8hex(SportDebug[2]) ;
////			Serial.write( ' ' ) ;
////			p8hex(SportDebug[3]) ;
////			crlf() ;
////			p8hex(SportDebug[4]) ;
////			Serial.write( ' ' ) ;
////			p8hex(SportDebug[5]) ;
////			crlf() ;
////			p8hex(SportDebug[6]) ;
////			Serial.write( ' ' ) ;
////			p8hex(SportDebug[7]) ;
////			crlf() ;
////			p8hex(SportDebug[8]) ;

//		}

//		if ( rxchar == 'z' )
//		{
//			uint32_t *p ;
			
//			Serial.write( 'z' ) ;
//			p = (uint32_t *) 0x3FF50000 ;
//			while ( p[7] & 0x0000007F )
//			{
//				Serial.write( *p ) ;
//			}
//			p8hex(SportIntCount) ;
//			Serial.write( '-' ) ;
//			p8hex(SportBadIntCount) ;
//		}
//		p = (uint32_t *) 0x3FF50010 ;
//		*p = 0x4000 ;			// Clear TxDone Int


//		if ( rxchar == 'r' )
//		{
//			read32_eeprom_data( 4096 * 128, Tbuffer, (uint32_t) 128, 0 ) ;
//			dispw_256( ( uint32_t)Tbuffer, 8 ) ;
//		}

//		if ( rxchar == 't' )
//		{
//			Serial.write( 'T' ) ;
//			crlf() ;
//			p4hex(StopCount) ;
//			crlf() ;
//			p4hex(ZeroReached) ;
//			crlf() ;
//			p4hex(T0IntCount) ;
//			crlf() ;
//			p2hex(ToneBuffer[0]) ;
//			p2hex(ToneBuffer[1]) ;
//			p2hex(ToneBuffer[2]) ;
//			p2hex(ToneBuffer[3]) ;
//			p2hex(ToneBuffer[4]) ;
//			p2hex(ToneBuffer[5]) ;
//			p2hex(ToneBuffer[6]) ;
//			p2hex(ToneBuffer[7]) ;
//			p2hex(ToneBuffer[8]) ;
//			p2hex(ToneBuffer[9]) ;
//			crlf() ;
//			uint32_t *p = (uint32_t *)0x3FF48484 ;
//			p8hex( *p) ;
//			Serial.write( ' ' ) ;
//			p8hex( *(p+1)) ;
//			crlf() ;

//		}
		
//		if ( rxchar == 's' )
//		{
//			Serial.write( 'S' ) ;
//			crlf() ;

//    	pinMode( 25, ANALOG) ;
//    	pinMode( 26, ANALOG) ;

//			dac_output_enable( DAC_CHANNEL_1) ;
//			dac_output_voltage( DAC_CHANNEL_1, 128 ) ;	// Value 0-255

//			dac_output_enable( DAC_CHANNEL_2) ;
//			dac_output_voltage( DAC_CHANNEL_2, 255 ) ;	// Value 0-255
			
//			// Keep DAC2 above about 23 to prevent AMP going into shutdown

//			// Turn on time is 280mS, allow 300mS

//			uint32_t i ;
//			// delay to allow volume to settle
//			for ( i = 0 ; i < 15 ; i += 1 )
//			{
//    		vTaskDelay( pdMS_TO_TICKS( 20 ) ) ;
//			}

//			toneFill( ToneBuffer ) ;
//			SoundBuffer = ToneBuffer ;
//			SoundCount = 4096 ;
//			timerStart( PTg0T0 ) ;
			 
//			PTg0T0 = timerBegin( 0, 20, true) ;
//			timerAlarmWrite( PTg0T0, 249, true) ;
//			timerAttachInterrupt( PTg0T0, timer0Interrupt, true) ;
//			timerAlarmEnable( PTg0T0 ) ;
//			timerStart( PTg0T0 ) ;

//		}
		
//		if ( rxchar == 'u' )
//		{
//			Serial.write( 'U' ) ;
//			crlf() ;
			
//			toneFill( ToneBuffer ) ;
//			SoundBuffer = ToneBuffer ;
//			SoundCount = 4096 ;
//			timerStart( PTg0T0 ) ;
//		} 

#if YMODEM
		if ( rxchar == 'Y' )
		{ // Enter Ymodem mode
			int32_t result ;
			txmit( 'Y' ) ;
			result = Ymodem_Receive( Ymbuffer ) ;
			p8hex( result ) ;
			crlf() ;
			char *fn ;
			fn = (char *)file_name ;
			while ( *fn )
			{
				txmit( *fn++ ) ;						
			}
			crlf() ;
		}
#endif
	}	
}


#if YMODEM

static int32_t Receive_Byte (uint8_t *c, uint32_t timeout)
{
	uint16_t rxchar ;

	while ( ( rxchar = Serial.read() ) == 0xFFFF )
	{
   	vTaskDelay( pdMS_TO_TICKS( 2 ) ) ;
		timeout -= 1 ;
		if ( timeout == 0 )
		{
			break ;			
		}
	}
	if ( rxchar == 0xFFFF )
	{
    return -1;
	}
	*c = rxchar ;
	return 0 ;
}

/*******************************************************************************
* Function Name  : Send_Byte
* Description    : Send a byte
* Input          : - c: Character
* Output         : None
* Return         : 0: Byte sent
*******************************************************************************/
static uint32_t Send_Byte (uint8_t c)
{
  txmit(c) ;
  return 0 ;
}

/*******************************************************************************
* Function Name  : Receive_Packet
* Description    : Receive a packet from sender
* Input 1        : - data
* Input 2        : - length
* Input 3        : - timeout
* Output         : *length:
*                  0: end of transmission
*                  -1: abort by sender
*                  >0: packet length
* Return         : 0: normally return
*                  -1: timeout or packet error
*                  1: abort by user
*******************************************************************************/
static int32_t Receive_Packet (uint8_t *data, int32_t *length, uint32_t timeout)
{
  uint16_t i, packet_size;
  uint8_t c;
  *length = 0;

  if (Receive_Byte(&c, timeout) != 0)
  {
    return -1;
  }

  switch (c)
  {
    case SOH:
      packet_size = PACKET_SIZE;
      break;
    case STX:
      packet_size = PACKET_1K_SIZE;
      break;
    case EOT:
      return 0;
    case CA:
      if ((Receive_Byte(&c, timeout) == 0) && (c == CA))
      {
        *length = -1;
        return 0;
      }
      else
      {
        return -1;
      }
    case ABORT1:
    case ABORT2:
      return 1;
    default:
      return -1;
  }
  *data = c;
  for (i = 1; i < (packet_size + PACKET_OVERHEAD); i ++)
  {
    if (Receive_Byte(data + i, PACKET_TIMEOUT) != 0)
    {
      return -1;
    }
  }
  if (data[PACKET_SEQNO_INDEX] != ((data[PACKET_SEQNO_COMP_INDEX] ^ 0xff) & 0xff))
  {
    return -1;
  }
  *length = packet_size;
  return 0;
}

uint8_t packet_data[PACKET_1K_SIZE + PACKET_OVERHEAD] ;

//uint8_t packet_0[PACKET_1K_SIZE + PACKET_OVERHEAD] ;
//uint8_t packet_1[PACKET_1K_SIZE + PACKET_OVERHEAD] ;
//uint8_t packet_2[PACKET_1K_SIZE + PACKET_OVERHEAD] ;

int32_t size = 0 ;

/*******************************************************************************
* Function Name  : Ymodem_Receive
* Description    : Receive a file using the ymodem protocol
* Input          : Address of the first byte
* Output         : None
* Return         : The size of the file
*******************************************************************************/

File YmFile ;

int32_t Ymodem_Receive (uint8_t *buf)
{
  uint8_t *file_ptr; //, *buf_ptr;
  int32_t i, packet_length, session_done, file_done, packets_received, errors, session_begin ;
	FRESULT fr ;
	uint32_t written ;
  /* Initialize FlashDestination variable */
//  FlashDestination = ApplicationAddress;

  
	size = 0 ;
 	file_name[0] = 0 ;

  for (session_done = 0, errors = 0, session_begin = 0 ; ; )
  {
    for (packets_received = 0, file_done = 0 /*, buf_ptr = buf*/; ;)
    {
      switch (Receive_Packet(packet_data, &packet_length, NAK_TIMEOUT))
      {
        case 0:
          errors = 0;
          switch (packet_length)
          {
              /* Abort by sender */
            case - 1:
              Send_Byte(ACK);
              return 0;
              /* End of transmission */
            case 0:
							YmFile.close() ;
              Send_Byte(ACK);
              file_done = 1;
              break;
              /* Normal packet */
            default:
              if ((packet_data[PACKET_SEQNO_INDEX] & 0xff) != (packets_received & 0xff))//
              {
                Send_Byte(NAK);
              }
              else
              {
                if (packets_received == 0)
                {/* Filename packet */
                  if (packet_data[PACKET_HEADER] != 0)
                  {/* Filename packet has valid data */
                    file_name[0] = '/' ;
                    for (i = 0, file_ptr = packet_data + PACKET_HEADER; (*file_ptr != 0) && (i < FILE_NAME_LENGTH);)
                    {
                      file_name[++i] = *file_ptr++ ;
                    }
                    file_name[++i] = '\0';
                    for (i = 0, file_ptr += 1; (*file_ptr) && (i < FILE_SIZE_LENGTH);)
                    {
											size *= 10 ;
											size += *file_ptr++ - '0' ;
                    }
//											f_unlink ( "Ymodtemp" ) ;					/* Delete any existing temp file */
//											fr = f_open( &Tfile, "Ymodtemp", FA_WRITE | FA_CREATE_ALWAYS ) ;
									  YmFile = SPIFFS.open(file_name, "w");

		  // Check fr value here

//                    }
                    Send_Byte(ACK);
                    Send_Byte(CRC16);
                  }
                  /* Filename packet is empty, end session */
                  else
                  {
                    Send_Byte(ACK);
                    file_done = 1;
                    session_done = 1;
                    break;
                  }
                }
                /* Data packet */
                else
                {
                  Send_Byte(ACK);
                }
								// Temp, copy and save first three buffers
//								if ( packets_received < 3 )
//								{
//									uint8_t *ptr ;
//									ptr = packet_0 ;
//									if ( packets_received == 1 ) ptr = packet_1 ;
//									if ( packets_received == 2 ) ptr = packet_2 ;
//									for ( i = 0 ; i < PACKET_1K_SIZE + PACKET_OVERHEAD ; i += 1 )
//									{
//										*ptr++ = packet_data[i] ;
//									}
//								}
								if ( packets_received == 1 )
								{
									YmFile.write(&packet_data[3], 128 ) ;
								}
                packets_received ++;
                session_begin = 1;
              }
          }
          break;
        case 1:
          Send_Byte(CA);
          Send_Byte(CA);
          return -3;
        default:
          if (session_begin > 0)
          {
            errors ++;
          }
          if (errors > MAX_ERRORS)
          {
            Send_Byte(CA);
            Send_Byte(CA);
            return 0;
          }
          Send_Byte(CRC16);
          break;
      }
      if (file_done != 0)
      {
        break;
      }
    }
    if (session_done != 0)
    {
      break;
    }
  }


//FRESULT f_unlink (const TCHAR*);					/* Delete an existing file or directory */
//FRESULT f_rename (const TCHAR*, const TCHAR*);		/* Rename/Move a file or directory */

	Voice.VoiceLock = 0 ;
  return (int32_t)size ;
}

#endif

//void disp_mem( register uint32_t address )
//{
//	p8hex( address ) ;
//	txmit('=') ;
//	p8hex( *( (uint32_t *)address ) ) ;
//	crlf() ;
//}

//void disp_256( register uint32_t address, register uint32_t lines )
//{
//	register uint32_t i ;
//	register uint32_t j ;
//	for ( i = 0 ; i < lines ; i += 1 )
//	{
//		p8hex( address ) ;
//		for ( j = 0 ; j < 16 ; j += 1 )
//		{
//			txmit(' ') ;
//			p2hex( *( (uint8_t *)address++ ) ) ;
//		}
//		crlf() ;
//	}
//}


void dispw_256( register uint32_t address, register uint32_t lines )
{
	register uint32_t i ;
	register uint32_t j ;
	for ( i = 0 ; i < lines ; i += 1 )
	{
		p8hex( address ) ;
		for ( j = 0 ; j < 4 ; j += 1 )
		{
			txmit(' ') ;
			p8hex( *( (uint32_t *)address ) ) ;
			address += 4 ;
		}
		crlf() ;
	}
}




