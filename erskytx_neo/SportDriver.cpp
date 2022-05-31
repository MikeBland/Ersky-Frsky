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

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define __disable_irq() XTOS_SET_INTLEVEL(15)
#define __enable_irq() XTOS_SET_INTLEVEL(0)

#include "erskyTx.h"
#include "soc/soc.h"
#include "soc/uart_struct.h"
#include "myeeprom.h"

// clock div reg = 00E0056C, E if frac, 56c is div = 1388 
// APB = 80000000
// uint32_t ClockDiv = (80000000*16)/baudrate ;
// #include "soc/uart_struct.h"
// #include "soc/soc.h"
// uart_dev_t *SPortUart = (uart_dev_t *)(DR_REG_UART1_BASE)
// SPortUart->clk_div.val = (ClockDiv>>4) | ( (ClockDiv & 0x0F) << 20 ) ;

#define UART_INTR_SOURCE(u) ((u==0)?ETS_UART0_INTR_SOURCE:( (u==1)?ETS_UART1_INTR_SOURCE:((u==2)?ETS_UART2_INTR_SOURCE:0)))

uint16_t getTmr2MHz() ;

void delay5uS()
{
	uint16_t now ;
	now = getTmr2MHz() ;
	while ( ( (uint16_t)( getTmr2MHz() - now ) ) < 10 )	// units 0.5uS
	{
		// wait
	}
}
void delay1uS()
{
	uint16_t now ;
	now = getTmr2MHz() ;
	while ( ( (uint16_t)( getTmr2MHz() - now ) ) < 2 )	// units 0.5uS
	{
		// wait
	}
}

intr_handle_t SportIntHandle ;
uint32_t SportIntCount ;
uint32_t SportBadIntCount ;

void IRAM_ATTR sportUartIsr(void *arg)
{
	uint32_t *p ;
	
	uart_dev_t *sportUart = (uart_dev_t *)(DR_REG_UART1_BASE) ;

	sportUart->int_ena.tx_done = 0 ;
	
//	uint32_t *p = (uint32_t *) 0x3FF50000 ;
//	p[3] &= ~0x4000 ;		// Disable TxDone Int

	if ( sportUart->status.txfifo_cnt == 0 )
//	if ( ( p[7] & 0x007F0000 ) == 0 )
	{
		GPIO.out1_w1ts.val = ( (uint32_t)1 << 14 ) ;	// Disabled
		delay5uS() ;
		p = (uint32_t *) 0x3FF44568 ;		// Output assign to uart
		*p = 0x0100 ;		// GPIO output, non-inverted
		GPIO.enable_w1tc = ( (uint32_t)1 << 14 ) ;	// Disabled
		// make sure output disabled, input enabled
		p = (uint32_t *) 0x3FF49030 ;		// IO_MUX_x_REG
		*p |= 0x00000200 ;

		SportIntCount += 1 ;
	}
	else
	{
		SportBadIntCount += 1 ;
	}
}

void enableSportRx()
{
	uint32_t *p ;
	GPIO.out1_w1ts.val = ( (uint32_t)1 << 14 ) ;	// Disabled
	delay5uS() ;
	p = (uint32_t *) 0x3FF44568 ;		// Output assign to uart
	*p = 0x0100 ;		// GPIO output, non-inverted
	GPIO.enable_w1tc = ( (uint32_t)1 << 14 ) ;	// Disabled
	// make sure output disabled, input enabled
	p = (uint32_t *) 0x3FF49030 ;		// IO_MUX_x_REG
	*p |= 0x00000200 ;

}

uint8_t ELRSconfigured = 0 ;

void sportUartEnableInterrupt()
{
	esp_intr_alloc(UART_INTR_SOURCE(1), (int)ESP_INTR_FLAG_IRAM, sportUartIsr, NULL, &SportIntHandle) ;
}

void setSportBaudrate( uint32_t baudrate )
{
	uint32_t ClockDiv = (80000000*16)/baudrate ;
	uart_dev_t *sportUart = (uart_dev_t *)(DR_REG_UART1_BASE) ;
	sportUart->clk_div.val = (ClockDiv>>4) | ( (ClockDiv & 0x0F) << 20 ) ;
}

void initSportUart()
{
	uart_dev_t *sportUart = (uart_dev_t *)(DR_REG_UART1_BASE) ;
	uint32_t *p ;
	uint32_t i ;
	
#define RXDSPORT 14
#define TXDSPORT 14

	for ( i = 0 ; i < 500 ; i += 1 )
	{
		delay5uS() ;
	}
	if ( ELRSconfigured == 0 )
	{
		ELRSconfigured = 1 ;
		Serial1.begin( 400000, SERIAL_8N1, RXDSPORT, TXDSPORT, true ) ;		// Invert serial signals
		Serial1.end() ;
		Serial.begin(115200) ;
		sportUartEnableInterrupt() ;
	}
	sportUart->conf0.val = 0x0800001C ;
	p = (uint32_t *) 0x3FF44568 ;		// Output assign to uart
	*p = 0x0211 ;
	// flush Rx fifo
	i = 0 ;
	while ( sportUart->status.rxfifo_cnt )
	{
		(void)sportUart->fifo.val ;
		if ( ++i > 256 )
		{
			break ;
		}
	}
}

void disableSportUart()
{
	
}

void sportTransmit( uint8_t *buffer, uint32_t length )
{
	uint32_t *p ;
	uart_dev_t *sportUart = (uart_dev_t *)(DR_REG_UART1_BASE) ;
	// disable input
//	Register 4.34. IO_MUX_x_REG (x: GPIO0GPIO39)(0x10+4*x) @ 0x3FF49030
// Bit FUN_IE (bit 9)
	p = (uint32_t *) 0x3FF49030 ;		// IO_MUX_x_REG
	*p &= ~0x00000200 ;

	// enable output	 
	
	p = (uint32_t *) 0x3FF44568 ;		// Output assign to uart
	*p = 0x0211 ;
	GPIO.enable_w1ts = ( (uint32_t)1 << 14 ) ;	// Disabled
	
	if ( length )
	{
		GPIO.enable_w1ts = ( (uint32_t)1 << 14 ) ;
		sportUart->fifo.val = *buffer++ ;
		length -= 1 ;
		delay5uS() ;
		while ( length )
		{
			delay1uS() ;
			sportUart->fifo.val = *buffer++ ;
			length -= 1 ;
		}
		sportUart->int_clr.tx_done = 1 ;
		sportUart->int_ena.tx_done = 1 ;
	}
}

int32_t pollSportFifo()
{
	uint8_t b ;
	if ( ELRSconfigured )
	{
		uart_dev_t *sportUart = (uart_dev_t *)(DR_REG_UART1_BASE) ;
		if ( sportUart->status.rxfifo_cnt )
		{
			return sportUart->fifo.val ;
		}
	}
	return -1 ;
}

// CRC8 implementation with polynom = x^8+x^7+x^6+x^4+x^2+1 (0xD5)
const unsigned char crc8tab[256] = {
  0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
  0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
  0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
  0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
  0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
  0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
  0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
  0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
  0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
  0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
  0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
  0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
  0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
  0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
  0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
  0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
  0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
  0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
  0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
  0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
  0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
  0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
  0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
  0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
  0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
  0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
  0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
  0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
  0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
  0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
  0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
  0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

uint8_t crc8(const uint8_t * ptr, uint32_t len)
{
  uint8_t crc = 0;
  for ( uint32_t i=0 ; i<len ; i += 1 )
	{
    crc = crc8tab[crc ^ *ptr++] ;
  }
  return crc;
}

#define CROSSFIRE_CH_CENTER         0x3E0
#define CROSSFIRE_CH_BITS           11
#define CROSSFIRE_CHANNELS_COUNT		16

#define MODULE_ADDRESS							0xEE
#define CHANNELS_ID									0x16

uint8_t Bit_pulses[80] ;			// To allow for Xfire telemetry
extern uint16_t XfireLength ;
extern struct t_telemetryTx TelemetryTx ;

// Range for pulses (channels output) is [-1024:+1024]
uint8_t setupPulsesXfire( uint32_t module )
{
	uint32_t startChan ;
 	uint8_t *buf = Bit_pulses ;
 	*buf++ = MODULE_ADDRESS ;

	if ( TelemetryTx.XfireTx.count )
	{
		uint32_t i ;
 		*buf++ = TelemetryTx.XfireTx.count + 2 ;
  	uint8_t *crc_start = buf ;
  	*buf++ = TelemetryTx.XfireTx.command ;
  	for ( i = 0 ; i < TelemetryTx.XfireTx.count ; i += 1 )
		{
			*buf++ = TelemetryTx.XfireTx.data[i] ;
			if ( i > 62 )
			{
				break ;
			}
		}
		i = buf - crc_start ;
		*buf++ = crc8( crc_start, i ) ;
		TelemetryTx.XfireTx.count = 0 ;
	}
	else
	{
		startChan = g_model.Module[1].startChannel ;
  	*buf++ = 24 ; // 1(ID) + 22 + 1(CRC)
  	uint8_t *crc_start = buf ;
  	*buf++ = CHANNELS_ID ;
  	uint32_t bits = 0 ;
  	uint32_t bitsavailable = 0 ;
  	for (uint32_t i=0 ; i < CROSSFIRE_CHANNELS_COUNT ; i += 1 )
		{
  	  uint32_t val = limit(0, CROSSFIRE_CH_CENTER + (((g_chans512[startChan+i]) * 4) / 5), 2*CROSSFIRE_CH_CENTER) ;
  	  bits |= val << bitsavailable ;
  	  bitsavailable += CROSSFIRE_CH_BITS ;
  	  while (bitsavailable >= 8)
			{
  	    *buf++ = bits ;
  	    bits >>= 8 ;
  	    bitsavailable -= 8 ;
  	  }
  	}
  	*buf++ = crc8( crc_start, 23) ;
	}
  
	sportTransmit( Bit_pulses, buf - Bit_pulses ) ;
	return 0 ;
}

uint32_t xfirePacketSend( uint8_t length, uint8_t command, uint8_t *data )
{
	uint32_t i ;

	if ( length == 0 )	// Test for buffer available
	{
		return TelemetryTx.XfireTx.count ? 0 : 1 ;
	}

	if ( TelemetryTx.XfireTx.count )
	{
		return 0 ;	// Can't send, packet already queued
	}
	TelemetryTx.XfireTx.command = command ;
//	j = TelemetryTx.XfireTx.count ;
	if ( length > 64 )
	{
		return 0 ;	// Can't send, packet too long
	}
	for ( i = 0 ; i < length ; i += 1 )
	{
		TelemetryTx.XfireTx.data[i] = *data++ ;
	}
	TelemetryTx.XfireTx.count = length ;
	return 1 ;
}




