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
#include <Wire.h>
#include "erskyTx.h"
#include "myeeprom.h"

#include "soc/i2c_struct.h"
#include "soc/i2c_reg.h"

#define AW1_ADDRESS		0x5B
#define AW2_ADDRESS		0x59
#define RTC_ADDRESS		0x51

extern uint8_t Aw1HighRequired ;
extern uint8_t Aw2LowRequired ;
extern uint8_t Aw1Outputs[2] ;
extern uint8_t Aw2Outputs[2] ;

void setI2C400kHz()
{
	uint32_t *r ;
	r = (uint32_t *) 0x3FF53000 ;
	*r = 0x64 ;     	// 0x3FF53000
	r = (uint32_t *) 0x3FF53030 ;
	*r++ = 0x19 ;   	// 0x3FF53030
	*r++ = 0x19 ;   	// 0x3FF53034
	*r = 0x64 ;				// 0x3FF53038
	r = (uint32_t *) 0x3FF53040 ;
	*r++ = 0x32 ;   	// 0x3FF53040
	*r++ = 0x32 ;   	// 0x3FF53044
	*r++ = 0x32 ;   	// 0x3FF53048
	*r = 0x32 ;     	// 0x3FF5304C
}


void initI2C()
{
	I2C0.int_ena.val = 0 ;

  I2C0.fifo_conf.tx_fifo_empty_thrhd = 0 ;
  I2C0.fifo_conf.rx_fifo_full_thrhd  = 30 ;
  I2C0.fifo_conf.nonfifo_rx_thres    = 32 ;
  I2C0.fifo_conf.nonfifo_tx_thres    = 32 ;
	
}

uint8_t readI2CByte( uint8_t address, uint8_t index )
{
	uint32_t result ;
	uint32_t i ;
	uint32_t save ;

	save = I2C0.int_ena.val ;
	I2C0.int_ena.val = 0 ;

  I2C0.fifo_conf.tx_fifo_empty_thrhd = 0 ;
  I2C0.fifo_conf.rx_fifo_full_thrhd  = 30 ;
  I2C0.fifo_conf.nonfifo_rx_thres    = 32 ;
  I2C0.fifo_conf.nonfifo_tx_thres    = 32 ;

	I2C0.command[0].val = 0 ;
	I2C0.command[1].val = 0x00000902 ;
	I2C0.command[2].val = 0x00000000 ;
	
	I2C0.command[3].val = 0x00000901 ;
	I2C0.command[4].val = 0x00001401 ;
	I2C0.command[5].val = 0x00001800 ;
	I2C0.command[6].val = 0x80000000 ;
		 
	I2C0.fifo_data.val = ( address << 1 ) ;
	I2C0.fifo_data.val = index ;
	I2C0.fifo_data.val = ( address << 1 ) | 1 ;
	
	I2C0.ctr.trans_start = 1 ;
	i = 0 ;
	while ( ( I2C0.command[5].val & 0x80000000 ) == 0 )
	{
		// wait
		if ( ++i > 500000 )
		{
			return 0 ;
		}
	}

	result = I2C0.fifo_data.val ;

	I2C0.int_ena.val = save ;
  
	return result ;
}

uint32_t writeI2CByte( uint8_t address, uint8_t index, uint8_t byte )
{
	uint32_t i ;
	uint32_t save ;

	save = I2C0.int_ena.val ;
	I2C0.int_ena.val = 0 ;

  I2C0.fifo_conf.tx_fifo_empty_thrhd = 0 ;
  I2C0.fifo_conf.rx_fifo_full_thrhd  = 30 ;
  I2C0.fifo_conf.nonfifo_rx_thres    = 32 ;
  I2C0.fifo_conf.nonfifo_tx_thres    = 32 ;

	I2C0.command[0].val = 0 ;
	I2C0.command[1].val = 0x00000903 ;
	I2C0.command[2].val = 0x00001800 ;
	I2C0.command[3].val = 0x80000000 ;
		 
	I2C0.fifo_data.val = ( address << 1 ) ;
	I2C0.fifo_data.val = index ;
	I2C0.fifo_data.val = byte ;
	
	I2C0.ctr.trans_start = 1 ;
	i = 0 ;
	while ( ( I2C0.command[2].val & 0x80000000 ) == 0 )
	{
		// wait
		if ( ++i > 500000 )
		{
			return 0 ;
		}
	}

	I2C0.int_ena.val = save ;

	return i > 0 ? 1 : 0 ;
}

void checkAw1()
{
	if ( Aw1HighRequired != Aw1Outputs[1] )
	{
		writeI2CByte( AW1_ADDRESS, 3, Aw1HighRequired ) ;
	
		Aw1Outputs[1] = Aw1HighRequired ;
	}
}

void checkAw2()
{
	if ( Aw2LowRequired != Aw2Outputs[0] )
	{
		writeI2CByte( AW2_ADDRESS, 2, Aw2LowRequired ) ;
	
		Aw2Outputs[0] = Aw2LowRequired ;
	}
}


uint32_t readI2Cmulti( uint8_t address, uint8_t index, uint8_t count, uint8_t *buffer )
{
	uint32_t result ;
	uint32_t i ;
	uint32_t save ;

//	result = I2C0.fifo_data.val ;

	save = I2C0.int_ena.val ;
	I2C0.int_ena.val = 0 ;

  I2C0.fifo_conf.tx_fifo_empty_thrhd = 0;
  I2C0.fifo_conf.rx_fifo_full_thrhd  = 30;
  I2C0.fifo_conf.nonfifo_rx_thres    = 32;
  I2C0.fifo_conf.nonfifo_tx_thres    = 32;
	
	I2C0.command[0].val = 0 ;
	I2C0.command[1].val = 0x00000902 ;
	I2C0.command[2].val = 0x00000000 ;
	
	I2C0.command[3].val = 0x00000901 ;
	I2C0.command[4].val = 0x00001000 + count - 1 ;
	I2C0.command[5].val = 0x00001401 ;
	I2C0.command[6].val = 0x00001800 ;
	I2C0.command[7].val = 0x80000000 ;
		 
	I2C0.fifo_data.val = ( address << 1 ) ;
	I2C0.fifo_data.val = index ;
	I2C0.fifo_data.val = ( address << 1 ) | 1 ;
	
	I2C0.ctr.trans_start = 1 ;
	i = 0 ;
	while ( ( I2C0.command[6].val & 0x80000000 ) == 0 )
	{
		// wait
		if ( ++i > 500000 )
		{
			return 0 ;
		}
	}

	while ( count )
	{
		*buffer++ = I2C0.fifo_data.val ;
		count -= 1 ;
	}

	I2C0.int_ena.val = save ;
  
	return result ;
}

uint32_t writeI2Cmulti( uint8_t address, uint8_t index, uint8_t count, uint8_t *buffer )
{
	uint32_t i ;
	uint32_t save ;

	save = I2C0.int_ena.val ;
	I2C0.int_ena.val = 0 ;

  I2C0.fifo_conf.tx_fifo_empty_thrhd = 0 ;
  I2C0.fifo_conf.rx_fifo_full_thrhd  = 30 ;
  I2C0.fifo_conf.nonfifo_rx_thres    = 32 ;
  I2C0.fifo_conf.nonfifo_tx_thres    = 32 ;

	I2C0.command[0].val = 0 ;
	I2C0.command[1].val = 0x00000902 + count ;
	I2C0.command[2].val = 0x00001800 ;
	I2C0.command[3].val = 0x80000000 ;
		 
	I2C0.fifo_data.val = ( address << 1 ) ;
	I2C0.fifo_data.val = index ;
	while ( count )
	{
		I2C0.fifo_data.val = *buffer++ ;
		count -= 1 ;
	}
	
	I2C0.ctr.trans_start = 1 ;
	i = 0 ;
	while ( ( I2C0.command[2].val & 0x80000000 ) == 0 )
	{
		// wait
		if ( ++i > 500000 )
		{
			return 0 ;
		}
	}

	I2C0.int_ena.val = save ;

	return i > 0 ? 1 : 0 ;
}

uint32_t readAws()
{
	uint32_t result ;
	uint32_t i ;

	I2C0.command[0].val = 0 ;
	I2C0.command[1].val = 0x00000902 ;
	I2C0.command[2].val = 0x00000000 ;
	I2C0.command[3].val = 0x00000901 ;
	I2C0.command[4].val = 0x00001401 ;
	I2C0.command[5].val = 0x00001800 ;
	I2C0.command[6].val = 0x80000000 ;

	I2C0.fifo_data.val = ( AW2_ADDRESS << 1 ) ;
	I2C0.fifo_data.val = 0 ;
	I2C0.fifo_data.val = ( AW2_ADDRESS << 1 ) | 1 ;

	I2C0.ctr.trans_start = 1 ;
	i = 0 ;
	while ( ( I2C0.command[5].val & 0x80000000 ) == 0 )
	{
		// wait
		if ( ++i > 500000 )
		{
			return 0 ;
		}
	}
	
	I2C0.command[5].val = 0x00001800 ;
	I2C0.fifo_data.val = ( AW2_ADDRESS << 1 ) ;
	I2C0.fifo_data.val = 1 ;
	I2C0.fifo_data.val = ( AW2_ADDRESS << 1 ) | 1 ;
	
	I2C0.ctr.trans_start = 1 ;
	i = 0 ;
	while ( ( I2C0.command[5].val & 0x80000000 ) == 0 )
	{
		// wait
		if ( ++i > 500000 )
		{
			return 0 ;
		}
	}
	
	I2C0.command[5].val = 0x00001800 ;
	I2C0.fifo_data.val = ( AW1_ADDRESS << 1 ) ;
	I2C0.fifo_data.val = 0 ;
	I2C0.fifo_data.val = ( AW1_ADDRESS << 1 ) | 1 ;

	I2C0.ctr.trans_start = 1 ;
	i = 0 ;
	while ( ( I2C0.command[5].val & 0x80000000 ) == 0 )
	{
		// wait
		if ( ++i > 500000 )
		{
			return 0 ;
		}
	}

	result = I2C0.fifo_data.val ;
	result |= I2C0.fifo_data.val << 8 ;
	result |= I2C0.fifo_data.val << 16 ;

	return result ;
}

//uint32_t readAws()
//{
//	uint32_t result ;
//	uint32_t i ;
//	uint32_t save ;

//	result = I2C0.fifo_data.val ;

//	save = I2C0.int_ena.val ;
//	I2C0.int_ena.val = 0 ;

//  I2C0.fifo_conf.tx_fifo_empty_thrhd = 0;
//  I2C0.fifo_conf.rx_fifo_full_thrhd  = 30;
//  I2C0.fifo_conf.nonfifo_rx_thres    = 32;
//  I2C0.fifo_conf.nonfifo_tx_thres    = 32;

//	I2C0.command[0].val = 0 ;
//	I2C0.command[1].val = 0x00000902 ;
//	I2C0.command[2].val = 0x00000000 ;

//	I2C0.command[3].val = 0x00000901 ;
//	I2C0.command[4].val = 0x00001001 ;
//	I2C0.command[5].val = 0x00001401 ;

//	I2C0.command[6].val = 0x00000000 ;
//	I2C0.command[7].val = 0x00000902 ;
//	I2C0.command[8].val = 0x00000000 ;
//	I2C0.command[9].val = 0x00000901 ;
//	I2C0.command[10].val = 0x00001401 ;
//	I2C0.command[11].val = 0x00001800 ;
//	I2C0.command[12].val = 0x80000000 ;
	 
//	I2C0.fifo_data.val = ( AW2_ADDRESS << 1 ) ;
//	I2C0.fifo_data.val = 0 ;
//	I2C0.fifo_data.val = ( AW2_ADDRESS << 1 ) | 1 ;
//	I2C0.fifo_data.val = ( AW1_ADDRESS << 1 ) ;
//	I2C0.fifo_data.val = 0 ;
//	I2C0.fifo_data.val = ( AW1_ADDRESS << 1 ) | 1 ;

//	I2C0.ctr.trans_start = 1 ;
//	i = 0 ;
//	while ( ( I2C0.command[11].val & 0x80000000 ) == 0 )
//	{
//		// wait
//		if ( ++i > 500000 )
//		{
//			return 0 ;
//		}
//	}

//	result = I2C0.fifo_data.val ;
//	result |= I2C0.fifo_data.val << 24 ;
//	result |= I2C0.fifo_data.val << 16 ;

//	result |= readI2CByte( AW2_ADDRESS, 1 ) << 8 ;

//	I2C0.int_ena.val = save ;

//	return result ;
//}


static uint32_t fromBCD( uint8_t bcd_value )
{
	return ( ( ( bcd_value & 0xF0 ) * 10 ) >> 4 ) + ( bcd_value & 0x0F ) ;
}

static uint32_t toBCD( uint32_t value )
{
	div_t qr ;
	qr = div( value, 10 ) ;
	return ( qr.quot << 4 ) + qr.rem ;
}

uint8_t ExternalRtc[7] ;

void readClock()
{
	t_time *p = &Time ;

	readI2Cmulti( RTC_ADDRESS, 2, 7, ExternalRtc ) ;
	 
	p->second = fromBCD( ExternalRtc[0] & 0x7F ) ;
	p->minute = fromBCD( ExternalRtc[1] & 0x7F ) ;
	p->hour = fromBCD( ExternalRtc[2] & 0x3F ) ;
	p->date = fromBCD( ExternalRtc[3] & 0x3F ) ;
	p->month = fromBCD( ExternalRtc[5] & 0x1F ) ;
	p->year = fromBCD( ExternalRtc[6] ) + 2000 ;
}

void writeClock( t_time *ptr )
{
	ExternalRtc[0] = toBCD( ptr->second ) ;
	ExternalRtc[1] = toBCD( ptr->minute ) ;
	ExternalRtc[2] = toBCD( ptr->hour ) ;
	ExternalRtc[3] = toBCD( ptr->date ) ;
	ExternalRtc[5] = toBCD( ptr->month ) ;
	ExternalRtc[6] = toBCD( ptr->year - 2000 ) ;

	writeI2Cmulti( RTC_ADDRESS, 2, 7, ExternalRtc ) ;

}





