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
#include "fifo.h"

void put_fifo128( struct t_fifo128 *pfifo, uint8_t byte )
{
  uint32_t next = (pfifo->in + 1) & 0x7f;
	if ( next != pfifo->out )
	{
		pfifo->fifo[pfifo->in] = byte ;
		pfifo->in = next ;
	}
}

uint32_t fifo128Space( struct t_fifo128 *pfifo )
{
	int32_t space ;

	space = (int32_t)pfifo->out - (int32_t)pfifo->in ;
	if ( space <= 0 )
	{
		space += 128 ;
	}    
	 
//	 + 127 ;
//	if ( pfifo->out > pfifo->in )
//	{
//		space -= 128 ;
//	}
	return space - 1 ;
}

int32_t get_fifo128( struct t_fifo128 *pfifo )
{
	int32_t rxbyte ;
	if ( pfifo->in != pfifo->out )				// Look for char available
	{
		rxbyte = pfifo->fifo[pfifo->out] ;
		pfifo->out = ( pfifo->out + 1 ) & 0x7F ;
		return rxbyte ;
	}
	return -1 ;
}

int32_t peek_fifo128( struct t_fifo128 *pfifo )
{
	if ( pfifo->in != pfifo->out )				// Look for char available
	{
		return pfifo->fifo[pfifo->out] ;
	}
	return -1 ;
}

// Bytes available in fifo
uint32_t fifo128Available( struct t_fifo128 *pfifo )
{
	uint32_t available ;
	available = pfifo->in - pfifo->out ;
	if ( pfifo->out > pfifo->in )
	{
		available += 128 ;
	}
	return available ;
}

void put_fifo64( struct t_fifo64 *pfifo, uint8_t byte )
{
  uint32_t next = (pfifo->in + 1) & 0x3f;
	if ( next != pfifo->out )
	{
		pfifo->fifo[pfifo->in] = byte ;
		pfifo->in = next ;
	}
}

int32_t get_fifo64( struct t_fifo64 *pfifo )
{
	int32_t rxbyte ;
	if ( pfifo->in != pfifo->out )				// Look for char available
	{
		rxbyte = pfifo->fifo[pfifo->out] ;
		pfifo->out = ( pfifo->out + 1 ) & 0x3F ;
		return rxbyte ;
	}
	return -1 ;
}

