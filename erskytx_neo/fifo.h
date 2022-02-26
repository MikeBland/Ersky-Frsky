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

struct t_fifo128
{
	uint8_t fifo[128] ;
	uint32_t in ;
	uint32_t out ;
} ;

struct t_fifo64
{
	uint8_t fifo[64] ;
	uint32_t in ;
	uint32_t out ;
} ;

int32_t get_fifo128( struct t_fifo128 *pfifo ) ;
void put_fifo128( struct t_fifo128 *pfifo, uint8_t byte ) ;
uint32_t fifo128Space( struct t_fifo128 *pfifo ) ;
uint32_t fifo128Available( struct t_fifo128 *pfifo ) ;
int32_t peek_fifo128( struct t_fifo128 *pfifo ) ;
void put_fifo64( struct t_fifo64 *pfifo, uint8_t byte ) ;
int32_t get_fifo64( struct t_fifo64 *pfifo ) ;



