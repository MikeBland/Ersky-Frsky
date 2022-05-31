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

struct t_file_entry
{
	uint32_t block_no ;
	uint32_t sequence_no ;
	uint16_t size ;
	uint8_t flags ;
} ;

#define FILE_TYPE_MODEL		0
#define FILE_TYPE_TEXT		1

extern uint8_t GdId[] ;
extern unsigned char ModelNames[][sizeof(g_model.name)+1] ;		// Allow for general
extern EE_X20General g_eeGeneral ;


void startSpiMemory() ;
//uint8_t eeprom_read_status() ;
void flash_read_pages( uint8_t *data, uint32_t page, uint32_t nPages) ;

extern uint32_t ee32_check_finished( void ) ;
extern void ee32WaitFinished( void ) ;
extern void ee32StoreGeneral( void ) ;
extern void ee32StoreModel( uint8_t modelNumber, uint8_t trim ) ;
//extern bool ee32LoadGeneral( void ) ;
extern void ee32LoadModel(uint8_t id) ;
extern void ee32WaitLoadModel(uint8_t id) ;
extern void ee32_delete_model( uint8_t id ) ;
extern bool eeModelExists(uint8_t id) ;
extern void ee32_process( void ) ;
void ee32SwapModels(uint8_t id1, uint8_t id2) ;
void init_eeprom() ;
uint16_t evalChkSum() ;
void eeDirty(uint8_t msk) ;
extern bool eeDuplicateModel(uint8_t id) ;

void setModelFilename( uint8_t *filename, uint8_t modelIndex, uint32_t type ) ;

extern void eeReadAll() ;

//extern uint32_t read32_eeprom_data( uint32_t eeAddress, register uint8_t *buffer, uint32_t size, uint32_t immediate ) ;

extern void eeDirty(uint8_t msk) ;


