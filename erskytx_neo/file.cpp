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
//#include <stdint.h>

#include <SPI.h>
#define GDCSPIN		0


#include "erskyTx.h"
#include "myeeprom.h"
#include "file.h"
#include "en.h"
#include "menus.h"
#include "templates.h"

#include "lfs.h"
#include "freertos/semphr.h"

#define RADIO_PATH           "/RADIO"   // no trailing slash = important

uint8_t Current_general_block ;		// 0 or 1 is active block
uint8_t Other_general_block_blank ;

uint8_t Spi_complete ;

X20ModelData TempModelData ;

struct lfs_config Cfg ;

lfs_t Lfs ;

uint8_t Lfs_read_buf[256] ;
uint8_t Lfs_prog_buf[256] ;
uint32_t Lfs_lookahead_buf[64] ;	// 128/8=16
//uint8_t Lfs_file_buf[256] ;


struct t_eeprom_header
{
	uint32_t sequence_no ;		// sequence # to decide which block is most recent
	uint16_t data_size ;			// # bytes in data area
	uint8_t flags ;
	uint8_t hcsum ;
} ;

void LFS_Config( void ) ;
void LfsBegin( void ) ;

// Structure of data in a block
struct t_eeprom_block
{
	struct t_eeprom_header header ;
	union
	{
		uint8_t bytes[4088] ;
		uint32_t words[1022] ;
	} data ;
} ;

struct t_dirty
{
	uint8_t General_dirty ;
	uint8_t Model_dirty ;
} Dirty ;


//#define EEPROM_BUFFER_SIZE ((sizeof(SKYModelData) + sizeof( struct t_eeprom_header ) + 3)/4)

struct t_eeprom_buffer
{
	struct t_eeprom_header header ;
	union t_eeprom_data
	{
		EE_X20General general_data ;
//		ModelData model_data ;
		X20ModelData sky_model_data ;
//		uint32_t words[ EEPROM_BUFFER_SIZE ] ;
		uint8_t buffer2K[2048] ;
	} data ;	
} Eeprom_buffer ;

#define CMD_READ_DATA          0x03


// These may not be needed, or might just be smaller
uint8_t Spi_tx_buf[8] ;

uint8_t Spi_rx_buf[8] ;

SPIClass SPIGD(HSPI) ;
uint8_t GdId[3] ;
uint16_t General_timer ;
uint16_t Model_timer ;

struct t_file_entry File_system[MAX_MODELS+1] ;

unsigned char ModelNames[MAX_MODELS+1][sizeof(g_model.name)+1] ;		// Allow for general

uint8_t Ee32_general_write_pending ;
uint8_t Ee32_model_write_pending ;
uint8_t Ee32_model_delete_pending ;

uint8_t	Eeprom32_process_state ;
uint8_t	Eeprom32_state_after_erase ;
uint8_t	Eeprom32_write_pending ;
uint8_t Eeprom32_file_index ;
uint8_t *Eeprom32_buffer_address ;
uint8_t *Eeprom32_source_address ;
uint32_t Eeprom32_address ;
uint32_t Eeprom32_data_size ;

#define EE_WAIT			0
#define EE_NO_WAIT	1


// States in Eeprom32_process_state
#define E32_IDLE							1
#define E32_ERASESENDING			2
#define E32_ERASEWAITING			3
#define E32_WRITESENDING			4
#define E32_WRITEWAITING			5
#define E32_READSENDING				6
#define E32_READWAITING				7
#define E32_BLANKCHECK				8
#define E32_WRITESTART				9
#define E32_LOCKED						10

bool eeModelExists(uint8_t id) ;
static uint32_t get_current_block_number( uint32_t block_no, uint16_t *p_size, uint32_t *p_seq ) ;
static uint32_t read32_eeprom_data( uint32_t eeAddress, register uint8_t *buffer, uint32_t size, uint32_t immediate ) ;
static uint32_t write32_eeprom_block( uint32_t eeAddress, register uint8_t *buffer, uint32_t size, uint32_t immediate ) ;
static void ee32_read_model_names( void ) ;
static void ee32LoadModelName(uint8_t id, unsigned char*buf,uint8_t len) ;
void ee32_update_name( uint32_t id, uint8_t *source ) ;
static uint32_t spi_operation( register uint8_t *tx, register uint8_t *rx, register uint32_t count ) ;
static bool ee32LoadGeneral( void ) ;
uint32_t ee32_check_finished() ;

static uint32_t eeprom_write_one( uint8_t byte, uint8_t count ) ;

extern SemaphoreHandle_t SpiMutex ;

void startSpiMemory()
{
  pinMode(GDCSPIN, OUTPUT);
	SPIGD.begin(13, 12, 15, 0) ; // sck, miso, mosi, ss (ss can be any GPIO)
	SPIGD.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0)) ;
  pinMode(GDCSPIN, OUTPUT) ;
  digitalWrite( GDCSPIN, HIGH ) ;
  delay(2) ;
  digitalWrite( GDCSPIN, LOW ) ;
  SPIGD.transfer(0x9F) ;
	SPIGD.transferBytes(GdId, GdId, 3 ) ;
  digitalWrite( GDCSPIN, HIGH ) ;

	LFS_Config() ;
	LfsBegin() ;
}

uint16_t evalChkSum()
{
  uint16_t sum=0;
	uint16_t *p ;
	p = ( uint16_t *)g_eeGeneral.calibMid ;
  for (int i=0; i<12;i++)
	{
    sum += *p++ ;
	}
  return sum;
}

static uint32_t spi_PDC_action( uint8_t *command, uint8_t *tx, uint8_t *rx, uint32_t comlen, uint32_t count )
{
	uint32_t condition ;
	static uint8_t discard_rx_command[4] ;

	Spi_complete = 0 ;
	if ( comlen > 4 )
	{
		Spi_complete = 1 ;
		return 0x4FFFF ;		
	}
	
  digitalWrite( GDCSPIN, LOW ) ;
	if  ( comlen )
	{
  	SPIGD.transferBytes(command, 0, comlen) ;
	}

	if ( tx == 0 )
	{
		tx = rx ;
	}
  if (count)
	{
		SPIGD.transferBytes( tx, rx, count ) ;
	}
  digitalWrite( GDCSPIN, HIGH ) ;
	Spi_complete = 1 ;
	return 0 ;
}

static void eeprom_write_enable()
{
	eeprom_write_one( 6, 0 ) ;
}

static uint8_t eeprom_read_status()
{
	return eeprom_write_one( 5, 1 ) ;
}

static uint32_t eeprom_write_one( uint8_t byte, uint8_t count )
{
	uint8_t received ;
  digitalWrite( GDCSPIN, LOW ) ;
  received = SPIGD.transfer(byte) ;
	if ( count == 0 )
	{
  	digitalWrite( GDCSPIN, HIGH ) ;
		return received ;
	}
  received = SPIGD.transfer(0) ;
 	digitalWrite( GDCSPIN, HIGH ) ;
	return received ;
}

// genaral data needs to be written to EEPROM
void ee32StoreGeneral()
{
	Dirty.General_dirty = 1 ;
	General_timer = 500 ;		// 5 seconds timeout before writing
}

void eeModelChanged()
{
	if ( Model_timer == 0 )
	{
		ee32StoreModel( g_eeGeneral.currModel, EE_MODEL ) ;
		Model_timer = 30000 ;	// 5 minutes
	}
}

// Store model to EEPROM, trim is non-zero if this is the result of a trim change
void ee32StoreModel( uint8_t modelNumber, uint8_t trim )
{
	Dirty.Model_dirty = modelNumber + 1 ;
	Model_timer = trim ? 12000 : 500 ;	// 2 minutes or 5 secs
	ee32_update_name( Dirty.Model_dirty, (uint8_t *)&g_model ) ;		// In case it's changed
}

void ee32_delete_model( uint8_t id )
{
	uint8_t buffer[sizeof(g_model.name)+1] ;
  memset( buffer, ' ', sizeof(g_model.name) ) ;
	ee32_update_name( id + 1, buffer ) ;
	Ee32_model_delete_pending = id + 1 ;
	ee32_process() ;		// Kick it off
	ee32WaitFinished() ;
}

void ee32WaitFinished()
{
  while ( ee32_check_finished() == 0 )
	{
		if ( General_timer )
		{
			General_timer = 1 ;		// Make these happen soon
		}
		if ( Model_timer )
		{
			Model_timer = 1 ;
		}
    ee32_process() ;
  }
}


static void ee32_read_model_names()
{
	uint32_t i ;

	for ( i = 1 ; i <= MAX_MODELS ; i += 1 )
	{
		ee32LoadModelName( i, ModelNames[i], sizeof(g_model.name) ) ;
	}
}

void ee32_update_name( uint32_t id, uint8_t *source )
{
	uint8_t * p ;
	uint32_t i ;

	p = ModelNames[id] ;
	for ( i = 0 ; i < sizeof(g_model.name) ; i += 1 )
	{
		*p++ = *source++ ;
	}
//	*p = '\0' ;
}

// Read eeprom data starting at random address
static uint32_t read32_eeprom_data( uint32_t eeAddress, register uint8_t *buffer, uint32_t size, uint32_t immediate )
{
	uint8_t *p ;
	uint32_t x ;

	p = Spi_tx_buf ;
	*p = 3 ;		// Read command
	*(p+1) = eeAddress >> 16 ;
	*(p+2) = eeAddress >> 8 ;
	*(p+3) = eeAddress ;		// 3 bytes address
	spi_PDC_action( p, 0, buffer, 4, size ) ;

	return 0 ;

//	if ( immediate )
//	{
//		return 0 ;		
//	}
//	for ( x = 0 ; x < 1000000 ; x += 1  )
//	{
//		if ( Spi_complete )
//		{
//			break ;				
//		}
//	}

//	return x ;
}


static uint32_t write32_eeprom_block( uint32_t eeAddress, register uint8_t *buffer, uint32_t size, uint32_t immediate )
{
	register uint8_t *p ;
	register uint32_t x ;

	eeprom_write_enable() ;

	p = Spi_tx_buf ;
	*p = 2 ;		// Write command
	*(p+1) = eeAddress >> 16 ;
	*(p+2) = eeAddress >> 8 ;
	*(p+3) = eeAddress ;		// 3 bytes address
	spi_PDC_action( p, buffer, 0, 4, size ) ;

	return 0 ;		

//	if ( immediate )
//	{
//		return 0 ;		
//	}
//	for ( x = 0 ; x < 100000 ; x += 1  )
//	{
//		if ( Spi_complete )
//		{
//			break ;				
//		}        			
//	}
//	return x ; 
}

uint8_t byte_checksum( uint8_t *p, uint32_t size )
{
	uint32_t csum ;

	csum = 0 ;
	while( size )
	{
		csum += *p++ ;
		size -= 1 ;
	}
	return csum ;
}

uint32_t ee32_check_header( struct t_eeprom_header *hptr )
{
	uint8_t csum ;

	csum = byte_checksum( ( uint8_t *) hptr, 7 ) ;
	if ( csum == hptr->hcsum )
	{
		return 1 ;
	}
	return 0 ;
}


	struct t_eeprom_header b0 ;
	struct t_eeprom_header b1 ;

// Pass in an even block number, this and the next block will be checked
// to see which is the most recent, the block_no of the most recent
// is returned, with the corresponding data size if required
// and the sequence number if required
static uint32_t get_current_block_number( uint32_t block_no, uint16_t *p_size, uint32_t *p_seq )
{
  uint32_t sequence_no ;
  uint16_t size ;
	read32_eeprom_data( block_no << 12, ( uint8_t *)&b0, sizeof(b0), EE_WAIT ) ;		// Sequence # 0
	read32_eeprom_data( (block_no+1) << 12, ( uint8_t *)&b1, sizeof(b1), EE_WAIT ) ;	// Sequence # 1

	if ( ee32_check_header( &b0 ) == 0 )
	{
		b0.sequence_no = 0 ;
		b0.data_size = 0 ;
		b0.flags = 0 ;
	}

	size = b0.data_size ;
  sequence_no = b0.sequence_no ;
	if ( ee32_check_header( &b0 ) == 0 )
	{
		if ( ee32_check_header( &b1 ) != 0 )
		{
  		size = b1.data_size ;
		  sequence_no = b1.sequence_no ;
			block_no += 1 ;
		}
		else
		{
			size = 0 ;
			sequence_no = 1 ;
		}
	}
	else
	{
		if ( ee32_check_header( &b1 ) != 0 )
		{
			
			if ( b1.sequence_no > b0.sequence_no )
			{
	  		size = b1.data_size ;
			  sequence_no = b1.sequence_no ;
				block_no += 1 ;
			}
		}
	}
  
	if ( size == 0xFFFF )
	{
		size = 0 ;
	}
  if ( p_size )
	{
		*p_size = size ;
	}
  if ( sequence_no == 0xFFFFFFFF )
	{  
		sequence_no = 0 ;
	}
  if ( p_seq )
	{
		*p_seq = sequence_no ;
	}
//	Block_needs_erasing = erase ;		
  
	return block_no ;
}


static bool ee32LoadGeneral()
{
	uint16_t size ;
	size = File_system[0].size ;

  memset(&g_eeGeneral, 0, sizeof(EE_X20General));

	if ( size > sizeof(EE_X20General) )
	{
		size = sizeof(EE_X20General) ;
	}

	if ( size )
	{
		read32_eeprom_data( ( File_system[0].block_no << 12) + sizeof( struct t_eeprom_header), ( uint8_t *)&g_eeGeneral, size, 0 ) ;
	}
	else
	{
		return false ;		// No data to load
	}

//  if(g_eeGeneral.myVers < MDX20VERS)
//      sysFlags = sysFLAG_OLD_EEPROM; // if old EEPROM - Raise flag

  g_eeGeneral.myVers   =  MDX20VERS ; // update myvers

	if ( g_eeGeneral.speakerPitch > 20 )
	{
		g_eeGeneral.speakerPitch = 20 ;
	}

  uint16_t sum=0;
  if(size>(43)) sum = evalChkSum() ;
	else return false ;
  return g_eeGeneral.chkSum == sum ;
}



void ee32_process()
{
	uint8_t *p ;
	uint8_t *q ;
	uint32_t x ;
	uint32_t eeAddress ;

	if ( General_timer )
	{
		if ( --General_timer == 0 )
		{
			// Time to write g_eeGeneral
			Ee32_general_write_pending = 1 ;
		}
	}
	if ( Model_timer )
	{
		if ( --Model_timer == 0 )
		{
			// Time to write model
			Ee32_model_write_pending = 1 ;
		}
	}

	if ( Eeprom32_process_state == E32_IDLE )
	{
		if ( ( Ee32_general_write_pending ) ||
			( Ee32_model_write_pending ) ||
			( Ee32_model_delete_pending ) )
		{
			while( xSemaphoreTake( SpiMutex, pdMS_TO_TICKS( 2 ) ) != pdTRUE )
			{
				// wait
			}
		}
		
		if ( Ee32_general_write_pending )
		{
			Ee32_general_write_pending = 0 ;			// clear flag

			// Check we can write, == block is blank

			Eeprom32_source_address = (uint8_t *)&g_eeGeneral ;		// Get data fromm here
			Eeprom32_data_size = sizeof(g_eeGeneral) ;						// This much
			Eeprom32_file_index = 0 ;								// This file system entry
			Eeprom32_process_state = E32_BLANKCHECK ;
		}
		else if ( Ee32_model_write_pending )
		{
			Ee32_model_write_pending = 0 ;			// clear flag

			// Check we can write, == block is blank

			Eeprom32_source_address = (uint8_t *)&g_model ;		// Get data from here
			Eeprom32_data_size = sizeof(g_model) ;						// This much
			Eeprom32_file_index = Dirty.Model_dirty ;								// This file system entry
			Eeprom32_process_state = E32_BLANKCHECK ;
//			Writing_model = Dirty.Model_dirty ;
		}
		else if ( Ee32_model_delete_pending )
		{
			Eeprom32_source_address = (uint8_t *)&g_model ;		// Get data from here
			Eeprom32_data_size = 0 ;													// This much
			Eeprom32_file_index = Ee32_model_delete_pending ;	// This file system entry
			Ee32_model_delete_pending = 0 ;
			Eeprom32_process_state = E32_BLANKCHECK ;
		}
	}

	if ( Eeprom32_process_state == E32_BLANKCHECK )
	{
//		DebugEeprom |= 1 ;
		eeAddress = File_system[Eeprom32_file_index].block_no ^ 1 ;
		eeAddress <<= 12 ;		// Block start address
		Eeprom32_address = eeAddress ;						// Where to put new data
				eeprom_write_enable() ;
//extern uint8_t EepromStatus ;
//				EepromStatus = eeprom_read_status() ;

				p = Spi_tx_buf ;
				*p = 0x20 ;		// Block Erase command
				*(p+1) = eeAddress >> 16 ;
				*(p+2) = eeAddress >> 8 ;
				*(p+3) = eeAddress ;		// 3 bytes address
				spi_PDC_action( p, 0, 0, 4, 0 ) ;
				Eeprom32_process_state = E32_ERASESENDING ;
				Eeprom32_state_after_erase = E32_WRITESTART ;
	}

	if ( Eeprom32_process_state == E32_WRITESTART )
	{
		uint32_t total_size ;
		p = Eeprom32_source_address ;
		q = (uint8_t *)&Eeprom_buffer.data ;
    if (p != q)
		{
			for ( x = 0 ; x < Eeprom32_data_size ; x += 1 )
			{
				*q++ = *p++ ;			// Copy the data to temp buffer
			}
		}
		Eeprom_buffer.header.sequence_no = ++File_system[Eeprom32_file_index].sequence_no ;
		File_system[Eeprom32_file_index].size = Eeprom_buffer.header.data_size = Eeprom32_data_size ;
		Eeprom_buffer.header.flags = 0 ;
		Eeprom_buffer.header.hcsum = byte_checksum( (uint8_t *)&Eeprom_buffer, 7 ) ;
		total_size = Eeprom32_data_size + sizeof( struct t_eeprom_header ) ;
		eeAddress = Eeprom32_address ;		// Block start address
		x = total_size / 256 ;	// # sub blocks
		x <<= 8 ;						// to offset address
		eeAddress += x ;		// Add it in
		p = (uint8_t *) &Eeprom_buffer ;
		p += x ;						// Add offset
		x = total_size % 256 ;	// Size of last bit
		if ( x == 0 )						// Last bit empty
		{
			x = 256 ;
			p -= x ;
			eeAddress -= x ;
		}
		Eeprom32_buffer_address = p ;
		Eeprom32_address = eeAddress ;
		eeprom_write_enable() ;
		write32_eeprom_block( eeAddress, p, x, 1 ) ;
		Eeprom32_process_state = E32_WRITESENDING ;
	}

	if ( Eeprom32_process_state == E32_WRITESENDING )
	{
		if ( Spi_complete )
		{
			Eeprom32_process_state = E32_WRITEWAITING ;
		}			
	}		

	if ( Eeprom32_process_state == E32_WRITEWAITING )
	{
		x = eeprom_read_status() ;
		if ( ( x & 1 ) == 0 )
		{
			if ( ( Eeprom32_address & 0x0FFF ) != 0 )		// More to write
			{
				Eeprom32_address -= 256 ;
				Eeprom32_buffer_address -= 256 ;
				write32_eeprom_block( Eeprom32_address, Eeprom32_buffer_address, 256, 1 ) ;
				Eeprom32_process_state = E32_WRITESENDING ;
			}
			else
			{
				File_system[Eeprom32_file_index].block_no ^= 1 ;		// This is now the current block
				xSemaphoreGive( SpiMutex ) ;
				Eeprom32_process_state = E32_IDLE ;
			}
		}
	}	

	if ( Eeprom32_process_state == E32_ERASESENDING )
	{
		if ( Spi_complete )
		{
			Eeprom32_process_state = E32_ERASEWAITING ;
		}			
	}	
		
	if ( Eeprom32_process_state == E32_ERASEWAITING )
	{
		x = eeprom_read_status() ;
		if ( ( x & 1 ) == 0 )
		{ // Command finished
			Eeprom32_process_state = Eeprom32_state_after_erase ;
		}			
	}
}

static uint32_t unprotect_eeprom()
{
 	register uint8_t *p ;
	uint32_t result ;
	eeprom_write_enable() ;
		
	p = Spi_tx_buf ;
	*p = 0x39 ;		// Unprotect sector command
	*(p+1) = 0 ;
	*(p+2) = 0 ;
	*(p+3) = 0 ;		// 3 bytes address

	result = spi_operation( p, Spi_rx_buf, 4 ) ;
	return result ;
}


static uint32_t spi_operation( register uint8_t *tx, register uint8_t *rx, register uint32_t count )
{
	uint32_t result ;
	uint8_t received ;

	result = 0 ; 
	
  digitalWrite( GDCSPIN, LOW ) ;
	while( count )
	{
		result = 0 ;
  	received = SPIGD.transfer(*tx++) ;
		*rx++ = received ;
	}
  digitalWrite( GDCSPIN, HIGH ) ;
	return result ;
}


//void flash_erase_block( uint32_t page )
//{
//	uint8_t x ;
//  uint32_t address ;
//  address = page << 12 ;
//	eeprom_write_enable() ;
//  digitalWrite( GDCSPIN, LOW ) ;
//	SPIGD.transfer(0x20) ;
//  SPIGD.transfer((address >> 16) & 0xFF);
//  SPIGD.transfer((address >> 8) & 0xFF);
//  SPIGD.transfer(address & 0xFF);
//  digitalWrite( GDCSPIN, HIGH ) ;
//	do
//	{
//		x = eeprom_read_status() ;
//	} while ( x & 1 ) ;
//}





//void flash_read_pages( uint8_t *data, uint32_t page, uint32_t nPages)
//{
//  uint32_t address ;
//  uint32_t count ;
  
//  digitalWrite( GDCSPIN, LOW ) ;
  
//	SPIGD.transfer(CMD_READ_DATA);
//  // Send the 3 byte address
//  address = page << 8 ;
//	count = nPages * 256 ;
//  SPIGD.transfer((address >> 16) & 0xFF);
//  SPIGD.transfer((address >> 8) & 0xFF);
//  SPIGD.transfer(address & 0xFF);
//  // Now read the page's data bytes
//  for( uint16_t i = 0 ; i < count ; i += 1 )
//	{
//    *data++ = SPIGD.transfer(0) ;
//  }
//  digitalWrite( GDCSPIN, HIGH ) ;
//}


uint8_t *ncpystr( uint8_t *dest, uint8_t *source, uint8_t count )
{
  while ( (*dest++ = *source++) )
	{
		if ( --count == 0 )
		{
			*dest++ = '\0' ;
			break ;
		}
	}	
  return dest - 1 ;
}

void eeDirty(uint8_t msk)
{
  if(!msk) return;

	// New file system operations
	if ( msk & EE_GENERAL )
	{
		ee32StoreGeneral() ;
	}
	if ( msk & EE_MODEL )
	{
		ee32StoreModel( g_eeGeneral.currModel, msk & EE_TRIM ) ;
	}
}

void generalDefault()
{
  memset(&g_eeGeneral,0,sizeof(g_eeGeneral));
  g_eeGeneral.myVers   =  MDX20VERS;
  g_eeGeneral.currModel=  0;

  g_eeGeneral.vBatWarn = 70 ;
  
	g_eeGeneral.stickMode=  1;
	g_eeGeneral.bright = 50 ;
	g_eeGeneral.volume = 2 ;
	g_eeGeneral.lightSw = MAX_SKYDRSWITCH ;	// ON
	
	g_eeGeneral.beeperVal = 3 ;
	g_eeGeneral.gpsFormat = 1 ;

//	g_eeGeneral.rotaryDivisor = 2 ;
	g_eeGeneral.hapticStrength = 4 ;

  for (int i = 0; i < 7 ; ++i )
	{
		g_eeGeneral.calibMid[i] = 0x800 ;
		g_eeGeneral.calibSpanPos[i] = 0x600 ;
		g_eeGeneral.calibSpanNeg[i] = 0x600 ;
  }
  ncpystr((uint8_t*)g_eeGeneral.ownerName,(uint8_t*)"ME        ", 10 ) ;
  g_eeGeneral.chkSum = evalChkSum() ;
	eeDirty(EE_GENERAL) ;
//	sysFlags |= sysFLAG_FORMAT_EEPROM ;

}

void modelDefault(uint8_t id)
{
  memset(&g_model, 0, sizeof(X20ModelData));
  ncpystr((uint8_t*)g_model.name,(uint8_t*)PSTR(STR_MODEL), 10 );
  g_model.name[5]='0'+(id+1)/10;
  g_model.name[6]='0'+(id+1)%10;
	g_model.modelVersion = 4 ;
	g_model.trimInc = 2 ;

  applyTemplate(0); //default 4 channel template

	// Set all mode trims to be copies of FM0
	for ( uint32_t i = 0 ; i < MAX_MODES ; i += 1 )
	{
		g_model.phaseData[i].trim[0] = TRIM_EXTENDED_MAX + 1 ;
		g_model.phaseData[i].trim[1] = TRIM_EXTENDED_MAX + 1 ;
		g_model.phaseData[i].trim[2] = TRIM_EXTENDED_MAX + 1 ;
		g_model.phaseData[i].trim[3] = TRIM_EXTENDED_MAX + 1 ;
	}
	
//	g_model.Module[0].protocol = PROTO_OFF ;
//	g_model.Module[1].protocol = PROTO_OFF ;
//	g_model.modelVoice = -1 ;
//	g_model.Module[0].pxxRxNum = id-1 ;
//	g_model.Module[1].pxxRxNum = id-1 ;
	g_model.rxVratio = 132 ;
	eeDirty(EE_MODEL) ;
}


void ee32LoadModel(uint8_t id)
{
	uint16_t size ;
	uint8_t version = 255 ;

//  closeLogs() ;

  if(id<MAX_MODELS)
  {
		size =  File_system[id+1].size ;
		memset(&g_model, 0, sizeof(g_model));

		if ( size > sizeof(g_model) )
		{
			size = sizeof(g_model) ;
		}
			 
    if(size<256) // if not loaded a fair amount
    {
      modelDefault(id) ;
    }
		else
		{
			while( xSemaphoreTake( SpiMutex, pdMS_TO_TICKS( 2 ) ) != pdTRUE )
			{
				// wait
			}
			read32_eeprom_data( ( File_system[id+1].block_no << 12) + sizeof( struct t_eeprom_header), ( uint8_t *)&g_model, size, 0 ) ;
			xSemaphoreGive( SpiMutex ) ;
		}

		validateName( (uint8_t *)g_model.name, sizeof(g_model.name) ) ;



//	ppmInValid = 0 ;

  }

// // Sort telemetry options
//	if ( g_model.telemetryProtocol == TELEMETRY_UNDEFINED )
//	{
//		g_model.telemetryProtocol = TELEMETRY_FRSKY ;	// default
//		if ( g_model.FrSkyUsrProto )
//		{
//			g_model.telemetryProtocol = TELEMETRY_WSHHI ;
//		}
//		if ( g_model.DsmTelemetry )
//		{
//			g_model.telemetryProtocol = TELEMETRY_DSM ;
//		}
//	}

	checkXyCurve() ;
	
//	if ( g_model.protocol > PROT_MAX )
//	{
//		if ( g_model.protocol != PROTO_OFF )
//		{
//			g_model.protocol = 0 ;
//		}
//	}
//	validateProtocolOptions( 0 ) ;
//	validateProtocolOptions( 1 ) ;

//	AltitudeZeroed = 0 ;
}




void eeReadAll()
{
	if(!ee32LoadGeneral() )
	{
    generalDefault() ;
    modelDefault(0) ;
		ee32StoreGeneral() ;
		ee32StoreModel( 0, 0 ) ;
	}
	else
	{
  	ee32LoadModel(g_eeGeneral.currModel) ;
	}
}


uint32_t ee32_check_finished()
{
	if ( ( Eeprom32_process_state != E32_IDLE )
			|| ( General_timer )
			|| ( Model_timer )
			|| ( Ee32_model_delete_pending)
			|| ( Ee32_general_write_pending)
			|| ( Ee32_model_write_pending)
			)
	{
		if ( General_timer )
		{
			General_timer = 1 ;		// Make these happen soon
		}
		if ( Model_timer )
		{
			Model_timer = 1 ;
		}
		ee32_process() ;
		return 0 ;
	}
	return 1 ;
}

bool eeModelExists(uint8_t id)
{
	return ( File_system[id+1].size > 0 ) ;
}

static void ee32LoadModelName( uint8_t id, unsigned char*buf, uint8_t len )
{
	if(id<=MAX_MODELS)
  {
		memset(buf,' ',len);
		if ( File_system[id].size > sizeof(g_model.name) )
		{
			while( xSemaphoreTake( SpiMutex, pdMS_TO_TICKS( 2 ) ) != pdTRUE )
			{
				// wait
			}
			read32_eeprom_data( ( File_system[id].block_no << 12) + 8, ( uint8_t *)buf, sizeof(g_model.name), 0 ) ;
			xSemaphoreGive( SpiMutex ) ;
		}
  }
}

static void fill_file_index()
{
	uint32_t i ;
	for ( i = 0 ; i < MAX_MODELS + 1 ; i += 1 )
	{
		File_system[i].block_no = get_current_block_number( i * 2, &File_system[i].size, &File_system[i].sequence_no ) ;
	}
	i = 0 ;
	File_system[i].block_no = get_current_block_number( i * 2, &File_system[i].size, &File_system[i].sequence_no ) ;
}

void init_eeprom()
{
	fill_file_index() ;
	ee32_read_model_names() ;
	Eeprom32_process_state = E32_IDLE ;
}


void waitForEepromFinished()
{
	while (ee32_check_finished() == 0)
	{	// wait
#ifndef SIMU
		if ( General_timer )
		{
			General_timer = 1 ;		// Make these happen soon
		}
		if ( Model_timer )
		{
			Model_timer = 1 ;
		}
   	vTaskDelay( pdMS_TO_TICKS( 2 ) ) ;
#endif
	}
}

bool ee32CopyModel(uint8_t dst, uint8_t src)
{
  uint16_t size = File_system[src].size ;
	
	waitForEepromFinished() ;
	while (ee32_check_finished() == 0)
	{	// wait
#ifndef SIMU
		if ( General_timer )
		{
			General_timer = 1 ;		// Make these happen soon
		}
		if ( Model_timer )
		{
			Model_timer = 1 ;
		}
   	vTaskDelay( pdMS_TO_TICKS( 2 ) ) ;
#endif
	}
	while( xSemaphoreTake( SpiMutex, pdMS_TO_TICKS( 2 ) ) != pdTRUE )
	{
		// wait
	}

  read32_eeprom_data( (File_system[src].block_no << 12) + sizeof( struct t_eeprom_header), ( uint8_t *)&Eeprom_buffer.data.sky_model_data, size, 0 ) ;
//	xSemaphoreGive( SpiMutex ) ;

  if (size > sizeof(g_model.name))
    memcpy( ModelNames[dst], Eeprom_buffer.data.sky_model_data.name, sizeof(g_model.name)) ;
  else
    memset( ModelNames[dst], ' ', sizeof(g_model.name)) ;

//	while( xSemaphoreTake( SpiMutex, pdMS_TO_TICKS( 2 ) ) != pdTRUE )
//	{
//		// wait
//	}
  Eeprom32_source_address = (uint8_t *)&Eeprom_buffer.data.sky_model_data ;		// Get data from here
  Eeprom32_data_size = sizeof(g_model) ;																	// This much
  Eeprom32_file_index = dst ;																							// This file system entry
  Eeprom32_process_state = E32_BLANKCHECK ;
	// Semaphore returned by ee32_process()
  ee32WaitFinished() ;
  return true;
}


void ee32SwapModels(uint8_t id1, uint8_t id2)
{
  ee32WaitFinished();
//  // eeCheck(true) should have been called before entering here

  uint32_t id2_block_no ;
  uint16_t id2_size = File_system[id2].size ;
  uint16_t id1_size = File_system[id1].size ;
	
	if ( id1_size == 0 )
	{ // Copying blank entry
		if ( id2_size == 0 )
		{ // To blank entry
			return ;	// Nothing to do
		}
		// Copy blank entry to model, swap the copy
		uint8_t temp = id1 ;
		id1 = id2 ;
		id2 = temp ;
	}
  
  id2_size = File_system[id2].size ;
	id2_block_no = File_system[id2].block_no ;

  ee32CopyModel(id2, id1);

//  // block_no(id1) has been shifted now, but we have the size
  if (id2_size > sizeof(g_model.name))
	{
		while( xSemaphoreTake( SpiMutex, pdMS_TO_TICKS( 2 ) ) != pdTRUE )
		{
			// wait
		}
    read32_eeprom_data( (id2_block_no << 12) + sizeof( struct t_eeprom_header), ( uint8_t *)&Eeprom_buffer.data.sky_model_data, id2_size, 0 ) ;
		xSemaphoreGive( SpiMutex ) ;
    memcpy( ModelNames[id1], Eeprom_buffer.data.sky_model_data.name, sizeof(g_model.name)) ;
  }
  else
	{
    memset( ModelNames[id1], ' ', sizeof(g_model.name)) ;
		id2_size = 0 ;
  }

  Eeprom32_source_address = (uint8_t *)&Eeprom_buffer.data.sky_model_data ;		// Get data from here
  Eeprom32_data_size = id2_size ;																					// This much
  Eeprom32_file_index = id1 ;																							// This file system entry
  Eeprom32_process_state = E32_BLANKCHECK ;
  ee32WaitFinished();
}

bool eeDuplicateModel(uint8_t id)
{
  uint32_t i;
  for( i=id ; i<MAX_MODELS; i++)
  {
    if(! eeModelExists(i) ) break;
  }
  if(i>=MAX_MODELS)
	{
  	for( i=0 ; i<id ; i++)
		{
    	if(! eeModelExists(i) ) break;
		}
  	if(i>=id) return false; //no free space in directory left
	}

	ee32CopyModel( i+1, id ) ;
//	ee32CopyModel( i, id ) ;

	return true ;
}

//void setModelAFilename( uint8_t *fname, uint8_t id )
//{
//	uint8_t *p ;
////	p = cpystr( fname, (uint8_t *)"RADIO/model" ) ;
//	p = cpystr( fname, (uint8_t *) RADIO_PATH "/model" ) ;
//	*p++ = '0'+(id+1)/10 ;
//	*p++ = '0'+(id+1)%10 ;
//	*p++ = 'A' ;
//	cpystr( p, (uint8_t *)".bin" ) ;
//}


void setModelFilename( uint8_t *filename, uint8_t modelIndex, uint32_t type )
{
	uint8_t *bptr ;
	uint32_t i ;
	
	bptr = cpystr( filename, type ? (uint8_t *)"/TEXT/" : (uint8_t *)"/MODELS/" ) ;
  memcpy( bptr, ModelNames[modelIndex], sizeof(g_model.name)) ;
	bptr += sizeof(g_model.name) - 1 ;
	for ( i = 0 ; i < sizeof(g_model.name) ; i += 1 )
	{
		if ( *bptr && ( *bptr != ' ' ) )
		{
			break ;
		}
		else
		{
			bptr -= 1 ;
		}
	}
	bptr += 1 ;
	if ( i >= sizeof(g_model.name) )
	{
		*bptr++ = 'x' ;
	}
	cpystr( bptr, type ? (uint8_t *)".txt" : (uint8_t *)".eepm" ) ;		// ".eepm"
}

//static const uint8_t base64digits[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/" ;






////	res = writeXMLfile( &LfsArchiveHandle, (uint8_t *)&TempModelData, sizeof(g_model), &written, ModelNames[modelIndex] ) ;
//uint32_t writeXMLfile( lfs_file_t *archiveFile, uint8_t *data, uint32_t size )
//{
	
//}


//lfs_file_t LfsArchiveHandle ;

//const char *ee32BackupModel( uint8_t modelIndex )
//{
//  uint16_t size ;
////	int result ;
////  DIR archiveFolder ;
////  FIL archiveFile ;
////  UINT written ;
//	uint8_t filename[50] ;
//	uint32_t res ;
//	lfs_dir_t my_dir ;

//  size = File_system[modelIndex].size ;
	
//	waitForEepromFinished() ;
//	memset(( uint8_t *)&TempModelData, 0, sizeof(g_model));
//  read32_eeprom_data( (File_system[modelIndex].block_no << 12) + sizeof( struct t_eeprom_header), ( uint8_t *)&Eeprom_buffer.data.sky_model_data, size, 0 ) ;
	
//	setModelFilename( filename, modelIndex, FILE_TYPE_MODEL ) ;
	
//	res = lfs_dir_open( &Lfs, &my_dir, "/MODELS" ) ;
//  if (res != 0)
//	{
////		WatchdogTimeout = 300 ;		// 3 seconds
//		res = lfs_mkdir( &Lfs, "/MODELS" ) ;
//   	if (res != 0)
//		{
//     	return "FILE ERROR" ;
//		}
//  }

//  res = lfs_file_open( &Lfs, &LfsArchiveHandle, (char *)filename, LFS_O_RDWR | LFS_O_CREAT | LFS_O_TRUNC ) ;
//  if (res != 0)
//	{
//   	return "CREATE ERROR" ;
//  }

//	res = writeXMLfile( &LfsArchiveHandle, (uint8_t *)&TempModelData, sizeof(g_model) ) ;
  
//	res = lfs_file_close(&Lfs, &LfsArchiveHandle ) ;
//  if (res != 0 ) //	|| written != size)
//	{
//    return "WRITE ERROR" ;
//  }

//  return "MODEL SAVED" ;
//}

// Drivers for LittleFs

// NEED to handle conflicts with above
int block_device_read(const struct lfs_config *c, lfs_block_t block,
	lfs_off_t off, void *buffer, lfs_size_t size)
{
	while( xSemaphoreTake( SpiMutex, pdMS_TO_TICKS( 2 ) ) != pdTRUE )
	{
		// wait
	}
	read32_eeprom_data( (block * c->block_size + off) + 4096 * 128, (uint8_t *)buffer, (uint32_t) size, 0 ) ;
	xSemaphoreGive( SpiMutex ) ;
	return 0;
}

// NEED to handle conflicts with above
int block_device_prog(const struct lfs_config *c, lfs_block_t block,
	lfs_off_t off, const void *buffer, lfs_size_t size)
{
	eeprom_write_enable() ;
	write32_eeprom_block( (block * c->block_size + off) + 4096 * 128, (uint8_t *)buffer, (uint32_t) size, 1 ) ;
	while ( eeprom_read_status() & 1 )
	{
		// wait for now
	}
	return 0;
}

int block_device_erase(const struct lfs_config *c, lfs_block_t block)
{
	uint8_t *p ;
	uint32_t eeAddress ;
//	W25X_Erase_Sector(block * c->block_size);
	eeAddress = (block * c->block_size) + 4096 * 128 ;
	eeprom_write_enable() ;
	p = Spi_tx_buf ;
	*p = 0x20 ;		// Block Erase command
	*(p+1) = eeAddress >> 16 ;
	*(p+2) = eeAddress >> 8 ;
	*(p+3) = eeAddress ;		// 3 bytes address
	spi_PDC_action( p, 0, 0, 4, 0 ) ;
	while ( eeprom_read_status() & 1 )
	{
		// wait for now
	}
	
	return 0;
}

int block_device_sync(const struct lfs_config *c)
{
	return 0;
}

void LFS_Config(void)
{
	// block device operations
	Cfg.read  = block_device_read ;
	Cfg.prog  = block_device_prog ;
	Cfg.erase = block_device_erase ;
	Cfg.sync  = block_device_sync ;

	// block device configuration
	Cfg.read_size = 256 ;
	Cfg.prog_size = 256 ;
	Cfg.block_size = 4096 ;
	Cfg.block_count = 3968 ;
	Cfg.lookahead_size = 256 ;
	
	Cfg.read_buffer = Lfs_read_buf ;
	Cfg.prog_buffer = Lfs_prog_buf ;
	Cfg.lookahead_buffer = Lfs_lookahead_buf ;
//	Cfg.file_buffer = Lfs_file_buf ;

	Cfg.cache_size = 256 ;
	Cfg.block_cycles = 500 ;
}

extern "C"{

void myconfig(struct lfs_config *cfg)
{
        cfg->read  = block_device_read;
        cfg->prog  = block_device_prog;
        cfg->erase = block_device_erase;
        cfg->sync  = block_device_sync;

        // block device configuration
        cfg->read_size = 256;
        cfg->prog_size = 256;
        cfg->block_size = 4096; 
        cfg->block_count = 3968;
        cfg->cache_size = 256;
        cfg->lookahead_size = 256;
        cfg->block_cycles = 500;
}		
}

void LfsBegin()
{
	int err = lfs_mount( &Lfs, &Cfg ) ;
	// reformat if we can't mount the filesystem
	// this should only happen on the first boot
	if (err)
	{
    lfs_format( &Lfs, &Cfg ) ;
    lfs_mount( &Lfs, &Cfg ) ;

		int32_t result ;
		result = lfs_mkdir( &Lfs, "/TEXT" ) ;
//		Serial.println(result) ;
		result = lfs_mkdir( &Lfs, "/voice" ) ;
//		Serial.println(result) ;
		result = lfs_mkdir( &Lfs, "/voice/system" ) ;
//		Serial.println(result) ;
		result = lfs_mkdir( &Lfs, "/voice/modelNames" ) ;
//		Serial.println(result) ;
		result = lfs_mkdir( &Lfs, "/voice/user" ) ;
//		Serial.println(result) ;
		result = lfs_mkdir( &Lfs, "/script" ) ;
//		Serial.println(result) ;
	}
}








