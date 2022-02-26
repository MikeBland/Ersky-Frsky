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


#include "erskyTx.h"
#include "myeeprom.h"
#include "telemetry.h"
#include "menus.h"
#include "fifo.h"

#define BASIC		1
extern struct t_fifo128 Script_fifo ;

enum CrossfireSensorIndexes {
  RX_RSSI1_INDEX,
  RX_RSSI2_INDEX,
  RX_QUALITY_INDEX,
  RX_SNR_INDEX,
  RX_ANTENNA_INDEX,
  RF_MODE_INDEX,
  TX_POWER_INDEX,
  TX_RSSI_INDEX,
  TX_QUALITY_INDEX,
  TX_SNR_INDEX,
  BATT_VOLTAGE_INDEX,
  BATT_CURRENT_INDEX,
  BATT_CAPACITY_INDEX,
  GPS_LATITUDE_INDEX,
  GPS_LONGITUDE_INDEX,
  GPS_GROUND_SPEED_INDEX,
  GPS_HEADING_INDEX,
  GPS_ALTITUDE_INDEX,
  GPS_SATELLITES_INDEX,
  ATTITUDE_PITCH_INDEX,
  ATTITUDE_ROLL_INDEX,
  ATTITUDE_YAW_INDEX,
  FLIGHT_MODE_INDEX,
  UNKNOWN_INDEX,
};

// Device address
#define BROADCAST_ADDRESS              0x00
#define RADIO_ADDRESS                  0xEA
#define MODULE_ADDRESS                 0xEE

// Frame id
#define CRSF_GPS_ID                         0x02
#define CRSF_BATTERY_ID                     0x08
#define CRSF_LINK_ID                        0x14
#define CRSF_CHANNELS_ID                    0x16
#define CRSF_ATTITUDE_ID                    0x1E
#define CRSF_FLIGHT_MODE_ID                 0x21
#define CRSF_PING_DEVICES_ID                0x28
#define CRSF_DEVICE_INFO_ID                 0x29
#define CRSF_REQUEST_SETTINGS_ID            0x2A
#define RADIO_ID			                      0x3A

#define TELEMETRY_RX_PACKET_SIZE	128

struct t_elrsConfig ElrsConfig ;

struct t_updateTiming
{
	uint32_t UpdateRate ;
	int32_t UpdateOffset ;
	uint16_t UpdateTimer ;
	uint16_t UpdateInterval ;
	uint16_t UpdateDelay ;
	uint16_t MinUpdateRate ;
	int16_t BaseUpdateOffset ;
	uint16_t CurrentRate ;
	int16_t CurrentLag ;
} ;

#define UPDATE_TIMEOUT	120


struct t_updateTiming UpdateTiming ;

uint8_t TelemetryRxBuffer[128] ;   // Receive buffer. 9 bytes (full packet), worst case 18 bytes with byte-stuffing (+1), 128 for XFIRE
uint8_t NumPktBytes = 0 ;

uint8_t SportStreamingStarted ;
uint8_t FrskyStreaming = 0 ;
uint8_t FrskyUsrStreaming = 0 ;
FrskyData FrskyTelemetry[9] ;

uint8_t AltitudeZeroed = 0 ;
uint8_t AltitudeDecimals ;
int16_t WholeAltitude ;
int16_t AltOffset ;

uint16_t XjtVersion ;

int16_t TelemetryData[TELEMETRYDATALENGTH] ;  // All 38 words
uint8_t TelemetryDataValid[TELEMETRYDATALENGTH] ;  // All 38 words

static uint8_t A1Received = 0 ;

void storeAltitude( int16_t value )
{
	TelemetryData[FR_ALT_BARO] = value ;
	TelemetryDataValid[FR_ALT_BARO] = 25 + g_model.telemetryTimeout ;
	if ( !AltitudeZeroed )
	{
		AltOffset = -TelemetryData[FR_ALT_BARO] ;
		if ( AltOffset )
		{
			AltitudeZeroed = 1 ;
		}
	}
}

uint16_t convertRxv( uint16_t value )
{
//	if ( ( FrskyTelemetryType != FRSKY_TEL_DSM ) && ( FrskyTelemetryType != FRSKY_TEL_AFH )
//			 && ( FrskyTelemetryType != FRSKY_TEL_HITEC ) )		// DSM or AFHDS2 or Hitec telemetry 
	{
		value *= g_model.rxVratio ;
		value /= 255 ;
	}
 	return value ;
}

uint16_t A1A2toScaledValue( uint8_t channel, uint8_t *dplaces )
{
	uint8_t val = TelemetryData[channel ? FR_A2_COPY : FR_A1_COPY] ;
	return scale_telem_value( val, channel, dplaces ) ;
}


void storeTelemetryData( uint8_t index, uint16_t value )
{
	if ( index == FR_ALT_BARO )
	{
		value *= 10 ;
		if ( AltitudeDecimals )
		{
			WholeAltitude = value ;
			index = FR_TRASH ;
		}
		else
		{
			storeAltitude( value ) ;
		}
	}

	if ( index == FR_ALT_BAROd )
	{
		if ( AltitudeDecimals == 0 )
		{
			AltitudeZeroed = 0 ;
		}
		AltitudeDecimals |= 1 ;
		if ( ( value > 9 ) || ( value < -9 ) )
		{
			AltitudeDecimals |= 2 ;
		}
		if ( AltitudeDecimals & 2 )
		{
			value /= 10 ;			
		}
		storeAltitude( WholeAltitude + ((WholeAltitude >= 0) ? value : -value) ) ;
		index = FR_ALT_BARO ;	// For max/min
	}

	if ( index == FR_SPORT_ALT )
	{
		index = FR_ALT_BARO ;         // For max and min
		storeAltitude(  value ) ;
	}
	
	if ( index == FR_SPORT_GALT )
	{
		index = TELEM_GPS_ALT ;         // For max and min
		TelemetryData[TELEM_GPS_ALT] = value ;
		TelemetryDataValid[TELEM_GPS_ALT] = 25 + g_model.telemetryTimeout ;
	}

	if ( index < TELEMETRYDATALENGTH )
	{
		TelemetryData[index] = value ;           /* ReSt */
		TelemetryDataValid[index] = 25 + g_model.telemetryTimeout ;
	}
}


void processSportData( uint8_t *packet, uint32_t receiver )
{
  uint8_t  prim   = packet[1];
//#if defined(LUA) || defined(BASIC)
//	if ( ( prim == 0x32 ) || ( (packet[3] & 0xF0) == 0x50 ) ) // || ( (packet[3] & 0xF0) == 0x10 ) )
//	{
//		postSportToScript( packet ) ;
//	}
//#endif

	if ( ( prim == DATA_FRAME ) || ( prim == 0x32 ) )
	{
		prim = packet[0] & 0x1F ;		// Sensor ID
		if ( packet[3] == 0xF1 )
		{ // Receiver specific
			uint8_t value = packet[4] ;
			if ( packet[2] == 1 )		// just RSSI
			{
				if ( SportStreamingStarted )
				{
					if ( value )
					{
						FrskyStreaming = FRSKY_TIMEOUT10ms * 3 ; // reset counter only if valid frsky packets are being detected
					}
				}
				else
				{
					if ( !( value == 0 ) )
					{
						SportStreamingStarted = 1 ;
						FrskyStreaming = FRSKY_TIMEOUT10ms * 3 ; // reset counter only if valid frsky packets are being detected
					}
				}
			}
			switch ( packet[2] )
			{
				case 1 :
	    		FrskyTelemetry[2].set(value, FR_RXRSI_COPY );	//FrskyHubData[] =  frskyTelemetry[2].value ;
//					setTxRssi( packet[5] ) ;			// packet[5] is  TX_RSSI for MULTI
//					setTxLqi( packet[7] ) ;			// packet[7] is  TX_LQI for MULTI
//					RssiSetTimer = 30 ;
				break ;

				case 2 :
			    FrskyTelemetry[0].set(value, FR_A1_COPY ); //FrskyHubData[] =  frskyTelemetry[0].value ;
					A1Received = 1 ;
				break ;
				case 4 :		// Battery from X8R
					if ( A1Received == 0 )
					{
			      FrskyTelemetry[0].set(value, FR_A1_COPY ); //FrskyHubData[] =  frskyTelemetry[0].value ;
					}
					storeTelemetryData( FR_RXV, value ) ;
				break ;
  		    
				case 3 :
					FrskyTelemetry[1].set(value, FR_A2_COPY ); //FrskyHubData[] =  frskyTelemetry[1].value ;
				break ;
    		  
//				case 5 : // SWR
//#if defined(PCBX9D) && (defined(REVPLUS) || defined(REV9E))
//					if ( !( XjtVersion != 0 && XjtVersion != 0xff ) )
//					{
//						if ( g_model.xprotocol != PROTO_PXX )
//						{						
//							value = 5 ;
//						}
//					}
//#endif
//					FrskyTelemetry[3].set(value, FR_TXRSI_COPY ); //FrskyHubData[] =  frskyTelemetry[3].value ;
//				break ;
				case 6 : // XJT VERSION
					XjtVersion = (*((uint16_t *)(packet+4))) ;
//#if defined(PCBX9D) && (defined(REVPLUS) || defined(REV9E))
//					if ( !( XjtVersion != 0 && XjtVersion != 0xff) )
//					{
//						if ( g_model.xprotocol != PROTO_PXX )
//						{						
//							frskyTelemetry[3].set( 5, FR_TXRSI_COPY ); //FrskyHubData[] =  frskyTelemetry[3].value ;
//						}
//					}
//#endif
				break ;
//				case 7 : // Multi Id
//					MultiId[0] = packet[4] ;
//					MultiId[1] = packet[5] ;
//					MultiId[2] = packet[6] ;
//					MultiId[3] = packet[7] ;
//				break ;
			}
		}
		else if ( packet[3] == 0xF0 )
		{
			if ( packet[2] == 0x10 )		// FrSky2 Frame loss count
			{
				storeTelemetryData( FR_VFR, packet[4] | (packet[5] << 8 ) ) ;
			}
		}
//		else if ( packet[3] == 0 )
//		{ // old sensors
//  	  FrskyUsrStreaming = 255 ; //FRSKY_USR_TIMEOUT10ms ; // reset counter only if valid frsky packets are being detected
//			FrskyStreaming = FRSKY_TIMEOUT10ms * 3 ; // reset counter only if valid frsky packets are being detected
//			uint16_t value = (*((uint16_t *)(packet+4))) ;
//			store_indexed_hub_data( packet[2], value ) ;
//		}
//		else
//		{ // new sensors
//			FrskyStreaming = FRSKY_TIMEOUT10ms * 3 ; // reset counter only if valid frsky packets are being detected
//  	  FrskyUsrStreaming = 255 ; //FRSKY_USR_TIMEOUT10ms ; // reset counter only if valid frsky packets are being detected
//			uint8_t id = (packet[3] << 4) | ( packet[2] >> 4 ) ;
//			uint32_t value = (*((uint32_t *)(packet+4))) ;

//			if ( (packet[3] & 0xF0) != 0x50 )
//			{
//				if ( ( packet[2] & 0x0F ) != 0 )
//				{
//			 		id = NON_STANDARD_ID_8 ;
//				}
//				switch ( id )
//				{
//					case ALT_ID_8 :
//						value = (int32_t)value / 10 ;
//						storeTelemetryData( FR_SPORT_ALT, value ) ;
//					break ;

//					case VARIO_ID_8 :
//						storeTelemetryData( FR_VSPD, value ) ;
//					break ;

//	//				case BETA_ALT_ID_8 :
//	//					value = (int32_t)value >> 8 ;
//	//					value = (int32_t)value ;
//	//					storeTelemetryData( FR_SPORT_ALT, value ) ;
//	//				break ;

//	////				case BETA_VARIO_ID_8 :
//	////					value = (int32_t)value >> 8 ;
//	////					storeTelemetryData( FR_VSPD, value ) ;
//	////				break ;

//	//				case CELLS_ID_8 :
//	//				{
//	//  	      uint8_t cells = value ;
//	//					cells >>= 4 ;
//	//  	      uint8_t battnumber = value ;
//	//					battnumber &= 0x0F ;
//	//					if ( prim == DATA_ID_FLVSS )
//	//					{
//	//	  				FrskyBattCells[0] = cells ;
//	//					}
//	//					else
//	//					{
//	//	  				FrskyBattCells[1] = cells ;
//	//						battnumber += 6 ;		
//	////						battnumber += FrskyBattCells[0] ;
//	//						cells += 6 ;
//	//					}
//	//					uint16_t cell ;

//	//					value >>= 8 ;
//	//					cell = value ;
//	//					store_cell_data( battnumber, cell ) ;
//	//					battnumber += 1 ;
//	//					if ( battnumber < cells )
//	//					{
//	//						value >>= 12 ;
//	//						cell = value ;
//	//						store_cell_data( battnumber, cell ) ;
//	//					}
//	//				}
//	//				break ;

//					case CURR_ID_8 :
//						storeTelemetryData( FR_CURRENT, value ) ;
//					break ;

//					case VFAS_ID_8 :
//						storeTelemetryData( FR_VOLTS, value / 10 ) ;
//						VfasVoltageTimer = 50 ;
//					break ;
				
//					case RPM_ID_8 :
//						storeTelemetryData( FR_RPM, value / 60 ) ;
//					break ;

//	//				case A3_ID_8 :
//	//				{	
//	//					uint16_t ratio = g_model.frsky.channels[0].ratio3_4 ;
//	//					if ( ratio == 0 )
//	//					{
//	//						ratio = 330 ;
//	//					}
//	//					value = value * ratio / 33000 ;
//	//					storeTelemetryData( FR_A3, value ) ;
//	//				}
//	//				break ;

//	//				case A4_ID_8 :
//	//				{	
//	//					uint16_t ratio = g_model.frsky.channels[1].ratio3_4 ;
//	//					if ( ratio == 0 )
//	//					{
//	//						ratio = 330 ;
//	//					}
//	//					value = value * ratio / 33000 ;					
//	//					storeTelemetryData( FR_A4, value ) ;
//	//				}
//	//				break ;

//					case T1_ID_8 :
//	//					if ( ( g_model.telemetryProtocol == TELEMETRY_ARDUCOPTER ) || ( g_model.telemetryProtocol == TELEMETRY_ARDUPLANE ) )
//	//					{
//	//						storeTelemetryData( FR_TEMP2, value ) ;
//	//					}
//	//					else
//						{
//							storeTelemetryData( FR_TEMP1, value ) ;
//						}
//					break ;
				
//					case T2_ID_8 :
//	//					if ( ( g_model.telemetryProtocol == TELEMETRY_ARDUCOPTER ) || ( g_model.telemetryProtocol == TELEMETRY_ARDUPLANE ) )
//	//					{
//	//						storeTelemetryData( FR_BASEMODE, value ) ;
//	//					}
//	//					else
//						{
//							storeTelemetryData( FR_TEMP2, value ) ;
//						}
//					break ;

//					case ACCX_ID_8 :
//						storeTelemetryData( FR_ACCX, value ) ;
//					break ;
				
//					case ACCY_ID_8 :
//						storeTelemetryData( FR_ACCY, value ) ;
//					break ;
			
//					case ACCZ_ID_8 :
//						storeTelemetryData( FR_ACCZ, value ) ;
//					break ;

//					case FUEL_ID_8 :
//	//					if ( ( g_model.telemetryProtocol == TELEMETRY_ARDUCOPTER ) || ( g_model.telemetryProtocol == TELEMETRY_ARDUPLANE ) )
//	//					{
//	//						storeTelemetryData( FR_TEMP1, value ) ;
//	//					}
//	//					else
//						{
//							storeTelemetryData( FR_FUEL, value ) ;
//						}
//					break ;

//					case GPS_ALT_ID_8 :
//						value = (int32_t)value / 10 ;
//						storeTelemetryData( FR_SPORT_GALT, value ) ;
//					break ;
				 
//	//				case GPS_LA_LO_ID_8 :
//	//				{	
//	////					Bits 31-30 00 = LAT min/10000 N
//	////					Bits 31-30 01 = LAT min/10000 S
//	////					Bits 31-30 10 = LON min/10000 E
//	////					Bits 31-30 11 = LON min/10000 W
//	//					uint32_t code = value >> 30 ;
//	//					value &= 0x3FFFFFFF ;
//	//					uint16_t bp ;
//	//					uint16_t ap ;
//	//					uint32_t temp ;
//	//					temp = value / 10000 ;
//	//					bp = (temp/ 60 * 100) + (temp % 60) ;
//	//		      ap = value % 10000;
//	//					if ( code & 2 )	// Long
//	//					{
//	//						storeTelemetryData( FR_GPS_LONG, bp ) ;
//	//						storeTelemetryData( FR_GPS_LONGd, ap ) ;
//	//						storeTelemetryData( FR_LONG_E_W, ( code & 1 ) ? 'W' : 'E' ) ;
//	//					}
//	//					else
//	//					{
//	//						storeTelemetryData( FR_GPS_LAT, bp ) ;
//	//						storeTelemetryData( FR_GPS_LATd, ap ) ;
//	//						storeTelemetryData( FR_LAT_N_S, ( code & 1 ) ? 'S' : 'N' ) ;
//	//					}
//	//				}
//	//				break ;
				
//					case GPS_HDG_ID_8 :
//						storeTelemetryData( FR_COURSE, value / 100 ) ;
//					break ;

//					case GPS_SPEED_ID_8 :
//						storeTelemetryData( FR_GPS_SPEED, value/1000 ) ;
//					break ;

//					case AIRSPEED_ID_8 :
//						storeTelemetryData( FR_AIRSPEED, value ) ;
//					break ;

//	//				// Rbox Battx
//	//				// 0xCCCCVVVV, C has 2dp, V has 3dp
//	//				case RBOX_BATT1_ID_8 :
//	//					storeTelemetryData( FR_RBOX_B1_V, (value & 0x0000FFFF)/10 ) ;	// To 2 dp
//	//					storeTelemetryData( FR_RBOX_B1_A, value >> 16 ) ;
//	//				break ;
				
//	//				case RBOX_BATT2_ID_8 :
//	//					storeTelemetryData( FR_RBOX_B2_V, (value & 0x0000FFFF)/10 ) ;	// To 2 dp
//	//					storeTelemetryData( FR_RBOX_B2_A, value >> 16 ) ;
//	//				break ;

//	//				// Rbox CNSP
//	//				// 0xBBBBAAAA, AAAA batt 1 mAh, BBBB batt 2 mAh
//	//				case RBOX_CNSP_ID_8 :
//	//					storeTelemetryData( FR_RBOX_B1_CAP, value & 0x0000FFFF ) ;
//	//					storeTelemetryData( FR_RBOX_B2_CAP, value >> 16 ) ;
//	//				break ;

//	//				// Rbox state
//	//				// Bits 0-15 Set if channel overload
//	//				// Bit 16 RX1IN overload
//	//				// Bit 17 RX2IN overload
//	//				// Bit 18 SBUS overload
//	//				// bit19 RX1_FAILSAFE
//	//				// Bit20 RX1_LOSTFRAME
//	//				// Bit21 RX2_FAILSAFE
//	//				// Bit22 RX2_LOSTFRAME
//	//				// Bit23 RX1_PHYSICAL_CONNECTION _LOST
//	//				// Bit24 RX2_PHYSICAL_CONNECTION _LOST
//	//				// Bit25 RX1_NO_SIGNAL
//	//				// Bit26 RX2_NO_SIGNAL
//	//				case RBOX_STATE_ID_8 :
//	//					storeTelemetryData( FR_RBOX_SERVO, value & 0x0000FFFF ) ;
//	//					storeTelemetryData( FR_RBOX_STATE, (value >> 16) & 0x07FF ) ;
//	//				break ;

//					case S6R_ID_8 :
//						S6Rdata.fieldIndex = packet[4] ;
//						if ( (S6Rdata.fieldIndex >= 0x9E) && (S6Rdata.fieldIndex <= 0xA0) )
//						{
//							S6Rdata.value = ( packet[5] << 8 ) | packet[6] ;
//						}
//						else
//						{
//							S6Rdata.value = packet[5] ;
//						}
//						S6Rdata.valid = 1 ;
//					break ;

//					case ESC_POWER_ID_8 :
//					// Low 16 bits volts 0.001 - 26.4
//					// High 16 bits amps 0.01 - 30
//						storeTelemetryData( FR_VOLTS, (value & 0x0000FFFF) / 10 ) ;
//						storeTelemetryData( FR_CURRENT, (value >> 16) / 10 ) ;
//					break ;

//					case ESC_RPM_ID_8 :
//					// Bit:0~15 RPM/1~65535RP
//	//					uint16_t ERpm = Electrical Rpm /100 so 100 are 10000 Erpm
//						storeTelemetryData( FR_RPM, (value & 0x0000FFFF) * 100 / 60 ) ;
//						storeTelemetryData( FR_AMP_MAH, (uint32_t)(value >> 16) ) ;
//					break ;

//					case ESC_TEMPERATURE_ID_8 :
//					// Bit:0-7 0.1Celsius/0~255
//						storeTelemetryData( FR_TEMP2, value & 0x000000FF ) ;
//					break ;

//					case SBEC_POWER_ID_8 :
//						storeTelemetryData( FR_SBEC_VOLT, (value & 0xffff) / 10 ) ;
//						storeTelemetryData( FR_SBEC_CURRENT, (value >> 16) / 10 ) ;
//					break ;
			
//					default :
//						// Handle unknown ID
//						handleUnknownId( (packet[3] << 8) | ( packet[2] ), value ) ;
//					break ;
//				}
//			}
		 
////		 if ( ( (packet[3] & 0xF0) == 0x50 ) ) //|| ( (packet[3] & 0xF0) == 0x10 ) )
////		 {
////			switch ( id )
////			{
////				case ARDUP_ID_8 :
////					{
////						uint32_t t ;
////						t = packet[2] & 0x0F ;
////						if ( t == ARDUP_AP_STAT_ID )	// 5001
////						{
////							// bits 0-4 Control mode
////							// bits 5-6 SimpleSS
////							// bit 7 LandComp
////							// bit 8 Armed
////							// bit 9 BatFS
////							// bit 10 EKFFS
////							storeTelemetryData( FR_BASEMODE, value & 0x0100 ? 0x80 : 0 ) ;
////							storeTelemetryData( FR_TEMP1, ((value & 0x1F)-1) | ( ARDUPtype << 8 ) ) ;
////						}
////						else if ( t == ARDUP_GPS_STAT_ID )	// 5002
////						{
////							uint16_t xvalue ;
////							int32_t ivalue ;
////							uint32_t negative ;
////							// Bits 0-3 #Sats
////							// Bits 4-5 GPs Fix NO_GPS = 0, NO_FIX = 1, GPS_OK_FIX_2D = 2, GPS_OK_FIX_3D = 3
////							// Bits 6-13 HDOP  7 bits (7-13) for value, 8th bit (bit 6) is 10^x
////							// Bits 14-21 VDOP
////							// Bits 22-30 GPS Alt? 7 bits for data, 8 and 9th bits 10^x
////							// Bit  32 GPS Alt negative
////							xvalue = value & 0x000F ;	// #sats
////							xvalue *= 10 ;
////							xvalue += (value >>4) & 0x0003 ;	// GPS Fix mode
////							storeTelemetryData( FR_TEMP2, xvalue ) ;
//////							FrskyHubData[FR_TEMP2] = xvalue ;
////							xvalue = (value >> 7) & 0x7F ;
////							if ( value & 0x0000040 )
////							{
////								xvalue *= 10 ;
////							}
////							storeTelemetryData( FR_GPS_HDOP, xvalue * 10 ) ;
//////							FrskyHubData[FR_GPS_HDOP] = xvalue * 10 ;
////							ivalue = (value >> 24) & 0x7F ;
////							negative = (value & 0x80000000) ? 1 : 0 ;
////							value >>= 22 ;
////							value &= 0x03 ;
////							while ( value )
////							{
////								ivalue *= 10 ;
////								value -= 1 ;
////							}
////							if ( negative )
////							{
////								ivalue = - ivalue ;
////							}
////							storeTelemetryData( FR_SPORT_GALT, ivalue ) ;
////						}
////						else if ( t == ARDUP_BATT_ID )	// 5003
////						{
////							// Bits 0-8 UAV Bat Volt
////							// Bits 9-16 UAV Curr bit 9 10^x, 10-16 mantissa
////							// Bits 17-31 mAh
////							uint16_t xvalue ;
////	//						xvalue = value & 0x01FF ;	// #battery in 0.1V
////							if ( VfasVoltageTimer )
////							{
////								VfasVoltageTimer -= 1 ;
////							}
////							else
////							{
////								storeTelemetryData( FR_VOLTS, value & 0x01FF ) ;
////							}
////							xvalue = (value >> 10) & 0x7F ;
////							if ( value & 0x0000200 )
////							{
////								xvalue *= 10 ;
////							}
////							storeTelemetryData( FR_CURRENT, xvalue ) ;
////							xvalue = (value >> 17) ;
////							storeTelemetryData( FR_AMP_MAH, xvalue ) ;
////							if ( PixHawkCapacity )
////							{
////								storeTelemetryData( FR_FUEL, 100 - (xvalue * 100 / PixHawkCapacity) ) ;
////							}
////						}
////						else if ( t == ARDUP_VandYAW_ID )	// 5005
////						{
////							// bits 0-8 Vspd 0 exponent, 1-7 mantissa, 8 sign
////							// bits 9-16 Hspd 9 exponent, 10-16 mantissa
////							// bits 17-27 Yaw ( * 0.2)
////							uint16_t xvalue ;
////							xvalue = (value >> 9) & 0x7F ;
////							if ( value & 0x0000100 )
////							{
////								xvalue *= 10 ;
////							}
////							storeTelemetryData( FR_AIRSPEED, xvalue ) ;
////						}
////						else if ( t == ARDUP_PARAM_ID )	// 5007
////						{
////							uint32_t id ;
////							id = value >> 24 ;
////							value &= 0x00FFFFFF ;

////							if ( id == 0x10 )
////							{
////								ARDUPtype = value ;
////							}
////	//						else if ( id == 0x20 )
////	//						{
//////2: FailSafe batt voltage in centivolts
////	//						}
////	//						else if ( id == 0x30 )
////	//						{
//////3: Failsafe Batt capacity in mAh
////	//						}
////							else if ( id == 0x04 )
////							{
////								if ( value )
////								{
////									PixHawkCapacity = value ;
////								}
////							}
////						}
////						else if ( t == ARDUP_HOME_ID ) // 5004
////						{
////							// bits 0-11 Home Dist 0-1 exponent, 2-11 mantissa
////							// bits 12-18 Home Angle, units of 3 degrees
////							// bits 19-31 HomeAlt 19-20 exponent, 21-30 mantissa, 31 sign
//////Distance to home
//////Angle from front of vehicle
//////Altitude relative to home - offset 19 bits - WRONG

//////The actual order of the data elements from LS bit side is:
//////Distance to home
//////Altitude relative to home - offset 12 bits
//////Angle from front of vehicle

//////as per these define statements in the APMv3.5.3 Arducopter source

//////// for home position related data
//////define HOME_ALT_OFFSET 12
//////define HOME_BEARING_LIMIT 0x7F
//////define HOME_BEARING_OFFSET 25
							
////// telemetry.homeDist = bit32.extract(VALUE,2,10) * (10^bit32.extract(VALUE,0,2))
////// telemetry.homeAlt = bit32.extract(VALUE,14,10) * (10^bit32.extract(VALUE,12,2)) * 0.1 * (bit32.extract(VALUE,24,1) == 1 and -1 or 1) --m
////// telemetry.homeAngle = bit32.extract(VALUE, 25,  7) * 3
							
////							uint16_t xvalue ;
////							int32_t ivalue ;
////							uint32_t negative ;
////							xvalue = ( value >> 2 ) & 0x3FF ;		// 10 bits
////							ivalue = value & 3 ;
////							while ( ivalue )
////							{
////								xvalue *= 10 ;
////								ivalue -= 1 ;
////							}
////							storeTelemetryData( FR_HOME_DIST, xvalue ) ;
							
////							xvalue = ( value >> 25 ) & 0x7F ;		// 7 bits
////							xvalue *= 3 ;
////							storeTelemetryData( FR_HOME_DIR, xvalue ) ;

////							// bits 12-24 HomeAlt 12-13 exponent, 14-23 mantissa, 24 sign
////							ivalue = (value >> 14) & 0x03FF ;
////							negative = (value & 0x01000000) ? 1 : 0 ;
////							value >>= 12 ;
////							value &= 0x03 ;
////							while ( value )
////							{
////								ivalue *= 10 ;
////								value -= 1 ;
////							}
////							if ( negative )
////							{
////								ivalue = - ivalue ;
////							}
////							storeAltitude( ivalue ) ;
//////							storeTelemetryData( FR_ALT_BARO, ivalue ) ;
////						}
////						else
////						{
////							// 5006 is roll and pitch
////							handleUnknownId( (packet[3] << 8) | ( packet[2] ), value ) ;
////						}
////					}
////				break ;

////				default :
////					// Handle unknown ID
////					handleUnknownId( (packet[3] << 8) | ( packet[2] ), value ) ;
////				break ;
////				}
////			}
//			else
//			{
//				handleUnknownId( (packet[3] << 8) | ( packet[2] ), value ) ;
//			}
//		}
	}
}

void FrskyData::set(uint8_t value, uint8_t copy)
{
	uint8_t x ;
	averaging_total += value ;
	uint8_t count = 16 ;
	uint8_t shift = 4 ;
//	if ( ( FrskyTelemetryType == FRSKY_TEL_SPORT ) || ( FrskyTelemetryType == FRSKY_TEL_AFH ) )	// SPORT or AFHDS2
	{
		count = 4 ;
		shift = 2 ;
	}
	if ( ++averageCount >= count )
	{
		averageCount = 0 ;
		raw = averaging_total >> shift ;
  	if ( raw > offset )
		{
		  x = raw - offset ;
		}
		else
		{
			x = 0 ;
		}
		this->value = x ;

		storeTelemetryData( copy, this->value ) ;
		averaging_total = 0 ;
	}
}

template<int N>
bool getCrossfireTelemetryValue(uint8_t index, uint32_t &value)
{
  bool result = false ;
  value = 0 ;
  uint8_t *byte = &TelemetryRxBuffer[index] ;
  for ( uint32_t i = 0 ; i < N ; i += 1 )
	{
    value <<= 8 ;
    if (*byte != 0xff)
		{
      result = true ;
    }
    value += *byte++ ;
  }
  return result ;
}

uint8_t crc8(const uint8_t * ptr, uint32_t len) ;

static bool checkCrossfireTelemetryFrameCRC()
{
  uint8_t len = TelemetryRxBuffer[1] ;
  uint8_t crc = crc8(&TelemetryRxBuffer[2], len-1) ;
  return (crc == TelemetryRxBuffer[len+1]) ;
}

uint32_t crossfireGpsConvert( uint32_t value )
{
	uint16_t degrees ;
	uint16_t minutes ;
	uint32_t result ;

	degrees = value / 10000000 ;
	value %= 10000000 ;		// Fractions of a degree
	value *= 60 ;
	minutes = value / 10000000 ;
	result = ( degrees * 100 + minutes ) << 16 ;
	value %= 10000000 ;		// Fractions of a minute
	value /= 1000 ;
	return result | value ;
}

void storeRSSI( uint8_t value )
{
//	if ( RssiSetTimer )
//	{
//		RssiSetTimer -= 1 ;
//	}
//	else
//	{
		FrskyTelemetry[2].set( value, FR_RXRSI_COPY );	//FrskyHubData[] =  frskyTelemetry[2].value ;
//	}
}


void processCrossfireTelemetryFrame()
{
//	XFDebug1 += 1 ;
  if (!checkCrossfireTelemetryFrameCRC())
	{
    return ;
  }
//	XFDebug2 += 1 ;

//	EA0D3AEA001000004E20FFFFCFF490
//  EA0E2DEAEEFF00000500060000FA03F7
//	EA0C14E900640A000701F3640B66
	
	uint8_t id = TelemetryRxBuffer[2] ;
  uint32_t value ;
  switch(id)
	{
    case CRSF_GPS_ID:
      FrskyUsrStreaming = FRSKY_USR_TIMEOUT10ms ; // reset counter only if valid frsky packets are being detected
      if (getCrossfireTelemetryValue<4>(3, value))
			{
				uint8_t code = 'N' ;
				int32_t ivalue = value ;
				if ( ivalue < 0 )
				{
					code = 'S' ;
					ivalue = -ivalue ;
				}
				value = ivalue ;
//        processCrossfireTelemetryValue(GPS_LATITUDE_INDEX, value/10);
				value = crossfireGpsConvert( value ) ;
				storeTelemetryData( FR_GPS_LAT, value >> 16 ) ;
				storeTelemetryData( FR_GPS_LATd, value ) ;
				storeTelemetryData( FR_LAT_N_S, code ) ;
			}
      if (getCrossfireTelemetryValue<4>(7, value))
			{
				uint8_t code = 'E' ;
				int32_t ivalue = value ;
				if ( ivalue < 0 )
				{
					code = 'W' ;
					ivalue = -ivalue ;
				}
				value = ivalue ;
//        processCrossfireTelemetryValue(GPS_LONGITUDE_INDEX, value/10);
				value = crossfireGpsConvert( value ) ;
				storeTelemetryData( FR_GPS_LONG, value >> 16 ) ;
				storeTelemetryData( FR_GPS_LONGd, value ) ;
				storeTelemetryData( FR_LONG_E_W, code ) ;
			}	
//      if (getCrossfireTelemetryValue<2>(11, value))
//        processCrossfireTelemetryValue(GPS_GROUND_SPEED_INDEX, value);
      if (getCrossfireTelemetryValue<2>(13, value))
			{
//        processCrossfireTelemetryValue(GPS_HEADING_INDEX, value);
					storeTelemetryData( FR_COURSE, value ) ;
			}
      if (getCrossfireTelemetryValue<2>(15, value))
			{
//        processCrossfireTelemetryValue(GPS_ALTITUDE_INDEX,  value - 1000);
				storeTelemetryData( FR_SPORT_GALT, value - 1000 ) ;
				
			}
      if (getCrossfireTelemetryValue<1>(17, value))
			{
//        processCrossfireTelemetryValue(GPS_SATELLITES_INDEX, value);
				
			}
    break;

    case CRSF_LINK_ID :
//			XFDebug3 += 1 ;
      FrskyStreaming = FRSKY_TIMEOUT10ms ;
      for ( uint32_t i=0 ; i<=TX_SNR_INDEX; i += 1 )
			{
        if (getCrossfireTelemetryValue<1>(3+i, value))
				{
//          if (i == TX_POWER_INDEX)
//					{
//            static const uint32_t power_values[] = { 0, 10, 25, 100, 500, 1000, 2000 } ;
//            value = (value < DIM(power_values) ? power_values[value] : 0) ;
//          }
//          processCrossfireTelemetryValue(i, value) ;
  				if ( i == TX_QUALITY_INDEX )
					{
			    	FrskyTelemetry[3].set(value, FR_TXRSI_COPY ) ;	// TSSI
					}
					
          if ( i == RX_QUALITY_INDEX )
					{
						storeRSSI( value ) ;
//            telemetryData.rssi.set(value) ;
          }
					if ( i < 6 )
					{
						storeTelemetryData( FR_CUST1 + i, value ) ;
					}
					else
					{
						if ( i < 10 )
						{
	          	if (i == TX_POWER_INDEX)
							{
    	      	  static const uint32_t power_values[] = { 0, 10, 25, 100, 500, 1000, 2000, 250, 50 } ;
      	    	  value = (value < DIM(power_values) ? power_values[value] : 0) ;
							}
							storeTelemetryData( FR_CUST7 + i - 6, value ) ;
						}
					}
        }
      }
    break ;

    case CRSF_BATTERY_ID :
      FrskyUsrStreaming = FRSKY_USR_TIMEOUT10ms ; // reset counter only if valid frsky packets are being detected
      if (getCrossfireTelemetryValue<2>( 3, value) )
			{
				storeTelemetryData( FR_VOLTS, value ) ;
			}
//        processCrossfireTelemetryValue(BATT_VOLTAGE_INDEX, value);
      if (getCrossfireTelemetryValue<2>(5, value))
			{
				storeTelemetryData( FR_CURRENT, value ) ;
			}
//        processCrossfireTelemetryValue(BATT_CURRENT_INDEX, value);
      if (getCrossfireTelemetryValue<3>(7, value))
			{
				storeTelemetryData( FR_AMP_MAH, value ) ;
			}
//      if (getCrossfireTelemetryValue<3>(7, value))
//        processCrossfireTelemetryValue(BATT_CAPACITY_INDEX, value);
    break ;

    case CRSF_ATTITUDE_ID:
      FrskyUsrStreaming = FRSKY_USR_TIMEOUT10ms ; // reset counter only if valid frsky packets are being detected
      if (getCrossfireTelemetryValue<2>(3, value))
			{
//        processCrossfireTelemetryValue(ATTITUDE_PITCH_INDEX, value/10);
				storeTelemetryData( FR_ACCX, value / 10 ) ;
			}	
      if (getCrossfireTelemetryValue<2>(5, value))
			{
//        processCrossfireTelemetryValue(ATTITUDE_ROLL_INDEX, value/10);
				storeTelemetryData( FR_ACCY, value / 10 ) ;
			}	
      if (getCrossfireTelemetryValue<2>(7, value))
			{
//        processCrossfireTelemetryValue(ATTITUDE_YAW_INDEX, value/10);
				storeTelemetryData( FR_ACCZ, value / 10 ) ;
			}	
    break;

//    case FLIGHT_MODE_ID:
//    {
//      const CrossfireSensor & sensor = crossfireSensors[FLIGHT_MODE_INDEX];
//      for (int i=0; i<min<int>(16, telemetryRxBuffer[1]-2); i+=4) {
//        uint32_t value = *((uint32_t *)&telemetryRxBuffer[3+i]);
//        setTelemetryValue(TELEM_PROTO_CROSSFIRE, sensor.id, 0, sensor.subId, value, sensor.unit, i);
//      }
//      break;
//    }

		case RADIO_ID :
		{
			if ( TelemetryRxBuffer[3] == RADIO_ADDRESS )
			{
				if ( TelemetryRxBuffer[5] == 0x10 )	// Timing Correction
				{
					uint32_t offset ;
      		if (getCrossfireTelemetryValue<4>( 6, value) )	// Update interval
					{
      			if (getCrossfireTelemetryValue<4>( 10, offset) )	// Update interval
						{
							// Report these
							UpdateTiming.UpdateRate = value / 10 ;			// Was 10ths of uS
							UpdateTiming.UpdateOffset = (int32_t)offset / 10 ;	// Was 10ths of uS
							UpdateTiming.UpdateTimer = UPDATE_TIMEOUT ;
						}
					}
				}
			}
		}
		break ;

#if defined(LUA) || defined(BASIC)
    default:
		{
			uint8_t *packet = &TelemetryRxBuffer[1] ;
			uint8_t len = *packet ;	// # bytes to copy, add length, drop crc

			if ( fifo128Space( &Script_fifo ) >= len )
			{
				uint32_t i ;
				for ( i = 0 ; i < len ; i += 1 )
				{
					put_fifo128( &Script_fifo, *packet++ ) ;
				}
			}
		}
    break;
#endif
//#if defined(LUA)
//    default:
//      if (luaInputTelemetryFifo && luaInputTelemetryFifo->hasSpace(telemetryRxBufferCount-2) ) {
//        for (uint8_t i=1; i<telemetryRxBufferCount-1; i++) {
//          // destination address and CRC are skipped
//          luaInputTelemetryFifo->push(telemetryRxBuffer[i]);
//        }
//      }
//    break;
//#endif
  }
}

//void p2hex( unsigned char c ) ;

void telemetryRecieveByte( uint8_t data, uint32_t module )
{

extern uint16_t TelRxCount ;
	TelRxCount += 1 ;

//	p2hex( data ) ;	
	
	// Handle ELRS data here
  uint8_t numbytes = NumPktBytes ;
	
  if ( numbytes == 0 && data != RADIO_ADDRESS)
	{
		return ;
	}
	
  if ( numbytes == 1 && (data < 2 || data > TELEMETRY_RX_PACKET_SIZE-2))
	{
    NumPktBytes = 0 ;
    return ;
  }
  
	if ( numbytes < TELEMETRY_RX_PACKET_SIZE)
	{
    TelemetryRxBuffer[numbytes++] = data ;
  }
  else
	{
    numbytes = 0 ;
  }
  
	if ( numbytes > 4)
	{
    uint8_t length = TelemetryRxBuffer[1] ;
    if (length + 2 == numbytes )
		{
      processCrossfireTelemetryFrame() ;
      numbytes = 0;
    }
  }
	NumPktBytes = numbytes ;
}


