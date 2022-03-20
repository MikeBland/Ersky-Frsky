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

#ifndef TELEMETRY_H
#define TELEMETRY_H


// Mapped indices for Telemetry Data
#define FR_A1_COPY		0
#define FR_A2_COPY		1
#define FR_RXRSI_COPY	2
#define FR_TXRSI_COPY	3
#define FR_ALT_BARO		4
#define FR_ALT_BAROd	5
#define TELEM_GPS_ALT		6
#define TELEM_GPS_ALTd		7
#define FR_GPS_SPEED	8 
#define FR_GPS_SPEEDd	9 
#define FR_TEMP1			10
#define FR_TEMP2			11
#define FR_RPM				12
#define FR_FUEL				13
#define FR_A1_MAH			14
#define FR_A2_MAH			15
#define FR_CELL_V			16
#define FR_COURSE			17
#define FR_COURSEd		18
#define FR_GPS_DATMON	19
#define FR_GPS_YEAR		20
#define FR_GPS_HRMIN	21
#define FR_GPS_SEC		22
#define FR_GPS_LONG		23
#define FR_GPS_LONGd	24
#define FR_LONG_E_W		25
#define FR_GPS_LAT		26
#define FR_GPS_LATd		27
#define FR_LAT_N_S		28
#define FR_ACCX				29
#define FR_ACCY				30
#define FR_ACCZ				31
#define FR_CURRENT		32
// next 2 moved from 58 and 59
#define FR_V_AMP			33
#define FR_V_AMPd			34
#define FR_CELL_MIN		35
#define FR_AMP_MAH		36
#define FR_CELLS_TOT	37
#define FR_VOLTS			38
#define FR_VSPD				39	// Moved from 0x30 (48)
#define FR_RXV				40
#define FR_A3					41
#define FR_A4					42

/* Extra data for Mavlink via FrSky */
#define FR_BASEMODE             43
#define FR_WP_DIST              44
#define FR_HEALTH               45
#define FR_MSG                  46
#define FR_HOME_DIR             47
#define FR_HOME_DIST            48
#define FR_CPU_LOAD             49
#define FR_GPS_HDOP             50
#define FR_WP_NUM               51
#define FR_WP_BEARING           52
#define FR_VCC                  53
/* Extra data for Mavlink via FrSky */
#define FR_AIRSPEED             54

#define FR_RBOX_B1_V						55
#define FR_RBOX_B1_A						56
#define FR_RBOX_B2_V            57
#define FR_RBOX_B2_A            58
#define FR_RBOX_B1_CAP          59
#define FR_RBOX_B2_CAP          60
#define FR_RBOX_SERVO	          61
#define FR_RBOX_STATE	          62
#define FR_CELL1			          63
#define FR_CELL2			          64
#define FR_CELL3			          65
#define FR_CELL4			          66
#define FR_CELL5			          67
#define FR_CELL6			          68
#define FR_CELL7			          69
#define FR_CELL8			          70
#define FR_CELL9			          71
#define FR_CELL10			          72
#define FR_CELL11			          73
#define FR_CELL12			          74

#define FR_CUST1			          75
#define FR_CUST2			          76
#define FR_CUST3			          77
#define FR_CUST4			          78
#define FR_CUST5			          79
#define FR_CUST6			          80
#define FR_CELLS_TOTAL1			    81
#define FR_CELLS_TOTAL2			    82
#define FR_SBEC_VOLT				    83
#define FR_SBEC_CURRENT			    84
#define FR_VFR							    85
#define FR_CUST7			          86
#define FR_CUST8			          87
#define FR_CUST9			          88
#define FR_CUST10			          89

#define FR_TRASH			90  // Used for invalid id
//#define FR_TRASH			43	// Used for invalid id

#define FR_SPORT_ALT	0xFF
#define FR_SPORT_GALT	0xFE

#define TELEMETRYDATALENGTH  (FR_TRASH+1)


#define RSSI_ID            0xf101
#define ADC1_ID            0xf102
#define ADC2_ID            0xf103
#define BATT_ID            0xf104
#define SWR_ID             0xf105
#define XJT_VERSION_ID     0xf106
#define VFR_ID				     0xf010

// Special
#define UART_FIRST_ID      0xFD00
#define UART_LAST_ID	     0xFD0F

// Sensors
#define ALT_FIRST_ID       0x0100
#define ALT_LAST_ID        0x010f
#define ALT_ID_8					0x10
#define VARIO_FIRST_ID     0x0110
#define VARIO_LAST_ID      0x011f
#define VARIO_ID_8				0x11
#define CURR_FIRST_ID      0x0200
#define CURR_LAST_ID       0x020f
#define CURR_ID_8					0x20
#define VFAS_FIRST_ID      0x0210
#define VFAS_LAST_ID       0x021f
#define VFAS_ID_8					0x21
#define CELLS_FIRST_ID     0x0300
#define CELLS_LAST_ID      0x030f
#define CELLS_ID_8				0x30
#define T1_FIRST_ID        0x0400
#define T1_LAST_ID         0x040f
#define T1_ID_8						0x40
#define T2_FIRST_ID        0x0410
#define T2_LAST_ID         0x041f
#define T2_ID_8						0x41
#define RPM_FIRST_ID       0x0500
#define RPM_LAST_ID        0x050f
#define RPM_ID_8					0x50
#define FUEL_FIRST_ID      0x0600
#define FUEL_LAST_ID       0x060f
#define FUEL_ID_8					0x60
#define ACCX_FIRST_ID      0x0700
#define ACCX_LAST_ID       0x070f
#define ACCX_ID_8					0x70
#define ACCY_FIRST_ID      0x0710
#define ACCY_LAST_ID       0x071f
#define ACCY_ID_8					0x71
#define ACCZ_FIRST_ID      0x0720
#define ACCZ_LAST_ID       0x072f
#define ACCZ_ID_8					0x72
#define GPS_SPEED_FIRST_ID 0x0830
#define GPS_SPEED_LAST_ID  0x083f
#define GPS_SPEED_ID_8		0x83
#define GPS_LA_LO_FIRST_ID	0x0800
#define GPS_LA_LO_LAST_ID	0x080F
#define GPS_LA_LO_ID_8		0x80
#define GPS_ALT_FIRST_ID 0x0820
#define GPS_ALT_LAST_ID  0x082f
#define GPS_ALT_ID_8			0x82
#define GPS_HDG_FIRST_ID 0x0840
#define GPS_HDG_LAST_ID  0x084f
#define GPS_HDG_ID_8			0x84
#define GPS_TIME_FIRST_ID 0x0850
#define GPS_TIME_LAST_ID  0x085f
#define GPS_TIME_ID_8			0x85

#define A3_FIRST_ID      0x0900
#define A3_LAST_ID       0x090f
#define A3_ID_8						0x90
#define A4_FIRST_ID      0x0910
#define A4_LAST_ID       0x091f
#define A4_ID_8						0x91
#define AIRSPEED_FIRST_ID 0x0A00
#define AIRSPEED_LAST_ID  0x0A0f
#define AIRSPEED_ID_8			0xA0

#define FUEL_QTY_FIRST_ID  0x0A10
#define FUEL_QTY_LAST_ID   0x0A1F
#define FUEL_QTY_ID_8      0xA1

#define RBOX_BATT1_FIRST_ID    0x0b00
#define RBOX_BATT1_LAST_ID     0x0b0f
#define RBOX_BATT1_ID_8        0xb0

#define RBOX_BATT2_FIRST_ID    0x0b10
#define RBOX_BATT2_LAST_ID     0x0b1f
#define RBOX_BATT2_ID_8        0xb1
#define RBOX_STATE_FIRST_ID    0x0b20
#define RBOX_STATE_LAST_ID     0x0b2f
#define RBOX_STATE_ID_8        0xb2
#define RBOX_CNSP_FIRST_ID     0x0b30
#define RBOX_CNSP_LAST_ID      0x0b3f
#define RBOX_CNSP_ID_8         0xb3

#define S6R_FIRST_ID			     0x0c30
#define S6R_LAST_ID						 0x0c3f
#define S6R_ID_8			         0xc3

#define DIY_FIRST_ID           0x5000
#define DIY_LAST_ID            0x52ff
#define DIY_ID_8	             0x00

#define DIY_STREAM_FIRST_ID       0x5000
#define DIY_STREAM_LAST_ID        0x50ff

#define FACT_TEST_ID              0xf000

#define ESC_POWER_FIRST_ID        0x0b50
#define ESC_POWER_LAST_ID         0x0b5f
#define ESC_POWER_ID_8	        	0xb5
#define ESC_RPM_CONS_FIRST_ID     0x0b60
#define ESC_RPM_CONS_LAST_ID      0x0b6f
#define ESC_RPM_ID_8	        		0xb6
#define ESC_TEMPERATURE_FIRST_ID  0x0b70
#define ESC_TEMPERATURE_LAST_ID   0x0b7f
#define ESC_TEMPERATURE_ID_8     	0xb7

#define X8R_FIRST_ID              0x0c20
#define X8R_LAST_ID               0x0c2f

#define GASSUIT_TEMP_FIRST_ID     0x0d00
#define GASSUIT_TEMP_LAST_ID      0x0d0f
#define GASSUIT_SPEED_FIRST_ID    0x0d10
#define GASSUIT_SPEED_LAST_ID     0x0d1f
#define GASSUIT_FUEL_FIRST_ID     0x0d20
#define GASSUIT_FUEL_LAST_ID      0x0d2f

#define SP2UART_A_ID              0xfd00
#define SP2UART_B_ID              0xfd01

#define SBEC_POWER_FIRST_ID       0x0e50
#define SBEC_POWER_LAST_ID        0x0e5f
#define SBEC_POWER_ID_8     			0xe5

#define NON_STANDARD_ID_8     		0x0F


// Craft and Theory
#define	ARDUP_ID_8				0x00
#define	ARDUP_AP_STAT_ID	0x01
#define	ARDUP_GPS_STAT_ID	0x02
#define	ARDUP_BATT_ID			0x03
#define	ARDUP_HOME_ID			0x04
#define	ARDUP_VandYAW_ID	0x05
#define	ARDUP_ATTandRNGID	0x06
#define	ARDUP_PARAM_ID		0x07


#define BETA_VARIO_ID      0x8030
#define BETA_VARIO_ID_8		 0x03
#define BETA_BARO_ALT_ID   0x8010
#define BETA_ALT_ID_8		   0x01



#define MAV_SYS_STATUS_SENSOR_3D_GYRO			 1  /* 0x01 3D gyro | */
#define MAV_SYS_STATUS_SENSOR_3D_ACCEL			 2  /* 0x02 3D accelerometer | */
#define MAV_SYS_STATUS_SENSOR_3D_MAG			 4  /* 0x04 3D magnetometer | */
#define MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE		 8  /* 0x08 absolute pressure | */
#define MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE     16  /* 0x10 differential pressure | */
#define MAV_SYS_STATUS_SENSOR_GPS			32  /* 0x20 GPS | */
#define MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW              64  /* 0x40 optical flow | */
#define MAV_SYS_STATUS_GEOFENCE                    1048576  /* 0x100000 geofence | */
#define MAV_SYS_STATUS_AHRS                        2097152  /* 0x200000 AHRS subsystem health | */
#define MAV_SYS_ERR_GYRO     "GYRO"
#define MAV_SYS_ERR_ACCEL    "ACCEL"
#define MAV_SYS_ERR_MAG      "MAG"
#define MAV_SYS_ERR_PRESSURE "PRESS"
#define MAV_SYS_ERR_AIRSPEED "AIRSP"
#define MAV_SYS_ERR_GPS      "GPS"
#define MAV_SYS_ERR_OPTICAL  "OPTIC"
#define MAV_SYS_ERR_GEOFENCE "FENCE"
#define MAV_SYS_ERR_AHRS     "AHRS"

#define FRSKY_TIMEOUT10ms 			35
#define FRSKY_USR_TIMEOUT10ms		90

#define DATA_FRAME         0x10


struct FrskyData
{
  uint8_t value;
  uint8_t raw;
//  uint8_t min;
//  uint8_t max;
	uint8_t offset ;
	uint16_t averaging_total ;
	uint8_t averageCount ;
  void set(uint8_t value, uint8_t copy);
	void setoffset();
};

struct t_s6r
{
	uint8_t fieldIndex ;
	uint8_t valid ;
	int16_t value ;
} ;

struct t_elrsConfig
{
	char name[20] ;
	uint8_t hwVers[4] ;
	uint8_t swVers[4] ;
	uint8_t numConfigs ;
	uint8_t protocolVers ;
	uint8_t filled ;
} ;



extern struct t_s6r S6Rdata ;

extern uint8_t TelemetryDataValid[] ;
extern int16_t TelemetryData[] ;

void processSportData( uint8_t *packet, uint32_t receiver ) ;
void checkTelemetry( void ) ;

extern uint8_t FrskyStreaming ;
extern uint8_t SportStreamingStarted ;
//uint8_t FrskyUsrStreaming = 0 ;
extern FrskyData FrskyTelemetry[] ;

extern int16_t AltOffset ;

uint16_t A1A2toScaledValue( uint8_t channel, uint8_t *dplaces ) ;
uint16_t convertRxv( uint16_t value ) ;






#endif

