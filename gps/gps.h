/*******************************************************************************
*
* FILE: 
* 		gps.h
*
* DESCRIPTION: 
* 		Contains API functions for gps
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GPS_H
#define GPS_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Includes 
------------------------------------------------------------------------------*/
#define GPSBUFSIZE  128       // GPS buffer size


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/


typedef struct _GPS_DATA{
    // calculated values
    float dec_longitude;
    float dec_latitude;
    float altitude_ft;

    // GGA - Global Positioning System Fixed Data
    float nmea_longitude;
    float nmea_latitude;
    float utc_time;
    char ns, ew;
    int lock;
    int satelites;
    float hdop;
    float msl_altitude;
    char msl_units;

    // RMC - Recommended Minimmum Specific GNS Data
    char rmc_status;
    float speed_k;
    float course_d;
    int date;

    // GLL
    char gll_status;

    // VTG - Course over ground, ground speed
    float course_t; // ground speed true
    char course_t_unit;
    float course_m; // magnetic
    char course_m_unit;
    char speed_k_unit;
    float speed_km; // speek km/hr
    char speed_km_unit;
} GPS_DATA;


/* Function return codes */
typedef enum GPS_STATUS
	{
	GPS_OK = 0,
    GPS_FAIL  ,
	GPS_TIMEOUT
	} GPS_STATUS;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* transmits bytes over USB */
GPS_STATUS gps_transmit 
	(
    void*    tx_data_ptr , /* Data to be sent       */	
	size_t   tx_data_size, /* Size of transmit data */ 
	uint32_t timeout       /* UART timeout          */
	);

/* Receives bytes from the USB port */
GPS_STATUS gps_receive 
	(
	void*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size, /* Size of the data to be received */
	uint32_t timeout       /* UART timeout */
	);

GPS_STATUS gps_receive_IT
	(
	uint8_t*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size /* Size of the data to be received */
	);

int gps_mesg_validate
    (
    char *nmeastr
    );

void GPS_parse
    (
    GPS_DATA* gps_ptr, 
    char *GPSstrParse
    );

static float gps_string_to_float
    (
    char *GPSstrParse, 
    int* inputIdx
    );

static char gps_string_to_char
    (
    char *GPSstrParse, 
    int* inputIdx
    );

float GPS_nmea_to_dec
    (
    float deg_coord,
    char nsew
    );

void gps_listener  
    (
    void    
    );

#ifdef __cplusplus
}
#endif

#endif /* USB_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
