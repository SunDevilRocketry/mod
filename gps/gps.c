/*******************************************************************************
*
* FILE: 
* 		gps.c
*
* DESCRIPTION: 
* 		Contains API functions for GPS 
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes  
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#if   defined( FLIGHT_COMPUTER      )
	#include "sdr_pin_defines_A0002.h"
#elif defined( ENGINE_CONTROLLER    )
	#include "sdr_pin_defines_L0002.h"
#elif defined( VALVE_CONTROLLER     )
	#include "sdr_pin_defines_L0005.h"
#elif defined( GROUND_STATION       )
	#include "sdr_pin_defines_A0005.h"
#elif defined( FLIGHT_COMPUTER_LITE )
	#include "sdr_pin_defines_A0007.h"
#endif


/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#include "main.h"
#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/*------------------------------------------------------------------------------
 Preprocesor Directives 
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
Global Variables                                                                  
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		gps_transmit                                                    *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		transmits a specified number of bytes over USB                         *
*                                                                              *
*******************************************************************************/
GPS_STATUS gps_transmit 
	(
    void*    tx_data_ptr , /* Data to be sent       */	
	size_t   tx_data_size, /* Size of transmit data */ 
	uint32_t timeout       /* UART timeout          */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef gps_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
gps_status = HAL_UART_Transmit( &( GPS_HUART ),
                                tx_data_ptr   , 
                                tx_data_size  , 
                                timeout );

/* Return HAL status */
if ( gps_status != HAL_OK )
	{
	return gps_status;
	}
else
	{
	return GPS_OK;
	}

} /* usb_transmit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		usb_recieve                                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Receives bytes from the USB port                                       *
*                                                                              *
*******************************************************************************/
GPS_STATUS gps_receive 
	(
	void*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size, /* Size of the data to be received */
	uint32_t timeout       /* UART timeout */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef gps_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
gps_status = HAL_UART_Receive( &( GPS_HUART ),
                               rx_data_ptr   , 
                               rx_data_size  , 
                               timeout );
/* Return HAL status */
switch ( gps_status )
	{
	case HAL_TIMEOUT:
		{
		return GPS_TIMEOUT;
		break;
		}
	case HAL_OK:
		{
		return GPS_OK;
		break;
		}
	default:
		{
		return GPS_FAIL;
		break;
        }
	}

} /* usb_receive */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		usb_recieve_IT                                                         *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Receives bytes from the USB port                                       *
*                                                                              *
*******************************************************************************/
GPS_STATUS gps_receive_IT 
	(
	uint8_t*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size /* Size of the data to be received */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef gps_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
gps_status = HAL_UART_Receive_IT( &( GPS_HUART ),
                               rx_data_ptr   , 
                               rx_data_size );

/* Return HAL status */
switch ( gps_status )
	{
	case HAL_TIMEOUT:
		{
		return GPS_TIMEOUT;
		break;
		}
	case HAL_OK:
		{
		return GPS_OK;
		break;
		}
	default:
		{
		return GPS_FAIL;
		break;
        }
	}

} /* usb_receive_IT */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		gps_mesg_validate                                                         *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Validate message returned from GPS                                       *
*                                                                              *
*******************************************************************************/
int gps_mesg_validate(char *nmeastr){
    char check[3];
    char checkcalcstr[3];
    int i;
    int calculated_check;

    i=0;
    calculated_check=0;

    // check to ensure that the string starts with a $
    if(nmeastr[i] == '$')
        i++;
    else
        return 0;

    //No NULL reached, 75 char largest possible NMEA message, no '*' reached
    while((nmeastr[i] != 0) && (nmeastr[i] != '*') && (i < 75)){
        calculated_check ^= nmeastr[i];// calculate the checksum
        i++;
    }

    if(i >= 75){
        return 0;// the string was too long so return an error
    }

    if (nmeastr[i] == '*'){
        check[0] = nmeastr[i+1];    //put hex chars in check string
        check[1] = nmeastr[i+2];
        check[2] = 0;
    }
    else
        return 0;// no checksum separator found there for invalid

    sprintf(checkcalcstr,"%02X",calculated_check);
    return((checkcalcstr[0] == check[0])
        && (checkcalcstr[1] == check[1])) ? 1 : 0 ;
} /*gps_mesg_validate*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		GPS_parse	                                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Convert raw NMEA string to usable data                                 *
*                                                                              *
*******************************************************************************/
void GPS_parse(GPS_DATA* gps_ptr, char *GPSstrParse){
	/* get message type */
	char token[7];
    strncpy(token, GPSstrParse, 6);
    token[7] = '\0';
    int idx = 7;

	/* GGA Message */
	if (!strcmp(token, "$GPGGA")) {
        gps_ptr->utc_time = gps_string_to_float(GPSstrParse, idx, &idx);
        gps_ptr->nmea_latitude = gps_string_to_float(GPSstrParse, idx, &idx);
        gps_ptr->ns = GPSstrParse[idx]; idx = idx + 2;
        gps_ptr->nmea_longitude = gps_string_to_float(GPSstrParse, idx, &idx);
        gps_ptr->ew = GPSstrParse[idx]; idx = idx + 2;
        gps_ptr->lock = GPSstrParse[idx]; idx = idx + 2;
        gps_ptr->satelites = (int)(gps_string_to_float(GPSstrParse, idx, &idx) + 0.5); // This is a decimal number.
        gps_ptr->hdop = gps_string_to_float(GPSstrParse, idx, &idx);
        gps_ptr->msl_altitude = gps_string_to_float(GPSstrParse, idx, &idx);
        gps_ptr->msl_units = GPSstrParse[idx]; idx = idx + 2;
	}

	/* RMC Message */
    else if (!strcmp(token, "$GPRMC")) {
        gps_ptr->utc_time = gps_string_to_float(GPSstrParse, idx, &idx);
        gps_ptr->rmc_status = GPSstrParse[idx]; idx = idx + 2; /* unused */
        gps_ptr->nmea_latitude = gps_string_to_float(GPSstrParse, idx, &idx);
        gps_ptr->ns = GPSstrParse[idx]; idx = idx + 2;
        gps_ptr->nmea_longitude = gps_string_to_float(GPSstrParse, idx, &idx);
        gps_ptr->ew = GPSstrParse[idx]; idx = idx + 2;
        gps_ptr->speed_k = gps_string_to_float(GPSstrParse, idx, &idx);
        gps_ptr->course_d = gps_string_to_float(GPSstrParse, idx, &idx);
        gps_ptr->date = (int)(0.5 + gps_string_to_float(GPSstrParse, idx, &idx));
	}

	/* GLL Message */
    else if (!strcmp(token, "$GPGLL")) {
        gps_ptr->nmea_latitude = gps_string_to_float(GPSstrParse, idx, &idx);
        gps_ptr->ns = GPSstrParse[idx]; idx = idx + 2;
        gps_ptr->nmea_longitude = gps_string_to_float(GPSstrParse, idx, &idx);
        gps_ptr->ew = GPSstrParse[idx]; idx = idx + 2;
        gps_ptr->utc_time = gps_string_to_float(GPSstrParse, idx, &idx);
        gps_ptr->gll_status = GPSstrParse[idx]; idx = idx + 2;
    }

	/* VTG Message */
    else if (!strcmp(token, "$GPVTG")) {
        gps_ptr->course_t = gps_string_to_float(GPSstrParse, idx, &idx);
        gps_ptr->course_t_unit = GPSstrParse[idx]; idx = idx + 2;
        gps_ptr->course_m = gps_string_to_float(GPSstrParse, idx, &idx);
        gps_ptr->course_m_unit = GPSstrParse[idx]; idx = idx + 2;
        gps_ptr->speed_k = gps_string_to_float(GPSstrParse, idx, &idx);
        gps_ptr->speed_k_unit = GPSstrParse[idx]; idx = idx + 2;
        gps_ptr->speed_km = gps_string_to_float(GPSstrParse, idx, &idx);
        gps_ptr->speed_km_unit = GPSstrParse[idx]; idx = idx + 2;
    }

} /* GPS_parse */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		gps_string_to_float                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Convert raw NMEA string to usable data                                 *
*                                                                              *
*******************************************************************************/
float gps_string_to_float(char *GPSstrParse, int startIdx, int* endIdx) {
    int idx = startIdx;
    char currChar = GPSstrParse[idx];
    char tempstr[16];
    int tempidx = 0;
    while (currChar != ',') {
        if (tempidx > 15) {
            /* ERROR HANDLING */
            // maybe just exit loop? and deal with bad data? or make it null.
            return 0.0f;
        }
        tempstr[tempidx] = GPSstrParse[idx];
        tempidx++;
        idx++;
        currChar = GPSstrParse[idx];
    }
    *endIdx = idx + 1;
    tempstr[tempidx] = '\0';
    return strtof(tempstr, NULL);
} /* gps_string_to_float */

float GPS_nmea_to_dec(float deg_coord, char nsew) {
    int degree = (int)(deg_coord/100);
    float minutes = deg_coord - degree*100;
    float dec_deg = minutes / 60;
    float decimal = degree + dec_deg;
    if (nsew == 'S' || nsew == 'W') { // return negative
        decimal *= -1;
    }
    return decimal;
}
/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/