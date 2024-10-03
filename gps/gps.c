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

void GPS_parse(GPS_DATA* gps_ptr, char *GPSstrParse){
    if(!strncmp(GPSstrParse, "$GPGGA", 6)){
    	if (sscanf(GPSstrParse, "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c", 
				&gps_ptr->utc_time, 
				&gps_ptr->nmea_latitude, 
				&gps_ptr->ns, 
				&gps_ptr->nmea_longitude, 
				&gps_ptr->ew, 
				&gps_ptr->lock, 
				&gps_ptr->satelites, 
				&gps_ptr->hdop, 
				&gps_ptr->msl_altitude, 
				&gps_ptr->msl_units) >= 1)
			{
			gps_ptr->dec_latitude = GPS_nmea_to_dec(gps_ptr->nmea_latitude, gps_ptr->ns);
    		gps_ptr->dec_longitude = GPS_nmea_to_dec(gps_ptr->nmea_longitude, gps_ptr->ew);
    		return;
    	}
    }
    else if (!strncmp(GPSstrParse, "$GPRMC", 6)){
    	if(sscanf(GPSstrParse, "$GPRMC,%f,%f,%c,%f,%c,%f,%f,%d",
				&gps_ptr->utc_time, 
				&gps_ptr->nmea_latitude, 
				&gps_ptr->ns, 
				&gps_ptr->nmea_longitude, 
				&gps_ptr->ew, 
				&gps_ptr->speed_k, 
				&gps_ptr->course_d, 
				&gps_ptr->date) >= 1)
    		return;

    }
    else if (!strncmp(GPSstrParse, "$GPGLL", 6)){
        if(sscanf(GPSstrParse, "$GPGLL,%f,%c,%f,%c,%f,%c", 
				&gps_ptr->nmea_latitude, 
				&gps_ptr->ns, 
				&gps_ptr->nmea_longitude, 
				&gps_ptr->ew, 
				&gps_ptr->utc_time, 
				&gps_ptr->gll_status) >= 1)
            return;
    }
    else if (!strncmp(GPSstrParse, "$GPVTG", 6)){
        if(sscanf(GPSstrParse, "$GPVTG,%f,%c,%f,%c,%f,%c,%f,%c", 
				&gps_ptr->course_t, 
				&gps_ptr->course_t_unit, 
				&gps_ptr->course_m, 
				&gps_ptr->course_m_unit, 
				&gps_ptr->speed_k, 
				&gps_ptr->speed_k_unit, 
				&gps_ptr->speed_km, 
				&gps_ptr->speed_km_unit) >= 1)
            return;
    }
}

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