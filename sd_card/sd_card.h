/*******************************************************************************
*
* FILE: 
* 		sd_card.h
*
* DESCRIPTION: 
* 		Contains API functions to operate the SD card 
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SD_CARD_H
#define SD_CARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sensor.h"

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Function return codes */

typedef enum SD_CARD_STATUS{
	SD_CARD_OK 	= 0	  ,
	SD_CARD_OPEN_FAIL ,
	SD_CARD_MOUNT_FAIL,
	SD_CARD_WRITE_FAIL,	
	SD_CARD_TIMEOUT
} SD_CARD_STATUS;

/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/
SD_CARD_STATUS write_to_sd_card
	(
	char* name,
	char* buffer
	);

void reverseString
    (
    char* pnum_str, /* Pointer to the string of a number */
    int len         /* The actual length of the string's length */
    );

int int2str
    (
    int num,        /* Integer number to be converted */
    char num_str[]  /* String of number to be returned by this function */
    );

void float2str
    (
    float float_num,    /* Float number to be converted to string */
    char* pnum_str,     /* Pointer to the string of a number */
    int decimal         /* Numbers after a decimal point */
    );

void dataframe_to_string
	(
	SENSOR_DATA*  sensor_data_ptr, /* Sensor dataframe to be converted */
	uint32_t	  time,			   /* Extracted time value */
	char*		  pbuffer_str  	   /* Converted data to be returned */
	);

#endif /* SD_CARD_H */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
