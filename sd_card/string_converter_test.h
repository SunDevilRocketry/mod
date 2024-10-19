/*******************************************************************************
*
* FILE: 
* 		string_converter_test.h
*
* DESCRIPTION: 
* 		Testing string converter 
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STRING_CONVERTER_TEST_H
#define STRING_CONVERTER_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sensor.h"

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/
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
    int str_len,        /* Length (size) of the string */
    int decimal         /* Numbers after a decimal point */
    );

void dataframe_to_string
	(
	SENSOR_DATA*  sensor_data_ptr, /* Sensor dataframe to be converted */
	uint32_t	  time,			   /* Extracted time value */
	char*		  pbuffer_str  	   /* Converted data to be returned */
	);

#endif /* STRING_CONVERTER_TEST_H */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
