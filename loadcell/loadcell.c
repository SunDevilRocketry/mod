/*******************************************************************************
*
* FILE: 
* 		loadcell.c
*
* DESCRIPTION: 
* 		Contains API functions for reading data from the engine's load cell 
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "sdr_pin_defines_L0002.h"
#include "loadcell.h"

/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/

/* Sample the load cell ADC a specified number of times in polling mode */
static LOADCELL_STATUS poll_loadcell_adc 
	(
    uint16_t    num_samples,
    uint32_t*   psample_buffer
    );

/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		loadcell_get_reading                                                   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Read the load cell force                                               *
*                                                                              *
*******************************************************************************/
LOADCELL_STATUS loadcell_get_reading 
	(
    uint32_t* loadcell_reading_ptr /* Pointer to output data */
    )
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
LOADCELL_STATUS loadcell_status; /* Return code from ADC operation */ 


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
loadcell_status  = LOADCELL_OK;


/*------------------------------------------------------------------------------
 Poll ADC once 
------------------------------------------------------------------------------*/
loadcell_status = poll_loadcell_adc( 1, loadcell_reading_ptr );
if ( loadcell_status != LOADCELL_OK )
	{
    return loadcell_status;
    }
return LOADCELL_OK;

} /* loadcell_get_reading */


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		poll_loadcell_adc                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Sample the load cell ADC a specified number of times in polling mode   *
*                                                                              *
*******************************************************************************/
static LOADCELL_STATUS poll_loadcell_adc 
	(
    uint16_t    num_samples,
    uint32_t*   psample_buffer
    )
{
/* Conversion status indicator */
HAL_StatusTypeDef adc_status;

/* Start the ADC */
HAL_ADC_Start( &( LOADCELL_ADC ) );

/* Poll ADC */
for ( int i = 0; i < num_samples; ++i )
	{
	/* Wait for end of conversion */
    adc_status = HAL_ADC_PollForConversion( &( LOADCELL_ADC ), 
	                                        LOADCELL_ADC_HAL_TIMEOUT );
    if      ( adc_status == HAL_TIMEOUT )
		{
        return LOADCELL_ADC_TIMEOUT;
        }
	else if ( adc_status != HAL_OK      )
		{
        return LOADCELL_ADC_POLL_ERROR;
        }
	else /* No error */
		{
		/* Read the ADC value */
		*(psample_buffer + i) = HAL_ADC_GetValue( &( LOADCELL_ADC ) ); 
        }
    }

/* Stop the ADC */
HAL_ADC_Stop( &( LOADCELL_ADC ) );

/* Conversion successful */
return LOADCELL_OK;

} /* poll_load_cell_adc */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/