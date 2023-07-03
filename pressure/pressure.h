/*******************************************************************************
*
* FILE: 
* 		pressure.h
*
* DESCRIPTION: 
* 		Contains API functions for reading data from the engine's pressure
*       transducers 
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PRESSURE_H 
#define PRESSURE_H 

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Includes 
------------------------------------------------------------------------------*/

/* GCC requires stdint.h for uint_t types*/
#ifdef UNIT_TEST
	#include <stdint.h>
#endif

/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

#define NUM_PTS              ( 8   )  /* Number of pressure transducers       */
#define PT_MUX_BITMASK_SHIFT ( 9   )  /* Bit shift to convert from pt number
                                         to GPIO pin bitmask                  */
#define ADC_TIMEOUT          ( 100 )  /* ADC timeout for a single conversion  
                                         in milliseconds                      */


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Pressure transducer subcommand codes */
typedef enum PRESSURE_STATUS 
    {
	PRESSURE_OK         = 0,
    PRESSURE_ADC_TIMEOUT   ,
	PRESSURE_ADC_POLL_ERROR,
    PRESSURE_FAIL        
    } PRESSURE_STATUS;

/* Pressure transducer numbers */
// TODO: Change the enumeration tags to be descriptive of what
//       each pressure transducer measures 
typedef enum PRESSURE_PT_NUMS
	{
    PT_NUM0 = 0, /* LOX Pressure                  */
    PT_NUM1    , /* LOX Flow Rate Upstream        */
    PT_NUM2    , /* LOX Flow Rate Downstream      */
    PT_NUM3    , /* Extra                         */
    PT_NUM4    , /* Engine Pressure               */
    PT_NUM5    , /* Kerosene Flow Rate Downstream */
    PT_NUM6    , /* Kerosene Flow Rate Upstream   */
    PT_NUM7      /* Kerosene Pressure             */
    } PRESSURE_PT_NUM;


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

/* Get a single pressure transducer reading */
PRESSURE_STATUS pressure_get_pt_reading 
	(
    PRESSURE_PT_NUM pt_num,
    uint32_t*       pt_readout_ptr
    );

/* Get readings from all pressure transducers */
PRESSURE_STATUS pressure_poll_pts
	(
    uint32_t* pPT_readings 
    );

/* Set the amplifier gain for a single pressure transducer */
void pressure_set_gain
	(
    PRESSURE_PT_NUM pt_num,
    uint8_t         gain
    );

/* Set the amplifier gain for all pressure transducers */
void pressure_set_all_gains
	(
    uint8_t gains[] 
    );

/* Get the gain for a single pressure transducer */
uint8_t pressure_get_gain
	(
    PRESSURE_PT_NUM pt_num
    );

/* Get the gain for all pressure transducers */
void pressure_get_all_gains
	(
    uint8_t* pgains 
    );

#ifdef __cplusplus
}
#endif

#endif /* PRESSURE_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/