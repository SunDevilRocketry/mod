/*******************************************************************************
*
* FILE: 
* 		loadcell.h
*
* DESCRIPTION: 
* 		Contains API functions for reading data from the engine's load cell 
*
* COPYRIGHT:                                                                   
*       Copyright (c) 2025 Sun Devil Rocketry.                                 
*       All rights reserved.                                                   
*                                                                              
*       This software is licensed under terms that can be found in the LICENSE 
*       file in the root directory of this software component.                 
*       If no LICENSE file comes with this software, it is covered under the   
*       BSD-3-Clause.                                                          
*                                                                              
*       https://opensource.org/license/bsd-3-clause          
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LOADCELL_H 
#define LOADCELL_H 

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/
#define LOADCELL_ADC_HAL_TIMEOUT  ( 100 ) /* ADC timeout for a single 
                                             conversion in milliseconds       */


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Loadcell return codes */
typedef enum LOADCELL_STATUS 
    {
	LOADCELL_OK = 0        ,
    LOADCELL_ADC_TIMEOUT   ,
	LOADCELL_ADC_POLL_ERROR,
    LOADCELL_FAIL        
    } LOADCELL_STATUS;


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

/* Read the load cell force */
LOADCELL_STATUS loadcell_get_reading 
	(
    uint32_t* loadcell_reading_ptr
    );

#ifdef __cplusplus
}
#endif

#endif /* LOADCELL_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/