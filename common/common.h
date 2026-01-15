/*******************************************************************************
*
* FILE: 
* 		common.h
*
* DESCRIPTION: 
* 		Contains utility functions for SDR code.
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
#ifndef COMMON_H 
#define COMMON_H 

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Includes 
------------------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* UID serial number (packed struct inhibits padding) */
typedef struct __attribute__((packed)) _ST_UID_TYPE 
    {
    uint32_t wafer_coords;
    char lot_num_1[3];
    uint8_t wafer_num;
    char lot_num_2[4];
    } ST_UID_TYPE;
_Static_assert( sizeof(ST_UID_TYPE) == 12, "ST_UID_TYPE packing incorrect." );

/*------------------------------------------------------------------------------
 Macros & Inlines
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* MACRO:                                                                       * 
*       array_size                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns the number of elements in an array where each element is a     *
*       fixed size. An error or warning from this macro indicates that it      *
*       can't be used in that context.                                         *
*                                                                              *
*******************************************************************************/
#define array_size( array ) ( sizeof( array ) / sizeof( array[0] ) )


/*******************************************************************************
*                                                                              *
* MACRO:                                                                       * 
*       util_set_bit                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Sets a certain bit and returns the new value                           *
*                                                                              *
*******************************************************************************/
#define util_set_bit( orig, idx ) ( orig | ( 1 << idx ) )


/*******************************************************************************
*                                                                              *
* INLINE:                                                                      * 
*       get_uid                                                                *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Gets the 12 byte UID value for the H7 chip.                            *
*                                                                              *
*******************************************************************************/
static inline void get_uid
    (
    ST_UID_TYPE* uid_buffer
    )
{
uint32_t uid[3];
uid[0] = HAL_GetUIDw0();
uid[1] = HAL_GetUIDw1();
uid[2] = HAL_GetUIDw2();

memcpy( uid_buffer, uid, sizeof( ST_UID_TYPE ) );

} /* get_uid */


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* common */

void delay_ms
    (
    uint32_t delay
    );

#ifdef __cplusplus
}
#endif
#endif /* COMMON_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/