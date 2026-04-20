/*******************************************************************************
*
* FILE:
* 		math_sdr.c
*
* DESCRIPTION: 
* 		Contains math functions for SDR code.
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

/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "math_sdr.h"

/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		crc32                                                                  *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns a 32bit checksum from the given data.                          *
*                                                                              *
*******************************************************************************/
uint32_t crc32
    (
    const uint8_t *data, 
    size_t len
    ) 
{
uint32_t crc = 0xFFFFFFFF;
while (len--) 
    {
    crc ^= *data++;
    for (int i = 0; i < 8; ++i)
        crc = (crc >> 1) ^ (0x82F63B78 & -(crc & 1));
    }
return ~crc;

} /* crc32 */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/