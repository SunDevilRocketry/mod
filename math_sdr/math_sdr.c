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


/* Standard ZYX conversion - maybe not the right order? */
/* TAKES RADIANS */
QUAT eul_to_quat(float yaw, float pitch, float roll)
{
float cos_yaw = cosf(yaw / 2.0f);
float cos_pitch = cosf(pitch / 2.0f);
float cos_roll = cosf(roll / 2.0f);

float sin_yaw = sinf(yaw / 2.0f);
float sin_pitch = sinf(pitch / 2.0f);
float sin_roll = sinf(roll / 2.0f);

QUAT q;
q.w = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;
q.x = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
q.y = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
q.z = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;

return q;

}

/* Note: quaternion multiplication is NOT commutative */
QUAT quat_mult
    (
    QUAT a,
    QUAT b
    )
{
QUAT result;

result.w = (a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z);
result.x = (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y);
result.y = (a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x);
result.z = (a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w);

return result;

}


QUAT quat_add
    (
    QUAT a,
    QUAT b
    )
{
QUAT result;

result.w = a.w + b.w;
result.x = a.x + b.x;
result.y = a.y + b.y;
result.z = a.z + b.z;

return result;

}


QUAT quat_scale
    (
    QUAT q,
    float s
    )
{
QUAT result;

result.w = q.w * s;
result.x = q.x * s;
result.y = q.y * s;
result.z = q.z * s;

return result;

}


QUAT quat_normalize
    (
    QUAT q
    )
{
QUAT result;

float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

/* Fallback for zero quaternion: initialize to unit quaternion
   This check is really only for potentially zero initialized quats to avoid divide by zero.
   Otherwise, values really close to zero could just have bad floating point roundoff */
if ( norm == 0.0f ) 
    {               
    result.w = 1.0f;
    result.x = 0.0f;
    result.y = 0.0f;
    result.z = 0.0f;
    }
else
    {
    result.w = q.w / norm;
    result.x = q.x / norm;
    result.y = q.y / norm;
    result.z = q.z / norm;
    }

return result;

}


QUAT quat_conj
    (
    QUAT q
    )
{
QUAT result = { q.w, -q.x, -q.y, -q.z };
return result;

}

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/