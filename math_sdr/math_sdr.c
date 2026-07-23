/**
  ******************************************************************************
  * @file           : math_sdr.c
  * @brief          : Contains math and utility functions for SDR code.
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2025 Sun Devil Rocketry.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is covered under the
  * BSD-3-Clause.
  *
  * https://opensource.org/license/bsd-3-clause
  *
  ******************************************************************************
  */

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

/**
  * @brief Computes a CRC-32 checksum over a block of data.
  *
  * @param[in] data Pointer to the input data buffer.
  * @param[in] len Number of bytes in the buffer.
  *
  * @return The 32-bit CRC checksum of the input data.
  */
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


/**
  * @brief Converts ZYX Euler angles to a quaternion.
  *
  * @note Angles must be provided in radians.
  * @note The rotation order is ZYX (standard aerospace convention). 
  * @todo The rotation sequence has not been validated yet.
  *
  * @param yaw   Rotation about the Z axis in radians.
  * @param pitch Rotation about the Y axis in radians.
  * @param roll  Rotation about the X axis in radians.
  *
  * @return The quaternion representing the rotation.
  */
QUAT eul_to_quat
    (
    float yaw,
    float pitch,
    float roll
    )
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

} /* eul_to_quat */


/**
  * @brief Multiplies two quaternions.
  *
  * @note Quaternion multiplication is NOT commutative.
  *
  * @param a The left-hand quaternion.
  * @param b The right-hand quaternion.
  *
  * @return The quaternion product a * b.
  */
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

} /* quat_mult */


/**
  * @brief Provides the dot product of two quaternions.
  *
  * @note Quaternion dot products are commutative.
  *
  * @param a The left-hand quaternion.
  * @param b The right-hand quaternion.
  *
  * @return The quaternion scalar product a . b
  */
float quat_dot
    (
    QUAT a,
    QUAT b
    )
{
return( a.w * b.w +
        a.x * b.x +
        a.y * b.y +
        a.z * b.z );
        
} /* quat_mult */


/**
  * @brief Adds two quaternions component-wise.
  *
  * @param a The first quaternion.
  * @param b The second quaternion.
  *
  * @return The quaternion sum a + b.
  */
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

} /* quat_add */


/**
  * @brief Scales a quaternion by a scalar value.
  *
  * @param q The quaternion to scale.
  * @param s The scalar factor.
  *
  * @return The quaternion @p q scaled by @p s (q * s).
  */
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

} /* quat_scale */


/**
  * @brief Normalizes a quaternion to unit length.
  *
  * @param q The quaternion to normalize.
  *
  * @return The normalized unit quaternion.
  *
  * Divides each component by the quaternion's norm. If the norm is
  * zero (e.g. a zero-initialized quaternion), returns the identity
  * quaternion (1, 0, 0, 0) to avoid a divide-by-zero. Values near 
  * but not equal to zero are divided normally and may exhibit 
  * floating-point roundoff.
  */
QUAT quat_normalize
    (
    QUAT q
    )
{
QUAT result;

float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

/* Fallback for zero quaternion: initialize to identity quaternion */
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

} /* quat_normalize */


/**
  * @brief Computes the conjugate of a quaternion.
  *
  * @param q The input quaternion.
  *
  * @return The conjugate quaternion (w, -x, -y, -z).
  */
QUAT quat_conj
    (
    QUAT q
    )
{
QUAT result = { q.w, -q.x, -q.y, -q.z };
return result;

} /* quat_conj */

/**
  * @brief Rotates a vector from the world frame into the body frame.
  *
  * The attitude quaternion represents the body-to-world rotation:
  *
  *     vector_body = conjugate(q) * vector_world * q
  *
  * @param attitude Body-to-world attitude quaternion.
  * @param vector_world Pure quaternion containing the world-frame vector.
  *
  * @return Pure quaternion containing the body-frame vector.
  */
QUAT quat_rotate_world_to_body
    (
    QUAT attitude,
    QUAT vector_world
    )
{
QUAT attitude_conj = quat_conj(attitude);

return quat_mult
    (
    quat_mult(attitude_conj, vector_world),
    attitude
    );

} /* quat_rotate_world_to_body */


/**
  * @brief Rotates a vector from the body frame into the world frame.
  *
  * The attitude quaternion represents the body-to-world rotation:
  *
  *     vector_world = q * vector_body * conjugate(q)
  *
  * @param attitude Body-to-world attitude quaternion.
  * @param vector_body Pure quaternion containing the body-frame vector.
  *
  * @return Pure quaternion containing the world-frame vector.
  */
QUAT quat_rotate_body_to_world
    (
    QUAT attitude,
    QUAT vector_body
    )
{
QUAT attitude_conj = quat_conj(attitude);

return quat_mult
    (
    quat_mult(attitude, vector_body),
    attitude_conj
    );

} /* quat_rotate_body_to_world */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
