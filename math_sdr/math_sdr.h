/**
  ******************************************************************************
  * @file           : math_sdr.h
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
  @verbatim
  ==============================================================================
                      ##### Math module features #####
  ==============================================================================
  [..]
  (+) Macros for common values, conversions, and utilities
  (+) CRC-32 checksum of data
  (+) Quaternion operations
  ******************************************************************************
  @endverbatim
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MATH_SDR_H 
#define MATH_SDR_H 

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>


/* Typedefs ------------------------------------------------------------------*/

/* Quaternion */
typedef struct _QUAT
	{
	float w, x, y, z;
	} QUAT;

/**
 * @brief Three-dimensional floating-point vector.
 */
typedef struct _VECTOR_3F
    {
    float x;
    float y;
    float z;
    } VECTOR_3F;

/* Macros --------------------------------------------------------------------*/

/* Constants */
#define GRAVITY 9.8f
#define IDENTITY_QUAT ((QUAT) {1.0f, 0.0f, 0.0f, 0.0f })

/* Conversion Constants */
#define FT_TO_M 0.3048f

/**
  * @brief Sets a certain bit and returns the new value.
  *
  * @param orig The original value.
  * @param idx Index of the bit to set.
  *
  * @return @p orig with bit @p idx set.
  */
#define util_set_bit( orig, idx ) ( orig | ( 1 << idx ) )


/**
  * @brief Convert a value in radians to degrees.
  *
  * @param x Value in radians.
  *
  * @return Equivalent value in degrees.
  */
#define rad_to_deg(x) ((x) * 57.29577951f)


/**
  * @brief Convert a value in degrees to radians.
  *
  * @param x Value in degrees.
  *
  * @return Equivalent value in radians.
  */
#define deg_to_rad(x) ((x) * 0.01745329252f)


/**
  * @brief Returns the number of elements in an array where each element is a fixed size.
  *
  * @note An error or warning from this macro indicates that it can't be used 
  *       in that context.
  *
  * @param array The array with desired element count.
  *
  * @return Number of elements in @p array.
  */
#define array_size( array ) ( sizeof( array ) / sizeof( array[0] ) )

/* Function Prototypes -------------------------------------------------------*/

uint32_t math_crc32
    (
    const uint8_t *data, 
    size_t len
    );

float math_clamp_float
    (
    float value,
    float minimum,
    float maximum
    );

/* Quaternions ---------------------------------------------------------------*/

QUAT math_eul_to_quat
    (
    float yaw,
    float pitch,
    float roll
    );

QUAT math_quat_mult
    (
    QUAT a,
    QUAT b
    );

float math_quat_dot
    (
    QUAT a,
    QUAT b
    );

QUAT math_quat_add
    (
    QUAT a,
    QUAT b
    );

QUAT math_quat_scale
    (
    QUAT q,
    float s
    );

QUAT math_quat_normalize
    (
    QUAT q
    );

QUAT math_quat_conj
    (
    QUAT q
    );

QUAT math_quat_rotate_world_to_body
    (
    QUAT attitude,
    QUAT vector_world
    );

QUAT math_quat_rotate_body_to_world
    (
    QUAT attitude,
    QUAT vector_body
    );

bool math_quat_is_finite
    (
    QUAT quaternion
    );

/* Vectors -------------------------------------------------------------------*/

bool math_vector_is_finite
    (
    VECTOR_3F vector
    );

float math_vector_magnitude
    (
    VECTOR_3F vector
    );

bool math_vector_normalize
    (
    VECTOR_3F *vector
    );

VECTOR_3F math_vector_cross
    (
    VECTOR_3F a,
    VECTOR_3F b
    );

VECTOR_3F math_vector_add
    (
    VECTOR_3F a,
    VECTOR_3F b
    );

VECTOR_3F math_vector_scale
    (
    VECTOR_3F vector,
    float scalar
    );

#ifdef __cplusplus
}
#endif
#endif /* MATH_SDR_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/