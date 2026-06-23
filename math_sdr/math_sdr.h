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
  (+) Quaternion arithmetic
  ******************************************************************************
  @endverbatim
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MATH_SDR_H 
#define MATH_SDR_H 

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Includes 
------------------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>


/*------------------------------------------------------------------------------
 Typedefs 
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


/* Quaternion */
typedef struct _QUAT
	{
	float w, x, y, z;
	} QUAT;


/*------------------------------------------------------------------------------
 Macros and Inlines
------------------------------------------------------------------------------*/

/* Constants */
#define COMP_ALPHA 0.98f /* Used in sensor fusion */
#define GRAVITY 9.8f


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


// NA TODO: preserving the old banner in case this is moved
/*******************************************************************************
*                                                                              *
* INLINE:                                                                      * 
*       get_uid                                                                *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Gets the 12 byte UID value for the H7 chip.                            *
*                                                                              *
*******************************************************************************/
/**
 * @brief Gets the 12 byte UID value for the H7 chip.
 * 
 * @param[out] uid_buffer Pointer to the buffer to receive the UID.
 */
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

uint32_t crc32
    (
    const uint8_t *data, 
    size_t len
    );

QUAT eul_to_quat
    (
    float yaw,
    float pitch,
    float roll
    );

QUAT quat_mult
    (
    QUAT a,
    QUAT b
    );

QUAT quat_add
    (
    QUAT a,
    QUAT b
    );

QUAT quat_scale
    (
    QUAT q,
    float s
    );

QUAT quat_normalize
    (
    QUAT q
    );

QUAT quat_conj
    (
    QUAT q
    );
    
#ifdef __cplusplus
}
#endif
#endif /* MATH_SDR_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/