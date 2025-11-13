/*******************************************************************************
*
* FILE: 
* 		math_sdr.h
*
* DESCRIPTION: 
* 		Contains utility functions for SDR code.
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MATH_SDR_H 
#define MATH_SDR_H 

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Constants */
#define COMP_ALPHA 0.98f /* acc-gyro sensor fusion trust (higher = trust gyro more) */
#define MAG_COMP_ALPHA 0.95f  /* mag-gyro sensor fusion trust (higher = trust gyro more) */

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
* MACRO:                                                                       * 
*       rad_to_deg                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Convert a value in radians to degrees.                                 *
*                                                                              *
*******************************************************************************/
#define rad_to_deg(x) ((x) * 57.29577951f)


/*******************************************************************************
*                                                                              *
* MACRO:                                                                       * 
*       deg_to_rad                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Convert a value in degrees to radians.                                 *
*                                                                              *
*******************************************************************************/
#define deg_to_rad(x) ((x) * 0.01745329252f)


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/
uint32_t crc32
    (
    const uint8_t *data, 
    size_t len
    );
    
#ifdef __cplusplus
}
#endif
#endif /* MATH_SDR_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/