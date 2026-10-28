/**
  ******************************************************************************
  * @file           : nav_utl.c
  * @brief          : Navigation & sensor fusion library utilities.
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2026 Sun Devil Rocketry.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
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
#include <string.h>
#include <stdbool.h>
#include <math.h>

/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "math_sdr.h"

#include "nav.h"
#include "nav_prv.h"

/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

/**
  * @brief Resets accumulated velocity values to prevent drift.
  * 
  * @param nav_state The nav library state
  * 
  * @return The status of this operation.
  */
NAV_STATUS nav_reset_velo
	(
	NAV_PUBLIC_STATE* nav_state
	)
{
/* Local Variables */
NAV_PRIVATE_STATE* prv_state = nav_state_to_prv( nav_state );
NAV_STATUS status = NAV_OK;

/* Reset Velocity */
if( nav_reserve_state( nav_state ) == NAV_OK )
    {
    status = _nav_reset_velo( prv_state );
    }
else
    {
    status = NAV_BUSY;
    }

return( status );

} /* nav_reset_velo */

/*------------------------------------------------------------------------------
 Private Functions 
------------------------------------------------------------------------------*/

/**
  * @brief Resets accumulated velocity values to prevent drift.
  * 
  * @param prv_state The nav library private state
  * 
  * @return The status of this operation.
  */
NAV_STATUS _nav_reset_velo
	(
	NAV_PRIVATE_STATE* prv_state
	)
{
/* Reset Velocity */
memset( &(prv_state->velocity), 0, sizeof( VECTOR_3F ) );

return( NAV_OK );

} /* _nav_reset_velo */


/**
 * @brief Calculate an attitude quaternion from an acceleration vector.
 * 
 * @param attitude Starting attitude (for yaw)
 * @param accel Acceleration vector
 * 
 * @return A quaternion representing the attitude derived from acceleration
 */
QUAT _nav_quat_grav_attitude
	(
    QUAT attitude,
    VECTOR_3F accel
	)
{
/* Compute pitch/roll from accelerometer */
float grav_pitch = atan2f(accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z));
float grav_roll  = atan2f(accel.y, accel.z);

float yaw = _nav_quat_to_yaw(attitude);

return eul_to_quat(yaw, grav_pitch, grav_roll);

} /* _nav_quat_grav_attitude */


/**
 * @brief Calculate the yaw angle from an attitude quaternion.
 * 
 * @param q An attitude quaternion.
 * 
 * @return The yaw angle.
 */
float _nav_quat_to_yaw
	(
	QUAT q
	)
{
float y = 2.0f * (q.w * q.z + q.x * q.y);
float x = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);

return atan2f(y, x);

} /* _nav_quat_to_yaw */


/**
  * @brief Calculates velocity from converted accelerometer measurements.
  * @note Assumes state reserved by the caller.
  * @note Requires attitude estimate to already be updated.
  * 
  * @param prv_state The private nav library state.
  * @param tdelta The delta between measurements
  * 
  * @return The status of the operation
  */
NAV_STATUS _nav_imu_velo
	(
	NAV_PRIVATE_STATE* prv_state,
    float tdelta
	)
{
float velo_x, velo_y, velo_z, velocity;
NAV_SENSORS* i = &(prv_state->public_state.nav_sensors);
NAV_OUTPUTS* o = &(prv_state->public_state.nav_outputs);

/*
 * The world frame uses North-East-Down (NED) coordinates,
 * so gravity points in the world-frame +Z direction.
 */
const QUAT gravity_world =
    {
    .w = 0.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = -GRAVITY
    };

/*
 * The attitude quaternion is body-to-world, so rotate world gravity
 * into the body frame before subtracting it from the accelerometer.
 */
QUAT gravity_body = quat_rotate_world_to_body
    (
    prv_state->public_state.nav_outputs.orientation,
    gravity_world
    );

QUAT linear_accel_body =
    {
    .w = 0.0f,
    .x = i->accel.x - gravity_body.x,
    .y = i->accel.y - gravity_body.y,
    .z = i->accel.z - gravity_body.z,
    };

/*
 * Rotate gravity-compensated acceleration into the world frame so the
 * integrated velocity components remain in a fixed coordinate frame.
 */
QUAT linear_accel_world = quat_rotate_body_to_world
    (
    prv_state->public_state.nav_outputs.orientation,
    linear_accel_body
    );

float accel_world_x = linear_accel_world.x;
float accel_world_y = linear_accel_world.y;
float accel_world_z = linear_accel_world.z;

// Calculate 3 velocity vectors using motion equations
velo_x = prv_state->velocity.x + accel_world_x * tdelta;
velo_y = prv_state->velocity.y + accel_world_y * tdelta;
velo_z = prv_state->velocity.z + accel_world_z * tdelta;

// Calculate the velocity scalar
velocity = sqrtf
    (
    velo_x * velo_x +
    velo_y * velo_y +
    velo_z * velo_z
    );

/* Update state estimations*/
o->velocity.x = velo_x;
o->velocity.y = velo_y;
o->velocity.z = velo_z;

o->velo_mag = velocity;

// Save current velocity for next computation
prv_state->velocity.x = velo_x;
prv_state->velocity.y = velo_y;
prv_state->velocity.z = velo_z;

return NAV_OK;

} /* _nav_imu_velo */

/**
  * END OF FILE
  */