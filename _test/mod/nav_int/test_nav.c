/*******************************************************************************
*
* FILE:
*      test_nav.c
*
* DESCRIPTION:
*      Unit tests for the nav module
*
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/*------------------------------------------------------------------------------
Project Includes
------------------------------------------------------------------------------*/
#include "sdrtf_pub.h"

#include "nav_prv.h"

/*------------------------------------------------------------------------------
Global Variables
------------------------------------------------------------------------------*/
NAV_PUBLIC_STATE nav_state;
NAV_PRIVATE_STATE* prv_state;

extern NAV_STATUS stub__state_reserve;
extern NAV_STATUS stub__state_release;

/*------------------------------------------------------------------------------
Procedures: Tests
------------------------------------------------------------------------------*/

/**
 * @brief This function tests the initialization function for NAV.
 */
void test_nav_init
	(
	void
	)
{
/*------------------------------------------------------------------------------
Case 1: Nominal Initialization
------------------------------------------------------------------------------*/
NAV_STATUS nav_status = NAV_OK;
NAV_CONFIG nav_config;
NAV_SENSORS calib;

nav_config.accel_enabled = true;
nav_config.gyro_enabled = true;
nav_config.mag_enabled = true;
nav_config.baro_enabled = true;
nav_config.gps_enabled = true;
nav_config.mahony_kp = 1.0f;

calib.accel.x = 9.81f;
calib.accel.y = 0.0f;
calib.accel.z = 0.0f;

prv_state->velocity.x = 100.0f;
prv_state->velocity.y = 100.0f;
prv_state->velocity.z = 100.0f;

VECTOR_3F zero_vec = { 0.0f };

/*------------------------------------------------------------------------------
Call FUT and verify
------------------------------------------------------------------------------*/
nav_status = nav_init(&nav_state, nav_config, calib);

TEST_ASSERT_EQ_UINT( "Verify that nav initialized normally.", nav_status, NAV_OK );
TEST_ASSERT_EQ_MEMORY( "Verify that the configuration was copied to the internal state", 
                       &(prv_state->saved_configs),
                       &(nav_config),
                       sizeof( NAV_CONFIG ) );
TEST_ASSERT_EQ_MEMORY( "Verify that the calibration was copied to the internal state", 
                       &(prv_state->cal_data),
                       &(calib),
                       sizeof( NAV_SENSORS ) );
TEST_ASSERT_EQ_MEMORY( "Verify that the velocity vector was reset",
                       &(prv_state->velocity),
                       &zero_vec,
                       sizeof( VECTOR_3F ) );

/*------------------------------------------------------------------------------
Case 2: Failed Reservation
------------------------------------------------------------------------------*/
memset( &nav_state, 0, sizeof( NAV_PUBLIC_STATE ) );
NAV_PUBLIC_STATE comparison = { 0 };

nav_status = NAV_OK;

nav_config.accel_enabled = true;
nav_config.gyro_enabled = true;
nav_config.mag_enabled = true;
nav_config.baro_enabled = true;
nav_config.gps_enabled = true;
nav_config.mahony_kp = 1.0f;

calib.accel.x = 9.81f;
calib.accel.y = 0.0f;
calib.accel.z = 0.0f;

stub__state_reserve = NAV_BUSY;

/*------------------------------------------------------------------------------
Call FUT and verify
------------------------------------------------------------------------------*/
nav_status = nav_init(&nav_state, nav_config, calib);

TEST_ASSERT_EQ_UINT( "Verify that nav returned a busy status.", nav_status, NAV_BUSY );
TEST_ASSERT_EQ_MEMORY( "Verify that NAV did not edit its state", 
                       &nav_state,
                       &comparison,
                       sizeof( NAV_PUBLIC_STATE ) );

/*------------------------------------------------------------------------------
Case 3: Bad Config
------------------------------------------------------------------------------*/
memset( &nav_state, 0, sizeof( NAV_PUBLIC_STATE ) );

nav_status = NAV_OK;

nav_config.accel_enabled = false;
nav_config.gyro_enabled = true;
nav_config.mag_enabled = true;
nav_config.baro_enabled = true;
nav_config.gps_enabled = true;
nav_config.mahony_kp = 1.0f;

calib.accel.x = 9.81f;
calib.accel.y = 0.0f;
calib.accel.z = 0.0f;

stub__state_reserve = NAV_OK;

/*------------------------------------------------------------------------------
Call FUT and verify
------------------------------------------------------------------------------*/
nav_status = nav_init(&nav_state, nav_config, calib);

TEST_ASSERT_EQ_UINT( "Verify that nav returned an invalid status.", nav_status, NAV_INVALID_CONFIG );
TEST_ASSERT_EQ_MEMORY( "Verify that NAV did not edit its state", 
                       &nav_state,
                       &comparison,
                       sizeof( NAV_PUBLIC_STATE ) );

} /* test_sensor_init */


/**
 * @brief This function tests the update function for NAV.
 */
void test_nav_update_errors
	(
	void
	)
{
/*------------------------------------------------------------------------------
Case 1: Failed Reservation
------------------------------------------------------------------------------*/
memset( &nav_state, 0, sizeof( NAV_PUBLIC_STATE ) );
NAV_PUBLIC_STATE comparison = { 0 };

NAV_STATUS nav_status = NAV_OK;

prv_state->saved_configs.accel_enabled = true;
prv_state->saved_configs.gyro_enabled = true;
prv_state->saved_configs.mag_enabled = true;
prv_state->saved_configs.baro_enabled = true;
prv_state->saved_configs.gps_enabled = true;
prv_state->saved_configs.mahony_kp = 1.0f;

prv_state->cal_data.accel.x = 9.81f;
prv_state->cal_data.accel.y = 0.0f;
prv_state->cal_data.accel.z = 0.0f;

memcpy( &comparison, &nav_state, sizeof( NAV_PUBLIC_STATE ) );

stub__state_reserve = NAV_BUSY;

/*------------------------------------------------------------------------------
Call FUT and verify
------------------------------------------------------------------------------*/
nav_status = nav_update(&nav_state, 100.0f);

TEST_ASSERT_EQ_UINT( "Verify that nav returned a busy status.", nav_status, NAV_BUSY );
TEST_ASSERT_EQ_MEMORY( "Verify that NAV did not edit its state", 
                       &nav_state,
                       &comparison,
                       sizeof( NAV_PUBLIC_STATE ) );

/*------------------------------------------------------------------------------
Case 2: Failed Release
------------------------------------------------------------------------------*/
memset( &nav_state, 0, sizeof( NAV_PUBLIC_STATE ) );

nav_status = NAV_OK;

prv_state->saved_configs.accel_enabled = true;
prv_state->saved_configs.gyro_enabled = true;
prv_state->saved_configs.mag_enabled = true;
prv_state->saved_configs.baro_enabled = true;
prv_state->saved_configs.gps_enabled = true;
prv_state->saved_configs.mahony_kp = 1.0f;

prv_state->cal_data.accel.x = 9.81f;
prv_state->cal_data.accel.y = 0.0f;
prv_state->cal_data.accel.z = 0.0f;

memcpy( &comparison, &nav_state, sizeof( NAV_PUBLIC_STATE ) );

stub__state_reserve = NAV_OK;
stub__state_release = NAV_FAIL;

/*------------------------------------------------------------------------------
Call FUT and verify
------------------------------------------------------------------------------*/
nav_status = nav_update(&nav_state, 100.0f);

TEST_ASSERT_EQ_UINT( "Verify that nav returned a fail status.", nav_status, NAV_FAIL );

} /* test_nav_update_errors */


/**
 * @brief This function tests the update function for NAV.
 */
void test_nav_update
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local Variables
------------------------------------------------------------------------------*/
NAV_STATUS nav_status = NAV_OK;

/*------------------------------------------------------------------------------
Local Predefined Options
------------------------------------------------------------------------------*/
NAV_CONFIG  on_ground_asl_config = 
    {
    .accel_enabled = true,
    .baro_enabled = true,
    .gps_enabled = true,
    .gyro_enabled = true,
    .mag_enabled = true,
    .mahony_kp = 1.0f
    };
NAV_SENSORS on_ground_asl_sensors =
    {
    .accel = { -9.8f, 0.0f, 0.0f },
    .baro_pres = 101325.0f,
    .baro_temp = 15.0f,
    .gps_alt = 0.0f,
    .gps_brg = 270.0f,
    .gps_posn = { 45.0f, 45.0f },
    .gyro = { 0.0f, 0.0f, 0.0f },
    .mag = { 0.0f, 0.0f, 0.0f } // don't care
    };
NAV_SENSORS on_ground_asl_calib = on_ground_asl_sensors;
NAV_OUTPUTS on_ground_asl_expected =
    {
    .fused_posn = { 45.0f, 45.0f },
    .ind_alt_agl = 0.0f,
    .ind_alt_asl = 0.0f,
    .orientation = { 0.707107f, 0.0f, -0.707107f, 0.0f },
    .velo_mag = 0.0f,
    .velocity = { 0.0f, 0.0f, 0.0f }
    };

NAV_CONFIG on_ground_asl_config_no_gps = 
    {
    .accel_enabled = true,
    .baro_enabled = true,
    .gps_enabled = false,
    .gyro_enabled = true,
    .mag_enabled = true,
    .mahony_kp = 1.0f
    };
NAV_OUTPUTS on_ground_asl_expected_no_gps =
    {
    .fused_posn = { INFINITY, INFINITY },
    .ind_alt_agl = 0.0f,
    .ind_alt_asl = 0.0f,
    .orientation = { 0.707107f, 0.0f, -0.707107f, 0.0f },
    .velo_mag = 0.0f,
    .velocity = { 0.0f, 0.0f, 0.0f }
    };


/*------------------------------------------------------------------------------
Local Case Struct
------------------------------------------------------------------------------*/
struct TEST_CASE {
    const char* description;
    bool in_air;
    NAV_CONFIG* config;
    NAV_SENSORS* sensors;
    NAV_SENSORS* calib;
    NAV_OUTPUTS* outputs;
};
struct TEST_CASE cases[] =
    {
    { "Test sensor fusion on ground pointing up vertically", false,
      &on_ground_asl_config, &on_ground_asl_sensors, &on_ground_asl_calib, &on_ground_asl_expected },
    { "Test sensor fusion on ground with GPS disabled", false,
      &on_ground_asl_config_no_gps, &on_ground_asl_sensors, &on_ground_asl_calib, &on_ground_asl_expected_no_gps }
    };

for( int i = 0; i < array_size( cases ); i++ )
    {
    TEST_begin_nested_case( cases[i].description );

    /*------------------------------------------------------------------------------
    Set Up Test
    ------------------------------------------------------------------------------*/
    stub__state_release = NAV_OK;
    stub__state_reserve = NAV_OK;

    memset( prv_state, 0, sizeof( NAV_PUBLIC_STATE ) );
    prv_state->cal_data = *(cases[i].calib);
    prv_state->saved_configs = *(cases[i].config);
    prv_state->public_state.nav_outputs = *(cases[i].outputs);
    prv_state->public_state.nav_sensors = *(cases[i].sensors);
    prv_state->public_state.in_air = cases[i].in_air;

    /* seed mahony */
    TEST_ASSERT_EQ_UINT( "Meta: Verify that mahony setup worked.", _nav_mahony_init( prv_state ), NAV_OK );

    /*------------------------------------------------------------------------------
    Call FUT
    ------------------------------------------------------------------------------*/
    nav_status = nav_update(&nav_state, 1.0f);
    
    /*------------------------------------------------------------------------------
    Verify Results
    ------------------------------------------------------------------------------*/
    TEST_ASSERT_EQ_UINT( "Verify the function returned a valid status.", nav_status, NAV_OK );
    TEST_ASSERT_NEAR_FLOAT( "Verify that the velocity magnitude matches the expected", 
        nav_state.pub.nav_outputs.velo_mag, 
        cases[i].outputs->velo_mag,
        0.001f );
    TEST_ASSERT_NEAR_FLOAT( "Verify that the velocity along the X axis matches the expected", 
        nav_state.pub.nav_outputs.velocity.x, 
        cases[i].outputs->velocity.x,
        0.001f );
    TEST_ASSERT_NEAR_FLOAT( "Verify that the velocity along the Y axis matches the expected", 
        nav_state.pub.nav_outputs.velocity.y, 
        cases[i].outputs->velocity.y,
        0.001f );
    TEST_ASSERT_NEAR_FLOAT( "Verify that the velocity along the Z axis matches the expected", 
        nav_state.pub.nav_outputs.velocity.z, 
        cases[i].outputs->velocity.z,
        0.001f );

    TEST_ASSERT_NEAR_FLOAT( "Verify that the AGL altitude matches the expected", 
        nav_state.pub.nav_outputs.ind_alt_agl, 
        cases[i].outputs->ind_alt_agl,
        0.001f );
    TEST_ASSERT_NEAR_FLOAT( "Verify that the ASL altitude matches the expected", 
        nav_state.pub.nav_outputs.ind_alt_asl, 
        cases[i].outputs->ind_alt_asl,
        0.001f );

    TEST_ASSERT_NEAR_FLOAT( "Verify that the GPS latitude matches the expected", 
        nav_state.pub.nav_outputs.fused_posn.latitude, 
        cases[i].outputs->fused_posn.latitude,
        0.001f );
    TEST_ASSERT_NEAR_FLOAT( "Verify that the GPS longitude matches the expected", 
        nav_state.pub.nav_outputs.fused_posn.longitude, 
        cases[i].outputs->fused_posn.longitude,
        0.001f );
    
    TEST_ASSERT_NEAR_FLOAT( "Verify that the Quat W component matches the expected", 
        nav_state.pub.nav_outputs.orientation.w, 
        cases[i].outputs->orientation.w,
        0.001f );
    TEST_ASSERT_NEAR_FLOAT( "Verify that the Quat X component matches the expected", 
        nav_state.pub.nav_outputs.orientation.x, 
        cases[i].outputs->orientation.x,
        0.001f );
    TEST_ASSERT_NEAR_FLOAT( "Verify that the Quat Y component matches the expected", 
        nav_state.pub.nav_outputs.orientation.y, 
        cases[i].outputs->orientation.y,
        0.001f );
    TEST_ASSERT_NEAR_FLOAT( "Verify that the Quat Z component matches the expected", 
        nav_state.pub.nav_outputs.orientation.z, 
        cases[i].outputs->orientation.z,
        0.001f );

    TEST_end_nested_case();
    }
} /* test_nav_update */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       main                                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Set up the testing environment and run the sensor error tests.          *
*                                                                              *
*******************************************************************************/
int main
	(
	void
	)
{
/*------------------------------------------------------------------------------
Test Cases
------------------------------------------------------------------------------*/
unit_test tests[] =
	{
	{ "Nav: Test initialization", test_nav_init, "RQ.MOD.00054, RQ.MOD.00055, RQ.MOD.00057" },
    { "Nav: Test update error handling", test_nav_update_errors, "RQ.MOD.00053, RQ.MOD.00055" },
    { "Nav: Test update", test_nav_update, "RQ.MOD.00053, RQ.MOD.00056" }
	};

/*------------------------------------------------------------------------------
Global Setup
------------------------------------------------------------------------------*/
prv_state = (NAV_PRIVATE_STATE*)&nav_state;

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_set_type( TEST_TYPE_SW_INTEGRATION );
TEST_INITIALIZE_TEST( "nav_int", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
