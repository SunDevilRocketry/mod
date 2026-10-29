/*******************************************************************************
*
* FILE:
*      test_state_transitions.c
*
* DESCRIPTION:
*      Unit tests for launch detection and apogee detection in APPA.
*
*******************************************************************************/

/*------------------------------------------------------------------------------
Standard Includes
------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*------------------------------------------------------------------------------
Project Includes
------------------------------------------------------------------------------*/
#include "sdrtf_pub.h"
#include "math_sdr.h"
#include "nav_prv.h"
#include "test_nav_stubs.h"

/*------------------------------------------------------------------------------
Globals
------------------------------------------------------------------------------*/
NAV_PUBLIC_STATE nav_state;
NAV_PRIVATE_STATE* prv_state = (NAV_PRIVATE_STATE*)(&nav_state);

/*------------------------------------------------------------------------------
Test Procedures
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_launch_detection                                                  *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test the launch detection function.                                    *
*                                                                              *
*******************************************************************************/
void test_launch_detection
    (
    void
    )
{
/*------------------------------------------------------------------------------
Set up test vectors
------------------------------------------------------------------------------*/
#define NUM_LAUNCH_DETECT_CASES 6
#define NUM_LAUNCH_DETECT_SAMPLES 11
int inputs_acc[NUM_LAUNCH_DETECT_CASES][NUM_LAUNCH_DETECT_SAMPLES] =
{
#include "cases/acc_inputs.txt"
};
int inputs_baro[NUM_LAUNCH_DETECT_CASES][NUM_LAUNCH_DETECT_SAMPLES] =
{
#include "cases/baro_inputs.txt"
};
int expected[NUM_LAUNCH_DETECT_CASES][NUM_LAUNCH_DETECT_SAMPLES] =
{
#include "cases/launch_detect_expected.txt"
};
NAV_SENSORS sensors = { 0 };
NAV_OUTPUTS outputs = { 0 };
NAV_STATUS status = NAV_OK;

nav_get_default_config( &(prv_state->saved_configs) );
prv_state->saved_configs.launch_detect_accel_threshold = 6;
prv_state->saved_configs.launch_detect_baro_threshold = 1000;
prv_state->saved_configs.launch_detect_accel_counts = 10;
prv_state->saved_configs.launch_detect_baro_counts = 10;
_nav_pof_init( prv_state );

/*------------------------------------------------------------------------------
Execute tests
------------------------------------------------------------------------------*/
for ( int test_num = 0; test_num < NUM_LAUNCH_DETECT_CASES; test_num++ )
    {
    TEST_begin_nested_case( "" );
    
    prv_state->saved_configs.launch_detect_accel_enabled = false;
    if ( test_num > 1 )
        {
        prv_state->saved_configs.launch_detect_accel_enabled = true;
        }
    if ( test_num > 3 )
        {
        prv_state->saved_configs.launch_detect_baro_enabled = true;
        }

    _nav_pof_init( prv_state );
    prv_state->pof = NAV_ON_GROUND;
    stub__pof_transitions_logged = 0;

    for ( int sample_num = 0; sample_num < NUM_LAUNCH_DETECT_SAMPLES; sample_num++ )
        {
        sensors.accel.x = inputs_acc[test_num][sample_num];
        sensors.accel.y = 0.0f;
        sensors.accel.z = 0.0f;
        sensors.baro_pres = inputs_baro[test_num][sample_num];

        // detected = launch_detection( &sample_launch_detect_time );
        status = _nav_pof_update( prv_state, &sensors, &outputs );
        
        TEST_ASSERT_EQ_SINT( "Test that the transition callback was called the right number of times.", stub__pof_transitions_logged, expected[test_num][sample_num] );
        TEST_ASSERT_EQ_UINT( "Test that the operation returned successfully", status, NAV_OK );

        if( expected[test_num][sample_num] )
            {
            TEST_ASSERT_EQ_UINT( "Test that the new state is powered ascent.", prv_state->pof, NAV_POWERED_ASCENT );
            }

        }

    TEST_end_nested_case();
    }

} /* test_launch_detection */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_apogee_detection                                                  *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test the apogee detection function in APPA.                            *
*                                                                              *
*******************************************************************************/
void test_apogee_detection
    (
    void
    )
{
/*------------------------------------------------------------------------------
Local Typedefs
------------------------------------------------------------------------------*/
#define MAX_SERIES_LEN 8
typedef struct
    {
    const char *test_name;
    float series[MAX_SERIES_LEN];
    int series_len;
    unsigned int window;
    bool expect_detected;
    } apogee_test_case;
apogee_test_case cases[] =
    {
    { "Simple Apogee (window=3)", { 100.0f, 110.0f, 105.0f, 104.0f, 103.0f, 102.0f }, 6, 3, true },
    { "No Apogee (window=3)", { 100.0f, 110.0f, 105.0f, 106.0f, 107.0f, 108.0f }, 6, 3, false },
    { "Interrupted Decrease (window=2)", { 120.0f, 115.0f, 114.0f, 115.0f, 113.0f, 112.0f }, 6, 2, true },
    { "No Decrease (window=2)", { 100.0f, 100.0f, 100.0f, 100.0f }, 4, 2, false },
    { "Single Drop (window=1)", { 300.0f, 290.0f }, 2, 1, true },
    { "First Nonzero Sample (window=2)", { 150.0f, 149.0f, 148.0f }, 3, 2, true },
    { "Oscillating (window=2)", { 100.0f, 99.0f, 100.0f, 99.5f, 101.0f, 100.5f }, 6, 2, false },
    { "Infinite (window=2)", { INFINITY, INFINITY, INFINITY, INFINITY, INFINITY, INFINITY }, 6, 2, false }
    };

for ( unsigned int case_num = 0; case_num < sizeof( cases ) / sizeof( cases[0] ); case_num++ )
    {
    NAV_SENSORS sensors = { 0 };
    NAV_OUTPUTS outputs = { 0 };
    sensors.accel.x = INFINITY; // invalidate accel to make sure coast detect isn't tripped
    NAV_STATUS status = NAV_OK;
    nav_get_default_config( &(prv_state->saved_configs) );
    _nav_pof_init( prv_state );
    prv_state->pof = case_num % 2 ? NAV_POWERED_ASCENT : NAV_UNPOWERED_ASCENT;
    prv_state->saved_configs.apogee_decreasing_counts = cases[case_num].window;
    stub__pof_transitions_logged = 0;
    for ( int sample_num = 0; sample_num < cases[case_num].series_len; sample_num++ )
        {
        outputs.ind_alt_agl = cases[case_num].series[sample_num];
        outputs.ind_alt_asl = cases[case_num].series[sample_num] + 1000;
        status = _nav_pof_update( prv_state, &sensors, &outputs );
        }

    TEST_ASSERT_EQ_UINT( cases[case_num].test_name, stub__pof_transitions_logged, cases[case_num].expect_detected );
    TEST_ASSERT_EQ_UINT( "Test that the operation returned successfully", status, NAV_OK );

    if( cases[case_num].expect_detected )
        {
        TEST_ASSERT_EQ_UINT( "Test that the new state is descent.", prv_state->pof, NAV_DESCENT );
        }
    }
} /* test_apogee_detection */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_coast_detect                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test coast detection using acceleration input vectors.                 *
*                                                                              *
*******************************************************************************/
void test_coast_detect
    (
    void
    )
{
/*------------------------------------------------------------------------------
Local Typedefs
------------------------------------------------------------------------------*/
#define MAX_COAST_SERIES_LEN 8
typedef struct
    {
    const char *test_name;
    float series[MAX_COAST_SERIES_LEN];
    int series_len;
    bool expect_detected;
    } coast_test_case;
float threshold = 0.75f * GRAVITY;
coast_test_case cases[] =
    {
    { "Detects coast after five low readings", { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }, 5, true },
    { "Does not detect coast before five low readings", { 0.0f, 0.0f, 0.0f, 0.0f }, 4, false },
    { "Threshold reading resets the low reading count", { 0.0f, 0.0f, threshold, 0.0f, 0.0f, 0.0f, 0.0f }, 7, false },
    { "Threshold readings do not count as low acceleration", { threshold, threshold, threshold, threshold, threshold }, 5, false },
    { "Detects coast after a reset and five low readings", { threshold, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }, 6, true }
    };

for ( unsigned int case_num = 0; case_num < sizeof( cases ) / sizeof( cases[0] ); case_num++ )
    {
    NAV_SENSORS sensors = { 0 };
    NAV_OUTPUTS outputs = { 0 };
    NAV_STATUS status = NAV_OK;
    nav_get_default_config( &(prv_state->saved_configs) );
    _nav_pof_init( prv_state );
    prv_state->pof = NAV_POWERED_ASCENT;
    prv_state->saved_configs.coast_detect_counts = 5;
    stub__pof_transitions_logged = 0;

    for ( int sample_num = 0; sample_num < cases[case_num].series_len; sample_num++ )
        {
        sensors.accel.x = cases[case_num].series[sample_num];
        status = _nav_pof_update( prv_state, &sensors, &outputs );
        }

    TEST_ASSERT_EQ_UINT( cases[case_num].test_name, stub__pof_transitions_logged, cases[case_num].expect_detected );
    TEST_ASSERT_EQ_UINT( "Test that the operation returned successfully", status, NAV_OK );

    if( cases[case_num].expect_detected )
        {
        TEST_ASSERT_EQ_UINT( "Test that the new state is unpowered ascent.", prv_state->pof, NAV_UNPOWERED_ASCENT );
        }
    }

} /* test_coast_detect */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_pof_init                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test phase-of-flight initialization.                                   *
*                                                                              *
*******************************************************************************/
void test_pof_init
    (
    void
    )
{
/*------------------------------------------------------------------------------
Case 1: Nominal Init
------------------------------------------------------------------------------*/
NAV_CONFIG config;
NAV_STATUS status = NAV_OK;
nav_get_default_config( &config );
prv_state->saved_configs = config;

status = _nav_pof_init( prv_state );

TEST_ASSERT_EQ_UINT( "Verify that initialization succeeded", status, NAV_OK );

/*------------------------------------------------------------------------------
Case 2: LD not configured
------------------------------------------------------------------------------*/
nav_get_default_config( &config );
config.launch_detect_accel_enabled = false;
config.launch_detect_baro_enabled = false;
prv_state->saved_configs = config;

status = _nav_pof_init( prv_state );

TEST_ASSERT_EQ_UINT( "Verify that initialization fails if launch detect isnt configured", status, NAV_INVALID_CONFIG );

/*------------------------------------------------------------------------------
Case 3: LD misconfigured (accel)
------------------------------------------------------------------------------*/
nav_get_default_config( &config );
config.launch_detect_accel_counts = 0;
prv_state->saved_configs = config;

status = _nav_pof_init( prv_state );

TEST_ASSERT_EQ_UINT( "Verify that initialization fails if launch detect accel is misconfigured", status, NAV_INVALID_CONFIG );

/*------------------------------------------------------------------------------
Case 4: LD misconfigured (baro)
------------------------------------------------------------------------------*/
nav_get_default_config( &config );
config.launch_detect_baro_enabled = true;
config.launch_detect_baro_counts = 0;
prv_state->saved_configs = config;

status = _nav_pof_init( prv_state );

TEST_ASSERT_EQ_UINT( "Verify that initialization fails if launch detect baro is misconfigured", status, NAV_INVALID_CONFIG );

/*------------------------------------------------------------------------------
Case 5: Coast misconfigured
------------------------------------------------------------------------------*/
nav_get_default_config( &config );
config.coast_detect_counts = 0;
prv_state->saved_configs = config;

status = _nav_pof_init( prv_state );

TEST_ASSERT_EQ_UINT( "Verify that initialization fails if coast detect is misconfigured", status, NAV_INVALID_CONFIG );

/*------------------------------------------------------------------------------
Case 6: Apogee misconfigured
------------------------------------------------------------------------------*/
nav_get_default_config( &config );
config.apogee_decreasing_counts = 0;
prv_state->saved_configs = config;

status = _nav_pof_init( prv_state );

TEST_ASSERT_EQ_UINT( "Verify that initialization fails if apogee detect is misconfigured", status, NAV_INVALID_CONFIG );

} /* test_pof_init */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       main                                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Set up the test environment, call tests, and tear down the test        *
*       environment.                                                           *
*                                                                              *
*******************************************************************************/
int main
    (
    void
    )
{
unit_test tests[] =
    {
    { "POF: launch_detection", test_launch_detection, "RQ.MOD.00060, RQ.MOD.00061" },
    { "POF: apogee_detect", test_apogee_detection, "RQ.MOD.00060, RQ.MOD.00063" },
    { "POF: coast_detect", test_coast_detect, "RQ.MOD.00060, RQ.MOD.00062" },
    { "POF: initialization", test_pof_init, "RQ.MOD.00060" }
    };

TEST_INITIALIZE_TEST( "nav_pof.c", tests );

} /* main */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/