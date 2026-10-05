/*******************************************************************************
*
* FILE:
*      test_isa.c
*
* DESCRIPTION:
*      Unit tests for error handling in the ISA module.
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
#include "isa.h"
#include "math_sdr.h"

/*------------------------------------------------------------------------------
Procedures
------------------------------------------------------------------------------*/

/**
 * Test ISA model for pressure altitude.
 */
void test_isa_altitudes
	(
	void
	)
{
struct test_case {
    const char* description;
    float pressure;
    float temperature;
    float altitude;
};
struct test_case cases[] =
    {
    /*  description                                                         pressure    temp    alt         */
    {   "Test sea-level pressure altitude.",                                101325.0f,  15.0f,  0.0f        },
    {   "Test ISA table entry @ 20,000 ft.",                                46563.2f,   -24.6f, 6096.0f     },
    {   "Test ISA table entry @ 40,000 ft.",                                18754.0f,   -56.5f, 12192.0f    },
    {   "Continuity: Test ISA table entry immediately below tropopause.",   22632.1f,   -56.5f, 11000.0f    },
    {   "Continuity: Test ISA table entry immediately above tropopause.",   22632.0f,   -56.5f, 11000.0f    },
    {   "Robustness: Test negative pressure.",                              -1.0f,      15.0f,  INFINITY    }
    };

for( int i = 0; i < array_size( cases ); i++ )
    {
    /* Assert with a small tolerance on the values (0.05%) */
    float altitude = isa_get_altitude_qnh( cases[i].pressure, cases[i].temperature );
    if( cases[i].altitude != INFINITY )
        {
        TEST_ASSERT_GE_FLOAT( cases[i].description, 
                            altitude, 
                            cases[i].altitude - (cases[i].altitude * 0.0005) );
        TEST_ASSERT_LE_FLOAT( cases[i].description, 
                            altitude, 
                            cases[i].altitude + (cases[i].altitude * 0.0005) );
        }
    else
        {
        /* Expected value is infinity, so no tolerance needed */
        TEST_ASSERT_EQ_FLOAT( cases[i].description, 
                            altitude, 
                            cases[i].altitude );
        }
    }

} /* test_isa_altitudes */


/**
 * Test QFE reference elevation calculator.
 */
void test_isa_qfe
    (
    void
    )
{
float baro_pressure = 91000.0f;
float baro_temp = 8.0f;
float baro_alt = isa_get_altitude_qnh( baro_pressure, baro_temp );
float offset_pressure = 101300.0f;
float offset_temp = 19.0f;
float offset_alt = isa_get_altitude_qnh( offset_pressure, offset_temp );

float qfe = isa_get_altitude_qfe( baro_pressure, baro_temp, offset_pressure, offset_temp );

TEST_ASSERT_EQ_FLOAT( "QFE altitude is the difference between current and calibrated alt.", qfe, baro_alt - offset_alt );
qfe = isa_get_altitude_qfe( offset_pressure, offset_temp, baro_pressure, baro_temp );
TEST_ASSERT_EQ_FLOAT( "QFE altitude can be negative.", qfe, offset_alt - baro_alt );

} /* test_isa_qfe */


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
	{ "ISA: QNH", test_isa_altitudes, "RQ.MOD.00031" },
    { "ISA: QFE", test_isa_qfe, "RQ.MOD.00051" }
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "isa", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
