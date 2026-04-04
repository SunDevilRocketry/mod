/*******************************************************************************
*
* FILE: 
* 		error_sdr.h
*
* DESCRIPTION: 
* 		Contains error handling functions for SDR code.
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ERROR_SDR_H 
#define ERROR_SDR_H 

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Includes 
------------------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>

/*------------------------------------------------------------------------------
 Constants 
------------------------------------------------------------------------------*/
#define TEXT_MESSAGE_LENGTH 72
#define Error_Handler( a ) error_fail_fast( a ) /* backwards compatible */

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Error codes */
typedef enum _ERROR_CODE 
    {
    ERROR_UNKNOWN_FATAL_ERROR          , /* An unknown error occurred         */
    ERROR_FLASH_INIT_ERROR             , /* Flash initialization error        */
    ERROR_BARO_INIT_ERROR              , /* Baro initialization error         */
    ERROR_IMU_INIT_ERROR               , /* IMU initialization error          */
    ERROR_DATA_LOG_LOAD_HEADER_ERROR   , /* Error loading flash header        */
    ERROR_DATA_LOG_CORRECT_HEADER_ERROR, /* Error correcting flash header     */
    ERROR_DATA_LOG_PREP_MEM_ERROR      , /* Error eraseing flight memory      */
    ERROR_CAL_GROUND_ALT_ERROR         , /* Error calibratiing the ground alt */
    ERROR_TERMINAL_ERROR               , /* Error from terminal command       */
    ERROR_FSM_STATE_ERROR              , /* State transition error            */
    ERROR_DUAL_DEPLOY_EXTRACT_ERROR    , /* Dual deploy extract data failed   */
    ERROR_RECORD_FLIGHT_EVENTS_ERROR   , /* Error recording flight evetns     */
    ERROR_SYSCLOCK_CONFIG_ERROR        , /* Error configuring the sysclock    */
    ERROR_COMMON_CLOCK_CONFIG_ERROR    , /* Error configuring shared clocks   */
    ERROR_IMU_I2C_INIT_ERROR           , /* Error initializing IMU I2C bus    */
    ERROR_BARO_I2C_INIT_ERROR          , /* Error initializing the baro I2C   */
    ERROR_FLASH_SPI_INIT_ERROR         , /* Error initializing the flash SPI  */
    ERROR_USB_UART_INIT_ERROR          , /* Error initializing the USB UART   */
    ERROR_BUZZER_TIM_INIT_ERROR        , /* Error initializing the buzzer TIM */
    ERROR_MICRO_TIM_INIT_ERROR         , /* Error initializing the micro TIM  */
    ERROR_I2C_HAL_MSP_ERROR            , /* Error in I2C HAL MSP              */
    ERROR_SPI_HAL_MSP_ERROR            , /* Error in SPI HAL MSP              */
    ERROR_UART_HAL_MSP_ERROR           , /* Error in UART HAL MSP             */
    ERROR_RF_ERROR                     , /* RF tx/rx error                    */
    ERROR_RF_UNRECOGNIZED_PING         , /* RF ping byte incorrect            */
    ERROR_USB_UART_ERROR               , /* General UART USB I/O error        */
    ERROR_XBEE_UART_CONFIG_ERROR       , /* Error initializing XBee UART      */
    ERROR_RF_UART_INIT_ERROR           , /* Error initializing wireless UART  */
    ERROR_UNSUPPORTED_OP_ERROR         , /* Unimplimented feature error       */
    ERROR_THERMO_INIT_ERROR            , /* Error initializaing thermocouple  */
    ERROR_PT_ADC_INIT_ERROR            , /* Error initializing PT ADC         */
    ERROR_LC_ADC_INIT_ERROR            , /* Error initializing LC ADC         */
    ERROR_THERMO_I2C_INIT_ERROR        , /* Error initializing TC I2C         */
    ERROR_VALVE_UART_INIT_ERROR        , /* Error initializing Valve UART     */
    ERROR_IGN_CMD_ERROR                , /* Error executing ignition command  */
    ERROR_FLASH_CMD_ERROR              , /* Error executing flash command     */
    ERROR_SENSOR_CMD_ERROR             , /* Error executing sensor command    */
    ERROR_SERVO_CMD_ERROR              , /* Error executing servo command     */
    ERROR_VALVE_CMD_ERROR              , /* Error executing valve command     */
    ERROR_BARO_CAL_ERROR               , /* Error calibrating ground pressure */
    ERROR_DATA_HAZARD_ERROR            , /* Potential data corruption error   */
    ERROR_FSM_INVALID_STATE_ERROR      , /* Invalid state variable value      */
    ERROR_VC_OFFLINE_ERROR             , /* Can't connect to valve controller */
    ERROR_VC_INIT_ERROR                , /* Error initializing main valves    */
    ERROR_PING_TIMEOUT_ERROR           , /* Ping Timeout                      */
    ERROR_RS485_UART_INIT_ERROR        , /* Error initializing RS485 UART     */
    ERROR_RS485_UART_ERROR             , /* Error during RS485 transmission   */
    ERROR_SOL_CMD_ERROR                , /* Error passing on solenoid command */
    ERROR_PT_ADC_CHANNEL_ERROR         , /* Error switching adc channel       */
    ERROR_FSM_INVALID_STATE_TRANSITION_ERROR,  /* Error changing state        */
    ERROR_GPS_UART4_INITIALIZATION     , /* Error initializing UART for GPS   */
    ERROR_GPS_UART4_DEMSP              , /* Error initializing DEMSP UART for GPS */
    ERROR_PWM4_ERROR                   , /* Error with PWM timer 4            */
    ERROR_PWM123_ERROR                 , /* Error with PWM timer 1,2,3        */
    ERROR_SERVO_INIT_ERROR             , /* Error initializing servos         */
    ERROR_SENSOR_FORMAT_ERROR          , /* unused                            */
    ERROR_INVALID_STATE_ERROR          , /* FSM has reached an invalid state  */
    ERROR_CONFIG_VALIDITY_ERROR        , /* Loaded configuration is invalid   */
    ERROR_IGNITION_CONTINUITY_ERROR    , /* No continuity on IGN terminals    */
    ERROR_LORA_SPI_INIT_ERROR          , /* Error with LoRa SPI init          */
    ERROR_LORA_INIT_ERROR              , /* Error with LoRa init              */
    ERROR_LORA_CMD_ERROR               , /* Error with LoRa command           */
    } ERROR_CODE;

/* Error callback table entry */
typedef struct _ERROR_CALLBACK {
    volatile ERROR_CODE error_code;
    void (*error_callback)(ERROR_CODE);
} ERROR_CALLBACK;

typedef struct TEXT_MESSAGE
    {
    uint32_t systick;
    char message[TEXT_MESSAGE_LENGTH];
    } TEXT_MESSAGE;

/*------------------------------------------------------------------------------
 Macros & Inlines
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* MACRO:                                                                       * 
*       assert_fail_fast                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Checks condition, if false calls error_fail_fast with error            *
*                                                                              *
*******************************************************************************/
#define assert_fail_fast( condition, error ) if ( !condition ) error_fail_fast( error )


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* error handler */
void error_fail_fast
    (
    volatile ERROR_CODE error_code
    );

/* warnings and info for telemetry */
void error_log_info
    (
    char* message
    );

void error_log_warning
    (
    char* message
    );

bool error_get_warning
    (
    TEXT_MESSAGE* buffer
    );

bool error_get_info
    (
    TEXT_MESSAGE* buffer
    );
    
bool error_is_pending_warning
    (
    void
    );

bool error_is_pending_info
    (
    void
    );

#ifdef __cplusplus
}
#endif
#endif /* ERROR_SDR_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/