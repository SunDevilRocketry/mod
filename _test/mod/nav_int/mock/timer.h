/**
 * @file timer.h
 * 
 * @brief Timer delays and tick
 * 
 * @copyright
 *       Copyright (c) 2025 Sun Devil Rocketry.                                 
 *       All rights reserved.                                                   
 *                                                                              
 *       This software is licensed under terms that can be found in the LICENSE 
 *       file in the root directory of this software component.                 
 *       If no LICENSE file comes with this software, it is covered under the   
 *       BSD-3-Clause.                                                          
 *                                                                              
 *       https://opensource.org/license/bsd-3-clause        
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TIMER_H 
#define TIMER_H 

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

#define MICROSEC_PER_SEC	( 1000000UL )
#define MILLISEC_PER_SEC	( 1000UL )
#define SEC_PER_MIN         ( 60UL )
#define MIN_PER_HOUR        ( 60UL )


/*------------------------------------------------------------------------------
 Types 
------------------------------------------------------------------------------*/
typedef uint16_t time_hours; /* we can't exceed this limit without rolling over the systick */
typedef uint8_t time_mins;   /* 0-59     */
typedef uint8_t time_secs;   /* 0-59     */
typedef uint16_t time_millis;/* 0-999    */

typedef struct _SYSTEM_TIME {
    time_hours      hours;
    time_mins       mins;
    time_secs       secs;  
    time_millis     millis;
} SYSTEM_TIME;

/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

uint64_t get_us_tick
    (
    void
    );

void delay_ms
    (
    uint32_t delay
    );

void delay_us
    (
    uint32_t delay
    );

void micro_tim_IT_handler
    (
    void
    );

SYSTEM_TIME get_system_time
    (
    void
    );

#ifdef __cplusplus
}
#endif
#endif /* TIMER_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
