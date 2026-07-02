/**
  ******************************************************************************
  * @file           : scheduler.h
  * @brief          : TODO
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
                      ##### Task scheduler features #####
  ==============================================================================
    TODO
  ******************************************************************************
  @endverbatim
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SCHEDULER_H 
#define SCHEDULER_H 

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

typedef enum 
    {
    TASK_START_BUZZ,
    TASK_STOP_BUZZ
    } TASK_TYPE;

typedef enum
    {
    TASK_SCHEDULER_OK,
    TASK_SCHEDULER_PASS,
    TASK_SCHEDULER_FAIL,
    TASK_SCHEDULER_INVALID_SYSTICK,
    TASK_SCHEDULER_NOT_FOUND,
    TASK_SCHEDULER_OCCUPIED,
    TASK_SCHEDULER_CALLBACK_ERROR
    } TASK_STATUS;

typedef uint16_t (*task_callback)(void); /* NA TODO: error handling (any negative returns?)*/

typedef struct 
    {
    const TASK_TYPE task;
    uint32_t scheduled_systick;
    const task_callback callback;
    } SCHEDULED_TASK;

typedef struct 
    {
    SCHEDULED_TASK task;
    struct TASK_LLIST* next;
    } TASK_LLIST;

/*------------------------------------------------------------------------------
 Macros
------------------------------------------------------------------------------*/



/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

TASK_STATUS schedule_task
    (
    TASK_TYPE task,
    uint32_t scheduled_systick
    );


TASK_STATUS task_check_and_execute
    (
    void
    );

void task_scheduler_IT_handler
    (
    void
    );
    
#ifdef __cplusplus
}
#endif
#endif /* SCHEDULER_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/