/**
  ******************************************************************************
  * @file           : scheduler.h
  * @brief          : Asynchronous task scheduler
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
    - Schedule `task_callback` functions to be run at a specified systick
  ******************************************************************************
  @endverbatim
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SCHEDULER_H 
#define SCHEDULER_H 

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Types ---------------------------------------------------------------------*/
typedef enum
    {
    SCHEDULER_OK,
    SCHEDULER_FAIL,
    SCHEDULER_INVALID_SYSTICK
    } SCHEDULER_STATUS;

typedef void (*task_callback)(void); /* NA TODO: error handling */

typedef struct TASK_LIST TASK_LIST;
struct TASK_LIST    
    {
    TASK_LIST* next;
    uint32_t scheduled_systick;
    task_callback task;
    };

/* Macros --------------------------------------------------------------------*/

/* Size defined as a macro by the project */
#define SCHEDULER_POOL_SIZE ( SCHEDULER_POOL_MAX_TASKS * sizeof(POOL_CHUNK) )

/* Exported functions prototypes ---------------------------------------------*/

void scheduler_init
    (
    void
    );

SCHEDULER_STATUS schedule_task
    (
    task_callback task,
    uint32_t scheduled_systick
    );


SCHEDULER_STATUS task_check_and_execute
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
