/**
  ******************************************************************************
  * @file           : scheduler.c
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
  */

/*------------------------------------------------------------------------------
 Standard Includes
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Project Includes
------------------------------------------------------------------------------*/
#include "main.h"
#include "scheduler.h"
#include "error_sdr.h"
#include "debug_sdr.h"
#include "pool_allocator.h"

/*------------------------------------------------------------------------------
 Global Variables  
------------------------------------------------------------------------------*/
/* Project defined scheduler pool */
extern Pool scheduler_pool;
TASK_LIST task_list_head = { 0 };


/*------------------------------------------------------------------------------
 API Functions
------------------------------------------------------------------------------*/

SCHEDULER_STATUS schedule_task
    (
    task_callback task,
    uint32_t scheduled_systick
    )
{
if ( scheduled_systick < HAL_GetTick() )
    {
    return SCHEDULER_INVALID_SYSTICK;
    }

TASK_LIST* new_task = pool_alloc( &scheduler_pool );
if ( new_task == NULL )
    {
    return SCHEDULER_FAIL;
    }
new_task->task = task;
new_task->scheduled_systick = scheduled_systick;
new_task->next = NULL;

TASK_LIST* previous = &task_list_head;

// potential race condition

/* Traverse to end of linked list */
while ( previous->next != NULL )
    {
    previous = previous->next;
    }

previous->next = new_task;

if ( !previous->next )
    {
    return SCHEDULER_FAIL;
    }


return SCHEDULER_OK;
}


SCHEDULER_STATUS task_check_and_execute
    (
    void
    )
{
uint32_t systick_now = HAL_GetTick();
uint16_t status = SCHEDULER_OK;

TASK_LIST* previous = &task_list_head;
TASK_LIST* current = task_list_head.next;

while ( current != NULL )
    {
    if ( current->scheduled_systick <= systick_now )
        {
        current->task();
        
        previous->next = current->next;
        TASK_LIST* next = current->next;
        
        pool_free(&scheduler_pool, current);
        current = next;
        }
    else
        {
        previous = current;
        current = current->next;
        }
    }


return status;
}


void task_scheduler_IT_handler
    (
    void
    )
{
SCHEDULER_STATUS status = task_check_and_execute();
if ( status == SCHEDULER_CALLBACK_ERROR )
    {
    error_fail_fast( ERROR_UNKNOWN_FATAL_ERROR ); // TODO
    }
}



/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/