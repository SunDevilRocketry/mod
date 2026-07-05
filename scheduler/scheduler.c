/**
  ******************************************************************************
  * @file           : scheduler.c
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
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "scheduler.h"
#include "error_sdr.h"
#include "debug_sdr.h"
#include "pool_allocator.h"

/* Static Variables ----------------------------------------------------------*/

/* Memory to be used by scheduler*/
static uint8_t scheduler_pool_mem[SCHEDULER_POOL_SIZE];
static POOL scheduler_pool;

/* Dummy head for task list */
static TASK_LIST task_list_head = { 0 };


/* Procedures ----------------------------------------------------------------*/

/**
 * @brief Initializes the scheduler's memory pool
 */
void scheduler_init
    (
    void
    )
{
scheduler_pool = pool_init(scheduler_pool_mem, SCHEDULER_POOL_SIZE);
}


/**
 * @brief Schedules a task.
 * 
 * @param task The task to perform.
 * @param scheduled_systick The systick at which the task will be run.
 * 
 * @return The status of the scheduler. 
 */
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

TASK_LIST* new_task = pool_alloc(&scheduler_pool);
if ( new_task == NULL )
    {
    return SCHEDULER_FAIL;
    }
new_task->task = task;
new_task->scheduled_systick = scheduled_systick;

/* Insert new task into the front of the list */
// potential race condition
if ( task_list_head.next != NULL )
    {
    new_task->next = task_list_head.next;
    }
else
    {
    new_task->next = NULL;
    }
task_list_head.next = new_task;


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
// if ( status == SCHEDULER_CALLBACK_ERROR )
//     {
//     error_fail_fast( ERROR_UNKNOWN_FATAL_ERROR ); // TODO
//     }
}
