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
#include "pool_allocator.h"

/*------------------------------------------------------------------------------
 Global Variables  
------------------------------------------------------------------------------*/
extern SCHEDULED_TASK task_scheduler_table[];
extern uint8_t task_scheduler_table_size;


/*------------------------------------------------------------------------------
 API Functions
------------------------------------------------------------------------------*/

TASK_STATUS schedule_task
    (
    TASK_TYPE task,
    uint32_t scheduled_systick
    )
{

/* CRITICAL SECTION BEGIN ?*/

for ( uint8_t i = 0; i < task_scheduler_table_size; i++ )
    {
    SCHEDULED_TASK* tabled_task = &task_scheduler_table[i];
    if ( tabled_task->task == task )
        {
            if ( tabled_task->scheduled_systick != 0 )
            {
                return TASK_SCHEDULER_OCCUPIED;
            }
            
            tabled_task->scheduled_systick = scheduled_systick;
            return TASK_SCHEDULER_OK;
        }
    }

/* CRITICAL SECTION END */

return TASK_SCHEDULER_NOT_FOUND;
}


TASK_STATUS task_check_and_execute
    (
    void
    )
{
uint32_t systick_now = HAL_GetTick();
uint16_t status = TASK_SCHEDULER_OK;

for ( uint8_t i = 0; i < task_scheduler_table_size; i++ )
    {
    SCHEDULED_TASK* tabled_task = &task_scheduler_table[i];
    if ( tabled_task->scheduled_systick <= systick_now && tabled_task->scheduled_systick != 0 )
        {
        /* At or passed scheduled tick, so execute callback */
        uint16_t callback_return = tabled_task->callback();

        /* Reset systick*/
        tabled_task->scheduled_systick = 0;

        if ( callback_return )
            {
            status = TASK_SCHEDULER_CALLBACK_ERROR;
            }
        }
    }

return status;
}


void task_scheduler_IT_handler
    (
    void
    )
{
TASK_STATUS status = task_check_and_execute();
if ( status == TASK_SCHEDULER_CALLBACK_ERROR )
    {
    error_fail_fast( ERROR_UNKNOWN_FATAL_ERROR ); // TODO
    }
}



/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/