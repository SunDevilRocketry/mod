/**
  ******************************************************************************
  * @file           : pool_allocator.c
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
#include "pool_allocator.h"
#include "error_sdr.h"
#include "debug_sdr.h"


/*------------------------------------------------------------------------------
 Global Variables  
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 API Functions
------------------------------------------------------------------------------*/
POOL pool_init
    (
    uint8_t* pool_memory,
    size_t size
    )
{
POOL pool;
size_t block_count = size / sizeof(POOL_CHUNK);
pool.free_chunk = (POOL_CHUNK*)pool_memory;
pool.chunk_arr = pool.free_chunk;

for ( size_t i = 0; i < block_count - 1; i++ )
    {
    pool.chunk_arr[i].next = &pool.chunk_arr[i + 1];
    }
pool.chunk_arr[block_count - 1].next = NULL;

return pool;
}


void* pool_alloc
    (
    POOL* pool
    )
{
if ( pool == NULL || pool->free_chunk == NULL)
    {
    return NULL;
    }

POOL_CHUNK* next_free = pool->free_chunk;
pool->free_chunk = pool->free_chunk->next;

return next_free;

}


void pool_free
    (
    POOL* pool, 
    void* ptr
    )
{
/* ptr can be NULL, but I want to check for now */
debug_assert(pool != NULL && ptr != NULL, ERROR_NULL_PTR_ERROR);
POOL_CHUNK* chunk = ptr;
chunk->next = pool->free_chunk;
pool->free_chunk = chunk;

}


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/