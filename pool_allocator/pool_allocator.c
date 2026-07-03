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
Pool pool_init
    (
    uint8_t* pool_memory,
    size_t size
    )
{
Pool pool;
size_t block_count = size / sizeof(Chunk);
pool.free_chunk = (Chunk*)pool_memory;
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
    Pool* pool
    )
{
if ( pool == NULL || pool->free_chunk == NULL)
    {
    return NULL;
    }

Chunk* next_free = pool->free_chunk;
pool->free_chunk = pool->free_chunk->next;

return next_free;

}


void pool_free
    (
    Pool* pool, 
    void* ptr
    )
{
/* ptr can be NULL, but I want to check for now */
debug_assert(pool != NULL && ptr != NULL, ERROR_UNKNOWN_FATAL_ERROR);
Chunk* chunk = ptr;
chunk->next = pool->free_chunk;
pool->free_chunk = chunk;

}


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/