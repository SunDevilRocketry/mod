/**
  ******************************************************************************
  * @file           : pool_allocator.h
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
                      ##### Pool allocator features #####
  ==============================================================================
    TODO
  ******************************************************************************
  @endverbatim
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef POOL_ALLOCATOR_H 
#define POOL_ALLOCATOR_H 

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Includes 
------------------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>


/*------------------------------------------------------------------------------
 Macros
------------------------------------------------------------------------------*/
#define CHUNK_SIZE 16 // TODO variable chunk size

/*------------------------------------------------------------------------------
 Typedefs 
------------------------------------------------------------------------------*/

typedef union Chunk Chunk;
union Chunk
    {
    Chunk* next;
    uint8_t arr[CHUNK_SIZE];
    };

typedef struct
    {
    Chunk* free_chunk;
    Chunk* chunk_arr;
    } Pool;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/
Pool pool_init
    (
    uint8_t* pool_memory,
    size_t size
    );

void* pool_alloc
    (
    Pool* pool
    );

void pool_free
    (
    Pool* pool, 
    void* ptr
    );

#ifdef __cplusplus
}
#endif
#endif /* POOL_ALLOCATOR_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/