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
#define CHUNK_SIZE 12 /* Defined as the size of a scheduler task for now */

/*------------------------------------------------------------------------------
 Typedefs 
------------------------------------------------------------------------------*/

typedef union POOL_CHUNK POOL_CHUNK;
union POOL_CHUNK
    {
    POOL_CHUNK* next;
    uint8_t arr[CHUNK_SIZE];
    };

typedef struct
    {
    POOL_CHUNK* free_chunk;
    POOL_CHUNK* chunk_arr;
    } POOL;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/
POOL pool_init
    (
    uint8_t* pool_memory,
    size_t size
    );

void* pool_alloc
    (
    POOL* pool
    );

void pool_free
    (
    POOL* pool, 
    void* ptr
    );

#ifdef __cplusplus
}
#endif
#endif /* POOL_ALLOCATOR_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/