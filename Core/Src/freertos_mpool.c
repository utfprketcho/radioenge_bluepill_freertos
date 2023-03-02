/* --------------------------------------------------------------------------
 * Copyright (c) 2013-2020 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *      Name:    cmsis_os2.c
 *      Purpose: CMSIS RTOS2 wrapper for FreeRTOS
 *
 *---------------------------------------------------------------------------*/

#include <string.h>

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "cmsis_compiler.h"             // Compiler agnostic definitions

#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
#include "task.h"                       // ARM.FreeRTOS::RTOS:Core
#include "event_groups.h"               // ARM.FreeRTOS::RTOS:Event Groups
#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core

#include "freertos_mpool.h"             // osMemoryPool definitions

#ifndef uxSemaphoreGetCountFromISR
#define uxSemaphoreGetCountFromISR( xSemaphore ) uxQueueMessagesWaitingFromISR( ( QueueHandle_t ) ( xSemaphore ) )
#endif

/*---------------------------------------------------------------------------*/
#if   ((__ARM_ARCH_7M__      == 1U) || \
       (__ARM_ARCH_7EM__     == 1U) || \
       (__ARM_ARCH_8M_MAIN__ == 1U))
#define IS_IRQ_MASKED()           ((__get_PRIMASK() != 0U) || (__get_BASEPRI() != 0U))
#elif  (__ARM_ARCH_6M__      == 1U)
#define IS_IRQ_MASKED()           (__get_PRIMASK() != 0U)
#elif (__ARM_ARCH_7A__       == 1U)
/* CPSR mask bits */
#define CPSR_MASKBIT_I            0x80U

#define IS_IRQ_MASKED()           ((__get_CPSR() & CPSR_MASKBIT_I) != 0U)
#else
#define IS_IRQ_MASKED()           (__get_PRIMASK() != 0U)
#endif

#if    (__ARM_ARCH_7A__      == 1U)
/* CPSR mode bitmasks */
#define CPSR_MODE_USER            0x10U
#define CPSR_MODE_SYSTEM          0x1FU

#define IS_IRQ_MODE()             ((__get_mode() != CPSR_MODE_USER) && (__get_mode() != CPSR_MODE_SYSTEM))
#else
#define IS_IRQ_MODE()             (__get_IPSR() != 0U)
#endif

#define IS_IRQ()                  IS_IRQ_MODE()


#ifdef FREERTOS_MPOOL_H_

/* Static memory pool functions */
static void  FreeBlock   (MemPool_t *mp, void *block);
static void *AllocBlock  (MemPool_t *mp);
static void *CreateBlock (MemPool_t *mp);

osMemoryPoolId_t osMemoryPoolNew (uint32_t block_count, uint32_t block_size, const osMemoryPoolAttr_t *attr) {
  MemPool_t *mp;
  const char *name;
  int32_t mem_cb, mem_mp;
  uint32_t sz;

  if (IS_IRQ()) {
    mp = NULL;
  }
  else if ((block_count == 0U) || (block_size == 0U)) {
    mp = NULL;
  }
  else {
    mp = NULL;
    sz = MEMPOOL_ARR_SIZE (block_count, block_size);

    name = NULL;
    mem_cb = -1;
    mem_mp = -1;

    if (attr != NULL) {
      if (attr->name != NULL) {
        name = attr->name;
      }

      if ((attr->cb_mem != NULL) && (attr->cb_size >= sizeof(MemPool_t))) {
        /* Static control block is provided */
        mem_cb = 1;
      }
      else if ((attr->cb_mem == NULL) && (attr->cb_size == 0U)) {
        /* Allocate control block memory on heap */
        mem_cb = 0;
      }

      if ((attr->mp_mem == NULL) && (attr->mp_size == 0U)) {
        /* Allocate memory array on heap */
          mem_mp = 0;
      }
      else {
        if (attr->mp_mem != NULL) {
          /* Check if array is 4-byte aligned */
          if (((uint32_t)attr->mp_mem & 3U) == 0U) {
            /* Check if array big enough */
            if (attr->mp_size >= sz) {
              /* Static memory pool array is provided */
              mem_mp = 1;
            }
          }
        }
      }
    }
    else {
      /* Attributes not provided, allocate memory on heap */
      mem_cb = 0;
      mem_mp = 0;
    }

    if (mem_cb == 0) {
      mp = pvPortMalloc (sizeof(MemPool_t));
    } else {
      mp = attr->cb_mem;
    }

    if (mp != NULL) {
      /* Create a semaphore (max count == initial count == block_count) */
      #if (configSUPPORT_STATIC_ALLOCATION == 1)
        mp->sem = xSemaphoreCreateCountingStatic (block_count, block_count, &mp->mem_sem);
      #elif (configSUPPORT_DYNAMIC_ALLOCATION == 1)
        mp->sem = xSemaphoreCreateCounting (block_count, block_count);
      #else
        mp->sem == NULL;
      #endif

      if (mp->sem != NULL) {
        /* Setup memory array */
        if (mem_mp == 0) {
          mp->mem_arr = pvPortMalloc (sz);
        } else {
          mp->mem_arr = attr->mp_mem;
        }
      }
    }

    if ((mp != NULL) && (mp->mem_arr != NULL)) {
      /* Memory pool can be created */
      mp->head    = NULL;
      mp->mem_sz  = sz;
      mp->name    = name;
      mp->bl_sz   = block_size;
      mp->bl_cnt  = block_count;
      mp->n       = 0U;

      /* Set heap allocated memory flags */
      mp->status = MPOOL_STATUS;

      if (mem_cb == 0) {
        /* Control block on heap */
        mp->status |= 1U;
      }
      if (mem_mp == 0) {
        /* Memory array on heap */
        mp->status |= 2U;
      }
    }
    else {
      /* Memory pool cannot be created, release allocated resources */
      if ((mem_cb == 0) && (mp != NULL)) {
        /* Free control block memory */
        vPortFree (mp);
      }
      mp = NULL;
    }
  }

  return (mp);
}

const char *osMemoryPoolGetName (osMemoryPoolId_t mp_id) {
  MemPool_t *mp = (osMemoryPoolId_t)mp_id;
  const char *p;

  if (IS_IRQ()) {
    p = NULL;
  }
  else if (mp_id == NULL) {
    p = NULL;
  }
  else {
    p = mp->name;
  }

  return (p);
}

void *osMemoryPoolAlloc (osMemoryPoolId_t mp_id, uint32_t timeout) {
  MemPool_t *mp;
  void *block;
  uint32_t isrm;

  if (mp_id == NULL) {
    /* Invalid input parameters */
    block = NULL;
  }
  else {
    block = NULL;

    mp = (MemPool_t *)mp_id;

    if ((mp->status & MPOOL_STATUS) == MPOOL_STATUS) {
      if (IS_IRQ()) {
        if (timeout == 0U) {
          if (xSemaphoreTakeFromISR (mp->sem, NULL) == pdTRUE) {
            if ((mp->status & MPOOL_STATUS) == MPOOL_STATUS) {
              isrm  = taskENTER_CRITICAL_FROM_ISR();

              /* Get a block from the free-list */
              block = AllocBlock(mp);

              if (block == NULL) {
                /* List of free blocks is empty, 'create' new block */
                block = CreateBlock(mp);
              }

              taskEXIT_CRITICAL_FROM_ISR(isrm);
            }
          }
        }
      }
      else {
        if (xSemaphoreTake (mp->sem, (TickType_t)timeout) == pdTRUE) {
          if ((mp->status & MPOOL_STATUS) == MPOOL_STATUS) {
            taskENTER_CRITICAL();

            /* Get a block from the free-list */
            block = AllocBlock(mp);

            if (block == NULL) {
              /* List of free blocks is empty, 'create' new block */
              block = CreateBlock(mp);
            }

            taskEXIT_CRITICAL();
          }
        }
      }
    }
  }

  return (block);
}

osStatus_t osMemoryPoolFree (osMemoryPoolId_t mp_id, void *block) {
  MemPool_t *mp;
  osStatus_t stat;
  uint32_t isrm;
  BaseType_t yield;

  if ((mp_id == NULL) || (block == NULL)) {
    /* Invalid input parameters */
    stat = osErrorParameter;
  }
  else {
    mp = (MemPool_t *)mp_id;

    if ((mp->status & MPOOL_STATUS) != MPOOL_STATUS) {
      /* Invalid object status */
      stat = osErrorResource;
    }
    else if ((block < (void *)&mp->mem_arr[0]) || (block > (void*)&mp->mem_arr[mp->mem_sz-1])) {
      /* Block pointer outside of memory array area */
      stat = osErrorParameter;
    }
    else {
      stat = osOK;

      if (IS_IRQ()) {
        if (uxSemaphoreGetCountFromISR (mp->sem) == mp->bl_cnt) {
          stat = osErrorResource;
        }
        else {
          isrm = taskENTER_CRITICAL_FROM_ISR();

          /* Add block to the list of free blocks */
          FreeBlock(mp, block);

          taskEXIT_CRITICAL_FROM_ISR(isrm);

          yield = pdFALSE;
          xSemaphoreGiveFromISR (mp->sem, &yield);
          portYIELD_FROM_ISR (yield);
        }
      }
      else {
        if (uxSemaphoreGetCount (mp->sem) == mp->bl_cnt) {
          stat = osErrorResource;
        }
        else {
          taskENTER_CRITICAL();

          /* Add block to the list of free blocks */
          FreeBlock(mp, block);

          taskEXIT_CRITICAL();

          xSemaphoreGive (mp->sem);
        }
      }
    }
  }

  return (stat);
}

uint32_t osMemoryPoolGetCapacity (osMemoryPoolId_t mp_id) {
  MemPool_t *mp;
  uint32_t  n;

  if (mp_id == NULL) {
    /* Invalid input parameters */
    n = 0U;
  }
  else {
    mp = (MemPool_t *)mp_id;

    if ((mp->status & MPOOL_STATUS) != MPOOL_STATUS) {
      /* Invalid object status */
      n = 0U;
    }
    else {
      n = mp->bl_cnt;
    }
  }

  /* Return maximum number of memory blocks */
  return (n);
}

uint32_t osMemoryPoolGetBlockSize (osMemoryPoolId_t mp_id) {
  MemPool_t *mp;
  uint32_t  sz;

  if (mp_id == NULL) {
    /* Invalid input parameters */
    sz = 0U;
  }
  else {
    mp = (MemPool_t *)mp_id;

    if ((mp->status & MPOOL_STATUS) != MPOOL_STATUS) {
      /* Invalid object status */
      sz = 0U;
    }
    else {
      sz = mp->bl_sz;
    }
  }

  /* Return memory block size in bytes */
  return (sz);
}

uint32_t osMemoryPoolGetCount (osMemoryPoolId_t mp_id) {
  MemPool_t *mp;
  uint32_t  n;

  if (mp_id == NULL) {
    /* Invalid input parameters */
    n = 0U;
  }
  else {
    mp = (MemPool_t *)mp_id;

    if ((mp->status & MPOOL_STATUS) != MPOOL_STATUS) {
      /* Invalid object status */
      n = 0U;
    }
    else {
      if (IS_IRQ()) {
        n = uxSemaphoreGetCountFromISR (mp->sem);
      } else {
        n = uxSemaphoreGetCount        (mp->sem);
      }

      n = mp->bl_cnt - n;
    }
  }

  /* Return number of memory blocks used */
  return (n);
}

uint32_t osMemoryPoolGetSpace (osMemoryPoolId_t mp_id) {
  MemPool_t *mp;
  uint32_t  n;

  if (mp_id == NULL) {
    /* Invalid input parameters */
    n = 0U;
  }
  else {
    mp = (MemPool_t *)mp_id;

    if ((mp->status & MPOOL_STATUS) != MPOOL_STATUS) {
      /* Invalid object status */
      n = 0U;
    }
    else {
      if (IS_IRQ()) {
        n = uxSemaphoreGetCountFromISR (mp->sem);
      } else {
        n = uxSemaphoreGetCount        (mp->sem);
      }
    }
  }

  /* Return number of memory blocks available */
  return (n);
}

osStatus_t osMemoryPoolDelete (osMemoryPoolId_t mp_id) {
  MemPool_t *mp;
  osStatus_t stat;

  if (mp_id == NULL) {
    /* Invalid input parameters */
    stat = osErrorParameter;
  }
  else if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else {
    mp = (MemPool_t *)mp_id;

    taskENTER_CRITICAL();

    /* Invalidate control block status */
    mp->status  = mp->status & 3U;

    /* Wake-up tasks waiting for pool semaphore */
    while (xSemaphoreGive (mp->sem) == pdTRUE);

    mp->head    = NULL;
    mp->bl_sz   = 0U;
    mp->bl_cnt  = 0U;

    if ((mp->status & 2U) != 0U) {
      /* Memory pool array allocated on heap */
      vPortFree (mp->mem_arr);
    }
    if ((mp->status & 1U) != 0U) {
      /* Memory pool control block allocated on heap */
      vPortFree (mp);
    }

    taskEXIT_CRITICAL();

    stat = osOK;
  }

  return (stat);
}

/*
  Create new block given according to the current block index.
*/
static void *CreateBlock (MemPool_t *mp) {
  MemPoolBlock_t *p = NULL;

  if (mp->n < mp->bl_cnt) {
    /* Unallocated blocks exist, set pointer to new block */
    p = (void *)(mp->mem_arr + (mp->bl_sz * mp->n));

    /* Increment block index */
    mp->n += 1U;
  }

  return (p);
}

/*
  Allocate a block by reading the list of free blocks.
*/
static void *AllocBlock (MemPool_t *mp) {
  MemPoolBlock_t *p = NULL;

  if (mp->head != NULL) {
    /* List of free block exists, get head block */
    p = mp->head;

    /* Head block is now next on the list */
    mp->head = p->next;
  }

  return (p);
}

/*
  Free block by putting it to the list of free blocks.
*/
static void FreeBlock (MemPool_t *mp, void *block) {
  MemPoolBlock_t *p = block;

  /* Store current head into block memory space */
  p->next = mp->head;

  /* Store current block as new head */
  mp->head = p;
}
#endif /* FREERTOS_MPOOL_H_ */
/*---------------------------------------------------------------------------*/
