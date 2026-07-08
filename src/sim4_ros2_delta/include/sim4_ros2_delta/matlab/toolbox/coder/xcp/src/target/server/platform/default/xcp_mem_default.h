/*
* Copyright 2016-2022 The MathWorks, Inc.
*
* File: xcp_mem_default.h
*
* Abstract:
*  The file contains data types and macros required by the default memory
*  allocator implementation
*/

#ifndef XCP_MEM_DEFAULT_H
#define XCP_MEM_DEFAULT_H

#include "xcp_common.h"
#include "xcp_internal.h"
#include "xcp_mem.h"
#include "xcp_mem_config.h"
#include "xcp_mem_config_helper.h"


#define XCP_FREE_POOL     NULL
#define XCP_FREE_POOL_ID (0xFF)


/** XcpMemHeader is a node of a singly linked. It is laid down in the memory
in front of each memory block.
@note poolId must between 0 and 0xFE, since its value is FREE_POOL_ID when the block is free
*/
XCP_PRAGMA_PACK_BEGIN(XCP_MEM_ALIGNMENT)
typedef struct XCP_ATTRIBUTE_ALIGNED(XCP_MEM_ALIGNMENT) XcpMemHeader{
  uint8_T poolId;             /**< the index of the pool to which this block of memory belongs */
  struct XcpMemHeader *next;  /**< points to the next node of the list  */
} XcpMemHeader;
XCP_PRAGMA_PACK_END()

#define XCP_MEM_HEADER_SIZE (sizeof(XcpMemHeader))
#define XCP_MEM_POOLID_SIZE (offsetof(XcpMemHeader, next))

#define XCP_MEM_BLOCK_SIZE_WITH_OVERHEAD(size)  ((size) + XCP_MEM_POOLID_SIZE)


/** XcpMemPoolDescriptor is specific to each block size. It points to the list
of free blocks of that size and keeps track of the number of free blocks.
@note totalBlocksCount does not change at runtime
*/
typedef struct XcpMemPoolDescriptor{
    XcpMemHeader *head;       /**< list of free blocks */
    size_t freeBlocksCount;   /**< number of free blocks */
    size_t totalBlocksCount;  /**< total number of blocks (free+allocated) */
    size_t blockSize;         /**< size (in bytes) of each block in the pool (without header) */
    size_t poolSize;          /**< size (in bytes) of all the block in the pool (including headers) */
#ifdef XCP_MEM_RESERVED_POOLS_LOCKLESS_SUPPORT
    XcpMemHeader *deferredFreeHead; /**< list of blocks that have been freed, but not yet available for allocation */
    size_t deferredFreeBlocksCount; /**< number of blocks that have been freed, but not yet available for allocation */
    XcpMemHeader *deferredFreeTail; /**< pointer to the first block that has been freed */
#else
    XCP_MUTEX_DEFINE(lock);   /**< lock to protect insertion/removal of blocks in the pool */
#endif
} XcpMemPoolDescriptor;

/** XcpMemCustomPoolDescriptor is specific to memory pools where the blocks are
allocated through custom allocation and de-allocation functions
*/
typedef struct XcpMemCustomPoolDescriptor{
    XcpCustomAllocHandler allocHandler;
    XcpCustomFreeHandler freeHandler;
} XcpMemCustomPoolDescriptor;


#if (XCP_MEM_BLOCK_1_NUMBER == 0)

#define XCP_MEM_MAIN_POOLS_NUMBER 0
#define XCP_NO_MAIN_MEM_ALLOCATED

#else

/** xcpMemMainBlockSizes contains the block sizes that the memory allocator
was configured at compile time to use for the "Main" memory
*/
static size_t xcpMemMainBlockSizes[] = {
#if XCP_MEM_BLOCK_1_NUMBER
  XCP_ALIGNED(XCP_MEM_BLOCK_1_SIZE),
#endif
#if XCP_MEM_BLOCK_2_NUMBER
  XCP_ALIGNED(XCP_MEM_BLOCK_2_SIZE),
#endif
#if XCP_MEM_BLOCK_3_NUMBER
  XCP_ALIGNED(XCP_MEM_BLOCK_3_SIZE),
#endif
#if XCP_MEM_BLOCK_4_NUMBER
  XCP_ALIGNED(XCP_MEM_BLOCK_4_SIZE),
#endif
#if XCP_MEM_BLOCK_5_NUMBER
  XCP_ALIGNED(XCP_MEM_BLOCK_5_SIZE),
#endif
#if XCP_MEM_BLOCK_6_NUMBER
  XCP_ALIGNED(XCP_MEM_BLOCK_6_SIZE),
#endif
#if XCP_MEM_BLOCK_7_NUMBER
  XCP_ALIGNED(XCP_MEM_BLOCK_7_SIZE),
#endif
#if XCP_MEM_BLOCK_8_NUMBER
  XCP_ALIGNED(XCP_MEM_BLOCK_8_SIZE),
#endif
#if XCP_MEM_BLOCK_9_NUMBER
  XCP_ALIGNED(XCP_MEM_BLOCK_9_SIZE),
#endif
#if XCP_MEM_BLOCK_10_NUMBER
  XCP_ALIGNED(XCP_MEM_BLOCK_10_SIZE),
#endif
#if XCP_MEM_BLOCK_11_NUMBER
  XCP_ALIGNED(XCP_MEM_BLOCK_11_SIZE),
#endif
#if XCP_MEM_BLOCK_12_NUMBER
  XCP_ALIGNED(XCP_MEM_BLOCK_12_SIZE),
#endif
#if XCP_MEM_BLOCK_13_NUMBER
  XCP_ALIGNED(XCP_MEM_BLOCK_13_SIZE),
#endif
#if XCP_MEM_BLOCK_14_NUMBER
  XCP_ALIGNED(XCP_MEM_BLOCK_14_SIZE),
#endif
#if XCP_MEM_BLOCK_15_NUMBER
  XCP_ALIGNED(XCP_MEM_BLOCK_15_SIZE),
#endif
#if XCP_MEM_BLOCK_16_NUMBER
  XCP_ALIGNED(XCP_MEM_BLOCK_16_SIZE),
#endif
};

/** xcpMemMainBlocksNumber contains the number of blocks of each size.
For a block size, the number of blocks appears only if that block size was
configured at compile time to be used by the memory allocator
*/
static size_t xcpMemMainBlocksNumber[] = {
#if XCP_MEM_BLOCK_1_NUMBER
  XCP_MEM_BLOCK_1_NUMBER,
#endif
#if XCP_MEM_BLOCK_2_NUMBER
  XCP_MEM_BLOCK_2_NUMBER,
#endif
#if XCP_MEM_BLOCK_3_NUMBER
  XCP_MEM_BLOCK_3_NUMBER,
#endif
#if XCP_MEM_BLOCK_4_NUMBER
  XCP_MEM_BLOCK_4_NUMBER,
#endif
#if XCP_MEM_BLOCK_5_NUMBER
  XCP_MEM_BLOCK_5_NUMBER,
#endif
#if XCP_MEM_BLOCK_6_NUMBER
  XCP_MEM_BLOCK_6_NUMBER,
#endif
#if XCP_MEM_BLOCK_7_NUMBER
  XCP_MEM_BLOCK_7_NUMBER,
#endif
#if XCP_MEM_BLOCK_8_NUMBER
  XCP_MEM_BLOCK_8_NUMBER,
#endif
#if XCP_MEM_BLOCK_9_NUMBER
  XCP_MEM_BLOCK_9_NUMBER,
#endif
#if XCP_MEM_BLOCK_10_NUMBER
  XCP_MEM_BLOCK_10_NUMBER,
#endif
#if XCP_MEM_BLOCK_11_NUMBER
  XCP_MEM_BLOCK_11_NUMBER,
#endif
#if XCP_MEM_BLOCK_12_NUMBER
  XCP_MEM_BLOCK_12_NUMBER,
#endif
#if XCP_MEM_BLOCK_13_NUMBER
  XCP_MEM_BLOCK_13_NUMBER,
#endif
#if XCP_MEM_BLOCK_14_NUMBER
  XCP_MEM_BLOCK_14_NUMBER,
#endif
#if XCP_MEM_BLOCK_15_NUMBER
  XCP_MEM_BLOCK_15_NUMBER,
#endif
#if XCP_MEM_BLOCK_16_NUMBER
  XCP_MEM_BLOCK_16_NUMBER,
#endif
};

#define XCP_MEM_MAIN_POOLS_NUMBER ((int8_T)(sizeof(xcpMemMainBlocksNumber)/sizeof(xcpMemMainBlocksNumber[0])))

#endif

#define XCP_MEM_MAIN_USER_SIZE ( (size_t)                         \
                ( XCP_MEM_BLOCK_1_NUMBER  * XCP_ALIGNED(XCP_MEM_BLOCK_1_SIZE)  )+ \
                ( XCP_MEM_BLOCK_2_NUMBER  * XCP_ALIGNED(XCP_MEM_BLOCK_2_SIZE)  )+ \
                ( XCP_MEM_BLOCK_3_NUMBER  * XCP_ALIGNED(XCP_MEM_BLOCK_3_SIZE)  )+ \
                ( XCP_MEM_BLOCK_4_NUMBER  * XCP_ALIGNED(XCP_MEM_BLOCK_4_SIZE)  )+ \
                ( XCP_MEM_BLOCK_5_NUMBER  * XCP_ALIGNED(XCP_MEM_BLOCK_5_SIZE)  )+ \
                ( XCP_MEM_BLOCK_6_NUMBER  * XCP_ALIGNED(XCP_MEM_BLOCK_6_SIZE)  )+ \
                ( XCP_MEM_BLOCK_7_NUMBER  * XCP_ALIGNED(XCP_MEM_BLOCK_7_SIZE)  )+ \
                ( XCP_MEM_BLOCK_8_NUMBER  * XCP_ALIGNED(XCP_MEM_BLOCK_8_SIZE)  )+ \
                ( XCP_MEM_BLOCK_9_NUMBER  * XCP_ALIGNED(XCP_MEM_BLOCK_9_SIZE)  )+ \
                ( XCP_MEM_BLOCK_10_NUMBER * XCP_ALIGNED(XCP_MEM_BLOCK_10_SIZE) )+ \
                ( XCP_MEM_BLOCK_11_NUMBER * XCP_ALIGNED(XCP_MEM_BLOCK_11_SIZE) )+ \
                ( XCP_MEM_BLOCK_12_NUMBER * XCP_ALIGNED(XCP_MEM_BLOCK_12_SIZE) )+ \
                ( XCP_MEM_BLOCK_13_NUMBER * XCP_ALIGNED(XCP_MEM_BLOCK_13_SIZE) )+ \
                ( XCP_MEM_BLOCK_14_NUMBER * XCP_ALIGNED(XCP_MEM_BLOCK_14_SIZE) )+ \
                ( XCP_MEM_BLOCK_15_NUMBER * XCP_ALIGNED(XCP_MEM_BLOCK_15_SIZE) )+ \
                ( XCP_MEM_BLOCK_16_NUMBER * XCP_ALIGNED(XCP_MEM_BLOCK_16_SIZE) ) )


#define XCP_MEM_MAIN_BLOCKS_NUMBER ( XCP_MEM_BLOCK_1_NUMBER  + \
                XCP_MEM_BLOCK_2_NUMBER  + \
                XCP_MEM_BLOCK_3_NUMBER  + \
                XCP_MEM_BLOCK_4_NUMBER  + \
                XCP_MEM_BLOCK_5_NUMBER  + \
                XCP_MEM_BLOCK_6_NUMBER  + \
                XCP_MEM_BLOCK_7_NUMBER  + \
                XCP_MEM_BLOCK_8_NUMBER  + \
                XCP_MEM_BLOCK_9_NUMBER  + \
                XCP_MEM_BLOCK_10_NUMBER + \
                XCP_MEM_BLOCK_11_NUMBER + \
                XCP_MEM_BLOCK_12_NUMBER + \
                XCP_MEM_BLOCK_13_NUMBER + \
                XCP_MEM_BLOCK_14_NUMBER + \
                XCP_MEM_BLOCK_15_NUMBER + \
                XCP_MEM_BLOCK_16_NUMBER )

#define XCP_MEM_MAIN_SYSTEM_SIZE ((XCP_MEM_POOLID_SIZE) * (XCP_MEM_MAIN_BLOCKS_NUMBER))

#define XCP_MEM_MAIN_TOTAL_SIZE (XCP_MEM_MAIN_USER_SIZE + XCP_MEM_MAIN_SYSTEM_SIZE)

#define XCP_MEM_MAIN_RESERVED_POOL_NUMBER  (XCP_MEM_MAIN_POOLS_NUMBER + XCP_MEM_RESERVED_POOLS_NUMBER)

/* Only a single custom pool manager is supported */
#define XCP_MEM_CUSTOM_POOL_MAX_NUMBER  1

#define XCP_MEM_CUSTOM_POOLS_OFFSET  XCP_MEM_MAIN_RESERVED_POOL_NUMBER

#define XCP_MEM_CUSTOM_POOLS_UPPER_BOUND (XCP_MEM_CUSTOM_POOLS_OFFSET + XCP_MEM_CUSTOM_POOL_MAX_NUMBER)

#define XCP_MEM_POOLS_TOTAL_NUMBER  (XCP_MEM_MAIN_RESERVED_POOL_NUMBER + XCP_MEM_CUSTOM_POOL_MAX_NUMBER)

/* @note the poolId is currently uint8_T and the first 16 pools are reserved to the "Main" memory */
#define XCP_MEM_MAX_RESERVED_POOLS_NUMBER (255 - (16 + XCP_MEM_CUSTOM_POOL_MAX_NUMBER))

#if (XCP_MEM_RESERVED_POOLS_NUMBER < 1) || (XCP_MEM_RESERVED_POOLS_NUMBER > XCP_MEM_MAX_RESERVED_POOLS_NUMBER )
#error "XCP_MEM_RESERVED_POOLS_NUMBER must be greater than 0, and the total number of pools (239) must not be exceeded."
#endif


#endif /* XCP_MEM_DEFAULT_H */
