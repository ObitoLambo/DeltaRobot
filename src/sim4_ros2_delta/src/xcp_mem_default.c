/*
* Copyright 2016-2023 The MathWorks, Inc.
*
* File: xcp_mem_default.c
*
* Abstract:
*  Implementation of default XCP Server memory allocator
*/

#include "xcp_common.h"
#include "xcp_mem.h"
#include "xcp_mem_default.h"
#include "xcp_utils.h"

#define PADDING (XCP_MEM_ALIGNMENT-1)

/* Array of XcpMemPoolDescriptor for each block size to allow freeing and
   allocating blocks.

   @note this includes both the Main Memory area and the Reserved Pools
         Memory Area */
static XcpMemPoolDescriptor xcpMemPoolDescriptors[XCP_MEM_MAIN_RESERVED_POOL_NUMBER];

/* xcpMemCustomPoolDescriptor stores the custom memory functions to allocate and
   free memory for a specific memory pool */
static XcpMemCustomPoolDescriptor xcpMemCustomPoolDescriptors[XCP_MEM_CUSTOM_POOL_MAX_NUMBER];


/* Main memory chunk from which blocks are allocated using the xcpMemAlloc
 * This may not be aligned to XCP_MEM_ALIGNMENT depending on the compiler 
 * and/or build flags.
 * Add padding bytes to the end to enable aligning blocks in xcpMemMainChunk
 * to aligned addresses if needed.
 */
XCP_MEM_DATA_SECTION_BEGIN
static uint8_T xcpMemMainChunk[XCP_MEM_MAIN_TOTAL_SIZE + PADDING];
XCP_MEM_DATA_SECTION_END


/* Reserved pools memory chunk from which pool blocks are allocated using the
 * xcpMemAllocFromPool()
 * Also refer to comments in xcpMemMainChunk
 */
XCP_MEM_DATA_SECTION_BEGIN
static uint8_T xcpMemReservedPoolsChunk[XCP_MEM_RESERVED_POOLS_TOTAL_SIZE + PADDING];
XCP_MEM_DATA_SECTION_END

static size_t  xcpMemReservedPoolsChunkUsedBytes;

/* Mutex to prevent concurrent access to xcpMemPoolDescriptors, xcpMemMainChunk
   and xcpMemReservedPoolsChunk
   @note: the insertion/removal of blocks within each reserved pool is protected by a dedicated
          lock, to reduce the interference between threads in multi-core applications */
static XCP_MUTEX_DEFINE(xcpMemLock);


/* Initialize pool's memory chunk, by creating the pool's memory blocks and as a linked list */
static void xcpInitializePoolMemoryArea(uint8_T *memoryPtr, size_t blockSize, size_t blocksNumber)
{
    if (memoryPtr != NULL) {
        XcpMemHeader *p, *prev = NULL;
        size_t i;
        /* Initialize memory to zero */
        size_t totSize = XCP_MEM_BLOCK_SIZE_WITH_OVERHEAD(blockSize) * blocksNumber;
        XCP_MEMSET(memoryPtr,  0, totSize);

        for(i = 0 ; i < blocksNumber; ++i) {
            p = (XcpMemHeader*) memoryPtr;
            p -> next = NULL;
            p -> poolId = XCP_FREE_POOL_ID;
            if (prev) {
                prev -> next = p;
            }
            memoryPtr += XCP_MEM_BLOCK_SIZE_WITH_OVERHEAD(blockSize);
            prev = p;
        }
    }
}

XcpErrorCode xcpMemInit(void)
{
    uint8_T poolIdx;
    uint8_T *memoryPtr = (uint8_T *)XCP_ALIGNED(xcpMemMainChunk);

    XCP_MUTEX_INIT(xcpMemLock);


#ifndef XCP_NO_MAIN_MEM_ALLOCATED
    /* Sort main memory block */
    xcpSortArray(xcpMemMainBlockSizes, xcpMemMainBlocksNumber, (size_t)XCP_MEM_MAIN_POOLS_NUMBER);

    /* Initialize Main memory */
    for(poolIdx = 0; poolIdx < XCP_MEM_MAIN_POOLS_NUMBER; ++poolIdx) {
        size_t blocksCount = xcpMemMainBlocksNumber[poolIdx];
        size_t blockSize = xcpMemMainBlockSizes[poolIdx];

        xcpMemPoolDescriptors[poolIdx].head = (XcpMemHeader*) memoryPtr;
        xcpMemPoolDescriptors[poolIdx].totalBlocksCount = blocksCount;
        xcpMemPoolDescriptors[poolIdx].freeBlocksCount = blocksCount;
        xcpMemPoolDescriptors[poolIdx].blockSize = blockSize;
        xcpMemPoolDescriptors[poolIdx].poolSize =
            blocksCount * XCP_MEM_BLOCK_SIZE_WITH_OVERHEAD(blockSize);
#ifdef XCP_MEM_RESERVED_POOLS_LOCKLESS_SUPPORT
        xcpMemPoolDescriptors[poolIdx].deferredFreeHead = NULL;
        xcpMemPoolDescriptors[poolIdx].deferredFreeTail = NULL;
        xcpMemPoolDescriptors[poolIdx].deferredFreeBlocksCount = 0;
#else
        XCP_MUTEX_INIT(xcpMemPoolDescriptors[poolIdx].lock);
#endif

        xcpInitializePoolMemoryArea(memoryPtr, blockSize, blocksCount);
        memoryPtr += xcpMemPoolDescriptors[poolIdx].poolSize;
    }
#else
    (void)memoryPtr;
#endif

    /* Initialize Reserved pools memory
       @note since the allocation of the memory pools is dynamic,
             the creation of the blocks for each pool is deferred
             and carried out within the xcpMemReservedPoolCreate() */
    XCP_MEMSET(xcpMemReservedPoolsChunk, 0, sizeof(xcpMemReservedPoolsChunk));
    xcpMemReservedPoolsChunkUsedBytes = 0;

    for(poolIdx = XCP_MEM_MAIN_POOLS_NUMBER; poolIdx < XCP_MEM_MAIN_RESERVED_POOL_NUMBER; ++poolIdx) {
        xcpMemPoolDescriptors[poolIdx].head = XCP_FREE_POOL;
        xcpMemPoolDescriptors[poolIdx].totalBlocksCount = 0;
        xcpMemPoolDescriptors[poolIdx].freeBlocksCount = 0;
        xcpMemPoolDescriptors[poolIdx].blockSize = 0;
        xcpMemPoolDescriptors[poolIdx].poolSize = 0;
#ifdef XCP_MEM_RESERVED_POOLS_LOCKLESS_SUPPORT
        xcpMemPoolDescriptors[poolIdx].deferredFreeHead = NULL;
        xcpMemPoolDescriptors[poolIdx].deferredFreeTail = NULL;
        xcpMemPoolDescriptors[poolIdx].deferredFreeBlocksCount = 0;
#else
        XCP_MUTEX_INIT(xcpMemPoolDescriptors[poolIdx].lock);
#endif
    }

    for(poolIdx = 0; poolIdx < XCP_MEM_CUSTOM_POOL_MAX_NUMBER; poolIdx++) {
        xcpMemCustomPoolDescriptors[poolIdx].allocHandler = NULL;
        xcpMemCustomPoolDescriptors[poolIdx].freeHandler = NULL;
    }

    return XCP_SUCCESS;
}


void* xcpMemAlloc(size_t size)
{

#ifndef XCP_NO_MAIN_MEM_ALLOCATED

    uint8_T poolIdx;
    size_t blockSize = 0;
    XcpMemHeader *poolHead = NULL;

    /* a size of 0 is considered an invalid argument */
    if (size == 0) {
        return NULL;
    }

    XCP_MUTEX_LOCK(xcpMemLock);

    /* find the first large enough pool */
    for(poolIdx = 0; poolIdx <  XCP_MEM_MAIN_POOLS_NUMBER; ++poolIdx) {
        blockSize = xcpMemMainBlockSizes[poolIdx];
        if ((size <= blockSize) && (xcpMemPoolDescriptors[poolIdx].freeBlocksCount > 0)) {
            poolHead = xcpMemPoolDescriptors[poolIdx].head;
            break;
        }
    }

    if (!poolHead) { /* size is too large, or there are no more free blocks */
       XCP_MUTEX_UNLOCK(xcpMemLock);
       return NULL;
    }

    xcpMemPoolDescriptors[poolIdx].head = poolHead->next;
    (xcpMemPoolDescriptors[poolIdx].freeBlocksCount)--;
    poolHead->poolId = poolIdx;

    XCP_MUTEX_UNLOCK(xcpMemLock);

    /* return a pointer past just the poolId */
    return ((uint8_T*) poolHead) + XCP_MEM_POOLID_SIZE;

#else

    (void)size;
    return NULL;

#endif

}


void xcpMemFree(void *ptr)
{
    if (ptr) {
        /* the XcpMemHeader cell is 'behind' the user's pointer */
        XcpMemHeader *hd = (XcpMemHeader*) ((uint8_T*)ptr - XCP_MEM_POOLID_SIZE);
        uint8_T poolIdx = hd -> poolId;

        if (poolIdx >= XCP_MEM_CUSTOM_POOLS_OFFSET && poolIdx < XCP_MEM_CUSTOM_POOLS_UPPER_BOUND) {
            poolIdx = poolIdx - XCP_MEM_CUSTOM_POOLS_OFFSET;
            if (xcpMemCustomPoolDescriptors[poolIdx].freeHandler) {
                /* For custom memory pools, protection against concurrent
                   exclusion must be guaranteed within the custom free method */
                xcpMemCustomPoolDescriptors[poolIdx].freeHandler((uint8_T*)ptr - XCP_MEM_POOLID_SIZE);
            }
            return;
        }

        if ((poolIdx == XCP_FREE_POOL_ID) || (poolIdx >= XCP_MEM_MAIN_RESERVED_POOL_NUMBER)){
            /* ill-formed block or already freed pointer */
            XCP_PRINTF("xcpMemFree: invalid or double-freed pointer %p\n", ptr);
            return;
        }

#ifndef XCP_NO_MAIN_MEM_ALLOCATED
        /* insert the new free block in the front of the pool */
        if (poolIdx < XCP_MEM_MAIN_POOLS_NUMBER) {
            XCP_MUTEX_LOCK(xcpMemLock);

            hd -> poolId = XCP_FREE_POOL_ID;
            hd -> next = xcpMemPoolDescriptors[poolIdx].head;

            xcpMemPoolDescriptors[poolIdx].head = hd;
            ++(xcpMemPoolDescriptors[poolIdx].freeBlocksCount);

            XCP_MUTEX_UNLOCK(xcpMemLock);
        }
        else
#endif
        {
        #ifdef XCP_MEM_RESERVED_POOLS_LOCKLESS_SUPPORT
            /* If lockless support is enabled, the freed memory block is
               not immediately available for subsequent allocations,
               until xcpMemReservedPoolFlushFreeMem is invoked */
            hd -> poolId = XCP_FREE_POOL_ID;
            hd -> next = xcpMemPoolDescriptors[poolIdx].deferredFreeHead;

            if (xcpMemPoolDescriptors[poolIdx].deferredFreeTail == NULL) {
                /* Let's save a reference to the first block in the pool
                   that has been freed */
                xcpMemPoolDescriptors[poolIdx].deferredFreeTail = hd;
            }

            xcpMemPoolDescriptors[poolIdx].deferredFreeHead = hd;
            ++(xcpMemPoolDescriptors[poolIdx].deferredFreeBlocksCount);
        #else
            XCP_MUTEX_LOCK(xcpMemPoolDescriptors[poolIdx].lock);

            hd -> poolId = XCP_FREE_POOL_ID;
            hd -> next = xcpMemPoolDescriptors[poolIdx].head;

            xcpMemPoolDescriptors[poolIdx].head = hd;
            ++(xcpMemPoolDescriptors[poolIdx].freeBlocksCount);
            XCP_MUTEX_UNLOCK(xcpMemPoolDescriptors[poolIdx].lock);
        #endif
        }
    }
}


XcpErrorCode xcpMemReservedPoolCreate(size_t blockSize, size_t blocksNumber, xcpPoolId_T* poolId)
{
    uint8_T poolIdx;
    uint8_T *currentHead = (uint8_T *) XCP_ALIGNED(xcpMemReservedPoolsChunk);
    XcpErrorCode errorCode = XCP_SUCCESS;

    xcpPoolId_T pool = XCP_INVALID_POOL_ID;
    size_t requestedPoolSize;

    if ((poolId == NULL) || (blockSize == 0) || (blocksNumber == 0)) {
        XCP_PRINTF("xcpMemReservedPoolCreate: invalid input parameter\n");
        return XCP_INV_ARG;
    }

    /* blockSize needs to be a multiple of XCP_MEM_ALIGNMENT in order to align blocks to aligned addresses.
     * If the size is not a multiple, adjustment is done here.
     */
    blockSize = (size_t) XCP_ALIGNED(blockSize);

    requestedPoolSize = blocksNumber * XCP_MEM_BLOCK_SIZE_WITH_OVERHEAD(blockSize);

    XCP_MUTEX_LOCK(xcpMemLock);

    for(poolIdx = XCP_MEM_MAIN_POOLS_NUMBER; poolIdx < XCP_MEM_MAIN_RESERVED_POOL_NUMBER; ++poolIdx) {
        if (xcpMemPoolDescriptors[poolIdx].head == XCP_FREE_POOL) {
            /* The pool is free, let's check if it is big enough */
            if ((xcpMemPoolDescriptors[poolIdx].poolSize) == 0) {
                /* The pool has never been allocated before, so this must be the last
                   usable pool in the list */
                if (requestedPoolSize <= (XCP_MEM_RESERVED_POOLS_TOTAL_SIZE - xcpMemReservedPoolsChunkUsedBytes)) {
                    /* It is big enough, so let's mark it as allocated */
                    xcpMemPoolDescriptors[poolIdx].head = (XcpMemHeader*) currentHead;
                    xcpMemPoolDescriptors[poolIdx].totalBlocksCount = blocksNumber;
                    xcpMemPoolDescriptors[poolIdx].freeBlocksCount = blocksNumber;
                    xcpMemPoolDescriptors[poolIdx].blockSize = blockSize;
                    xcpMemPoolDescriptors[poolIdx].poolSize = requestedPoolSize;
                #ifdef XCP_MEM_RESERVED_POOLS_LOCKLESS_SUPPORT
                    xcpMemPoolDescriptors[poolIdx].deferredFreeHead = NULL;
                    xcpMemPoolDescriptors[poolIdx].deferredFreeTail = NULL;
                    xcpMemPoolDescriptors[poolIdx].deferredFreeBlocksCount = 0;
                #endif

                    xcpInitializePoolMemoryArea(currentHead, blockSize, blocksNumber);
                    xcpMemReservedPoolsChunkUsedBytes += requestedPoolSize;

                    pool = poolIdx;
                }
                break; /* No more pools to process, exit from the loop */
            } else {
                /* The pool has already been allocated so it can't be re-sized */
                if (requestedPoolSize <= xcpMemPoolDescriptors[poolIdx].poolSize) {
                    /* It is big enough, so let's mark it as allocated */
                    xcpMemPoolDescriptors[poolIdx].head = (XcpMemHeader*) currentHead;
                    xcpMemPoolDescriptors[poolIdx].totalBlocksCount = blocksNumber;
                    xcpMemPoolDescriptors[poolIdx].freeBlocksCount = blocksNumber;
                    xcpMemPoolDescriptors[poolIdx].blockSize = blockSize;
                #ifdef XCP_MEM_RESERVED_POOLS_LOCKLESS_SUPPORT
                    xcpMemPoolDescriptors[poolIdx].deferredFreeHead = NULL;
                    xcpMemPoolDescriptors[poolIdx].deferredFreeTail = NULL;
                    xcpMemPoolDescriptors[poolIdx].deferredFreeBlocksCount = 0;
                #endif

                    xcpInitializePoolMemoryArea(currentHead, blockSize, blocksNumber);
                    pool = poolIdx;
                    break; /* We found what we were looking for, exit from the loop */
                }
            }
        }

        /* Let's continue to see if there are bigger memory areas available */
        currentHead += xcpMemPoolDescriptors[poolIdx].poolSize;
    }

    XCP_MUTEX_UNLOCK(xcpMemLock);

    if (pool == XCP_INVALID_POOL_ID) {
        errorCode = XCP_NO_MEMORY;
    }

    *poolId = pool;

    return errorCode;
}


XcpErrorCode xcpMemReservedPoolDestroy(xcpPoolId_T poolId)
{
    uint8_T poolIdx;
    boolean_T isLastPool = true;

    if ((poolId < XCP_MEM_MAIN_POOLS_NUMBER) || (poolId >= XCP_MEM_MAIN_RESERVED_POOL_NUMBER)) {
        XCP_PRINTF("xcpMemReservedPoolDestroy: invalid input parameter\n");
        return XCP_INV_ARG;
    }

    XCP_MUTEX_LOCK(xcpMemLock);

    /* Check if the pool is the last one (i.e. the used pool with the highest id) */
    for(poolIdx = (XCP_MEM_MAIN_RESERVED_POOL_NUMBER - 1); poolIdx > poolId; --poolIdx) {
        if ((xcpMemPoolDescriptors[poolIdx].head != XCP_FREE_POOL) ||
            (xcpMemPoolDescriptors[poolIdx].poolSize != 0)) {
            isLastPool = false;
            break; /* the pool is not the last, exit from the loop */
        }
    }

    xcpMemPoolDescriptors[poolId].head = XCP_FREE_POOL;
    xcpMemPoolDescriptors[poolId].totalBlocksCount = 0;
    xcpMemPoolDescriptors[poolId].freeBlocksCount = 0;
    xcpMemPoolDescriptors[poolId].blockSize = 0;

#ifdef XCP_MEM_RESERVED_POOLS_LOCKLESS_SUPPORT
    xcpMemPoolDescriptors[poolId].deferredFreeHead = NULL;
    xcpMemPoolDescriptors[poolId].deferredFreeTail = NULL;
    xcpMemPoolDescriptors[poolId].deferredFreeBlocksCount = 0;
#endif

    if (isLastPool) {
        /* If it's the last pool we can de-allocate the memory, taking into account this pool
           as well as the previous ones that have already been deallocated (out of order) */
        for(poolIdx = XCP_MEM_MAIN_POOLS_NUMBER; poolIdx < XCP_MEM_MAIN_RESERVED_POOL_NUMBER; ++poolIdx) {
            if ((xcpMemPoolDescriptors[poolIdx].head == XCP_FREE_POOL) &&
                (xcpMemPoolDescriptors[poolIdx].poolSize != 0)) {
                xcpMemReservedPoolsChunkUsedBytes -= xcpMemPoolDescriptors[poolIdx].poolSize;
                xcpMemPoolDescriptors[poolIdx].poolSize = 0;
            }
        }
    }

    XCP_MUTEX_UNLOCK(xcpMemLock);

    return XCP_SUCCESS;
}


XcpErrorCode xcpMemCustomPoolCreate(XcpCustomAllocHandler allocHandler, XcpCustomFreeHandler freeHandler, xcpPoolId_T* poolId)
{
    XcpErrorCode errorCode = XCP_SUCCESS;
    uint8_T poolIdx;

    xcpPoolId_T pool = XCP_INVALID_POOL_ID;

    if ((poolId == NULL) || (allocHandler == NULL) || (freeHandler == NULL)) {
        XCP_PRINTF("xcpMemCustomPoolCreate: invalid input parameter\n");
        return XCP_INV_ARG;
    }

    XCP_MUTEX_LOCK(xcpMemLock);

    for (poolIdx = 0; poolIdx < XCP_MEM_CUSTOM_POOL_MAX_NUMBER; poolIdx++) {
        if (xcpMemCustomPoolDescriptors[poolIdx].allocHandler == NULL && xcpMemCustomPoolDescriptors[poolIdx].freeHandler == NULL) {
            pool = poolIdx + XCP_MEM_CUSTOM_POOLS_OFFSET;
            xcpMemCustomPoolDescriptors[poolIdx].allocHandler = allocHandler;
            xcpMemCustomPoolDescriptors[poolIdx].freeHandler = freeHandler;
            break;
        }
    }

    XCP_MUTEX_UNLOCK(xcpMemLock);

    if (pool == XCP_INVALID_POOL_ID) {
        errorCode = XCP_NO_MEMORY;
    }

    *poolId = pool;

    return errorCode;
}


XcpErrorCode xcpMemCustomPoolDestroy(xcpPoolId_T poolId)
{
    if (poolId < XCP_MEM_CUSTOM_POOLS_OFFSET || poolId >= XCP_MEM_CUSTOM_POOLS_UPPER_BOUND) {
        XCP_PRINTF("xcpMemCustomPoolDestroy: invalid poolId\n");
        return XCP_INV_ARG;
    }

    XCP_MUTEX_LOCK(xcpMemLock);

    xcpMemCustomPoolDescriptors[poolId - XCP_MEM_CUSTOM_POOLS_OFFSET].allocHandler = NULL;
    xcpMemCustomPoolDescriptors[poolId - XCP_MEM_CUSTOM_POOLS_OFFSET].freeHandler = NULL;

    XCP_MUTEX_UNLOCK(xcpMemLock);

    return XCP_SUCCESS;
}


void* xcpMemAllocFromPool(xcpPoolId_T poolId, size_t size)
{
    XcpMemHeader *poolHead = NULL;

    /* a size of 0 is considered an invalid argument */
    if (size == 0) {
        return NULL;
    }

    /* Check if memory allocation is delegated to a custom handler.
       For custom memory pools, protection against concurrent execution must be
       implemented withing the allocation method */
    if (poolId >= XCP_MEM_CUSTOM_POOLS_OFFSET && poolId < XCP_MEM_CUSTOM_POOLS_UPPER_BOUND) {
        if (xcpMemCustomPoolDescriptors[poolId - XCP_MEM_CUSTOM_POOLS_OFFSET].allocHandler) {
            /* Memory must be added explicitly to store the pool ID */
            poolHead = (XcpMemHeader *) xcpMemCustomPoolDescriptors[poolId - XCP_MEM_CUSTOM_POOLS_OFFSET].allocHandler(size + XCP_MEM_POOLID_SIZE);
            if (!poolHead) {
                return NULL;
            }
            poolHead->poolId = (uint8_T) poolId;
            return ((uint8_T*) poolHead) + XCP_MEM_POOLID_SIZE;
        }
        return NULL;
    }
    
    if ((poolId < XCP_MEM_MAIN_POOLS_NUMBER) || (poolId >= XCP_MEM_MAIN_RESERVED_POOL_NUMBER) ||
        (xcpMemPoolDescriptors[poolId].head == XCP_FREE_POOL) ||
        (xcpMemPoolDescriptors[poolId].blockSize < size)) {
        /* No memory block of the given size is available from this pool */
        return NULL;
    }

#ifndef XCP_MEM_RESERVED_POOLS_LOCKLESS_SUPPORT
    XCP_MUTEX_LOCK(xcpMemPoolDescriptors[poolId].lock);
#endif

    if (xcpMemPoolDescriptors[poolId].freeBlocksCount > 0) {
        poolHead = xcpMemPoolDescriptors[poolId].head;
    }

    if (!poolHead){ /* there are no more free blocks */
#ifndef XCP_MEM_RESERVED_POOLS_LOCKLESS_SUPPORT
       XCP_MUTEX_UNLOCK(xcpMemPoolDescriptors[poolId].lock);
#endif
       return NULL;
    }

    xcpMemPoolDescriptors[poolId].head = poolHead->next;
    (xcpMemPoolDescriptors[poolId].freeBlocksCount)--;
    poolHead->poolId = (uint8_T) poolId;

#ifndef XCP_MEM_RESERVED_POOLS_LOCKLESS_SUPPORT
    XCP_MUTEX_UNLOCK(xcpMemPoolDescriptors[poolId].lock);
#endif

    /* return a pointer past just the poolId */
    return ((uint8_T*) poolHead) + XCP_MEM_POOLID_SIZE;

}


#ifdef XCP_MEM_RESERVED_POOLS_LOCKLESS_SUPPORT
void xcpMemReservedPoolFlushFreeMem(xcpPoolId_T poolId)
{
    if ((poolId >= XCP_MEM_MAIN_POOLS_NUMBER) &&
        (poolId < XCP_MEM_MAIN_RESERVED_POOL_NUMBER) &&
        (xcpMemPoolDescriptors[poolId].deferredFreeTail != NULL)) {
        /* Let's move the deferred free blocks to the list of free blocks
           available for subsequent allocations */
        xcpMemPoolDescriptors[poolId].deferredFreeTail->next = xcpMemPoolDescriptors[poolId].head;
        xcpMemPoolDescriptors[poolId].head = xcpMemPoolDescriptors[poolId].deferredFreeHead;
        xcpMemPoolDescriptors[poolId].freeBlocksCount += xcpMemPoolDescriptors[poolId].deferredFreeBlocksCount;
        xcpMemPoolDescriptors[poolId].deferredFreeBlocksCount = 0;
        xcpMemPoolDescriptors[poolId].deferredFreeTail = NULL;
    }
}
#endif


XcpErrorCode xcpMemReset(void){
    XCP_MEMSET(xcpMemMainChunk,          0, sizeof(xcpMemMainChunk));
    XCP_MEMSET(xcpMemReservedPoolsChunk, 0, sizeof(xcpMemReservedPoolsChunk));
    xcpMemReservedPoolsChunkUsedBytes = 0;

    XCP_MEMSET(xcpMemPoolDescriptors,    0, sizeof(xcpMemPoolDescriptors));

    XCP_MEMSET(xcpMemCustomPoolDescriptors, 0, sizeof(xcpMemCustomPoolDescriptors));

    return XCP_SUCCESS;
}

#ifdef XCP_DEBUG_SUPPORT

void xcpMemPrintDiagnostics(void)
{
    uint8_T poolIdx;
    XCP_PRINTF("#############################\n");
    XCP_PRINTF("             Header size: %7zu B\n", XCP_MEM_HEADER_SIZE);
    XCP_PRINTF("             Offset size: %7zu B\n", XCP_MEM_POOLID_SIZE);
    XCP_PRINTF("      Main System memory: %7zu B\n", XCP_MEM_MAIN_SYSTEM_SIZE);
    XCP_PRINTF("        Main User memory: %7zu B\n", XCP_MEM_MAIN_USER_SIZE);
    XCP_PRINTF("       Main Total memory: %7zu B\n", XCP_MEM_MAIN_TOTAL_SIZE);

    for(poolIdx = 0 ; poolIdx < XCP_MEM_MAIN_POOLS_NUMBER ; ++poolIdx) {
        size_t bsThisPool = xcpMemMainBlockSizes[poolIdx];
        size_t nbBlocksThisPool = xcpMemPoolDescriptors[poolIdx].totalBlocksCount;
        size_t nbFreeBlocksThisPool = xcpMemPoolDescriptors[poolIdx].freeBlocksCount;
        XCP_PRINTF("%6zu B blocks: %4zu/%-4zu\n", bsThisPool, nbFreeBlocksThisPool, nbBlocksThisPool);
    }
    XCP_PRINTF("               (free)/(total)\n");
    XCP_PRINTF("#############################\n");

    XCP_PRINTF("        Res Pools Number: %7d   \n", XCP_MEM_RESERVED_POOLS_NUMBER);
    XCP_PRINTF("  Res Pools Total memory: %7zu B\n", (size_t)XCP_MEM_RESERVED_POOLS_TOTAL_SIZE);
    XCP_PRINTF("   Res Pools Used memory: %7zu B\n", xcpMemReservedPoolsChunkUsedBytes);

    for(poolIdx = XCP_MEM_MAIN_POOLS_NUMBER ; poolIdx < XCP_MEM_MAIN_RESERVED_POOL_NUMBER ; ++poolIdx) {
        size_t nbBlocksThisPool = xcpMemPoolDescriptors[poolIdx].totalBlocksCount;
        size_t nbFreeBlocksThisPool = xcpMemPoolDescriptors[poolIdx].freeBlocksCount;
        size_t sizeBlocksThisPool = xcpMemPoolDescriptors[poolIdx].blockSize;
        XCP_PRINTF("Pool %3d blocks: %4zu/%-4zu of size %7zu\n", poolIdx, nbFreeBlocksThisPool, nbBlocksThisPool, sizeBlocksThisPool);
    }
    XCP_PRINTF("               (free)/(total)\n");
}

#endif

#ifdef XCP_MEM_BYTE_COPY_SUPPORT
/* xcpMemcpyByte() currently supports byte-wise copy on WORD (HW_AG=2) addressable targets only */
void xcpMemcpyByte(void *pDst, uint8_T dstOffsetBytes, void const* pSrc, uint8_T srcOffsetBytes, size_t numOfBytes)
{
    uint16_T *dst = (uint16_T *)pDst;
    uint16_T const* src = (uint16_T const*)pSrc;

    if (numOfBytes >= 1) {
        size_t numOfAGUnits = ((numOfBytes + 1) >> 1);
        
        if ((dstOffsetBytes == 0) && (srcOffsetBytes == 0)){
            if (numOfBytes % 2){
                numOfAGUnits--;
                XCP_MEMCPY(dst, src, numOfAGUnits);
                dst[numOfAGUnits] &= 0xff00;
                dst[numOfAGUnits] |= (src[numOfAGUnits] & 0x00ff);
            }
            else {
                XCP_MEMCPY(dst, src, numOfAGUnits);
            }
        }
        else if ((dstOffsetBytes == 1) && (srcOffsetBytes == 0)){
            uint32_T i = 0;
            dst[0] &= 0x00ff;
            dst[0] |= ((src[0] & 0x00ff) << 8);
            for (i = 1; i < numOfAGUnits; i++){
                dst[i]  = ((src[i-1] & 0xff00) >> 8);
                dst[i] |= ((src[i] & 0x00ff) << 8);
            }
            if ((numOfBytes % 2) == 0){
                dst[i] &= 0xff00;
                dst[i] |= ((src[i-1] & 0xff00) >> 8);
            }
        }
        else if ((dstOffsetBytes == 0) && (srcOffsetBytes == 1)){
            uint32_T i = 0;
            dst[0] = ((src[0] & 0xff00) >> 8);
            if (numOfBytes >= 2) {
                dst[0] |= ((src[1] & 0x00ff) << 8);
                for (i = 1; i < numOfAGUnits-1; i++){
                    dst[i]  = ((src[i] & 0xff00) >> 8);
                    dst[i] |= ((src[i+1] & 0x00ff) << 8);
                }
                if (numOfBytes % 2){
                    dst[i] &= 0xff00;
                    dst[i] |= ((src[i] & 0xff00) >> 8);
                }
            }
        }
        else if ((dstOffsetBytes == 1) && (srcOffsetBytes == 1)){
            dst[0] &= 0x00ff;
            dst[0] |= (src[0] & 0xff00);
            numOfBytes--;
            numOfAGUnits--;
            dst++;
            src++;
            XCP_MEMCPY(dst, src, numOfAGUnits);
            if ((numOfBytes % 2) != 0){
                dst[numOfAGUnits] &= 0xff00;
                dst[numOfAGUnits] |= (src[numOfAGUnits] & 0x00ff);
            }
        }
    }
}

#endif
