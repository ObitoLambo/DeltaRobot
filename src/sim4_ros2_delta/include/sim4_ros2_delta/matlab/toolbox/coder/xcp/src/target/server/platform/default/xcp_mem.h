/*
* Copyright 2016-2022 The MathWorks, Inc.
*
* File: xcp_mem.h
*
* Abstract:
*  XCP memory allocator interface
*
*  The memory allocator provides dynamic access to contiguous blocks
*  of memory that are statically allocated and defined at compile time.
*
*  The initial design provided access to a single memory chunk (called
*  "Main" memory) that was divided in different sets of blocks of
*  different size, configurable as described in xcp_mem_config.h
*  and accessible using a simple xcpMemAlloc() API.
*
*  In order to support multi-threaded applications that require:
*  - size of the memory blocks defined at run-time
*  - a guaranteed reserved memory area (not accessible by
*    other services)
*  the concept of reserved memory pool has been introduced.
*
*  In particular a new reserved memory pool can be created
*  using a xcpMemReservedPoolCreate() API.
*
*  The allocation of a memory area from a specific reserved pool
*  can be achieved using a xcpMemAllocFromPool() method.
*
*/

#ifndef XCP_MEM_H
#define XCP_MEM_H

#include <stddef.h>

#include "xcp.h"
#include "xcp_internal.h"

#ifndef XCP_MEM_RESERVED_POOLS_LOCKLESS_SUPPORT
#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
#define XCP_MEM_RESERVED_POOLS_LOCKLESS_SUPPORT
#endif
#endif


typedef int xcpPoolId_T;

#define XCP_INVALID_POOL_ID  ((xcpPoolId_T) -1)


/** Initialize XCP memory allocator */
XcpErrorCode xcpMemInit(void);


/** Retrieve a contiguous memory area of size bytes.
    It returns NULL if no memory is available */
void* xcpMemAlloc(size_t size);


/** Release a contiguous memory area (previously allocated).
    @note: if XCP_MEM_RESERVED_POOLS_LOCKLESS_SUPPORT is enabled,
           the freed memory block is not immediately available
           for subsequent allocations, until xcpMemReservedPoolFlushFreeMem
           is explicitly invoked */
void xcpMemFree(void *ptr);


/** Create a reserved memory pool of 'blockNumber' blocks of 'blockSize' size */
XcpErrorCode xcpMemReservedPoolCreate(size_t blockSize, size_t blocksNumber, xcpPoolId_T* poolId);


/** Destroy a reserved memory pool */
XcpErrorCode xcpMemReservedPoolDestroy(xcpPoolId_T poolId);


/** Create a custom memory pool which delegates the memory management externally
    @param allocHandler function handler called to allocate memory
    @param freeHandler function handler called to release memory
*/
XcpErrorCode xcpMemCustomPoolCreate(XcpCustomAllocHandler allocHandler,
                         XcpCustomFreeHandler freeHandler, xcpPoolId_T* poolId);


/** Destroy a custom memory pool */
XcpErrorCode xcpMemCustomPoolDestroy(xcpPoolId_T poolId);


/** Retrieve a contiguous memory area of size bytes.
    from a given memory pool.  */
void* xcpMemAllocFromPool(xcpPoolId_T poolId, size_t size);


#ifdef XCP_MEM_RESERVED_POOLS_LOCKLESS_SUPPORT
/** Flush all the memory blocks that have been freed, and make them available
    for subsequent allocations */
void xcpMemReservedPoolFlushFreeMem(xcpPoolId_T poolId);
#endif


/** Reset XCP memory allocator to the default initial state */
XcpErrorCode xcpMemReset(void);


#ifdef XCP_DEBUG_SUPPORT
/** Print the current state of XCP memory allocator */
void xcpMemPrintDiagnostics(void);
#endif

#ifndef XCP_MEM_BYTE_COPY_SUPPORT
/* If XCP_ADDRESS_GRANULARITY is BYTE and XCP_HARDWARE_ADDRESS_GRANULARITY is WORD, 
   then it means that we are emulating a 16-bit addressable target as an 8-bit 
   addressable target. Bytewise memcopy is supported only when we are emulating 
   as 8-bit addressable target.   */
#if (defined(XCP_ADDRESS_GRANULARITY) && defined(XCP_HARDWARE_ADDRESS_GRANULARITY))
    #if ((XCP_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE) && \
            (XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD))
        #define XCP_MEM_BYTE_COPY_SUPPORT
    #endif
#endif
#endif


#ifdef XCP_MEM_BYTE_COPY_SUPPORT
/** Custom memory copy function for byte-wise copy when emulating a
    non-BYTE addressable target (Eg.: C2000) as a BYTE addressable target.
    Currently, supports WORD addressable targets only. But, can be extended
    to DWORD addressable targets.
            
    @param  pDst           Pointer to the destination
    @param  dstOffsetBytes Offset (in 8-bit bytes) from the destination pointer (pDst) where the bytes are copied to
    @param  pSrc           Pointer to the source
    @param  srcOffsetBytes Offset (in 8-bit bytes) from the source pointer (pSrc) where the bytes are copied from
    @param  numOfBytes     Number of 8-bit bytes to copy                                                          */
extern void xcpMemcpyByte(void *pDst, uint8_T dstOffsetBytes, void const* pSrc, uint8_T srcOffsetBytes, size_t numOfBytes);
#endif

#endif /* XCP_MEM_H */
