/*
* Copyright 2016-2023 The MathWorks, Inc.
*
* File: xcp_mem_config.h
*
* Abstract:
*  XCP memory allocator default configuration file
*
* *******************
*   Main Memory Area
* *******************
*  The "Main" memory area can be configured to use up to 16 different
*  block sets, that are accessible when xcpMemAlloc() is invoked.
*
*  Block sets sizes and count:
*
*  For each set, the following must be provided (with x in [1,16]):
* 1- the block size of the set using XCP_MEM_BLOCK_x_SIZE
* 2- the number of blocks for the set using XCP_MEM_BLOCK_x_NUMBER
*
* The allocator checks that 
*      XCP_MEM_BLOCK_i_SIZE and XCP_MEM_BLOCK_i_NUMBER are set
* in order to use
*      XCP_MEM_BLOCK_i+1_SIZE and XCP_MEM_BLOCK_i+1_NUMBER
*
* Example: if 3 blocks are to be used, they must be numbered (1,2,3).
* The numberings (2,3,4), (1,3,4), or (1,2,4) will fail to compile.
*
* The allocator reorders the blocks sizes during initialization,
* so that internally sizes are treated as 
*      XCP_MEM_BLOCK_i_SIZE < XCP_MEM_BLOCK_(i+1)_SIZE
*
* Alignment & Minimum size:
*
* The macro XCP_MEM_ALIGNMENT defines the allocator's memory alignment.
*
* To ensure alignment, the allocator rounds up all block sizes to
* guarantee that XCP_MEM_BLOCK_i_SIZE are multiples of XCP_MEM_ALIGNMENT.
*
* After this round up all block sizes must be larger than a pointer,
* this condition is checked at compile time.
*
* Padding is inserted after the ID (which is 1 byte) to ensure that the
* following Block starts at an address aligned with XCP_MEM_ALIGNMENT:
*
*    XCP_MEM_ALIGNMENT
*        bytes
*    <------------>
*
*    +----+-------+---------------+----+-------+---------------+---
*    |    |       |               |    |       |               |
*    | id |padding|     Block     | id |padding|     Block     | ...
*    |    |       |               |    |       |               |
*    +----+-------+---------------+----+-------+---------------+---
*
*    <---->       <--------------->
*    1 byte          multiple of
*                  XCP_MEM_ALIGNMENT
*
*
* *****************************
*   Reserved Pools Memory Area
* *****************************
*  The "Reserved Pools" memory area allows the creation of
*  memory pools with a size that can be dynamically assigned
*  using the xcpMemReservedPoolCreate() method. The blocks in
*  a given pool can then be accessed by invoking xcpMemAllocFromPool().
*
*  The max number of reserved memory pools that can be allocated
*  is defined by the macro XCP_MEM_RESERVED_POOLS_NUMBER.
*
*  The size of the overall memory available for the pools is
*  defined by the macro XCP_MEM_RESERVED_POOLS_TOTAL_SIZE
*/

#ifndef XCP_MEM_CONFIG_H
#define XCP_MEM_CONFIG_H

#include "xcp_common.h"

#ifndef XCP_MEM_ALIGNMENT
#define XCP_MEM_ALIGNMENT           8
#endif

/* If required, it is possible to enforce the allocation of the 
   Main Memory and Reserved Pools Memory areas in a specific 
   data section. 
   E.g.:
    on Infineon micro-controllers: 
     #define XCP_MEM_DATA_SECTION_BEGIN  BEGIN_DATA_SECTION(.lmubss)
     #define XCP_MEM_DATA_SECTION_END    END_DATA_SECTION
    Currently we use XCP_MEM_DATA_SECTION_BEGIN and XCP_MEM_DATA_SECTION_END
    around each internal variable, and therefore we also support 
    the following GCC's syntax:
     #define XCP_MEM_DATA_SECTION_BEGIN  __attribute__((section("MyMemSection")))
     #define XCP_MEM_DATA_SECTION_END
*/
#ifndef XCP_MEM_DATA_SECTION_BEGIN
#define XCP_MEM_DATA_SECTION_BEGIN
#endif

#ifndef XCP_MEM_DATA_SECTION_END
#define XCP_MEM_DATA_SECTION_END
#endif

/* Given an address x, return its round-up to XCP_MEM_ALIGNMENT */
#define XCP_ALIGNED(x)  ( ( ( (uintptr_t)(x) + XCP_MEM_ALIGNMENT - 1) / XCP_MEM_ALIGNMENT ) * XCP_MEM_ALIGNMENT )

#if defined(EXTMODE_CODE_EXEC_PROFILING) && !defined(EXTMODE_CODE_EXEC_PROFILING_CUSTOM)
/* @note if the memory allocator is compiled for External Mode
         the following macros are used to allocate the memory
         required by the Code Execution Profiler */

#ifndef XCP_MAX_PROFILING_BUFFER_SIZE
/* By default, assuming a buffer size of 255 bytes.
   XCP Header and Tail of 6 bytes and 16 bytes of xcp_fifo header
   need to be taken into account as well. */
#define XCP_MAX_PROFILING_BUFFER_SIZE  278
#endif

#ifndef XCP_MAX_PROFILING_BUFFERS_NUMBER
/* By default, assuming 10 additional buffers for
   sending the profiling data */
#define XCP_MAX_PROFILING_BUFFERS_NUMBER 10
#endif

#endif

/**
 * INTERNAL defines are used by the XCP-based external mode (since 22b) to configure the memory
 * allocator while allowing the use of the non-internal defines to override any configuration. For
 * each define that is _needed_ when using XCP based extmode:
 * - if the macro is defined use that
 * - otherwise, if the macro prepended with INTERNAL_ is defined use that
 * - otherwise, run preprocessor logic
 **/

#ifndef XCP_MEM_BLOCK_1_SIZE
  #ifdef INTERNAL_XCP_MEM_BLOCK_1_SIZE
    #define XCP_MEM_BLOCK_1_SIZE INTERNAL_XCP_MEM_BLOCK_1_SIZE
  #else
    /* @note if the memory allocator is compiled for External Mode
      this size is typically used by ALLOC_ODT command to allocate
      sizeof(XcpOdt) bytes */
    #define XCP_MEM_BLOCK_1_SIZE        48
  #endif
#endif


#ifndef XCP_MEM_BLOCK_1_NUMBER
  #ifdef INTERNAL_XCP_MEM_BLOCK_1_NUMBER
    #define XCP_MEM_BLOCK_1_NUMBER INTERNAL_XCP_MEM_BLOCK_1_NUMBER
  #else

    #if defined(XCP_MEM_DAQ_RESERVED_POOLS_NUMBER) && (XCP_MEM_DAQ_RESERVED_POOLS_NUMBER > 0)
      /* @note if the memory allocator is compiled for External Mode
        this number needs to match the number of times the ALLOC_ODT 
        command is invoked (daqCount). Each rate has an associated DAQ list. 
        Profiling uses another DAQ list. Adding 2 extra ones to take into account 
        of other XCP events on the target. */
      #define XCP_MEM_BLOCK_1_NUMBER      (XCP_MEM_DAQ_RESERVED_POOLS_NUMBER + 3)
    #else
      /* Default value if not compiled for External Mode */
      #define XCP_MEM_BLOCK_1_NUMBER      10
    #endif 

  #endif
#endif /* XCP_MEM_BLOCK_1_NUMBER */


#ifndef XCP_MEM_BLOCK_2_SIZE
  #ifdef INTERNAL_XCP_MEM_BLOCK_2_SIZE
    #define XCP_MEM_BLOCK_2_SIZE INTERNAL_XCP_MEM_BLOCK_2_SIZE
  #else

    #if defined(XCP_MEM_DAQ_RESERVED_POOLS_NUMBER) && (XCP_MEM_DAQ_RESERVED_POOLS_NUMBER > 0)
      /* @note if the memory allocator is compiled for External Mode
        this size is typically used by ALLOC_DAQ command to allocate
        daqCount * sizeof(XcpDaq) bytes
        Each rate has an associated DAQ list. Profiling uses another DAQ list. 
        Adding 2 extra ones to take into account of other XCP events on the target.*/
        #define XCP_MEM_BLOCK_2_SIZE      ((XCP_MEM_DAQ_RESERVED_POOLS_NUMBER + 3) * 32)
    #else
      /* Default value if not compiled for External Mode */
      #define XCP_MEM_BLOCK_2_SIZE        320
    #endif

  #endif
#endif /* XCP_MEM_BLOCK_2_SIZE */


#ifndef XCP_MEM_BLOCK_2_NUMBER
  #ifdef INTERNAL_XCP_MEM_BLOCK_2_NUMBER
    #define XCP_MEM_BLOCK_2_NUMBER INTERNAL_XCP_MEM_BLOCK_2_NUMBER
  #else
    #if !defined(EXTMODE_CODE_EXEC_PROFILING) || defined(EXTMODE_CODE_EXEC_PROFILING_CUSTOM) ||\
            (XCP_MEM_BLOCK_2_SIZE < XCP_MAX_PROFILING_BUFFER_SIZE)
      /* @note if the memory allocator is compiled for External Mode
        this number should match the number of times the ALLOC_DAQ
        command is executed: 1 */
      #define XCP_MEM_BLOCK_2_NUMBER      1
    #else
      /* Additional blocks have been added and available for the
        transfer of Profiling data. */
      #define XCP_MEM_BLOCK_2_NUMBER      (1 + XCP_MAX_PROFILING_BUFFERS_NUMBER)
    #endif
  #endif
#endif /* XCP_MEM_BLOCK_2_NUMBER */


#ifndef XCP_MEM_BLOCK_3_SIZE
  #ifdef INTERNAL_XCP_MEM_BLOCK_3_SIZE
    #define XCP_MEM_BLOCK_3_SIZE INTERNAL_XCP_MEM_BLOCK_3_SIZE
  #else
    /* @note if the memory allocator is compiled for External Mode
      this size is typically used by ALLOC_ODT_ENTRY command to allocate
      odtEntriesCount * sizeof(XcpOdtEntry) bytes.
      According to the ASAM specifications, the max number of ODT entries
      is 255, however if the targets requires a smaller memory footprint
      the XCP_MAX_ODT_ENTRIES_COUNT macro can specify a smaller number.
      The smaller limit (35 by default) is chosen to have a XCP_MEM_BLOCK_3_SIZE
      big enough to allocate a profiling buffer (278) */

    #ifndef XCP_MAX_ODT_ENTRIES_COUNT
      #define XCP_MAX_ODT_ENTRIES_COUNT 255
    #endif

    #if defined(EXTMODE_CODE_EXEC_PROFILING) && !defined(EXTMODE_CODE_EXEC_PROFILING_CUSTOM) &&\
            (XCP_MEM_BLOCK_2_SIZE < XCP_MAX_PROFILING_BUFFER_SIZE)
      /* XCP_MAX_ODT_ENTRIES_COUNT range check if Profiling is enabled */
      #if (XCP_MAX_ODT_ENTRIES_COUNT < ((XCP_MAX_PROFILING_BUFFER_SIZE / 8) + 1)) || (XCP_MAX_ODT_ENTRIES_COUNT > 255)
        #error "Invalid XCP_MAX_ODT_ENTRIES_COUNT value"
      #endif
    #else
      /* Default XCP_MAX_ODT_ENTRIES_COUNT range check */
      #if (XCP_MAX_ODT_ENTRIES_COUNT < 1) || (XCP_MAX_ODT_ENTRIES_COUNT > 255)
        #error "Invalid XCP_MAX_ODT_ENTRIES_COUNT value"
      #endif
    #endif

    #define XCP_MEM_BLOCK_3_SIZE        (8 * XCP_MAX_ODT_ENTRIES_COUNT)
  #endif
#endif /* XCP_MEM_BLOCK_3_SIZE */


#ifndef XCP_MEM_BLOCK_3_NUMBER
#ifdef INTERNAL_XCP_MEM_BLOCK_3_NUMBER
  #define XCP_MEM_BLOCK_3_NUMBER INTERNAL_XCP_MEM_BLOCK_3_NUMBER
#else
  #if defined(XCP_MEM_DAQ_RESERVED_POOLS_NUMBER) && (XCP_MEM_DAQ_RESERVED_POOLS_NUMBER > 0)
    /* @note if the memory allocator is compiled for External Mode
      this number needs to match the number of times the ALLOC_ODT_ENTRY 
      command is invoked (odtCount * daqCount). Typically odtCount = 1.
      Each rate has an associated DAQ list. Profiling uses another DAQ list and doesn't
      support packed mode. 
      Adding 2 extra ones to take into account of other XCP events on the target. */

    #if defined(EXTMODE_CODE_EXEC_PROFILING) && !defined(EXTMODE_CODE_EXEC_PROFILING_CUSTOM) &&\
            (XCP_MEM_BLOCK_2_SIZE < XCP_MAX_PROFILING_BUFFER_SIZE)
      /* Since they were not added to the BLOCK 2 set, additional blocks 
        have been added and available for the transfer of Profiling data.  */
      #define XCP_MEM_BLOCK_3_NUMBER     (XCP_MEM_DAQ_RESERVED_POOLS_NUMBER + 3 + XCP_MAX_PROFILING_BUFFERS_NUMBER)
    #else
      #define XCP_MEM_BLOCK_3_NUMBER     (XCP_MEM_DAQ_RESERVED_POOLS_NUMBER + 3)
    #endif
  #else
  /* Default value if not compiled for External Mode */
    #define XCP_MEM_BLOCK_3_NUMBER      10
  #endif
#endif
#endif /* XCP_MEM_BLOCK_3_NUMBER */

#if !defined(XCP_MEM_BLOCK_4_SIZE) && defined(INTERNAL_XCP_MEM_BLOCK_4_SIZE)
  #define XCP_MEM_BLOCK_4_SIZE INTERNAL_XCP_MEM_BLOCK_4_SIZE
#endif

#if !defined(XCP_MEM_BLOCK_4_NUMBER) && defined(INTERNAL_XCP_MEM_BLOCK_4_NUMBER)
  #define XCP_MEM_BLOCK_4_NUMBER INTERNAL_XCP_MEM_BLOCK_4_NUMBER
#endif

#ifndef XCP_MEM_RESERVED_POOLS_NUMBER
  #ifdef INTERNAL_XCP_MEM_RESERVED_POOLS_NUMBER
    #define XCP_MEM_RESERVED_POOLS_NUMBER INTERNAL_XCP_MEM_RESERVED_POOLS_NUMBER
  #else
    #if defined(XCP_MEM_DAQ_RESERVED_POOLS_NUMBER) && (XCP_MEM_DAQ_RESERVED_POOLS_NUMBER > 0)
      /* @note if the memory allocator is compiled for External Mode
        one pool is used by the XCP Server to receive/transmit CTO.
        Each DAQ list associated to a rate in the model also has 
        a reserved memory pool */
      #define XCP_MEM_RESERVED_POOLS_NUMBER   (XCP_MEM_DAQ_RESERVED_POOLS_NUMBER + 1)
    #else
      /* by default only one pool is enabled and it used
        by the XCP Server to receive/transmit CTO */
      #define XCP_MEM_RESERVED_POOLS_NUMBER  1
    #endif
  #endif
#endif /* XCP_MEM_RESERVED_POOLS_NUMBER */


#ifndef XCP_MEM_RESERVED_POOLS_TOTAL_SIZE
  #ifdef INTERNAL_XCP_MEM_RESERVED_POOLS_TOTAL_SIZE
    #define XCP_MEM_RESERVED_POOLS_TOTAL_SIZE INTERNAL_XCP_MEM_RESERVED_POOLS_TOTAL_SIZE
  #else
    #ifdef EXTMODE_STATIC_SIZE
      /* @note if the memory allocator is compiled for External Mode
        one pool is used by the XCP Server to receive/transmit CTO.
        Therefore we need to reserve enough space to store 2 CTO packets
        (1 RX, 1 TX), considering a MAX_CTO_SIZE, XCP Header and Tail of 
        6 bytes and 16 bytes of xcp_fifo header.
        The remaining space size is used for signals streaming and 
        the size is obtained from the EXTMODE_STATIC_SIZE parameter. */
      #ifdef XCP_MAX_CTO_SIZE
        #define XCP_MEM_RESERVED_POOLS_TOTAL_SIZE  ((2 * (XCP_ALIGNED(XCP_MAX_CTO_SIZE + 22) + XCP_MEM_POOLID_SIZE)) +\
                                                  XCP_ALIGNED(EXTMODE_STATIC_SIZE))
      #else
        /* Assuming MAX_CTO_SIZE of 255 */
        #define XCP_MEM_RESERVED_POOLS_TOTAL_SIZE  ((2 * (XCP_ALIGNED(278) + XCP_MEM_POOLID_SIZE)) +\
                                                  XCP_ALIGNED(EXTMODE_STATIC_SIZE))
      #endif
    #else
    /* @note by default only one pool is used by the XCP Server 
      to receive/transmit CTO. 
      Therefore we need to reserve enough space to store 2 CTO packets
      (1 RX, 1 TX), considering a MAX_CTO_SIZE, XCP Header and Tail of 
      6 bytes and 16 bytes of xcp_fifo header. */
      #ifdef XCP_MAX_CTO_SIZE
        #define XCP_MEM_RESERVED_POOLS_TOTAL_SIZE  (2 * (XCP_ALIGNED(XCP_MAX_CTO_SIZE + 22) + XCP_MEM_POOLID_SIZE))
      #else
        /* Assuming MAX_CTO_SIZE of 255 */
        #define XCP_MEM_RESERVED_POOLS_TOTAL_SIZE  (2 * (XCP_ALIGNED(278) + XCP_MEM_POOLID_SIZE))
      #endif
    #endif
  #endif
#endif /* XCP_MEM_RESERVED_POOLS_TOTAL_SIZE */

#endif /* XCP_MEM_CONFIG_H */
