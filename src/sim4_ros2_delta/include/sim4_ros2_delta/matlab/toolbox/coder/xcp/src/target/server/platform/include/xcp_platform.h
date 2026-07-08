/*
* Copyright 2016-2021 The MathWorks, Inc.
*
* File: xcp_platform.h
*
* Abstract:
*  XCP Platform Abstraction Layer interface
*/

#ifndef XCP_PLATFORM_H
#define XCP_PLATFORM_H

#ifdef XCP_CUSTOM_PLATFORM

#include "xcp_platform_custom.h"

#else

#include "xcp_platform_default.h"

#endif /* XCP_CUSTOM_PLATFORM */

/* Platform header file validation */

#if !defined(XCP_MEMCPY) || !defined(XCP_MEMSET)
#include <string.h>
#endif

/* A "printf-like" logging function can be provided */
#ifndef XCP_PRINTF
#include <stdarg.h>
extern void xcp_void_printf(const char_T *fmt,...);
#define XCP_PRINTF xcp_void_printf
#endif

/* A memcpy implementation can be provided */
#ifndef XCP_MEMCPY
#define XCP_MEMCPY memcpy
#endif

/* A memset implementation can be provided */
#ifndef XCP_MEMSET
#define XCP_MEMSET memset
#endif

/* An API to define a mutex needs to be provided */
#ifndef XCP_MUTEX_DEFINE
#error "XCP_MUTEX_DEFINE not defined in platform specific header file"
#endif

/* An API to initialize a mutex data structure needs to be provided */
#ifndef XCP_MUTEX_INIT
#error "XCP_MUTEX_INIT not defined in platform specific header file"
#endif

/* An API to lock a mutex needs to be provided */
#ifndef XCP_MUTEX_LOCK
#error "XCP_MUTEX_LOCK not defined in platform specific header file"
#endif

/* An API to unlock a mutex needs to be provided */
#ifndef XCP_MUTEX_UNLOCK
#error "XCP_MUTEX_UNLOCK not defined in platform specific header file"
#endif

/* An API to get the current timestamp needs to be provided */
#ifndef XCP_TIMESTAMP_GET
#ifdef XCP_TIMESTAMP_BASED_ON_SIMULATION_TIME

extern uint32_T xcpGetTimestamp(void);
#define XCP_TIMESTAMP_GET         xcpGetTimestamp

#else

#error "XCP_TIMESTAMP_GET not defined in platform specific header file"

#endif
#endif

/* An API to get the variable memory address, starting from XCP (addressExtension,  address) needs to be provided */
#ifndef XCP_ADDRESS_GET
#error "XCP_ADDRESS_GET not defined in platform specific header file"
#endif

/* An API to pause the execution (sleep) for a specific amount of time needs to be provided */
#ifndef XCP_SLEEP
#error "XCP_SLEEP not defined in platform specific header file"
#endif

/* A memory barrier instruction can be provided if the processor supports out of order execution */
#ifndef XCP_MEM_BARRIER
#define XCP_MEM_BARRIER()  do{}while(0)
#endif

/* Some APIs to enforce struct alignment should be provided.
   If not defined, #pragma pack(push, n) and #pragma pack(pop) 
   are used by default. Users should replace them with 
   the appropriate compiler-specific constructs if a warning 
   or error message is generated */
#if !defined(XCP_PRAGMA_PACK_BEGIN) && !defined(XCP_PRAGMA_PACK_END) && !defined(XCP_ATTRIBUTE_ALIGNED) && !defined(XCP_ATTRIBUTE_PACKED)

#define PRAGMA(n)   _Pragma(#n)
#define XCP_PRAGMA_PACK_BEGIN(n)  PRAGMA(pack(push, n))
#define XCP_PRAGMA_PACK_END()     PRAGMA(pack(pop))
#define XCP_ATTRIBUTE_ALIGNED(n)
#define XCP_ATTRIBUTE_PACKED

#endif


#endif /* XCP_PLATFORM_H */
