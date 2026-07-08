/*
* Copyright 2016-2023 The MathWorks, Inc.
*
* File: xcp_platform_default.h
*
* Abstract:
*  Default implementation for XCP Platform Abstraction Layer interface
*  supporting generic Linux, Windows and Mac
*/

#ifndef XCP_PLATFORM_DEFAULT_H
#define XCP_PLATFORM_DEFAULT_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Include the appropriate header files, depending on the platform */
#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32)

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include "windows.h"

#elif defined(__linux__) || defined(__APPLE__) && defined(__MACH__)

#include <sys/time.h>    /* gettimeofday */ 

#if defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 199309L
#include <time.h>       /* for nanosleep */
#else
#include <stddef.h>
#include <sys/select.h> /* for select */
#endif /* _POSIX_C_SOURCE >= 199309L */

#else

#error "platform not supported: a custom Platform Abstraction Layer needs to be provided."

#endif

#include "rtwtypes.h"

/* The default "printf-like" logging function is the standard printf */
#define XCP_PRINTF  printf

#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32)

/* Define Mutual exclusion APIs */
#define XCP_MUTEX_DEFINE(lock)    HANDLE lock
#define XCP_MUTEX_INIT(lock)      lock = CreateMutex(0, false, 0)
#define XCP_MUTEX_LOCK(lock)      WaitForSingleObject((lock), INFINITE)
#define XCP_MUTEX_UNLOCK(lock)    ReleaseMutex(lock)

#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
/* The above mutex implementation does not support independent
   event locks to protect DAQ list data structures */
#define XCP_DAQ_LIST_INDEPENDENT_EVENT_LOCK 0
#endif

#elif defined(__linux__) || defined(__APPLE__) && defined(__MACH__)

#include <pthread.h>

/* Define Mutual exclusion APIs */
#define XCP_MUTEX_DEFINE(lock)    pthread_mutex_t lock
#define XCP_MUTEX_INIT(lock)      pthread_mutex_init(&(lock), NULL)
#define XCP_MUTEX_LOCK(lock)      pthread_mutex_lock(&(lock))
#define XCP_MUTEX_UNLOCK(lock)    pthread_mutex_unlock(&(lock))

#else

#error "platform not supported: a custom Platform Abstraction Layer needs to be provided."

#endif

/* Memory barrier */

#if defined(__GNUC__)

/* Covers compilers that implement "GNU C" features,
 other than gcc also clang on linux and mac defines the macro */

#define XCP_MEM_BARRIER() __sync_synchronize()

#elif defined(_MSC_VER)

#include <Windows.h>

#define XCP_MEM_BARRIER() MemoryBarrier()

#endif

/* Define the API to convert the XCP address and extension into the corresponding
   variable address in the target address space */
extern uint8_T *xcpAddressGet(uint8_T addressExtension, uint32_T address);
#define XCP_ADDRESS_GET(addressExtension, address) xcpAddressGet(addressExtension,address)


#ifndef XCP_TIMESTAMP_BASED_ON_SIMULATION_TIME

/* Define the API to get the current timestamp */
extern uint32_T xcpGetTimestamp(void);
#define XCP_TIMESTAMP_GET xcpGetTimestamp

#endif

/* Define the API to pause the execution (sleep) for a specific amount of time */
extern void xcpSleep(uint32_T seconds, uint32_T microseconds);
#define XCP_SLEEP(seconds,microseconds) xcpSleep(seconds,microseconds)


/* Define the APIs used to enforce struct alignment. */
#if defined(__linux__)
/* Use __attribute__((packed)) and __attribute__((aligned(n))) on GCC */
#define XCP_PRAGMA_PACK_BEGIN(n)
#define XCP_PRAGMA_PACK_END()
#define XCP_ATTRIBUTE_ALIGNED(n)  __attribute__((aligned(n)))
#define XCP_ATTRIBUTE_PACKED      __attribute__((packed))
#else
/* Use #pragma pack(push, n) and #pragma pack(pop) for all the other platforms */

#if (defined(_WIN32) || defined(__WIN32__) || defined(WIN32)) && defined(_MSC_VER)
/* _Pragma was introduced in C99, but it's supported by VS only
   from VS2019 v.16.6. To allow earlier versions of the compiler 
   we use the Microsoft-specific __pragma instead */
#define PRAGMA(n)   __pragma(n)
#else
/* Default to the standard implementation in all other cases, including MinGW */
#define PRAGMA(n)   _Pragma(#n)
#endif

#define XCP_PRAGMA_PACK_BEGIN(n)  PRAGMA(pack(push, n))
#define XCP_PRAGMA_PACK_END()     PRAGMA(pack(pop))
#define XCP_ATTRIBUTE_ALIGNED(n)
#define XCP_ATTRIBUTE_PACKED
#endif /* defined(__linux__) */

#endif /* XCP_PLATFORM_DEFAULT_H */
