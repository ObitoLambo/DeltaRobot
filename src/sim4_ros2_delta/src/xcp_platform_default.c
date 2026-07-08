/*
* Copyright 2016-2023 The MathWorks, Inc.
*
* File: xcp_platform_default.c
*
* Abstract:
*  Implementation of default XCP Server platform services
*/

#include <stdlib.h>

#include "xcp_common.h"
#include "xcp_platform_default.h"
#include "xcp_cfg.h"

#if defined(__APPLE__) && defined(__MACH__)
#include <mach-o/dyld.h>
#endif

/* Define the API to pause the execution (sleep) for a specific amount of time */
#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32)

#define XCP_MICROSECONDS_PER_SECOND  1000000

static boolean_T xcpPerfCounterAvailable      = true;
static boolean_T xcpPerfCounterInitialized    = false;
static uint64_t  xcpPerfCounterTicksPerSecond = 1;

/* Initialize performance counters */
static void xcpPerfCounterInit(void)
{
    if (!xcpPerfCounterInitialized) {
        LARGE_INTEGER frequency;

        if (QueryPerformanceFrequency(&frequency)) {
            if ((frequency.u.HighPart != 0) || (frequency.u.LowPart == 0)) {
                XCP_PRINTF("xcpTimestampInit: QueryPerformanceFrequency error, unexpected frequency\n");
                xcpPerfCounterAvailable = false;
            } else {
                xcpPerfCounterTicksPerSecond = frequency.u.LowPart;
            }
        }
        else {
            XCP_PRINTF("xcpTimestampInit: QueryPerformanceFrequency error, performance Counter not available\n");
            xcpPerfCounterAvailable = false;
        }

        xcpPerfCounterInitialized = true;
    }
}


/* Convert a performance counter value (in ticks) into a value according to a specific time unit */
static uint64_t xcpPerfCounterValue2TimeUnits(LARGE_INTEGER value, uint64_t timeUnitsPerSecond)
{
    return ((((uint64_t)value.u.HighPart) << 32) | (uint64_t)value.u.LowPart) * timeUnitsPerSecond / xcpPerfCounterTicksPerSecond;
}



void xcpSleep(uint32_T seconds, uint32_T microseconds)
{
    if ((seconds == 0) && (microseconds == 0)) {
        /* According to MSDN, a value of zero causes the thread to relinquish
           the remainder of its time slice to any other thread that is ready to run.
           If there are no other threads ready to run, the function returns immediately
           and the thread continues execution */
        Sleep(0);
    } else if ((seconds == 0) && (microseconds < 1000)) {
        /* Sleep() doesn't allow to sleep for less than 1ms.
           If a sleep time of less than 1ms is requested on Windows, a busy wait
           based on performance counters is used instead */
        if (!xcpPerfCounterInitialized) {
            /* Initialize performance counters */
            xcpPerfCounterInit();
        }

        if (xcpPerfCounterAvailable) {
            LARGE_INTEGER current;
            uint64_t startTime = 0;
            uint64_t currentTime = 0;
            uint64_t elapsedTime = 0;

            QueryPerformanceCounter(&current);
            startTime = xcpPerfCounterValue2TimeUnits(current, XCP_MICROSECONDS_PER_SECOND);

            do
            {
                QueryPerformanceCounter(&current);
                currentTime = xcpPerfCounterValue2TimeUnits(current, XCP_MICROSECONDS_PER_SECOND);

                if (currentTime >= startTime) {
                    elapsedTime = currentTime - startTime;
                } else {
                    elapsedTime = currentTime + UINT64_MAX - startTime;
                }
            }
            while(elapsedTime <= microseconds);
        }
    } else {
        /* Sleep for the required amount of ms */
        Sleep((seconds) * 1000 + (((microseconds) + 1000 - 1) / 1000));
    }
}

#elif defined(__linux__) || defined(__APPLE__) && defined(__MACH__)

void xcpSleep(uint32_T seconds, uint32_T microseconds)
{
#if defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 199309L
	/* nanosleep() is the preferred sleep method on Linux */
	struct timespec t;
	t.tv_sec = seconds;
	t.tv_nsec = microseconds * 1000;

	nanosleep(&t, NULL);
#else
	/* if nanosleep() is not available, use the select() instead */
	struct timeval t;
	t.tv_sec = seconds;
	t.tv_usec = (suseconds_t)microseconds; /* tv_usec is signed */

	select(0, NULL, NULL, NULL, &t);
#endif /* _POSIX_C_SOURCE >= 199309L */
}

#else

#error "platform not supported: a custom Platform Abstraction Layer needs to be provided."

#endif


#ifndef XCP_TIMESTAMP_BASED_ON_SIMULATION_TIME
/* @note The following section provides the default implementation
         of the xcpGetTimestamp() API, based on the 'real' time
         for Linux and Windows platforms.
         If XCP_TIMESTAMP_BASED_ON_SIMULATION_TIME is defined,
         the same function will be implemented in the External Mode
         abstraction layer, based on the model simulation time */

/* Define the XCP_TIMESTAMP_UNITS_PER_SECOND value based on the
   selected XCP_TIMESTAMP_UNIT */
#if (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_1NS)
#define XCP_TIMESTAMP_UNITS_PER_SECOND          1e9
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_10NS)
#define XCP_TIMESTAMP_UNITS_PER_SECOND          1e8
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_100NS)
#define XCP_TIMESTAMP_UNITS_PER_SECOND          1e7
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_1US)
#define XCP_TIMESTAMP_UNITS_PER_SECOND          1e6
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_10US)
#define XCP_TIMESTAMP_UNITS_PER_SECOND          1e5
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_100US)
#define XCP_TIMESTAMP_UNITS_PER_SECOND          1e4
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_1MS)
#define XCP_TIMESTAMP_UNITS_PER_SECOND          1e3
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_10MS)
#define XCP_TIMESTAMP_UNITS_PER_SECOND          1e2
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_100MS)
#define XCP_TIMESTAMP_UNITS_PER_SECOND          1e1
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_1S)
#define XCP_TIMESTAMP_UNITS_PER_SECOND            1
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_1PS)
#define XCP_TIMESTAMP_UNITS_PER_SECOND         1e12
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_10PS)
#define XCP_TIMESTAMP_UNITS_PER_SECOND         1e11
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_100PS)
#define XCP_TIMESTAMP_UNITS_PER_SECOND         1e10
#else
#error "invalid XCP_TIMESTAMP_UNIT value"
#endif

/* Define the corresponding XCP_TIMESTAMP_UNITS_PER_MICROSECOND value */
#define XCP_TIMESTAMP_UNITS_PER_MICROSECOND (XCP_TIMESTAMP_UNITS_PER_SECOND / 1e6)

static boolean_T xcpTimestampInitialized = false;
static uint64_t  xcpTimestampStartTime = 0;


#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32)

/* Initialize timestamp based on performance counters */
static void xcpTimestampInit(void)
{
    if (!xcpTimestampInitialized) {
        if (!xcpPerfCounterInitialized) {
            /* Initialize performance counters */
            xcpPerfCounterInit();
        }

        if (xcpPerfCounterAvailable) {
            LARGE_INTEGER current;

            /* Initialize xcpTimestampStartTime based on current time */
            QueryPerformanceCounter(&current);

            xcpTimestampStartTime = xcpPerfCounterValue2TimeUnits(current, (uint64_t) XCP_TIMESTAMP_UNITS_PER_SECOND);
        }

        xcpTimestampInitialized = true;
    }
}

/* Get timestamp value in XCP_TIMESTAMP_UNITs
    @note in the current implementation the time starts counting from 0 on the first call */
uint32_T xcpGetTimestamp(void)
{
    LARGE_INTEGER current;
    uint64_t currentTime;
    uint32_T ret = 0;

    if (!xcpTimestampInitialized) {
        xcpTimestampInit();
        return ret;
    }

    if (xcpPerfCounterAvailable) {
        QueryPerformanceCounter(&current);
        currentTime = xcpPerfCounterValue2TimeUnits(current, (uint64_t) XCP_TIMESTAMP_UNITS_PER_SECOND);

        if (currentTime >= xcpTimestampStartTime) {
            ret = (uint32_T) (currentTime - xcpTimestampStartTime);
        } else {
            ret = (uint32_T) (currentTime + UINT64_MAX - xcpTimestampStartTime);
        }
    }

    return ret;
}

#elif defined(__linux__) || defined(__APPLE__) && defined(__MACH__)

/* Return value in XCP_TIMESTAMP_UNITs
   @note in the current implementation the time starts counting from 0 on the first call */
uint32_T xcpGetTimestamp(void)
{
    struct timeval timeVal;
    uint64_t currentTime = 0;
    uint32_T ret = 0;

    gettimeofday(&timeVal, NULL);
    currentTime = timeVal.tv_sec * XCP_TIMESTAMP_UNITS_PER_SECOND + timeVal.tv_usec * XCP_TIMESTAMP_UNITS_PER_MICROSECOND;

    if (!xcpTimestampInitialized) {
        /* Read initial time and store it in xcpTimestampStartTime variable */
        xcpTimestampStartTime = currentTime;

        xcpTimestampInitialized = true;
    }

    if (currentTime >= xcpTimestampStartTime) {
        ret = (uint32_T) (currentTime - xcpTimestampStartTime);
    } else {
        ret = (uint32_T) (currentTime + UINT64_MAX - xcpTimestampStartTime);
    }

    return ret;
}

#else

#error "platform not supported: a custom Platform Abstraction Layer needs to be provided."

#endif

#endif /* XCP_TIMESTAMP_BASED_ON_SIMULATION_TIME */


/* Define the API to convert the XCP address and extension into the corresponding
   variable address in the target address space */
#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32)

#if defined(__MINGW32__) || defined(__MINGW64__) || defined(XCP_PLATFORM_WINDOWS_NO_OFFSET_SUPPORT)

uint8_T *xcpAddressGet(uint8_T addressExtension, uint32_T address)
{
    return (uint8_T*) ((uintptr_t) address | (((uintptr_t)addressExtension) << 32));
}

#else

extern IMAGE_DOS_HEADER __ImageBase;
uint8_T *xcpAddressGet(uint8_T addressExtension, uint32_T address)
{
    return (uint8_T*) (((uintptr_t) address | (((uintptr_t)addressExtension) << 32)) + 
                       (uintptr_t)&__ImageBase);
}

#endif

#elif defined(__linux__)

extern char __executable_start;

#if defined(XCP_PLATFORM_LINUX_PIE_SUPPORT) && defined(XCP_PLATFORM_LINUX_NO_PIE_SUPPORT)
#error "XCP_PLATFORM_LINUX_PIE_SUPPORT and XCP_PLATFORM_LINUX_NO_PIE_SUPPORT cannot be defined at the same time."
#endif

#ifdef XCP_PLATFORM_LINUX_PIE_SUPPORT

/* If the binary is compiled as Position Independent Executable, the address sent by the XCP Client
   is treated as an offset from the __executable_start address */
uint8_T *xcpAddressGet(uint8_T addressExtension, uint32_T address)
{
    return (uint8_T*) (((uintptr_t) address | (((uintptr_t)addressExtension) << 32)) + 
                        (uintptr_t)&__executable_start);
}

#elif defined(XCP_PLATFORM_LINUX_NO_PIE_SUPPORT)

/* If the binary is NOT compiled as Position Independent Executable, the address sent by the XCP Client
   is treated as the actual address of the variable */
uint8_T *xcpAddressGet(uint8_T addressExtension, uint32_T address)
{
    return (uint8_T*) ((uintptr_t) address | (((uintptr_t)addressExtension) << 32));
}

#else

/* if the code is loaded below 32 bits, the default implementation is assuming
   that PIE is not enabled and that address represents the actual address
   in memory where the variable is loaded.

   However if we detect that the code has been loaded beyond that limit,
   then a relocation mechanism must be enabled and therefore we apply
   the same conversion rule as PIE */
uint8_T *xcpAddressGet(uint8_T addressExtension, uint32_T address)
{
    if ((uintptr_t)&__executable_start < (uintptr_t)UINT32_MAX) {
        return (uint8_T*) ((uintptr_t) address | (((uintptr_t)addressExtension) << 32));
    } else {
        return (uint8_T*) (((uintptr_t) address | (((uintptr_t)addressExtension) << 32)) +
                            (uintptr_t)&__executable_start);
    }
}

#endif /*  XCP_PLATFORM_LINUX_PIE_SUPPORT */

#elif defined(__APPLE__) && defined(__MACH__)

#define MAX_EXECUTABLE_PATH_SIZE 1024
#define UNINITIALIZED_ADDRESS ((intptr_t) -1)

/* Get the dynamic image slide of system when the executable is deployed */
uintptr_t xcpExecutableStartAddress(void)
{    
    static intptr_t imgSlideValue = UNINITIALIZED_ADDRESS;
    
    if (imgSlideValue == UNINITIALIZED_ADDRESS) {
        char path[MAX_EXECUTABLE_PATH_SIZE];
        uint32_T size = sizeof(path);
        path[MAX_EXECUTABLE_PATH_SIZE-1] = '\0';
        /* _NSGetExecutablePath returns 0 if the path of the main executable
         * is successfully copied */
        if (_NSGetExecutablePath(path, &size) == 0) {
            uint32_T i = 0;
            for (i = 0; (i < _dyld_image_count()) && (imgSlideValue == UNINITIALIZED_ADDRESS); i++)
            {
             /* If the path of the executable is the same as the indexed image
             that means the image is the ASLR slide */
                if (strcmp(_dyld_get_image_name(i), path) == 0)
                    imgSlideValue = _dyld_get_image_vmaddr_slide(i);
            }
        }
    }
    return (uintptr_t)imgSlideValue;
}

uint8_T *xcpAddressGet(uint8_T addressExtension, uint32_T address)
{
    return (uint8_T*) (((uintptr_t) address | (((uintptr_t)addressExtension) << 32)) + 
                       xcpExecutableStartAddress());
}

#else

#error "platform not supported: a custom Platform Abstraction Layer needs to be provided."

#endif

/* LocalWords:  MSDN
 */
