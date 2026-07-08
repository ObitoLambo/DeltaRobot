/*
 * Copyright 2018-2023 The MathWorks, Inc.
 *
 * File: xcp_ext_common.c
 *
 * Abstract:
 *  This file contains the implementation of the common functionalities
 *  used within the XCP External Mode Platform Abstraction layer.
 *
 *  This includes the handling of absolute simulation time and the global
 *  variables to control the execution of the model on the target
 *  (and accessed remotely via XCP communication protocol).
 */
#if !defined(INTEGER_CODE) || INTEGER_CODE == 0
#include <float.h>         /* for DBL_EPSILON */
#include <math.h>
#endif

#include "xcp_ext_common.h"
#include "xcp_ext_param.h"
#include "xcp.h"
#include "xcp_internal.h"
#include "xcp_transport.h"
#include "xcp_cfg.h"

#include "rtw_extmode.h"

#ifdef XCP_CUSTOM_ADDRESS_TRANSLATION
void xcpInitCustomAddressGet(void);
#endif

#ifndef EXTMODE_FLUSH_ALL_DATA_DELAY_TIME_IN_MICROSECONDS
/* Dummy delay, that forces the background task to pause the execution
   and reduces the packets transmission rate. 
   This is useful for host-based targets to prevent the saturation
   of transmission buffers */
#define EXTMODE_FLUSH_ALL_DATA_DELAY_TIME_IN_MICROSECONDS  10L  /* 10us */
#endif


/* XCP Client can 'directly' access these global variables, in order to control
   the model state machine */

#if defined(ON_TARGET_WAIT_FOR_START) && ON_TARGET_WAIT_FOR_START == 1
boolean_T volatile xcpModelStartRequest = false;
#else
boolean_T volatile xcpModelStartRequest = true;
#endif
boolean_T volatile xcpModelStopRequest = false;
XcpExtModeStatus volatile xcpModelStatus = XCP_EXTMODE_STATUS_RESET;

uint32_T volatile xcpModelChecksum0   = 0;
uint32_T volatile xcpModelChecksum1   = 0;
uint32_T volatile xcpModelChecksum2   = 0;
uint32_T volatile xcpModelChecksum3   = 0;
uint32_T volatile xcpModelIntegerCode = 0;

/* Global variable defined when a non-BYTE addressable target is
   being emulated as a BYTE addressable target. Currently defined 
   only for WORD addressable targets (Eg.:C2000). But, can be
   extended to DWORD addressable targets */
#ifdef XCP_EMULATE_BYTE_ADDRESSABLE_TARGET
uint8_T volatile xcpEmulateWordTargetAsByteTarget = 1;
#endif

#if defined(INTEGER_CODE) && INTEGER_CODE == 1
extmodeSimulationTime_T volatile xcpExtmodeFinalSimulationTime = EXTMODE_SIMULATION_RUN_FOREVER;
#else
extmodeSimulationTime_T volatile xcpExtmodeFinalSimulationTime = EXTMODE_SIMULATION_TIME_NOT_INITIALIZED;
#endif

boolean_T               volatile xcpExtmodeSimulationComplete  = false;

#if !defined(INTEGER_CODE) || INTEGER_CODE == 0
/* Dummy double variable, required to obtain the size of double data type on the target 
   using the symbols parser. */
extmodeDouble_T volatile xcpDummyDoubleVariable = (extmodeDouble_T) 0;
#endif

#ifdef MAX_uint64_T
/* Dummy uint64_T variable, required to obtain the size of uint64_T data type
   on the target  using the symbols parser. */
uint64_T volatile xcpDummyUint64Variable = (uint64_T) 0;
#endif

/* Definition of a dummy printf function */
void xcp_void_printf(const char_T *fmt,...){UNUSED_PARAMETER(fmt);}


#if (defined(INTEGER_CODE) && INTEGER_CODE == 1) || defined(XCP_EXTMODE_SIMULATION_TIME_IN_TICKS)

/* The mutex is used to protect against the concurrent access to the global
   variables (related to the simulation time) when xcpExtModeUpdateTime()
   is executed by different threads.
   In particular it guarantees that the state of the variables is updated
   atomically and it is always consistent when the function is executed */
static XCP_MUTEX_DEFINE(xcpExtModeLock);


uint32_T xcpCurrentSimulationTimeInTicks[2] = {0};

/* The variable contains the last value of model simulation time 
   received via extmodeEvent() for the Base Rate thread.
   In particular, the extmodeSimulationTime_T represents absolute time 
   (in baserate ticks)
   Note: xcpBaseRateSimulationTime is assumed to be monotonically increasing, 
         but a counter overflow could occur */
XCP_STATIC extmodeSimulationTime_T xcpBaseRateSimulationTime = 0;

/* When LifeSpan is short, the xcpBaseRateSimulationTime variable could 
  "overflow earlier than expected", as the generated code may use only 
  a 16bit counter to store the baserate ticks.

  The purpose of xcpTicksCounterL is to convert the xcpBaseRateSimulationTime 
  into a "normal" 32bit variable (and it relies on the EXTMODE_MAX_BASE_RATE_SIMULATION_TIME
  value to understand when the overflow is supposed to occur).

  xcpTicksCounterL and xcpTicksCounterH are then used to obtain 
  the xcpCurrentSimulationTimeInTicks global tick counter */
XCP_STATIC uint32_T xcpTicksCounterL = 0;
XCP_STATIC uint32_T xcpTicksCounterH = 0;

/* The variable contains the last value of simulation time converted into a 32 bit
   value with XCP_TIMESTAMP_UNIT resolution. This variable is only updated when
   base rate is executed */
XCP_STATIC uint32_T xcpBaseRateSimulationTimestamp = 0;

#else /* INTEGER_CODE == 0 */

uint32_T xcpCurrentSimulationTimeInMs[2] = {0};
XCP_STATIC extmodeSimulationTime_T xcpBaseRateSimulationTime = 0;

#endif

#ifndef XCP_BIG_ENDIAN

XCP_STATIC void xcpCopyTimeValue(volatile uint32_T *dst, uint32_T lsb, uint32_T msb)
{
    dst[0] = lsb;
    dst[1] = msb;
}

#else /* BIG ENDIAN*/

#define XCP_BYTES_SWAP(value)    ((((value) >> 24) & 0x000000ff)| \
                                  (((value) >> 8)  & 0x0000ff00)| \
                                  (((value) << 8)  & 0x00ff0000)| \
                                  (((value) << 24) & 0xff000000))

XCP_STATIC void xcpCopyTimeValue(volatile uint32_T *dst, uint32_T lsb, uint32_T msb)
{
    dst[0] = XCP_BYTES_SWAP(lsb);
    dst[1] = XCP_BYTES_SWAP(msb);
}

#endif /* BIG ENDIAN*/


#if (defined(INTEGER_CODE) && INTEGER_CODE == 1) || defined(XCP_EXTMODE_SIMULATION_TIME_IN_TICKS)

/* Define the XCP_TIMESTAMP_UNITS_PER_MICROSECOND value based on the
   selected XCP_TIMESTAMP_UNIT */
#if (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_1US)
#define XCP_TIMESTAMP_UNITS_PER_MICROSECOND     1
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_10US)
#define XCP_TIMESTAMP_UNITS_PER_MICROSECOND     10
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_100US)
#define XCP_TIMESTAMP_UNITS_PER_MICROSECOND     100
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_1MS)
#define XCP_TIMESTAMP_UNITS_PER_MICROSECOND     1000
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_10MS)
#define XCP_TIMESTAMP_UNITS_PER_MICROSECOND     10000
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_100MS)
#define XCP_TIMESTAMP_UNITS_PER_MICROSECOND     100000
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_1S)
#define XCP_TIMESTAMP_UNITS_PER_MICROSECOND     1000000
#else
#error "Invalid XCP_TIMESTAMP_UNIT value. When Purely Integer code is selected, it must be greater than or equal to XCP_TIMESTAMP_UNIT_1US."
#endif


static extmodeSimulationTime_T xcpExtModeMaxBaseRateSimulationTime = EXTMODE_MAX_BASE_RATE_SIMULATION_TIME;


/* Internal function that updates absolute simulation time variables:
   - xcpCurrentSimulationTimeInTicks 
   - xcpBaseRateSimulationTime
   - xcpBaseRateSimulationTimestamp
   and detect if the simulation is actually complete
   (by updating xcpExtmodeSimulationComplete variable)
 
   The function returns the 32bit XCP Timestamp counter value 
   (in XCP_TIMESTAMP_UNITs) corresponding to the given newTime
 
   Note: the newTime input represents the model simulation time in base rate ticks */
static uint32_T updateTime(extmodeSimulationTime_T newTime, extmodeEventId_T eventId)
{
    uint32_T timestamp = 0;

    XCP_MUTEX_LOCK(xcpExtModeLock);
    
    if (eventId == EXTMODE_BASE_RATE_EVENT_ID) {
        /* Absolute time variables are only updated when 
           base rate thread is executed */
        extmodeSimulationTime_T ticksIncrement;
        extmodeSimulationTime_T newCounterTicks;

        /* Detect increment (in ticks) */
        if (newTime >= xcpBaseRateSimulationTime) {
            ticksIncrement = newTime - xcpBaseRateSimulationTime;
        } else {
            ticksIncrement = newTime + EXTMODE_MAX_BASE_RATE_SIMULATION_TIME - xcpBaseRateSimulationTime + 1;
        }
    
        /* Update xcpTicksCounterL and xcpTicksCounterH tick counters */
        newCounterTicks = xcpTicksCounterL + ticksIncrement;
    
        if (newCounterTicks < xcpTicksCounterL) {
            xcpTicksCounterH++;
        }
        xcpTicksCounterL = newCounterTicks;

        /* Update absolute simulation time values */
        xcpCopyTimeValue(xcpCurrentSimulationTimeInTicks, xcpTicksCounterL, xcpTicksCounterH);

        /* Save last received base rate simulation time */
        xcpBaseRateSimulationTime = newTime;
    
        /* Update the 32bit XCP Timestamp counter for the base rate, using 32bit unsigned integer arithmetic */
        xcpBaseRateSimulationTimestamp += ticksIncrement * EXTMODE_STEP_SIZE_IN_MICROSECONDS * XCP_TIMESTAMP_UNITS_PER_MICROSECOND;

        timestamp = xcpBaseRateSimulationTimestamp;
        
        /* Check if simulation is complete */
        if ((xcpExtmodeFinalSimulationTime != EXTMODE_SIMULATION_RUN_FOREVER) &&
            (xcpExtmodeFinalSimulationTime != EXTMODE_SIMULATION_TIME_NOT_INITIALIZED)) {
            /* By default, assuming EXTMODE_SIMULATION_RUN_FOREVER */
            xcpExtmodeSimulationComplete = (xcpTicksCounterL >= xcpExtmodeFinalSimulationTime);
        }
    } else {
        /* For threads different from the base rate, if the execution is
           really concurrent, the simulationTime value might be different 
           from xcpBaseRateSimulationTime (ahead or behind).
           The resulting timestamp value is calculated starting from 
           xcpBaseRateSimulationTimestamp and adding/subtracting the 
           difference */
        extmodeSimulationTime_T ticksIncrement = 0;
        extmodeSimulationTime_T ticksDecrement = 0;
        
        if (xcpExtModeMaxBaseRateSimulationTime < MAX_extmodeSimulationTime_T) {
            /* If the size of extmodeSimulationTime_T data type allows
               the representation of numbers bigger than EXTMODE_MAX_BASE_RATE_SIMULATION_TIME
               then the newTime value needs to be "converted" to just use the same
               amount of bits as the Base Rate (ClockTick0)
               Note: this happens for example when ClockTicks are uint16_T and
               extmodeSimulationTime_T is uint32_T. If the first subrate is half
               of the base rate then
                   newTime = ClockTick1 * 2
               and this time would end up being bigger than EXTMODE_MAX_BASE_RATE_SIMULATION_TIME */
            newTime = newTime % (xcpExtModeMaxBaseRateSimulationTime + 1);
        }

        if (newTime >= xcpBaseRateSimulationTime) {
            /* We need to identify one of the two cases:
               - newTime is ahead of xcpBaseRateSimulationTime
               - newTime is behind but an overflow occurred
             */
            ticksIncrement = newTime - xcpBaseRateSimulationTime;
            ticksDecrement = xcpBaseRateSimulationTime + EXTMODE_MAX_BASE_RATE_SIMULATION_TIME - newTime + 1;

            if (ticksDecrement <= ticksIncrement) {
                /* Assuming that newTime is behind and an overflow has occurred */
                ticksIncrement = 0;
            } else {
            	/* assuming that newTime is ahead */
                ticksDecrement = 0;
            }
        } else {
            /* We need to identify one of the two cases:
               - newTime is behind of xcpBaseRateSimulationTime
               - newTime is ahead but an overflow occurred
             */
            ticksIncrement = newTime + EXTMODE_MAX_BASE_RATE_SIMULATION_TIME - xcpBaseRateSimulationTime + 1;
            ticksDecrement = xcpBaseRateSimulationTime - newTime;

            if (ticksDecrement <= ticksIncrement) {
                /* Assuming that newTime is behind, as more likely */
                ticksIncrement = 0;
            } else {
            	/* assuming that newTime is ahead and an overflow has occurred */
                ticksDecrement = 0;
            }
        }

        if (ticksIncrement > 0) {
            timestamp = xcpBaseRateSimulationTimestamp +
                        (ticksIncrement * EXTMODE_STEP_SIZE_IN_MICROSECONDS * XCP_TIMESTAMP_UNITS_PER_MICROSECOND);
        } else {
            timestamp = xcpBaseRateSimulationTimestamp -
                        (ticksDecrement * EXTMODE_STEP_SIZE_IN_MICROSECONDS * XCP_TIMESTAMP_UNITS_PER_MICROSECOND);
        }
    }

    XCP_MUTEX_UNLOCK(xcpExtModeLock);
    
    return timestamp;
}

/* Update absolute simulation time variables
   and detect if the simulation is actually complete

   Note: the newTime input represents the current model simulation time */
void xcpExtModeUpdateTime(extmodeSimulationTime_T newTime, extmodeEventId_T eventId)
{
    updateTime(newTime, eventId);
}

/* In addition to updating absolute simulation time variables
   and detecting if the simulation is actually complete, 
   the function returns the 32bit XCP Timestamp counter value
   (in XCP_TIMESTAMP_UNITs) corresponding to the given newTime

   Note: the newTime input represents the model simulation time */
uint32_T xcpExtModeGetUpdatedTimestamp(extmodeSimulationTime_T newTime, extmodeEventId_T eventId)
{
    return updateTime(newTime, eventId);
}


#ifdef XCP_TIMESTAMP_BASED_ON_SIMULATION_TIME

/* Return value in XCP_TIMESTAMP_UNITs, based on the current model simulation time */
uint32_T xcpGetTimestamp(void)
{
    uint32_T timestamp;

    /* @note: theoretically the base thread could pre-preempt any thread during the
       read operation. Since the read of a 32bit value may not be atomic
       in some architectures we have to protect it with a mutex.
       We may be able to remove this lock if we extend the Platform Abstraction Layer
       to include atomic read operations */
    XCP_MUTEX_LOCK(xcpExtModeLock);

    timestamp = xcpBaseRateSimulationTimestamp;

    XCP_MUTEX_UNLOCK(xcpExtModeLock);

    return timestamp;
}

#endif /* XCP_TIMESTAMP_BASED_ON_SIMULATION_TIME */

#else /* INTEGER_CODE == 0 */

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


/* Update absolute simulation time variable xcpCurrentSimulationTimeInMs
   and detect if the simulation is actually complete
   (by updating xcpExtmodeSimulationComplete variable)

   The function returns the 32bit XCP Timestamp counter value
   (in XCP_TIMESTAMP_UNITs) corresponding to the given newTime

   Note: the newTime input represents the model simulation time in seconds */
void xcpExtModeUpdateTime(extmodeSimulationTime_T newTime, extmodeEventId_T eventId)
{
    if (eventId == EXTMODE_BASE_RATE_EVENT_ID) {
        /* Absolute time variables are only updated when base rate
           thread is executed, no need to use any lock */
        real_T   timeInMs = newTime * 1000.0;
        uint32_T timeInMsL = (uint32_T)fmod(timeInMs, (extmodeSimulationTime_T) XCP_UINT32_MAX + 1);
        uint32_T timeInMsH = 0;

        if (timeInMs > XCP_UINT32_MAX) {
            /* cast truncates, rounding to zero */
            timeInMsH = (uint32_T)(timeInMs / XCP_UINT32_MAX);
        }

        /* Update absolute simulation time variables */
        xcpCopyTimeValue(xcpCurrentSimulationTimeInMs, timeInMsL, timeInMsH);

        /* Check if simulation is complete */
        if ((xcpExtmodeFinalSimulationTime != EXTMODE_SIMULATION_RUN_FOREVER) &&
            (xcpExtmodeFinalSimulationTime != EXTMODE_SIMULATION_TIME_NOT_INITIALIZED)) {
            /* By default, assuming EXTMODE_SIMULATION_RUN_FOREVER */
           xcpExtmodeSimulationComplete = (xcpExtmodeFinalSimulationTime - newTime) <
                                          (newTime * (DBL_EPSILON));
        }

        /* Update the current simulation time */
        xcpBaseRateSimulationTime = newTime;
    }
}

/* In addition to updating absolute simulation time variables
   and detecting if the simulation is actually complete, 
   the function returns the 32bit XCP Timestamp counter value
   (in XCP_TIMESTAMP_UNITs) corresponding to the given newTime

   Note: the newTime input represents the model simulation time */
uint32_T xcpExtModeGetUpdatedTimestamp(extmodeSimulationTime_T newTime, extmodeEventId_T eventId)
{
    uint32_T timestamp = 0;

    xcpExtModeUpdateTime(newTime, eventId);

    /* Convert the newTime into the 32 bits XCP Timestamp counter, based on XCP configuration */
    timestamp = (uint32_T)fmod(newTime * (extmodeSimulationTime_T) XCP_TIMESTAMP_UNITS_PER_SECOND + 0.5, 
                              (extmodeSimulationTime_T)XCP_UINT32_MAX + 1);

    return timestamp;
}


#ifdef XCP_TIMESTAMP_BASED_ON_SIMULATION_TIME

/* This function is supposed to return the value of the XCP Timestamp in XCP_TIMESTAMP_UNITs.
   Specifically it is invoked within the xcpEvent().
   When XCP_TIMESTAMP_BASED_ON_SIMULATION_TIME is defined, both extmodeEvent() and 
   rtExtModeUpload() use xcpEventExternalTimestamp() instead, so we should never hit this function.
   However we need to provide a dummy implementation as the function is required by the 
   Platform Abstraction Layer. */
uint32_T xcpGetTimestamp(void)
{
    return 0;
}

#endif /* XCP_TIMESTAMP_BASED_ON_SIMULATION_TIME */

#endif /* INTEGER_CODE == 1 */


uint32_T xcpExtModeGetSimulationTime(void)
{  
#ifdef XCP_TIMESTAMP_BASED_ON_SIMULATION_TIME
    /* Even if not thread-safe, this implementation has been added for backward compatibility 
       as Code Execution Profiling was invoking the xcpEvent() API directly */
    return xcpExtModeGetUpdatedTimestamp(xcpBaseRateSimulationTime, EXTMODE_BASE_RATE_EVENT_ID);
#else
    return 0;
#endif
}


XcpErrorCode xcpExtModeInit(void)
{
    int_T parNumber = 0;
    void **parList = NULL;
    XcpErrorCode errorCode = XCP_SUCCESS;
    
#if !defined(INTEGER_CODE) || INTEGER_CODE == 0
    /* Use dummy variable to prevent optimization in some compilers */
    xcpDummyDoubleVariable = (extmodeDouble_T) 0;
#endif

#ifdef MAX_uint64_T
    xcpDummyUint64Variable = (uint64_T) 0;
#endif

#if defined(INTEGER_CODE) && INTEGER_CODE == 1
    XCP_MUTEX_INIT(xcpExtModeLock);
#endif

#ifdef XCP_CUSTOM_ADDRESS_TRANSLATION
    xcpInitCustomAddressGet();
#endif

    /* Retrieve XCP Transport Layer initialization parameters */
    xcpTransportGetInitParameters(&parNumber, &parList);

    /* Initialize XCP Transport Layer */
    errorCode = xcpTransportInit(parNumber, parList);
    if (errorCode != XCP_SUCCESS) {
        XCP_PRINTF("xcpExtModeInit: xcpTransportInit error\n");
        return errorCode;
    }

    /* Retrieve XCP Protocol Layer initialization parameters */
    xcpGetInitParameters(&parNumber, &parList);

    /* Initialize XCP Protocol layer */
    errorCode = xcpInit(parNumber, parList);
    if (errorCode != XCP_SUCCESS) {
        XCP_PRINTF("xcpExtModeInit: xcpInit error: code %d\n", errorCode);
    }

    return errorCode;
}


XcpErrorCode xcpExtModeRunBackground(boolean_T flushAllData)
{
    XcpErrorCode errorCode = XCP_SUCCESS;
    boolean_T done = false;

    /* Add a dummy delay, that forces the task to pause the execution
       (leaving the 'running' state). This gives Simulink the opportunity
       to run when connecting to host-based targets in External Mode */
    XCP_SLEEP(0, 0);

    do {
        boolean_T dataAvailable = false;

        /* Retrieve a new Rx packet from interface */
        errorCode = xcpTransportRx();

        if ((errorCode != XCP_SUCCESS) && (errorCode != XCP_EMPTY)) {
            XCP_PRINTF("xcpExtModeRunBackground: xcpTransportRx error, code %d\n", errorCode);
            done = true;
        } else {
            dataAvailable = dataAvailable || (errorCode != XCP_EMPTY);
        }

        if (!done) {
            /* Process it within the protocol layer.
               By checking the return code value, diagnostic info on
               XCP packet processing can be collected.
               Every run corresponds to the processing of one single XCP packet */
            errorCode = xcpRun();

            if ((errorCode != XCP_SUCCESS) && (errorCode != XCP_EMPTY)) {
                XCP_PRINTF("xcpExtModeRunBackground: xcpRun error, code %d\n", errorCode);
                done = true;
            } else {
                dataAvailable = dataAvailable || (errorCode != XCP_EMPTY);
            }
        }

        if (!done) {
            /* Send one of the pending Tx packets over the network */
            errorCode = xcpTransportTx();
            if ((errorCode != XCP_SUCCESS) && (errorCode != XCP_EMPTY)) {
                XCP_PRINTF("xcpExtModeRunBackground: xcpTransportTx error, code %d\n", errorCode);
                done = true;
            } else {
                dataAvailable = dataAvailable || (errorCode != XCP_EMPTY);
            }
        }

        if (flushAllData) {
#if EXTMODE_FLUSH_ALL_DATA_DELAY_TIME_IN_MICROSECONDS > 0
            /* Dummy delay, that forces the background task to pause the execution
               and reduces the packets transmission rate, for host-based targets. */
            XCP_SLEEP(0, EXTMODE_FLUSH_ALL_DATA_DELAY_TIME_IN_MICROSECONDS);
#endif
            /* The xcpExtModeRunBackground will continue to process packets until
               the TX and RX queues are empty or an error occurred. */
            done = done || !dataAvailable;
        } else {
            /* The xcpExtModeRunBackground will only carry out one iteration and then return.
               The remaining packets will be processed at the next round. */
            done = true;
        }
    } while (!done);

    return errorCode;
}


XcpErrorCode xcpExtModeReset(void)
{
    XcpErrorCode errorCode = XCP_SUCCESS;
    extmodeRealTime_T elapsedTime;
    XcpStatus status;

    if (xcpExtmodeSimulationComplete) {
        /* When DAQ Packed Mode is enabled, if we reached the nominal end of the
           simulation we trigger the transmission of a packet containing the samples 
           received so far. The trailing '0's will be removed and not displayed
           in the output of the simulation */
        xcpPackedModeEventsFlush(0);
    }

    /* Wait for the XCP client to complete the clean disconnection procedure */
    status = xcpGetStatus();

    elapsedTime = 0;
    while ((elapsedTime < EXTMODE_SHUTDOWN_TIMEOUT_IN_MICROSECONDS) &&
           (status != XCP_DISCONNECTED)) {

        XCP_SLEEP(0, EXTMODE_RETRY_TIME_IN_MICROSECONDS);
        elapsedTime += EXTMODE_RETRY_TIME_IN_MICROSECONDS;

        /* Run the XCP Stack to keep the communication 'alive',
           forcing the flush of all data */
        xcpExtModeRunBackground(true);

        /* Check the XCP stack status again */
        status = xcpGetStatus();
    }

    /* Reset XCP Protocol Layer */
    errorCode = xcpReset();
    if (errorCode != XCP_SUCCESS) {
        XCP_PRINTF("xcpReset error: code %d\n", errorCode);
    }

    /* Reset XCP Transport Layer */
    errorCode = xcpTransportReset();
    if (errorCode != XCP_SUCCESS) {
        XCP_PRINTF("xcpTransportReset error: code %d\n", errorCode);
    }

    /* Restore the global variables to the default value */
#if defined(ON_TARGET_WAIT_FOR_START) && ON_TARGET_WAIT_FOR_START == 1
    xcpModelStartRequest = false;
#else
    xcpModelStartRequest = true;
#endif

    xcpModelStopRequest = false;

    xcpModelChecksum0   = 0;
    xcpModelChecksum1   = 0;
    xcpModelChecksum2   = 0;
    xcpModelChecksum3   = 0;
    xcpModelIntegerCode = 0;

#if defined(INTEGER_CODE) && INTEGER_CODE == 1
    xcpExtmodeFinalSimulationTime = EXTMODE_SIMULATION_RUN_FOREVER;

    xcpBaseRateSimulationTime = 0;

    xcpTicksCounterL = 0;
    xcpTicksCounterH = 0;

    xcpBaseRateSimulationTimestamp = 0;
#else
    xcpExtmodeFinalSimulationTime = EXTMODE_SIMULATION_TIME_NOT_INITIALIZED;
#endif

    xcpExtmodeSimulationComplete  = false;

#if (defined(INTEGER_CODE) && INTEGER_CODE == 1) || defined(XCP_EXTMODE_SIMULATION_TIME_IN_TICKS)
    XCP_MEMSET(xcpCurrentSimulationTimeInTicks, 0, sizeof(xcpCurrentSimulationTimeInTicks));

    xcpBaseRateSimulationTime = 0;

    xcpTicksCounterL = 0;
    xcpTicksCounterH = 0;

    xcpBaseRateSimulationTimestamp = 0;
#else
    XCP_MEMSET(xcpCurrentSimulationTimeInMs, 0, sizeof(xcpCurrentSimulationTimeInMs));
    xcpBaseRateSimulationTime = 0;
#endif

    return errorCode;
}
