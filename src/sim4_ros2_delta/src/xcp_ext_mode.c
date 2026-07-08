/*
 * Copyright 2017-2024 The MathWorks, Inc.
 *
 * File: xcp_ext_mode.c
 *
 * Abstract:
 *  The file provides the implementation of the ext_mode.h interface
 *  based on XCP communication protocol
 */

#if !defined(EXTMODE_DISABLE_ARGS_PROCESSING)
#include <stdio.h>
#endif

#include <string.h>        /* optional for strcmp */

#include "xcp_ext_common.h"
#include "xcp_ext_param.h"
#include "xcp_internal.h"
#include "ext_mode.h"


#ifdef EXTMODE_XCP_TRIGGER_SUPPORT

#include "xcp_ext_classic_trigger.h"

#ifndef EXTMODE_XCP_MAX_TRIGGER_NUMBER
#if defined(XCP_MEM_DAQ_RESERVED_POOLS_NUMBER) && (XCP_MEM_DAQ_RESERVED_POOLS_NUMBER > 0)
/* If XCP_MEM_DAQ_RESERVED_POOLS_NUMBER is defined, the value of this macro should be at least
   equal to the number of sample times in the model. This can be one more than the number of DAQ
   lists that use reserved pools if Tid 0 and 1 are mapped to the same DAQ list. */
#define EXTMODE_XCP_MAX_TRIGGER_NUMBER XCP_MEM_DAQ_RESERVED_POOLS_NUMBER + 1
#else
#define EXTMODE_XCP_MAX_TRIGGER_NUMBER EXTMODE_MAX_EVENT_ID
#endif
#endif /* EXTMODE_XCP_MAX_TRIGGER_NUMBER */

#if (EXTMODE_XCP_MAX_TRIGGER_NUMBER > EXTMODE_MAX_EVENT_ID) || (EXTMODE_XCP_MAX_TRIGGER_NUMBER < 1)
#error Invalid EXTMODE_XCP_MAX_TRIGGER_NUMBER value.
#endif

static extmodeEventTriggerEnable xcpCustomTriggerFunction[EXTMODE_XCP_MAX_TRIGGER_NUMBER];

#endif


extmodeErrorCode_T extmodeParseArgs(int_T   argc,
                                    const char_T *argv[])
{
    extmodeErrorCode_T errorCode = EXTMODE_SUCCESS;

#ifdef  EXTMODE_DISABLE_ARGS_PROCESSING
    /* Some targets do not support command line args */
    UNUSED_PARAMETER(argc);
    UNUSED_PARAMETER(argv);
#else
    boolean_T  parseError = false;
    int_T  count = 1;

    if (argc < 0) {
        XCP_PRINTF("extmodeParseArgs: argc must be a positive number\n");
        return EXTMODE_INV_ARG;
    }

    if ((argc > 0) && (argv == NULL)) {
        XCP_PRINTF("extmodeParseArgs: invalid argv value\n");
        return EXTMODE_INV_ARG;
    }

    /*
     * Parse the External Mode Platform-independent Abstraction Layer parameters.
     * Let all unrecognized parameters pass through to the XCP External Mode 
     * Platform-specific Abstraction Layer.
     * NULL out all args handled so that they can ignored by the lower layer.
     */
    while (count < argc) {
        const char_T *option = argv[count++];

        if (option != NULL) {
            /* final time */
            if ((strcmp(option, "-tf") == 0) && (count != argc)) {
                const char_T *timeValueString = argv[count++];
                extmodeSimulationTime_T  timeValue = 0;
                char_T        stringBuffer[201];

                XCP_MEMSET(stringBuffer, 0, sizeof(stringBuffer));

                sscanf(timeValueString, "%200s", stringBuffer);
                if (strcmp(stringBuffer, "inf") == 0) {
                    timeValue = EXTMODE_SIMULATION_RUN_FOREVER;
                }
                else {
                    char_T tmpString[2];

#if (defined(INTEGER_CODE) && INTEGER_CODE == 1) || defined(XCP_EXTMODE_SIMULATION_TIME_IN_TICKS)
                    int tmpValue;

                    if ((sscanf(stringBuffer, "%d%1s", &tmpValue, tmpString) != 1) ||
                        (tmpValue < 0)) {
                        XCP_PRINTF("External mode final simulation time must be a positive, integer value or inf\n");
                        parseError = true;
                        break;
                    }
                    timeValue = (extmodeSimulationTime_T) tmpValue;
#else
                    if ((sscanf(stringBuffer, "%lf%1s", &timeValue, tmpString) != 1) ||
                        (timeValue < (extmodeSimulationTime_T)0)) {
                        XCP_PRINTF("External mode final simulation time must be a positive, real value or inf\n");
                        parseError = true;
                        break;
                    }
#endif
                }
                xcpExtmodeFinalSimulationTime = timeValue;

                argv[count - 2] = NULL;
                argv[count - 1] = NULL;
            }
            /* -w (wait for packet from host) option */
            else if (strcmp(option, "-w") == 0) {
                xcpModelStartRequest = false;
                argv[count - 1] = NULL;
            }
        }
    }

    if (parseError) {
        XCP_PRINTF("\nUsage: model_name -option1 val1 -option2 val2 -option3 "
            "...\n\n");

#if (defined(INTEGER_CODE) && INTEGER_CODE == 1) || defined(XCP_EXTMODE_SIMULATION_TIME_IN_TICKS)
        XCP_PRINTF("\t-tf 20 - sets final time to 20 ticks\n");
#else
        XCP_PRINTF("\t-tf 20 - sets final time to 20 seconds\n");
#endif

        return EXTMODE_INV_ARG;
    }

    /* Extract XCP specific initialization parameters, by invoking the 
       XCP External Mode Platform-specific Abstraction Layer API */
    xcpExtModeParseArgs(argc, argv);

    /*
     * Check for unprocessed ("unhandled") args.
     */
    {
        int i;
        for (i = 1; i<argc; i++) {
            if (argv[i] != NULL) {
                XCP_PRINTF("Unexpected command line argument: %s\n", argv[i]);
                return EXTMODE_INV_ARG;
            }
        }
    }

#endif /* EXTMODE_DISABLE_ARGS_PROCESSING */

    return errorCode;
}


extmodeErrorCode_T extmodeInit(RTWExtModeInfo *extmodeInfo, extmodeSimulationTime_T *finalTime)
{
    XcpErrorCode errorCode = XCP_SUCCESS;

    if (extmodeInfo == NULL) {
        XCP_PRINTF("extmodeInit: invalid extmodeInfo\n");
        return EXTMODE_INV_ARG;
    }

    if (finalTime == NULL) {
        XCP_PRINTF("extmodeInit: invalid finalTime variable\n");
        return EXTMODE_INV_ARG;
    }

    if (xcpExtmodeFinalSimulationTime == EXTMODE_SIMULATION_TIME_NOT_INITIALIZED) {
#if (defined(INTEGER_CODE) && INTEGER_CODE == 1)
        /* If the model final simulation time in External Mode has NOT been initialized,
           then EXTMODE_SIMULATION_RUN_FOREVER is assumed, as the Real-time Model
           structure doesn't contain any final time information when PurelyIntegerCode
           is selected */
        xcpExtmodeFinalSimulationTime = EXTMODE_SIMULATION_RUN_FOREVER;
#else
        /* If the model final simulation time in External Mode has NOT been initialized,
           then the finalTime argument is considered an INPUT */
        xcpExtmodeFinalSimulationTime = *finalTime;
#endif
    }
    else {
        /* If the model final simulation time in External Mode has been initialized
           (e.g. via '-tf ' parameter detected by extmodeParseArgs() or
            via explicit call of extmodeSetFinalSimulationTime())
           then the finalTime argument is considered an OUTPUT */
        *finalTime = xcpExtmodeFinalSimulationTime;
    }

    if (xcpExtmodeFinalSimulationTime == EXTMODE_SIMULATION_RUN_FOREVER) {
        XCP_PRINTF("\n**warning: the simulation will run with no stop time due "
            "to external mode infinite final simulation time.\n");
    }

    /* Initialize the model checksum information */
    xcpModelChecksum0 = rteiGetChecksum0(extmodeInfo);
    xcpModelChecksum1 = rteiGetChecksum1(extmodeInfo);
    xcpModelChecksum2 = rteiGetChecksum2(extmodeInfo);
    xcpModelChecksum3 = rteiGetChecksum3(extmodeInfo);

#if defined(INTEGER_CODE) && INTEGER_CODE == 1
    xcpModelIntegerCode = 1;
#else
    xcpModelIntegerCode = 0;
#endif

#ifdef EXTMODE_XCP_TRIGGER_SUPPORT
    xcpExtModeClassicTriggerInit();

    /* By default, use Classic Triggering */
    {
        unsigned i = 0;
        for (i = 0; i < EXTMODE_XCP_MAX_TRIGGER_NUMBER; i++) {
            xcpCustomTriggerFunction[i] = xcpExtModeClassicTriggerEnabled;
        }
    }
#endif

    /* Initialize the platform abstraction layer common services */
    errorCode = xcpExtModeInit();
    
    if (errorCode == XCP_SUCCESS) {
        xcpModelStatus = XCP_EXTMODE_STATUS_INITIALIZED;
    }

    return XCP_TO_EXTMODE_ERROR_CODE(errorCode);
}


extmodeErrorCode_T extmodeWaitForHostRequest(extmodeRealTime_T timeoutInMicroseconds)
{
    extmodeErrorCode_T errorCode = EXTMODE_SUCCESS;
    extmodeRealTime_T elapsedTime = 0;
    boolean_T timeoutExpired = false;
    boolean_T waitForHostRequest = !xcpModelStartRequest;
    xcpModelStatus = XCP_EXTMODE_STATUS_WAITING_TO_START;

    timeoutExpired = (timeoutInMicroseconds == 0) && !xcpModelStartRequest && !xcpModelStopRequest;

    /*
     * Pause until the XCP client modifies the xcpModelStartRequest value
     * or a stop request has been issued
     */
    while (!xcpModelStartRequest && !xcpModelStopRequest && !timeoutExpired) {
        XCP_SLEEP(0L, EXTMODE_RETRY_TIME_IN_MICROSECONDS);
        elapsedTime += EXTMODE_RETRY_TIME_IN_MICROSECONDS;

        timeoutExpired = (timeoutInMicroseconds != EXTMODE_WAIT_FOREVER) &&
                         (elapsedTime > timeoutInMicroseconds);

        xcpExtModeRunBackground(DEFAULT_XCP_EXTMODE_RUN_BACKGROUND_FLUSH);
    }

    xcpModelStatus = XCP_EXTMODE_STATUS_READY_TO_RUN;

    if (timeoutExpired) {
        errorCode = EXTMODE_TIMEOUT_ERROR;
    }

#ifdef EXTMODE_XCP_TRIGGER_SUPPORT
    /* Avoid missing logged data points at t = 0 in concurrent execution workflows. */
    if (waitForHostRequest) {
        xcpExtModeClassicTriggerForceEnableStatusUpdate();
    }
#endif
    return errorCode;
}


extmodeErrorCode_T extmodeEvent(extmodeEventId_T eventId, extmodeSimulationTime_T simulationTime)
{
    XcpErrorCode errorCode  = XCP_SUCCESS;
    boolean_T triggerEnable = true;

#if (!defined(INTEGER_CODE) || (INTEGER_CODE == 0)) && (!defined(XCP_EXTMODE_SIMULATION_TIME_IN_TICKS))
    if (simulationTime < 0) {
        XCP_PRINTF("extmodeEvent error: invalid simulationTime, must be non-negative\n");
        return EXTMODE_INV_ARG;
    }
#endif

    if (eventId > EXTMODE_MAX_EVENT_ID) {
        XCP_PRINTF("extmodeEvent error: invalid eventId (%d), must be less than %d\n", eventId, EXTMODE_MAX_EVENT_ID);
        return EXTMODE_INV_ARG;
    }
    
    if ((xcpModelStatus == XCP_EXTMODE_STATUS_READY_TO_RUN) ||
        (xcpModelStatus == XCP_EXTMODE_STATUS_INITIALIZED)) {
        /* Update model status, as soon as the extmodeEvent() gets invoked */
        xcpModelStatus = XCP_EXTMODE_STATUS_RUNNING;
    }

#ifdef EXTMODE_XCP_TRIGGER_SUPPORT
    if ((eventId < EXTMODE_XCP_MAX_TRIGGER_NUMBER) &&
        (xcpCustomTriggerFunction[eventId] != NULL)) {
        /* Trigger logic should be applied only when the synchronous
           data transfer is active */
        XcpStatus xcpStatus = xcpGetStatus();
        if (xcpStatus == XCP_SYNC_DATA_TRANSFER) {
            triggerEnable = xcpCustomTriggerFunction[eventId](eventId);
        }
    }
#endif
    {
        XcpEventIdType xcpEventId = (XcpEventIdType) eventId;

#ifdef XCP_TIMESTAMP_BASED_ON_SIMULATION_TIME
        /* Update local absolute time variables*/
        uint32_T timestampBasedOnSimTime = xcpExtModeGetUpdatedTimestamp(simulationTime, eventId);
#else
        /* Update local absolute time variables*/
        xcpExtModeUpdateTime(simulationTime, eventId);
#endif

        if (triggerEnable) {
#ifdef XCP_TIMESTAMP_BASED_ON_SIMULATION_TIME
            /* Notify XCP Stack about the eventId */
            errorCode = xcpEventExternalTimestamp(xcpEventId, timestampBasedOnSimTime);
#else
            /* Notify XCP Stack about the eventId, using the timestamp read
               from the HW timer supported by XCP Platform Abstraction layer */
            errorCode = xcpEvent(xcpEventId);
#endif
        } else {
            /* reset pending packed DAQ lists if the trigger did not fire so that we do not send
               packets with non-consecutive samples */
            xcpPackedModeEventReset(xcpEventId);
        }
    }

    if (errorCode != XCP_SUCCESS) {
        XCP_PRINTF("extmodeEvent error: code %d\n", errorCode);
    }

    return XCP_TO_EXTMODE_ERROR_CODE(errorCode);
}


extmodeErrorCode_T extmodeBackgroundRun(void)
{
    XcpErrorCode errorCode = XCP_SUCCESS;

    errorCode = xcpExtModeRunBackground(DEFAULT_XCP_EXTMODE_RUN_BACKGROUND_FLUSH);

    return XCP_TO_EXTMODE_ERROR_CODE(errorCode);
}


extmodeErrorCode_T extmodeReset(void)
{
    XcpErrorCode errorCode = XCP_SUCCESS;

    xcpModelStatus = XCP_EXTMODE_STATUS_RESETTING;
    
    errorCode = xcpExtModeReset();

#ifdef EXTMODE_XCP_TRIGGER_SUPPORT
    XCP_MEMSET(xcpCustomTriggerFunction, 0, sizeof(xcpCustomTriggerFunction));

    xcpExtModeClassicTriggerReset();
#endif

    xcpModelStatus = XCP_EXTMODE_STATUS_RESET;
    
    return XCP_TO_EXTMODE_ERROR_CODE(errorCode);
}

boolean_T extmodeStopRequested(void)
{
    return xcpModelStopRequest;
}

boolean_T extmodeStartRequested(void)
{
    return xcpModelStartRequest;
}


boolean_T extmodeSimulationComplete(void)
{
    return xcpExtmodeSimulationComplete;
}

extmodeErrorCode_T extmodeGetFinalSimulationTime(extmodeSimulationTime_T *finalTime)
{
    if (finalTime == NULL) {
        XCP_PRINTF("extmodeGetFinalSimulationTime: invalid finalTime variable\n");
        return EXTMODE_INV_ARG;
    }

    if (xcpExtmodeFinalSimulationTime == EXTMODE_SIMULATION_TIME_NOT_INITIALIZED) {
        XCP_PRINTF("extmodeGetFinalSimulationTime: finalTime not initialized\n");
        return EXTMODE_NOT_INITIALIZED;
    }

    *finalTime = xcpExtmodeFinalSimulationTime;

    return EXTMODE_SUCCESS;
}

extmodeErrorCode_T extmodeSetFinalSimulationTime(extmodeSimulationTime_T finalTime)
{
#if (!defined(INTEGER_CODE) || (INTEGER_CODE == 0)) && (!defined(XCP_EXTMODE_SIMULATION_TIME_IN_TICKS))
    if ((finalTime < 0) && (finalTime != EXTMODE_SIMULATION_RUN_FOREVER)) {
        XCP_PRINTF("extmodeSetFinalSimulationTime: invalid finalTime value\n");
        return EXTMODE_INV_ARG;
    }
#endif

    xcpExtmodeFinalSimulationTime = finalTime;

    return EXTMODE_SUCCESS;
}

#ifdef EXTMODE_XCP_TRIGGER_SUPPORT

extmodeErrorCode_T extmodeGetEventTriggerEnable(extmodeEventId_T eventId, extmodeEventTriggerEnable *triggerEnable)
{
    if (eventId >= EXTMODE_XCP_MAX_TRIGGER_NUMBER) {
        XCP_PRINTF("extmodeGetEventTriggerEnable: invalid eventId value\n");
        return EXTMODE_INV_ARG;
    }

    if (triggerEnable == NULL) {
        XCP_PRINTF("extmodeGetEventTriggerEnable: invalid triggerEnabled value\n");
        return EXTMODE_INV_ARG;
    }

    *triggerEnable = xcpCustomTriggerFunction[eventId];

    return EXTMODE_SUCCESS;
}

extmodeErrorCode_T extmodeSetEventTriggerEnable(extmodeEventId_T eventId, extmodeEventTriggerEnable triggerEnable)
{
    if (eventId >= EXTMODE_XCP_MAX_TRIGGER_NUMBER) {
        XCP_PRINTF("extmodeSetEventTriggerEnable: invalid eventId value\n");
        return EXTMODE_INV_ARG;
    }

    xcpCustomTriggerFunction[eventId] = triggerEnable;
    return EXTMODE_SUCCESS;
}

#else

extmodeErrorCode_T extmodeGetEventTriggerEnable(extmodeEventId_T eventId, extmodeEventTriggerEnable *triggerEnable)
{
    UNUSED_PARAMETER(eventId);

    if (triggerEnable != NULL) {
        *triggerEnable = NULL;
    }

    return EXTMODE_SUCCESS;
}

extmodeErrorCode_T extmodeSetEventTriggerEnable(extmodeEventId_T eventId, extmodeEventTriggerEnable triggerEnable)
{
    UNUSED_PARAMETER(eventId);
    UNUSED_PARAMETER(triggerEnable);

    return EXTMODE_SUCCESS;
}

#endif
