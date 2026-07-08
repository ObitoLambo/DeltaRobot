/*
* Copyright 2018-2024 The MathWorks, Inc.
*
* File: xcp_ext_common.h
*
* Abstract:
*  This file contains the common declarations and internal functions
*  used within the External Mode Platform Abstraction layer
*/

#ifndef XCP_EXT_COMMON_H
#define XCP_EXT_COMMON_H

#include "ext_mode_types.h"
#include "xcp.h"

#ifndef XCP_UT
#define XCP_STATIC static
#else
#define XCP_STATIC
#endif


/* Global variables directly accessible by the XCP Client */

#if (defined(INTEGER_CODE) && INTEGER_CODE == 1) || defined(XCP_EXTMODE_SIMULATION_TIME_IN_TICKS)

/** Current (absolute) simulation time (in baserate ticks).
   The time value is stored as a uint64_t and it's always in LITTLE ENDIAN format */
extern uint32_T xcpCurrentSimulationTimeInTicks[2];

#else

/** Current (absolute) simulation time (in ms).
   The time value is stored as a uint64_t and it's always in LITTLE ENDIAN format */
extern uint32_T xcpCurrentSimulationTimeInMs[2];

#endif

/** Request the start of the model execution */
extern boolean_T volatile xcpModelStartRequest;

/** Request the stop of the model execution */
extern boolean_T volatile xcpModelStopRequest;

/** Current model status */
typedef enum {
    XCP_EXTMODE_STATUS_RESET,             /*< Initial status */
    XCP_EXTMODE_STATUS_INITIALIZED,       /*< Initialization complete */
    XCP_EXTMODE_STATUS_WAITING_TO_START,  /*< Waiting for request to start */
    XCP_EXTMODE_STATUS_READY_TO_RUN,      /*< Request to start received, ready to run */
    XCP_EXTMODE_STATUS_RUNNING,           /*< Model in execution */
    XCP_EXTMODE_STATUS_PAUSED,            /*< Model execution paused */
    XCP_EXTMODE_STATUS_RESETTING          /*< Reset in progress */
} XcpExtModeStatus;

extern XcpExtModeStatus volatile xcpModelStatus;

/** Model Checksum information */
extern uint32_T volatile xcpModelChecksum0;
extern uint32_T volatile xcpModelChecksum1;
extern uint32_T volatile xcpModelChecksum2;
extern uint32_T volatile xcpModelChecksum3;

/** Support for PurelyIntegerCode */
extern uint32_T volatile xcpModelIntegerCode;

/** External Mode Final Simulation time */
extern extmodeSimulationTime_T volatile xcpExtmodeFinalSimulationTime;

/** External Mode Simulation Complete */
extern boolean_T volatile xcpExtmodeSimulationComplete;


/** Conversion function, from XCP to EXTMODE error code */
#define XCP_TO_EXTMODE_ERROR_CODE(code) ((extmodeErrorCode_T) code)


/** Simulation time not initialized */
#define EXTMODE_SIMULATION_TIME_NOT_INITIALIZED ((extmodeSimulationTime_T)-2)


/** Wait EXTMODE_RETRY_TIME before attempting a new internal operation depending on the state */
#ifndef EXTMODE_RETRY_TIME_IN_MICROSECONDS
#define EXTMODE_RETRY_TIME_IN_MICROSECONDS           10000L  /* 10ms */
#endif

/** Once the model execution has terminated, wait for EXTMODE_SHUTDOWN_TIMEOUT
    before closing the XCP connection */
#ifndef EXTMODE_SHUTDOWN_TIMEOUT_IN_MICROSECONDS
#define EXTMODE_SHUTDOWN_TIMEOUT_IN_MICROSECONDS  60000000L  /* 60s   */
#endif

#ifdef XCP_EXTMODE_RUN_BACKGROUND_FLUSH
/* If the macro is defined, the xcpExtModeRunBackground() will continue to process
   packets until the TX and RX queues are empty or an error occurred. */
#define DEFAULT_XCP_EXTMODE_RUN_BACKGROUND_FLUSH   true
#else
/* If the macro is NOT defined, the xcpExtModeRunBackground() will only carry out one iteration
   and then return. The remaining packets will be processed at the next round. */
#define DEFAULT_XCP_EXTMODE_RUN_BACKGROUND_FLUSH false
#endif


/** Initialize XCP common services part of the External Mode Abstraction Layer */
extern XcpErrorCode xcpExtModeInit(void);


/* Update absolute simulation time variables
   and detect if the simulation is actually complete

   Note: the newTime input represents the current model simulation time */
void xcpExtModeUpdateTime(extmodeSimulationTime_T newTime, extmodeEventId_T eventId);


/* In addition to updating absolute simulation time variables
   and detecting if the simulation is actually complete, 
   the function returns the 32bit XCP Timestamp counter value
   (in XCP_TIMESTAMP_UNITs) corresponding to the given newTime

   Note: the newTime input represents the model simulation time */
uint32_T xcpExtModeGetUpdatedTimestamp(extmodeSimulationTime_T newTime, extmodeEventId_T eventId);


/** Run XCP Transport and Protocol Layer Background activities */
extern XcpErrorCode xcpExtModeRunBackground(boolean_T flushAllData);


/** Reset XCP common services part of the External Mode Abstraction Layer, by restoring the default status */
extern XcpErrorCode xcpExtModeReset(void);


/** Get the current absolute simulation time */
extern uint32_T xcpExtModeGetSimulationTime(void);

/* Definition of a dummy printf function */
void xcp_void_printf(const char_T *fmt,...);

#endif /* XCP_EXT_COMMON_H */
