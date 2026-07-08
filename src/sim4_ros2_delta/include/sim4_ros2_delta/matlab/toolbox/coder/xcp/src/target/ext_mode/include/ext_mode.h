/*
 * Copyright 2017-2020 The MathWorks, Inc.
 *
 * File: ext_mode.h     
 *
 * Abstract:
 *   External Mode Abstraction Layer
 */

#ifndef EXT_MODE_H
#define EXT_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ext_mode_types.h"
#include "rtw_extmode.h"
    

/** External Mode error codes */
typedef enum {
    EXTMODE_SUCCESS =                0, /**< No error detected */
    EXTMODE_INV_ARG =               -1, /**< Arguments invalid */
    EXTMODE_EMPTY =                 -2, /**< No data available to carry out the given operation */
    EXTMODE_FULL =                  -3, /**< Resource full detected (e.g. transmission buffer) */
    EXTMODE_PKT_OUT_OF_SEQUENCE =   -4, /**< Out-of-sequence packet detected by external mode communication protocol */
    EXTMODE_PKT_LOST =              -5, /**< Packet loss detected by external mode communication protocol */
    EXTMODE_BUSY =                  -6, /**< Resource busy detected, try later */
    EXTMODE_INV_MSG_FORMAT =        -7, /**< Invalid message format detected by external mode communication protocol */
    EXTMODE_INV_SIZE =              -8, /**< Invalid size detected by external mode communication protocol */
    EXTMODE_NOT_INITIALIZED =       -9, /**< External mode not initialized yet */
    EXTMODE_NO_MEMORY =            -10, /**< No memory available on the target hardware */
    EXTMODE_NOT_SUPPORTED =        -11, /**< Operation not supported */
    EXTMODE_ERROR =                -12, /**< External mode generic error detected */
    EXTMODE_PKT_CHECKSUM_ERROR =   -13, /**< Checksum inconsistency detected by external mode communication protocol */
    EXTMODE_PKT_RX_TIMEOUT_ERROR = -14, /**< Timeout error detected during the reception of a packet */
    EXTMODE_PKT_TX_TIMEOUT_ERROR = -15, /**< Timeout error detected during the transmission of a packet */
    EXTMODE_TIMEOUT_ERROR =       -100  /**< External mode timeout error detected */
}   extmodeErrorCode_T;

/** Wait forever (infinite timeout value) */
#define  EXTMODE_WAIT_FOREVER  ((extmodeRealTime_T)-1)


/** Extract values of configuration parameters supported by external mode abstraction layer.
    The function parses the array of strings passed as input arguments. The array of strings
    is from the command line arguments of the executable file running on the target hardware.

    The external mode abstraction layer interprets only these options:
     '-w':  Enables the extmodeWaitForStartRequest() function, which waits for a model start 
            request from Simulink in external mode. If you do not specify this option, 
            the extmodeWaitForStartRequest() function will have no effect.

     '-tf <finalSimulationTime>': finalSimulationTime overrides the Simulink 
                                  configuration parameter, StopTime (inf specifies no time limit)

    If the command contains more options, they are passed to rtIOStreamOpen as configuration
    parameters for the communication driver. 
 */
extern extmodeErrorCode_T extmodeParseArgs(int_T argc, const char_T *argv[]);


/** Initialize external mode target connectivity. 
    The initialization includes the underlying communication stack.
 
    If the model's final simulation time in the external mode abstraction layer 
    is initialized, for example, through the '-tf ' option detected by extmodeParseArgs() 
    or extmodeSetFinalSimulationTime(), then finalTime is an output and the pointer 
    location is updated with the initialized value.
    
    If the model's final simulation time in the external mode abstraction layer 
    is not initialized, then finalTime is an input and the model's final simulation time 
    in external mode is updated accordingly. 
 */
extern extmodeErrorCode_T extmodeInit(RTWExtModeInfo *extmodeInfo, extmodeSimulationTime_T *finalTime);


/** Wait for request from development computer to start or stop external mode simulation.
    The function times out when the timeout value is reached.

    @note This is a blocking function. Use it during initialization or in a background task.
          If the specified timeoutInMicroseconds is set to EXTMODE_WAIT_FOREVER, the function waits forever.
          If '-w' is not extracted by the extmodeParseArgs(), the function has no effect.
*/
extern extmodeErrorCode_T extmodeWaitForHostRequest(extmodeRealTime_T timeoutInMicroseconds);


/** External mode event trigger.
    The function informs the external mode abstraction layer of the occurrence of an event.

    The function:
    - samples all signals associated with a given rate
    - stores signal values in a new packet buffer
    - passes the packet buffer to the underlying transport layer for subsequent transmission
      to the development computer

    For correct sampling of signal values, run the function immediately after model_step()
    for the corresponding sample time ID.

    As the function is thread-safe, you can invoke the function with different sample time IDs
    in separate threads.

    @param[in] eventId the Id of the rate for which the signals must be sampled. This corresponds to
        the TID assigned by Simulink. @note in case TID01EQ == 1 (code supports and contains
        continuous time), the base rate should use TID 1.
    @param[in] simulationTime simulation time to which the sampling is associated. If the model is
        configured for IntegerOnlyCode or the macro XCP_EXTMODE_SIMULATION_TIME_IN_TICKS is defined
        the simulationTime must be in base-rate ticks (in particular, an integer), otherwise, the 
        actual simulation time should be passed (a real number).

    @note in some cases the call to extmodeEvent is added to the generated code, so that it should
        not be called externally. This happens when the model is *not* a multi tasking model (both
        'ConcurrentTasks' and 'EnableMultiTask' set to 'off') and 'CombineOuputUpdateFcns' is set to
        'on'. In all other cases calls to extmodeEvent should be made in the same code that calls
        the generated code.
*/
extern extmodeErrorCode_T extmodeEvent(extmodeEventId_T eventId, extmodeSimulationTime_T simulationTime);


/** Perform external mode background activity
    For example, retrieving packets from the network, running the packets protocol layer, 
    and sending packets to the development computer.

    Do not invoke the function in a thread with real-time constraints.
*/
extern extmodeErrorCode_T extmodeBackgroundRun(void);


/** Reset external mode target connectivity.
    Restores the external mode abstraction layer, including the communication stack, 
    to the initial, default state.
 */
extern extmodeErrorCode_T extmodeReset(void);


/** Check whether request to stop external mode simulation is received from model on the development computer */
extern boolean_T extmodeStopRequested(void);


/** Check whether request to start external mode simulation is received from model on the development computer */
extern boolean_T extmodeStartRequested(void);


/** Check that external mode simulation is complete.
    The function checks, during an external mode simulation, whether the model simulation time 
    has reached the final simulation time specified by the command-line '-tf' option or 
    the configuration parameter, StopTime.
 */
extern boolean_T extmodeSimulationComplete(void);


/** Get final simulation time for external mode platform abstraction layer */
extern extmodeErrorCode_T extmodeGetFinalSimulationTime(extmodeSimulationTime_T *finalTime);


/** Set final simulation time for external mode platform abstraction layer */
extern extmodeErrorCode_T extmodeSetFinalSimulationTime(extmodeSimulationTime_T finalTime);


/** The function returns true if the external mode event must be triggered */
typedef boolean_T (*extmodeEventTriggerEnable)(extmodeEventId_T eventId);


/** Get the current custom trigger enable function for a specific eventId, or NULL if no custom trigger functions are installed */
extern extmodeErrorCode_T extmodeGetEventTriggerEnable(extmodeEventId_T eventId, extmodeEventTriggerEnable *triggerEnable);


/** Set a new custom trigger enable function for a specific eventId */
extern extmodeErrorCode_T extmodeSetEventTriggerEnable(extmodeEventId_T eventId, extmodeEventTriggerEnable triggerEnable);

#ifdef __cplusplus
}
#endif

#endif /* EXT_MODE_H */
