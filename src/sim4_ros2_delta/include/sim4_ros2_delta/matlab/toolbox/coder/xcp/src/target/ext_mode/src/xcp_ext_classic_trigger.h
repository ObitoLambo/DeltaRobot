/*
* Copyright 2020-2023 The MathWorks, Inc.
*
* File: xcp_ext_classic_trigger.h
*
* Abstract:
*  This file contains the declarations and internal functions
*  to support External Mode "Classic Trigger" logic
*/

#ifndef XCP_EXT_CLASSIC_TRIGGER_H
#define XCP_EXT_CLASSIC_TRIGGER_H

#include "ext_mode_types.h"
#include "xcp.h"

/** External Mode classic triggering direction */
typedef enum {
    XCP_EXTMODE_TRIGGER_RISING,
    XCP_EXTMODE_TRIGGER_FALLING,
    XCP_EXTMODE_TRIGGER_RISING_OR_FALLING
} XcpClassicTriggerDirection;


/** External Mode classic triggering source */
typedef enum {
    XCP_EXTMODE_TRIGGER_MANUAL,
    XCP_EXTMODE_TRIGGER_SIGNAL
} XcpClassicTriggerSource;


/** External Mode classic triggering mode */
typedef enum {
    XCP_EXTMODE_TRIGGER_NORMAL,
    XCP_EXTMODE_TRIGGER_ONESHOT
} XcpClassicTriggerMode;


/** External Mode classic triggering status */
typedef enum {
    XCP_EXTMODE_TRIGGER_UNARMED,
    XCP_EXTMODE_TRIGGER_HOLDING_OFF,
    XCP_EXTMODE_TRIGGER_ARMED,
    XCP_EXTMODE_TRIGGER_DELAYED,
    XCP_EXTMODE_TRIGGER_FIRING
} XcpClassicTriggerStatus;


/* Global variables directly accessible by the XCP Client */


/** Event ID associated to the signal selected as
    External Mode classic trigger */
extern extmodeEventId_T volatile xcpClassicTriggerEventId;


/** External Mode classic trigger signal address */
extern uint32_T volatile xcpClassicTriggerSignalAddress;


/** External Mode classic trigger signal address extension */
extern uint8_T volatile xcpClassicTriggerSignalAddressExtension;


/** External Mode classic trigger Level */
extern extmodeClassicTriggerSignal_T volatile xcpClassicTriggerLevel;


/** External Mode classic trigger duration */
extern uint32_T volatile xcpClassicTriggerDuration;


/** External Mode classic trigger Hold-off */
extern uint32_T volatile xcpClassicTriggerHoldOff;


/** External Mode classic trigger Delay
    @note: currently only positive values are supported
           and therefore no pre-triggering is possible */
extern int32_T volatile xcpClassicTriggerDelay;


/** External Mode classic trigger Direction */
extern XcpClassicTriggerDirection volatile xcpClassicTriggerDirection;


/** External Mode classic Arm Trigger request */
extern boolean_T volatile xcpClassicTriggerArmRequest;


/** External Mode classic Cancel Trigger request */
extern boolean_T volatile xcpClassicTriggerCancelRequest;


/** External Mode classic trigger Source */
extern XcpClassicTriggerSource volatile xcpClassicTriggerSource;


/** External Mode classic trigger Mode */
extern XcpClassicTriggerMode volatile xcpClassicTriggerMode;


/** Current External Mode classic Trigger status
    @note this variable is only for information purposes */
extern XcpClassicTriggerStatus volatile xcpClassicTriggerStatus;


/** Initialize External Mode classic triggering logic */
extern XcpErrorCode xcpExtModeClassicTriggerInit(void);


/** The function returns true if the external mode event must be triggered */
extern boolean_T xcpExtModeClassicTriggerEnabled(extmodeEventId_T eventId);


/** Reset External Mode classic triggering logic, by restoring the default status */
extern XcpErrorCode xcpExtModeClassicTriggerReset(void);


/** When Manual triggering is enabled, force output of xcpExtModeClassicTriggerEnabled 
 *  to match pending Arm or Cancel request for all non-base rate events until requests 
 *  are processed at the next base rate event.  This is a NO-OP when Signal triggering
 *  is active. */
void xcpExtModeClassicTriggerForceEnableStatusUpdate(void);

#endif /* XCP_EXT_CLASSIC_TRIGGER_H */
