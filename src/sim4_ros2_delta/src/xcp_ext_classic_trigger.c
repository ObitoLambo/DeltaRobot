/*
 * Copyright 2020-2024 The MathWorks, Inc.
 *
 * File: xcp_ext_classic_trigger.c
 *
 * Abstract:
 *  This file contains the implementation of the External Mode
 *  "Classic Trigger" logic.
 *
 */

#include "xcp_ext_classic_trigger.h"
#include "xcp_ext_common.h"

#define EXTMODE_TRIG_BASE_RATE_EVENT_ID EXTMODE_BASE_RATE_EVENT_ID

#define DEFAULT_XCP_CLASSIC_TRIGGER_DURATION 1000


/* Global variables directly accessible by the XCP Client */
extmodeEventId_T volatile xcpClassicTriggerEventId = 0;

uint32_T volatile xcpClassicTriggerSignalAddress = 0;
uint8_T volatile  xcpClassicTriggerSignalAddressExtension = 0;

extmodeClassicTriggerSignal_T volatile xcpClassicTriggerLevel = 0;

uint32_T volatile xcpClassicTriggerDuration = DEFAULT_XCP_CLASSIC_TRIGGER_DURATION;
uint32_T volatile xcpClassicTriggerHoldOff  = 0;
int32_T  volatile xcpClassicTriggerDelay    = 0;

XcpClassicTriggerDirection volatile xcpClassicTriggerDirection = XCP_EXTMODE_TRIGGER_RISING;

#if defined(EXTMODE_TRIG_ARMED_ON_START) && EXTMODE_TRIG_ARMED_ON_START == 0
boolean_T volatile xcpClassicTriggerArmRequest = false;
#else
/* Classic Trigger is armed by default */
boolean_T volatile xcpClassicTriggerArmRequest = true;
#endif

boolean_T volatile xcpClassicTriggerCancelRequest = false;

XcpClassicTriggerSource volatile xcpClassicTriggerSource = XCP_EXTMODE_TRIGGER_MANUAL;

XcpClassicTriggerMode volatile xcpClassicTriggerMode = XCP_EXTMODE_TRIGGER_NORMAL;

XcpClassicTriggerStatus volatile xcpClassicTriggerStatus = XCP_EXTMODE_TRIGGER_UNARMED;


XCP_STATIC extmodeClassicTriggerSignal_T xcpOldSignalValue = 0;
XCP_STATIC boolean_T                     xcpOldSignalValueAvailable = false;

XCP_STATIC XcpClassicTriggerSource       xcpOldClassicTriggerSource = XCP_EXTMODE_TRIGGER_MANUAL;

XCP_STATIC boolean_T                     xcpTriggerEnabled = false;
XCP_STATIC boolean_T                     xcpSignalTriggerFired = false;
XCP_STATIC uint32_T                      xcpTriggerCount = 0;

XCP_STATIC boolean_T                     xcpForcedTriggerEnabled = false;
XCP_STATIC boolean_T                     xcpForceTriggerEnableRequest = false;

/** The function is responsible for sampling the signal trigger value
    and setting the xcpSignalTriggerFired boolean to true if
    the trigger signal has the expected transition across the xcpClassicTriggerLevel.
    The boolean will always be restored back to false by the base rate
    when the event is acknowledged and the state transition completed.
    The function is also setting xcpTriggerEnabled to true if no Delay is required. */
static void sampleSignalTrigger(void)
{
    extmodeClassicTriggerSignal_T const* signal = (extmodeClassicTriggerSignal_T const*)
                                            XCP_ADDRESS_GET_READ(xcpClassicTriggerSignalAddressExtension,
                                                            xcpClassicTriggerSignalAddress);
    if ((signal != NULL) &&
       ((xcpClassicTriggerSignalAddressExtension != 0) ||
        (xcpClassicTriggerSignalAddress != 0))) {
        if ((xcpClassicTriggerStatus ==	XCP_EXTMODE_TRIGGER_ARMED) &&
            !xcpSignalTriggerFired) {
            /* Read the new signal value */
            extmodeClassicTriggerSignal_T signalValue = *signal;

            if (xcpOldSignalValueAvailable) {
                /* If the trigger is armed and not already fired,
                   check for the expected transition */
                boolean_T checkForRising =  (xcpClassicTriggerDirection == XCP_EXTMODE_TRIGGER_RISING) ||
                                            (xcpClassicTriggerDirection == XCP_EXTMODE_TRIGGER_RISING_OR_FALLING);

                boolean_T checkForFalling = (xcpClassicTriggerDirection == XCP_EXTMODE_TRIGGER_FALLING) ||
                                            (xcpClassicTriggerDirection == XCP_EXTMODE_TRIGGER_RISING_OR_FALLING);

                if (checkForRising &&
                   (((signalValue >= xcpClassicTriggerLevel) && (xcpOldSignalValue <  xcpClassicTriggerLevel)) ||
                    ((signalValue >  xcpClassicTriggerLevel) && (xcpOldSignalValue == xcpClassicTriggerLevel)))) {

                    if (xcpClassicTriggerDelay == 0) {
                        /* fire the event immediately if needed no Delay is required */
                        xcpTriggerEnabled = true;
                    }
                    xcpSignalTriggerFired   = true; /* the transition from ARMED to FIRING is updated in base rate */
                }
                if (checkForFalling &&
                   (((signalValue < xcpClassicTriggerLevel)  && (xcpOldSignalValue >= xcpClassicTriggerLevel)) ||
                    ((signalValue == xcpClassicTriggerLevel) && (xcpOldSignalValue >  xcpClassicTriggerLevel)))) {

                    if (xcpClassicTriggerDelay == 0) {
                        /* fire the event immediately if needed no Delay is required */
                        xcpTriggerEnabled = true;
                    }
                    xcpSignalTriggerFired   = true; /* the transition from ARMED to FIRING is updated in base rate */
                }
            }

            /* Update the old signal value. Note: if we just fired, the sampling will start
               next time the trigger is armed and therefore we reset xcpOldSignalValueAvailable */
            xcpOldSignalValueAvailable = !xcpSignalTriggerFired;
            xcpOldSignalValue = signalValue;
        }
    }
}


/** The function is executed in the base rate and it is responsible
    for the handling of the triggering status transitions */
static void updateTriggerStatus(void)
{
    /* Process user Arm/Cancel requests common to all states
       and update the triggerFired variable (depending on the
       active triggering source) */
    boolean_T triggerFired = false;

    if (xcpClassicTriggerStatus == XCP_EXTMODE_TRIGGER_UNARMED) {
        /* This case deserves a separate handling
           because we would like the transition to the ARMED status
           to occur in the same cycle.
           Since we are already UNARMED, any request to cancel 
           the trigger can be ignored */
        xcpClassicTriggerCancelRequest = false;
        xcpSignalTriggerFired = false;

        if (xcpClassicTriggerArmRequest) {
            /* Process arm request */
            xcpClassicTriggerStatus = XCP_EXTMODE_TRIGGER_ARMED;
            xcpClassicTriggerArmRequest = false;

            /* If we are using MANUAL trigger, we want to start 
               firing the trigger event straightaway, e.g.
               when the user selects ExtModeArmWhenConnect */
            if (xcpClassicTriggerSource == XCP_EXTMODE_TRIGGER_MANUAL) {
                triggerFired = true;
            }
        }
    } else {
        /* We are already armed, ignore any request to arm the trigger */
        xcpClassicTriggerArmRequest = false;

        if (xcpClassicTriggerCancelRequest) {
            /* No matter what the Status is, a cancel request should
               always bring the status to UNARMED.
               We implement the transition here, to avoid code
               duplication in the states. */
            xcpTriggerEnabled = false;
            xcpClassicTriggerStatus = XCP_EXTMODE_TRIGGER_UNARMED;
            xcpClassicTriggerCancelRequest = false;
        } else {
            if (xcpClassicTriggerSource == XCP_EXTMODE_TRIGGER_MANUAL) {
                /* when manual triggering is selected, we assume that
                   the trigger event has always been fired when armed */
                triggerFired = true;
            } else {
                /* when signal triggering is selected, we check the status
                   of the xcpSignalTriggerFired (updated by the
                   sampleSignalTrigger() logic */
                triggerFired = xcpSignalTriggerFired;
                xcpSignalTriggerFired = false;
            }
        }
    }

    /* Handle the remaining state transitions */
    switch (xcpClassicTriggerStatus) {
    case XCP_EXTMODE_TRIGGER_UNARMED:
        if (xcpClassicTriggerArmRequest) {
            /* Process arm request */
            xcpClassicTriggerStatus = XCP_EXTMODE_TRIGGER_ARMED;
            xcpClassicTriggerArmRequest = false;
        }
        break;

    case XCP_EXTMODE_TRIGGER_ARMED:
        if (triggerFired) {
            if ((xcpClassicTriggerDelay > 0) &&
                (xcpClassicTriggerSource == XCP_EXTMODE_TRIGGER_SIGNAL)) {
                /* We need to postpone the enabling of the trigger
                   by xcpClassicTriggerDelay base periods */
                xcpTriggerCount = (uint32_T) xcpClassicTriggerDelay;
                xcpClassicTriggerStatus = XCP_EXTMODE_TRIGGER_DELAYED;
            } else {
                /* Negative xcpClassicTriggerDelay are ignored.
                   The parameter is also ignored if the xcpClassicTriggerSource
                   is XCP_EXTMODE_TRIGGER_MANUAL
                   If xcpClassicTriggerDelay is 0 we enable the trigger straight away */
                if (xcpClassicTriggerDuration > 0) {
                    xcpTriggerEnabled = true;
                    xcpTriggerCount = xcpClassicTriggerDuration;
                    xcpClassicTriggerStatus = XCP_EXTMODE_TRIGGER_FIRING;
                } else {
                    /* If duration is 0, we would never be able to send any data.
                       This situation should be prevented on the host,
                       we just handle it gracefully here, in case it happens */
                    XCP_PRINTF("Invalid duration value (0) detected.\n");
                    xcpTriggerEnabled = false;
                    xcpClassicTriggerStatus = XCP_EXTMODE_TRIGGER_UNARMED;
                }
            }
        }
        break;

    case XCP_EXTMODE_TRIGGER_FIRING:
        /* Update the base rate counter */
        if (xcpTriggerCount > 0) {
            xcpTriggerCount--;
        }

        if (xcpTriggerCount == 0) {
            /* Duration has expired */
            if (xcpClassicTriggerMode == XCP_EXTMODE_TRIGGER_ONESHOT) {
                /* In the one-shot scenario we just disable the trigger */
                xcpTriggerEnabled = false;
                xcpClassicTriggerStatus  = XCP_EXTMODE_TRIGGER_UNARMED;
            } else {
                /* In normal mode, we might have to handle the HoldOff and
                   pause the triggering  */
                if (xcpClassicTriggerHoldOff > 0) {
                    xcpTriggerEnabled = false;
                    xcpTriggerCount = xcpClassicTriggerHoldOff;
                    xcpClassicTriggerStatus = XCP_EXTMODE_TRIGGER_HOLDING_OFF;
                } else {
                    if (triggerFired) {
                        /* If the trigger has been fired, we start another duration
                           straight away */
                        xcpTriggerCount = xcpClassicTriggerDuration;
                        xcpClassicTriggerStatus = XCP_EXTMODE_TRIGGER_FIRING;
                    } else {
                        /* We wait for the next trigger to be fired */
                        xcpTriggerEnabled = false;
                        xcpClassicTriggerStatus = XCP_EXTMODE_TRIGGER_ARMED;
                    }
                }
            }
        }
        break;

    case XCP_EXTMODE_TRIGGER_DELAYED:
        /* Update the base rate counter */
        if (xcpTriggerCount > 0) {
            xcpTriggerCount--;
        }

        if (xcpTriggerCount == 0) {
            /* Delay is over, we can enable the trigger now */
            if (xcpClassicTriggerDuration > 0) {
                xcpTriggerEnabled = true;
                xcpTriggerCount = xcpClassicTriggerDuration;
                xcpClassicTriggerStatus = XCP_EXTMODE_TRIGGER_FIRING;
            } else {
                /* If duration is 0, we would never be able to send any data.
                   This situation should be prevented on the host,
                   we just handle it gracefully here, in case it happens */
                XCP_PRINTF("Invalid duration value (0) detected.\n");
                xcpTriggerEnabled = false;
                xcpClassicTriggerStatus = XCP_EXTMODE_TRIGGER_UNARMED;
            }
        }
        break;

    case XCP_EXTMODE_TRIGGER_HOLDING_OFF:
        /* Update the base rate counter */
        if (xcpTriggerCount > 0) {
            xcpTriggerCount--;
        }

        if (xcpTriggerCount == 0) {
            /* HoldOff time has expired */
            if (triggerFired) {
                /* If the trigger has been fired, we start another duration
                   straight away */
                if (xcpClassicTriggerDuration > 0) {
                    xcpTriggerEnabled = true;
                    xcpTriggerCount = xcpClassicTriggerDuration;
                    xcpClassicTriggerStatus = XCP_EXTMODE_TRIGGER_FIRING;
                } else {
                    /* If duration is 0, we would never be able to send any data.
                       This situation should be prevented on the host,
                       we just handle it gracefully here, in case it happens */
                    XCP_PRINTF("Invalid duration value (0) detected.\n");
                    xcpTriggerEnabled = false;
                    xcpClassicTriggerStatus = XCP_EXTMODE_TRIGGER_UNARMED;
                }
            } else {
                /* We wait for the next trigger to be fired */
                xcpClassicTriggerStatus = XCP_EXTMODE_TRIGGER_ARMED;
            }
        }
        break;

    default:
        XCP_PRINTF("Invalid Trigger Status %u detected\n", xcpClassicTriggerStatus);
        xcpTriggerEnabled = false;
        xcpSignalTriggerFired = false;
        xcpClassicTriggerCancelRequest = false;
        xcpClassicTriggerStatus = XCP_EXTMODE_TRIGGER_UNARMED;
    }
}



XcpErrorCode xcpExtModeClassicTriggerInit(void)
{
    xcpOldSignalValue = 0;
    xcpOldSignalValueAvailable = false;

    xcpOldClassicTriggerSource = xcpClassicTriggerSource;

    xcpTriggerEnabled = false;
    xcpSignalTriggerFired = false;
    xcpTriggerCount = 0;

    xcpClassicTriggerStatus  = XCP_EXTMODE_TRIGGER_UNARMED;

    return XCP_SUCCESS;
}


/** The function returns true if the external mode event must be triggered */
boolean_T xcpExtModeClassicTriggerEnabled(extmodeEventId_T eventId)
{
    if (eventId == EXTMODE_TRIG_BASE_RATE_EVENT_ID) {
        /* Whenever a change of the xcpClassicTriggerSource is detected
           the xcpClassicTriggerStatus is reset */
        boolean_T triggerSrcChangeDetected = (xcpClassicTriggerSource != xcpOldClassicTriggerSource);
        xcpOldClassicTriggerSource = xcpClassicTriggerSource;

        if (triggerSrcChangeDetected) {
            xcpTriggerEnabled     = false;
            xcpSignalTriggerFired = false;
            xcpTriggerCount       = 0;

            xcpClassicTriggerStatus  = XCP_EXTMODE_TRIGGER_UNARMED;
        }

        /* If XCP_EXTMODE_TRIGGER_SIGNAL is selected we want to start
           processing the Arm request (or the Hold-off expiration) 
           straightaway and carry out the state transition before 
           sampling for the first time */
        if ((xcpClassicTriggerSource == XCP_EXTMODE_TRIGGER_SIGNAL)  &&
            (((xcpClassicTriggerStatus == XCP_EXTMODE_TRIGGER_UNARMED) &&
               xcpClassicTriggerArmRequest) ||
             ((xcpClassicTriggerStatus == XCP_EXTMODE_TRIGGER_HOLDING_OFF) &&
              (xcpTriggerCount == 1)))) {
               xcpTriggerCount = 0;
               xcpClassicTriggerArmRequest = false;
               xcpClassicTriggerStatus = XCP_EXTMODE_TRIGGER_ARMED;
        }
    }

    if ((xcpClassicTriggerSource == XCP_EXTMODE_TRIGGER_SIGNAL) &&
        (eventId == xcpClassicTriggerEventId))  {
        /* Since we are running at the correct eventID, sample the
           trigger signal value to determine if the trigger signal
           needs to be enabled.
           Note: this will update the boolean straight away and request
           the state transition (updated at the next base rate) */
        sampleSignalTrigger();
    }

    if (eventId == EXTMODE_TRIG_BASE_RATE_EVENT_ID) {
        /* update trigger status to fulfill user requests */
        updateTriggerStatus();

        /* If the base rate task has run, then reset the
         * xcpForceTriggerEnableRequest flag.  The state machine
         * is up to date. */
        xcpForceTriggerEnableRequest = false;
    } else if (xcpForceTriggerEnableRequest) {
        return xcpForcedTriggerEnabled;
    }

    return xcpTriggerEnabled;
}


/** Reset External Mode classic triggering logic, by restoring the default status */
XcpErrorCode xcpExtModeClassicTriggerReset(void)
{
    xcpOldSignalValue = 0;
    xcpOldSignalValueAvailable = false;

    xcpClassicTriggerStatus  = XCP_EXTMODE_TRIGGER_UNARMED;
    xcpOldClassicTriggerSource = XCP_EXTMODE_TRIGGER_MANUAL;
    xcpClassicTriggerEventId = 0;
    xcpTriggerEnabled = false;
    xcpSignalTriggerFired = false;
    xcpTriggerCount = 0;
    xcpForcedTriggerEnabled = false;
    xcpForceTriggerEnableRequest = false;

    xcpClassicTriggerSignalAddress = 0;
    xcpClassicTriggerSignalAddressExtension = 0;

    xcpClassicTriggerLevel    = 0;
    xcpClassicTriggerDuration = DEFAULT_XCP_CLASSIC_TRIGGER_DURATION;
    xcpClassicTriggerHoldOff  = 0;
    xcpClassicTriggerDelay    = 0;

    xcpClassicTriggerDirection = XCP_EXTMODE_TRIGGER_RISING;

#if defined(EXTMODE_TRIG_ARMED_ON_START) && EXTMODE_TRIG_ARMED_ON_START == 0
    xcpClassicTriggerArmRequest = false;
#else
/* Classic Trigger is armed by default */
    xcpClassicTriggerArmRequest = true;
#endif

    xcpClassicTriggerCancelRequest = false;

    xcpClassicTriggerSource = XCP_EXTMODE_TRIGGER_MANUAL;
    xcpClassicTriggerMode = XCP_EXTMODE_TRIGGER_NORMAL;

    return XCP_SUCCESS;
}


/** When Manual triggering is enabled, force output of xcpExtModeClassicTriggerEnabled to
 *  match pending Arm or Cancel request for all non-base rate events until requests are 
 *  processed at the next base rate event.
 * 
 *  In the concurrent execution workflow, it is used to avoid omission
 *  of logging data points if a non-base rate task completes
 *  before the base rate at t = 0.
 * 
 *  This is a NO-OP when XCP_EXTMODE_TRIGGER_SIGNAL is active.
 */
void xcpExtModeClassicTriggerForceEnableStatusUpdate(void) {
    if (xcpClassicTriggerSource == XCP_EXTMODE_TRIGGER_MANUAL) {
        if (xcpClassicTriggerArmRequest && !xcpClassicTriggerCancelRequest) {
            /* Force xcpExtModeClassicTriggerEnabled to return true
            * until the next base rate event. */
            xcpForcedTriggerEnabled = true;
            xcpForceTriggerEnableRequest = true;
        } else if (xcpClassicTriggerCancelRequest && !xcpClassicTriggerArmRequest) {
            xcpForcedTriggerEnabled = false;
            xcpForceTriggerEnableRequest = true;
        } else {
            xcpForceTriggerEnableRequest = false;
        }
    }
}
