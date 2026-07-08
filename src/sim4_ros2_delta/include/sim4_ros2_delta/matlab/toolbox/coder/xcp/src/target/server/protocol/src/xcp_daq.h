/*
* Copyright 2016-2022 The MathWorks, Inc.
*
* File: xcp_daq.h
*
* Abstract:
*  XCP Protocol layer DAQ lists (and STIM) support internal interface
*/

#ifndef XCP_DAQ_H
#define XCP_DAQ_H

#include "xcp_internal.h"
#include "xcp_types.h"

/** Initialize DAQ lists (and STIM) support */
void xcpDaqInit(void);

/** Return the XcpPacketLookupFunction to be invoked
    to get access to the Input and Output Packet
    handlers for the supported DAQ Commands */
XcpPacketLookupFunction xcpDaqGetPacketLookup(void);

/** Override the XcpPacketLookupFunction to be invoked
    to get access to the Input and Output Packet
    handlers for the supported DAQ Commands */
void xcpDaqSetPacketLookup(XcpPacketLookupFunction getPacket);

/** Process DAQ (and STIM) Lists associated to the event
    @note when DAQ Packed Mode is enabled, setting resetSamples
          to true forces the resets the status of pending packed DAQ lists,
          discarding incomplete packets. */
XcpErrorCode xcpDaqEvent(XcpEventIdType eventId, uint32_T timestamp, boolean_T resetSamples);

/**
   When DAQ Packed Mode is enabled, trigger the transmission of a packet
   containing the samples received so far.
   @note timestamp input parameter is ignored when DAQ Packed Mode Timestamp 
         is configured as XCP_DAQ_SINGLE_TIMESTAMP_FIRST_SAMPLE */
XcpErrorCode xcpDaqEventsFlush(uint32_T timestamp);

/** Reset DAQ lists status, returning true if successfully executed */
boolean_T xcpResetDaqListStatus(void);

/** Lock DAQ list transmission, returning true if successfully executed */
boolean_T xcpDaqLock(void);

/** Unlock DAQ list transmission */
void xcpDaqUnlock(void);

/** Reset DAQ lists (and STIM) support to the initial status */
void xcpDaqReset(void);

/** Configure a specific eventId to use custom functions to allocate and free 
    memory instead of relying on generic/reserved memory pool */
XcpErrorCode xcpDaqSetCustomPoolMemoryManager(XcpEventIdType eventId,
                                              XcpCustomAllocHandler allocHandler,
                                              XcpCustomFreeHandler freeHandler);

/** Query custom functions to allocate and free memory for a specific eventId */
XcpErrorCode xcpDaqGetCustomPoolMemoryManager(XcpEventIdType *eventId,
                                              XcpCustomAllocHandler *allocHandler,
                                              XcpCustomFreeHandler *freeHandler);


#ifdef XCP_INTERNAL_DAQ_CONFIG_ACCESS_SUPPORT

/** XCP DAQ Direction */
typedef enum {
    XCP_DIRECTION_DAQ  = 0,
    XCP_DIRECTION_STIM = 1
} XcpDaqDirection;

/** Get the list of all the ODT Entries associated to the active DAQ lists for a given eventId.
    The user must provide a pointer to the output array of XcpOdtEntry where the data
    needs to be copied to and the maximum number of elements of such array.
    The function returns XCP_NO_MEMORY if the number of available entries
    exceeds the size of the output array provided.
    @note this API is available for internal use only. The access to the internal
          data structures is not protected by any mutual exclusion 
          mechanism. Therefore it's user's responsibility to guarantee that the
          Dynamic DAQ lists are available and the function is invoked when the the dynamic 
          DAQ lists configuration is not in progress. */
XcpErrorCode xcpDaqGetOdtEntries(XcpEventIdType eventId,    /**< Event Channel Id */
                                 XcpDaqDirection direction, /**< DAQ direction: DAQ or STIM */
                                 XcpOdtEntry* entries,      /**< Pointer to the output array where data are copied to  */
                                 size_t* entriesNumber,     /**< Number of entries that have been successfully copied  */
                                 size_t maxEntriesNumber);  /**< Max number of elements allocated in the output buffer */


#endif /* XCP_INTERNAL_DAQ_CONFIG_ACCESS_SUPPORT */

#endif /* XCP_DAQ_H */
