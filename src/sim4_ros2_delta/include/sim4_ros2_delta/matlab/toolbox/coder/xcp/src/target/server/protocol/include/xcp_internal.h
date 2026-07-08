/*
* Copyright 2020-2021 The MathWorks, Inc.
*
* File: xcp_internal.h
*
* Abstract:
*   Internal interface for XCP Protocol Layer
*/

#ifndef XCP_INTERNAL_H
#define XCP_INTERNAL_H

#include "xcp.h"

/** XCP ODT Entry */
typedef struct XcpOdtEntry {
    uint8_T bitOffset;        /**< Allows bitwise access (it must be between 0 and 0x1F - ignored if equal to 0xFF) */
    uint8_T size;             /**< size of DAQ element (the unit depends on the XCP_ADDRESS_GRANULARITY) */
    uint8_T addressExtension; /**< address extension of DAQ element */
    uint32_T address;         /**< address of DAQ element */
}   XcpOdtEntry;


typedef void* (*XcpCustomAllocHandler)(size_t memSize);
typedef void  (*XcpCustomFreeHandler)(void* ptr);

/** Register a custom memory manager for a specific eventId */
XcpErrorCode xcpSetCustomPoolMemoryManager(XcpEventIdType eventId, XcpCustomAllocHandler allocHandler, XcpCustomFreeHandler freeHandler);

/** Get the custom memory manager for a specific eventId */
XcpErrorCode xcpGetCustomPoolMemoryManager(XcpEventIdType *eventId, XcpCustomAllocHandler *allocHandler, XcpCustomFreeHandler *freeHandler);

/**
 * When DAQ Packed Mode is enabled, trigger the transmission of a packet
 * containing the samples received so far.
 * The function is thread safe and it can be invoked from a background thread,
 * typically before the reset of the XCP stack.
 * @note timestamp input parameter is ignored when DAQ Packed Mode Timestamp 
 *       is configured as XCP_DAQ_SINGLE_TIMESTAMP_FIRST_SAMPLE */
XcpErrorCode xcpPackedModeEventsFlush(uint32_T timestamp);

/**
 * When DAQ Packed Mode is enabled, resets the status of pending packed DAQ lists
 * associated to a given event, discarding incomplete packets.
 * The function must be invoked from the thread associated to event.
 */
XcpErrorCode xcpPackedModeEventReset(XcpEventIdType eventId);


#ifdef XCP_INTERNAL_DAQ_CONFIG_ACCESS_SUPPORT
/** Get the list of all the ODT Entries associated to the active DAQ lists for a given eventId.
    The user must provide a pointer to the output array of XcpOdtEntry where the data
    needs to be copied to and the maximum number of elements of such array.
    The function returns XCP_NO_MEMORY if the number of available entries
    exceeds the size of the output array provided.
    The list only contains entries associated to DAQ direction, whereas the entries 
    with STIM direction are excluded.
    @note this API is available for internal use only. The access to the internal
          data structures is  not protected by any mutual exclusion 
          mechanism. Therefore it's user's responsibility to guarantee that the
          Dynamic DAQ lists are available by checking xcpGetStatus() and
          invoking it once the dynamic DAQ lists configuration is complete.
          In general thread safety is only guaranteed if xcpGetDaqOdtEntries() and 
          xcpRun() are NOT executed concurrently. */
XcpErrorCode xcpGetDaqOdtEntries(XcpEventIdType eventId,   /**< Event Channel Id */
                                 XcpOdtEntry* entries,     /**< Pointer to the output array where data are copied to  */
                                 size_t* entriesNumber,    /**< Number of entries that have been successfully copied  */
                                 size_t maxEntriesNumber); /**< Max number of elements allocated in the output buffer */

#endif /* XCP_INTERNAL_DAQ_CONFIG_ACCESS_SUPPORT */

#endif /* XCP_INTERNAL_H */
