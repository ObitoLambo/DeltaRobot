/*
* Copyright 2016-2023 The MathWorks, Inc.
*
* File: xcp_daq.c
*
* Abstract:
*  Implementation of XCP Protocol Layer DAQ Lists (and STIM) support
*/

#include "xcp_common.h"
#include "xcp.h"
#include "xcp_internal.h"
#include "xcp_cfg.h"
#include "xcp_daq.h"

#ifdef XCP_DAQ_SUPPORT
#include "xcp_daq_ext.h"
#include "xcp_transport_internal.h"
#include "xcp_types.h"
#include "xcp_daq_types.h"
#include "xcp_mem.h"

/*****************************************************************************
    Internal Global variables specific to DAQ support
******************************************************************************/

/** Dynamic DAQ Lists data structures */
static XcpDaqLists xcpDynamicDaqLists;

/** Number of event threads currently in execution */
static size_t xcpRunningEventCounter = 0;

/** ID of the selected DAQ List on START_STOP_DAQ_LIST command */
#define XCP_INVALID_DAQ_LIST_ID -1
static int32_T startStopDaqListId = XCP_INVALID_DAQ_LIST_ID;

/** Pointer to current ODT entry */
static XcpDaqPtr   xcpCurrentDaq = { 0xFFFF, 0xFF, 0xFF };

static XcpEventCustomMemoryManager xcpEventCustomMemoryManager = {XCP_DAQ_CUSTOM_MEMORY_INVALID_EVENT_ID, NULL, NULL};

#ifndef XCP_DAQ_LIST_INDEPENDENT_EVENT_LOCK
#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
/* XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT is typically enabled
   for multi-core applications, where it is critical to limit
   the interference between threads streaming data.
   For this reason, by default we enable the independent lock
   of the DAQ list data structures between events */
#define XCP_DAQ_LIST_INDEPENDENT_EVENT_LOCK 1
#else
/* XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT is typically disabled
   for single-core bare-metal applications, where it is critical
   to limit the memory footprint of the target executable.
   For this reason, by default we disable the independent lock
   of the DAQ list data structures between events and use a single
   mutex to protect the DAQ lists data structures */
#define XCP_DAQ_LIST_INDEPENDENT_EVENT_LOCK 0
#endif

#endif


#if !defined(XCP_DAQ_LIST_INDEPENDENT_EVENT_LOCK) || (XCP_DAQ_LIST_INDEPENDENT_EVENT_LOCK == 0)
/* Mutex protecting against the concurrent access to DAQ Lists data structures */
static XCP_MUTEX_DEFINE(daqLock);

static boolean_T xcpDaqWriteLock(XcpEventIdType eventId)
{
    boolean_T locked = false;
    XCP_UNUSED_PARAM(eventId);
    XCP_WRITE_TRY_LOCK(daqLock, xcpRunningEventCounter, locked);

    return locked;
}

static void xcpDaqWriteUnlock(XcpEventIdType eventId)
{
    XCP_UNUSED_PARAM(eventId);
    XCP_WRITE_UNLOCK(daqLock);
}

static void xcpDaqReadLock(XcpEventIdType eventId)
{
    XCP_UNUSED_PARAM(eventId);
    XCP_READ_LOCK(daqLock, xcpRunningEventCounter);
}

static void xcpDaqReadUnlock(XcpEventIdType eventId)
{
    XCP_UNUSED_PARAM(eventId);
    XCP_READ_UNLOCK(daqLock, xcpRunningEventCounter);
}

#else
/* Each XCP event has an independent mutex to protect the read/write access
   to the DAQ list data structures */
#ifndef XCP_MAX_EVENT_DATA_NUMBER
#define XCP_MAX_EVENT_DATA_NUMBER XCP_MAX_EVENT_CHANNEL
#endif

typedef struct XcpEventData {
    XCP_MUTEX_DEFINE(lock);
} XcpEventData;

/** Array of event-specific data */
static XcpEventData xcpEventData[XCP_MAX_EVENT_DATA_NUMBER];

static boolean_T xcpDaqWriteLock(XcpEventIdType eventId)
{
    XCP_MUTEX_LOCK(xcpEventData[eventId].lock);
    return true;
}

static void xcpDaqWriteUnlock(XcpEventIdType eventId)
{
    XCP_MUTEX_UNLOCK(xcpEventData[eventId].lock);
}

static void xcpDaqReadLock(XcpEventIdType eventId)
{
    XCP_MUTEX_LOCK(xcpEventData[eventId].lock);
}

static void xcpDaqReadUnlock(XcpEventIdType eventId)
{
    XCP_MUTEX_UNLOCK(xcpEventData[eventId].lock);
}

#endif

#ifndef XCP_MIN_EVENT_NO_RESERVED_POOL
#define XCP_MIN_EVENT_NO_RESERVED_POOL 0xFFFF
#endif

/*****************************************************************************
    Internal Functions specific to DAQ support
******************************************************************************/

/* When enabled, timestamp is required for first ODT in the DAQ list */
#if XCP_TIMESTAMP_FIXED == 0
#define IS_TIMESTAMP_REQUIRED(mode, odtNumber) ((odtNumber == 0) && \
            XCP_READ_BIT_VALUE(mode, XCP_DAQ_MODE_TIMESTAMP_MASK))
#else
#define IS_TIMESTAMP_REQUIRED(mode, odtNumber) (odtNumber == 0)
#endif

/* Incrementing pointer. byteOffset argument will have a valid value only when
   we are emulating as a BYTE addressable target. Otherwise, it is 0. */
#ifdef XCP_EMULATE_BYTE_ADDRESSABLE_TARGET
#define XCP_HW_PTR_INCREMENT(ptr, incr, byteOffset) (uint8_T *)(byteOffset ? (ptr + XCP_IN_HW_AG(incr - 1)) : (ptr + XCP_IN_HW_AG(incr)))
#else
#define XCP_HW_PTR_INCREMENT(ptr, incr, byteOffset) (uint8_T *)(ptr + incr)
#endif

/* Update byteOffset when emulating as a BYTE addressable target */
#ifdef XCP_EMULATE_BYTE_ADDRESSABLE_TARGET
#define XCP_HW_BYTE_OFFSET_UPDATE(offset, size) ((offset + size) % XCP_HARDWARE_ADDRESS_GRANULARITY_BYTES_NUMBER)
#else
#define XCP_HW_BYTE_OFFSET_UPDATE(offset, size) 0
#endif

#define IS_USING_CUSTOM_POOL(x) (x == xcpEventCustomMemoryManager.eventId)

static boolean_T isActiveDaqList(uint16_T daqListId)
{
    return (xcpDynamicDaqLists.daq[daqListId].status == XCP_DAQ_SELECTED) ||
           (xcpDynamicDaqLists.daq[daqListId].status == XCP_DAQ_STARTED);
}

static boolean_T noActiveDaqLists(void)
{
    boolean_T found = 0;
    uint16_T i = 0;

    for (i = 0; (i < xcpDynamicDaqLists.daqCount) && !found; i++) {
        found = isActiveDaqList(i);
    }

    return !found;
}

static boolean_T isValidDaqPtr(uint16_T daqListId, uint8_T odtId, uint8_T odtEntryId)
{
    boolean_T ret = (xcpDynamicDaqLists.daq != NULL) &&
                    (daqListId < xcpDynamicDaqLists.daqCount) &&
                    (xcpDynamicDaqLists.daq[daqListId].odt != NULL) &&
                    (odtId < xcpDynamicDaqLists.daq[daqListId].odtCount) &&
                    (xcpDynamicDaqLists.daq[daqListId].odt[odtId].entry != NULL) &&
                    (odtEntryId < xcpDynamicDaqLists.daq[daqListId].odt[odtId].entriesCount);

    return ret;
}

static boolean_T isValidDaqEntry(uint8_T bitOffset, uint8_T size, uint32_T address)
{
    boolean_T ret = false;
    uint16_T  entrySize = size;     /* cast to 16-bit avoids -Werror=type-limits */

    if (bitOffset == 0xFF) {
        /* "Normal" (non-bitwise) access has been selected */
        if ( (entrySize <= XCP_MAX_ODT_ENTRY_SIZE) &&
             (entrySize % (XCP_ODT_ENTRY_SIZE_GRANULARITY/XCP_ADDRESS_GRANULARITY_BYTES_NUMBER) == 0) &&
             (  address % (XCP_ODT_ENTRY_SIZE_GRANULARITY/XCP_ADDRESS_GRANULARITY_BYTES_NUMBER) == 0) ) {
            ret = true;
        }
    }
    else {
        /* Bitwise access has been selected */
        if ((bitOffset <= 0x1F) && (address % XCP_ODT_ENTRY_SIZE_GRANULARITY) &&
            (entrySize == XCP_ODT_ENTRY_SIZE_GRANULARITY)) {
            ret = true;
        }
    }

    return ret;
}

/** The current supported mode includes:
- alternating = 0
- direction = 0 (DAQ only)
- timestamp = 1 or 0
- PID OFF = 0
*/
static boolean_T isSupportedDaqListMode(uint8_T mode)
{
    #if XCP_TIMESTAMP_FIXED == 0
        return ( (mode & ~XCP_DAQ_MODE_TIMESTAMP_MASK) == 0 );
    #else
        return (mode == XCP_DAQ_MODE_TIMESTAMP_MASK);
    #endif
}

static boolean_T isValidStartStopMode(uint8_T mode)
{
    boolean_T ret = false;

    if ((mode == XCP_DAQ_LIST_STOP) ||
        (mode == XCP_DAQ_LIST_START) ||
        (mode == XCP_DAQ_LIST_SELECT))
        ret = true;

    return ret;
}

static boolean_T isValidStartStopSynchMode(uint8_T mode)
{
    boolean_T ret = false;

    if ((mode == XCP_DAQ_LIST_STOP_ALL) ||
        (mode == XCP_DAQ_LIST_START_SELECTED) ||
        (mode == XCP_DAQ_LIST_STOP_SELECTED))
        ret = true;

    return ret;
}

static boolean_T absoluteOdtNumberIdentification(void)
{
    return (XCP_ID_FIELD_TYPE == XCP_ID_ABSOLUTE_ODT_NUMBER);
}

static boolean_T isDaqPackedModeEnabled(uint16_T daqListId)
{
    /* DAQ Packed Mode is only supported for event-grouped packing and 
       reserved memory pool */

    boolean_T isEnabled = (xcpDynamicDaqLists.daq[daqListId].packedMode == XCP_DAQ_EVENT_GROUPED_PACKING) &&
                          !IS_USING_CUSTOM_POOL(xcpDynamicDaqLists.daq[daqListId].eventId);
    
#if XCP_MEM_DAQ_RESERVED_POOLS_NUMBER > 0
    isEnabled = isEnabled && (daqListId < XCP_MEM_DAQ_RESERVED_POOLS_NUMBER);
#endif

    return isEnabled;
}

/* Return size of packet associated with the odt at the protocol layer */
static size_t odtPacketSize(XcpDaq *daq, uint8_T odtNumber) {
    XcpOdt *odt = &daq->odt[odtNumber];
    size_t packetSize = XCP_IN_BYTES(odt->size * daq->sampleCount + XCP_IN_AG(XCP_ID_FIELD_SIZE));

    if (IS_TIMESTAMP_REQUIRED(daq->mode, odtNumber)) {
        packetSize += XCP_TIMESTAMP_SIZE;
    }

    return packetSize;
}

/* Try to allocate a reserved memory pool dedicated to the packets in the DAQ list.
   @note depending on the configuration of the memory allocator, this may fail
         so we just set daq->poolId to XCP_INVALID_POOL_ID in that case */
static XcpErrorCode createDaqReservedPool(XcpDaq *daq)
{
    XcpErrorCode errorCode = XCP_SUCCESS;
    size_t poolBlockSize = 0;
    uint8_T i = 0;

    /* Calculate the max size for the ODT packets in the given DAQ list */
    for (i = 0; i < daq->odtCount; i++) {
        /* Calculate the required message buffer size */
        size_t packetSize = odtPacketSize(daq, i);
        size_t msgBufferSize = 0;

        msgBufferSize = xcpTransportMsgBufferSize(packetSize, XCP_DTO);

        if (msgBufferSize > poolBlockSize) {
            poolBlockSize = msgBufferSize;
        }
    }

    /* Try the allocation of the dedicate pool */
    errorCode = xcpMemReservedPoolCreate(poolBlockSize,
        (size_t)XCP_MEM_DAQ_RESERVED_POOL_BLOCKS_NUMBER * daq->odtCount,
        &daq->poolId);
    if (errorCode != XCP_SUCCESS) {
        daq->poolId = XCP_INVALID_POOL_ID;
    }

    #ifdef XCP_DEBUG_SUPPORT
    xcpMemPrintDiagnostics();
    #endif

    return errorCode;
}

static void destroyDaqReservedPool(XcpDaq *daq)
{
    if (daq->poolId != XCP_INVALID_POOL_ID) {
        xcpMemReservedPoolDestroy(daq->poolId);
    }

    daq->poolId = XCP_INVALID_POOL_ID;
}


/*****************************************************************************
    XCP SET DAQ PTR
******************************************************************************/
static XcpProtoErrorCode setDaqPtrInputPacketHandler(void   *msgBuffer,
                                                     size_t  xcpPacketOffset,
                                                     size_t *outputPacketSize)
{
    XcpProtoErrorCode protoErrorCode = XCP_PROTO_SUCCESS;
    uint8_T *packet = (uint8_T *)msgBuffer + xcpPacketOffset;
    XcpSetDaqPtrCmdPacketFrame *frame = (XcpSetDaqPtrCmdPacketFrame *) packet;
    uint16_T daqListId  = (uint16_T)frame->daqListId;
    uint8_T  odtId      = (uint8_T)frame->odtId;
    uint8_T  odtEntryId = (uint8_T)frame->odtEntryId;
    boolean_T ok = false;

    /* Check if the selected ODT Entry is available */
    ok = isValidDaqPtr(daqListId, odtId, odtEntryId);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE,
        ("SET_DAQ_PTR: ODT entry not available (DAQ %d, ODT %d, Entry %d)\n", daqListId, odtId, odtEntryId));

    /* Check if the DAQ list is currently active */
    ok = !isActiveDaqList(daqListId);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_DAQ_ACTIVE, ("SET_DAQ_PTR: DAQ list %d is currently active\n", daqListId));

    /* Everything is fine -> proceed and update the current ODT entry pointer */
    XCP_PRINTF("SET_DAQ_PTR: updating current ODT entry pointer to DAQ %d, ODT %d, Entry %d\n",
               daqListId, odtId, odtEntryId);

    xcpCurrentDaq.daqListId  = daqListId;
    xcpCurrentDaq.odtId      = odtId;
    xcpCurrentDaq.odtEntryId = odtEntryId;

    *outputPacketSize = XCP_GENERIC_RES_PACKET_SIZE_IN_BYTES;

    return protoErrorCode;
}

/*****************************************************************************
    XCP WRITE DAQ
******************************************************************************/
static XcpProtoErrorCode writeDaqInputPacketHandler(void   *msgBuffer,
                                                    size_t  xcpPacketOffset,
                                                    size_t *outputPacketSize)
{
    XcpProtoErrorCode protoErrorCode = XCP_PROTO_SUCCESS;
    uint8_T *packet = (uint8_T *)msgBuffer + xcpPacketOffset;
    XcpWriteDaqCmdPacketFrame *frame = (XcpWriteDaqCmdPacketFrame *)packet;
    uint16_T daqListId = xcpCurrentDaq.daqListId;
    uint8_T  odtId = xcpCurrentDaq.odtId;
    uint8_T  odtEntryId = xcpCurrentDaq.odtEntryId;
    XcpOdtEntry *entry = NULL;
    boolean_T ok = false;

    /* Check if the selected ODT Entry is available */
    ok = isValidDaqPtr(daqListId, odtId, odtEntryId);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_WRITE_PROTECTED,
        ("WRITE_DAQ: current ODT entry not available for write operations (DAQ %d, ODT %d, Entry %d)\n", daqListId, odtId, odtEntryId));

    /* Check if the DAQ list is currently active */
    ok = !isActiveDaqList(daqListId);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_DAQ_ACTIVE, ("XCP_WRITE_DAQ: DAQ list %d is currently active\n", daqListId));

    /* Check if the Entry values are valid */
    ok = isValidDaqEntry((uint8_T)frame->bitOffset, (uint8_T)frame->size, (uint32_T)frame->address);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE,
        ("XCP_WRITE_DAQ: invalid DAQ %d ODT %d Entry %d value: bitOffset = %xH size = %d addressExtension = %dH address = %xH\n",
         daqListId, odtId, odtEntryId, frame->bitOffset, frame->size, frame->addressExtension, frame->address));

    /* Everything is fine -> proceed and write the entry */
    XCP_PRINTF("XCP_WRITE_DAQ: writing DAQ %d ODT %d Entry %d: bitOffset = %xH size = %d addressExtension = %dH address = %xH\n",
               daqListId, odtId, odtEntryId, frame->bitOffset, frame->size, frame->addressExtension, frame->address);

    entry = &(xcpDynamicDaqLists.daq[daqListId].odt[odtId].entry[odtEntryId]);

    entry->bitOffset        = (uint8_T)frame->bitOffset;
    entry->size             = (uint8_T)frame->size;
    entry->addressExtension = (uint8_T)frame->addressExtension;
    entry->address          = (uint32_T)frame->address;

    /* After a successful WRITE_DAQ, the xcpDaqPtr odtEntryId needs to be post incremented */
    xcpCurrentDaq.odtEntryId++;

    *outputPacketSize = XCP_GENERIC_RES_PACKET_SIZE_IN_BYTES;

    return protoErrorCode;
}

/*****************************************************************************
    XCP SET DAQ LIST MODE
******************************************************************************/
static XcpProtoErrorCode setDaqListModeInputPacketHandler(void   *msgBuffer,
                                                          size_t  xcpPacketOffset,
                                                          size_t *outputPacketSize)
{
    XcpProtoErrorCode protoErrorCode = XCP_PROTO_SUCCESS;
    uint8_T *packet = (uint8_T *)msgBuffer + xcpPacketOffset;
    XcpSetDaqListModeCmdPacketFrame *frame = (XcpSetDaqListModeCmdPacketFrame *)packet;
    XcpDaq *daqList = NULL;
    boolean_T ok = false;

    /* Check if the DAQ List is available */
    ok = (xcpDynamicDaqLists.daq != NULL) && (frame->daqListId < xcpDynamicDaqLists.daqCount);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE, ("SET_DAQ_LIST_MODE: DAQ list %d not available\n", frame->daqListId));

    daqList = &(xcpDynamicDaqLists.daq[frame->daqListId]);

    /* Check if the DAQ list is currently active */
    ok = !isActiveDaqList((uint16_T)frame->daqListId);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_DAQ_ACTIVE, ("SET_DAQ_LIST_MODE: DAQ list %d is currently active\n", frame->daqListId));

    /* Check if the Event Id is valid */
    ok = (frame->eventId < XCP_MAX_EVENT_CHANNEL);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE, ("SET_DAQ_LIST_MODE: invalid event channel Id %d\n", frame->eventId));

    /* Check DAQ list priority */
    ok = (frame->priority == 0);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE, ("SET_DAQ_LIST_MODE: priority not supported, must be zero\n"));

    /* Check if DAQ list mode is supported */
    ok = isSupportedDaqListMode((uint8_T)frame->mode);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_MODE_NOT_VALID, ("SET_DAQ_LIST_MODE: DAQ list mode not supported %xH\n", frame->mode));

    /* Everything is fine -> proceed and set the DAQ list mode */
    XCP_PRINTF("SET_DAQ_LIST_MODE: setting DAQ list %d: mode = %xH, eventId = %d, prescaler = %d, priority = %d\n",
        frame->daqListId, frame->mode, frame->eventId, frame->prescaler, frame->priority);

    daqList->mode       = (uint8_T)frame->mode;
    daqList->eventId    = (uint16_T)frame->eventId;
    daqList->prescaler  = (uint8_T)frame->prescaler;
    daqList->priority   = (uint8_T)frame->priority;

    /* Assuming that prescaler = 0 means value that you are not interested in using it */
    if (daqList->prescaler <= 1)
        daqList->prescaler = 1;

    *outputPacketSize = XCP_GENERIC_RES_PACKET_SIZE_IN_BYTES;

    return protoErrorCode;
}

/*****************************************************************************
    XCP START_STOP_DAQ_LIST
******************************************************************************/
static XcpProtoErrorCode startStopDaqListInputPacketHandler(void   *msgBuffer,
                                                            size_t  xcpPacketOffset,
                                                            size_t *outputPacketSize)
{
    XcpProtoErrorCode protoErrorCode = XCP_PROTO_SUCCESS;
    uint8_T *packet = (uint8_T *)msgBuffer + xcpPacketOffset;
    XcpStartStopDaqListCmdPacketFrame *frame = (XcpStartStopDaqListCmdPacketFrame *)packet;
    XcpDaq *daqList = NULL;
    int32_T daqListId = frame->daqListId;
    boolean_T ok = false;
    boolean_T locked = false;

    /* Check if the DAQ List is available */
    ok = (xcpDynamicDaqLists.daq != NULL) && (daqListId < xcpDynamicDaqLists.daqCount);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE,
        ("START_STOP_DAQ_LIST: DAQ list %d not available\n", daqListId));

    startStopDaqListId = daqListId;
    daqList = &(xcpDynamicDaqLists.daq[daqListId]);

    /* Check if requested mode is valid */
    ok = isValidStartStopMode((uint8_T)frame->mode);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_MODE_NOT_VALID,
        ("START_STOP_DAQ_LIST: invalid mode %d selected \n", frame->mode));

    /* Assign the Absolute ODT number */
    if (absoluteOdtNumberIdentification()) {
        if (daqList->status == XCP_DAQ_INIT) {
            /* The DAQ list has never been started/stopped/selected before
            -> PIDs for the ODTs needs to be reserved */
            if (XCP_READ_BIT_VALUE(daqList->mode, XCP_DAQ_MODE_DIRECTION_MASK)) {
                /* STIM */
                ok = (daqList->odt != NULL) &&
                       ((xcpDynamicDaqLists.firstAvailableStimPid +
                        daqList->odtCount) <= XCP_MAX_STIM_ODT_NUMBER);

                if (ok) {
                    daqList->firstPid = xcpDynamicDaqLists.firstAvailableStimPid;
                    xcpDynamicDaqLists.firstAvailableStimPid += daqList->odtCount;
                }
            }
            else {
                /* DAQ */
                ok = (daqList->odt != NULL) &&
                       ((xcpDynamicDaqLists.firstAvailableDaqPid +
                        daqList->odtCount) <= XCP_MAX_DAQ_ODT_NUMBER);

                if (ok) {
                    daqList->firstPid = xcpDynamicDaqLists.firstAvailableDaqPid;
                    xcpDynamicDaqLists.firstAvailableDaqPid += daqList->odtCount;
                }
            }

            XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_DAQ_CONFIG_ERROR,
                ("START_STOP_DAQ_LIST: cannot assign PIDs for DAQ list %d \n", daqListId));
        }
    }

    /* Check the ODT configuration and also if the ODT sizes fits the Max DTO size */
    if ((frame->mode == XCP_DAQ_LIST_START) || (frame->mode == XCP_DAQ_LIST_SELECT)) {
        int i = 0;

        ok = true;
        for (i = 0; (i < daqList->odtCount) && ok; i++) {
            int j = 0;
            size_t odtSize = 0;

            /* Calculate the resulting ODT size */
            for (j = 0; j < daqList->odt[i].entriesCount; j++) {
                odtSize += daqList->odt[i].entry[j].size;
            }

            /* Check that the resulting ODT size is valid */
            if (IS_TIMESTAMP_REQUIRED(daqList->mode, i)){
                ok = (XCP_IN_BYTES(odtSize * daqList->sampleCount + XCP_IN_AG(XCP_ID_FIELD_SIZE)) + XCP_TIMESTAMP_SIZE) <= xcpTransportMaxDtoSize();
            }
            else{
                ok = (XCP_IN_BYTES(odtSize * daqList->sampleCount + XCP_IN_AG(XCP_ID_FIELD_SIZE))) <= xcpTransportMaxDtoSize();
            }
            XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_DAQ_CONFIG_ERROR,
                ("START_STOP_DAQ_LIST: in DAQ list %d the ODT entries size is larger than Max DTO size for ODT %d \n", daqListId, i));

            /* Update the ODT size */
            daqList->odt[i].size = odtSize;
        }
    }

    /* Everything is fine -> proceed and change the DAQ list status */
    locked = xcpDaqWriteLock(daqList->eventId);

    if (locked) {
        XcpErrorCode createPoolError = XCP_SUCCESS;

        switch (frame->mode) {
        case XCP_DAQ_LIST_STOP:
            XCP_PRINTF("START_STOP_DAQ_LIST: stopping DAQ list %d\n", daqListId);
            daqList->status = XCP_DAQ_STOPPED;
            if (noActiveDaqLists()) {
                /* update status if this was the only active DAQ list */
                xcpStatusSet(XCP_CONNECTED);
                xcpSessionStatusClearMask(XCP_SESSION_DAQ_RUNNING_MASK);
            }
            break;

        case XCP_DAQ_LIST_START:
            XCP_PRINTF("START_STOP_DAQ_LIST: starting DAQ list %d\n", daqListId);

            /* daqList->poolId is initialized with XCP_INVALID_POOL_ID in
               allocDaqInputPacketHandler.

               Allocate a reserved memory pool for the DAQ list, if needed. */
            if (daqList->poolId == XCP_INVALID_POOL_ID) {
                if (IS_USING_CUSTOM_POOL(daqList->eventId)) {
                    /* Forward custom memory information to the memory manager */
                    createPoolError = xcpMemCustomPoolCreate(
                                          xcpEventCustomMemoryManager.allocHandler, 
                                          xcpEventCustomMemoryManager.freeHandler,
                                          &daqList->poolId);
                } else if (daqListId < XCP_MEM_DAQ_RESERVED_POOLS_NUMBER
                    && daqList->eventId < XCP_MIN_EVENT_NO_RESERVED_POOL) {
                    /* Create a reserved memory pool, if not already created */
                    createPoolError = createDaqReservedPool(daqList);
                    daqList->currentSample = 0;
                }
            }

            if (createPoolError == XCP_SUCCESS) {
                daqList->status = XCP_DAQ_STARTED;
                xcpStatusSet(XCP_SYNC_DATA_TRANSFER);
                xcpSessionStatusSetMask(XCP_SESSION_DAQ_RUNNING_MASK);
            }
            break;

        case XCP_DAQ_LIST_SELECT:
            XCP_PRINTF("START_STOP_DAQ_LIST: selected DAQ list %d\n", frame->daqListId);
            daqList->status = XCP_DAQ_SELECTED;
            break;
        }

        xcpDaqWriteUnlock(daqList->eventId);

        *outputPacketSize = XCP_START_STOP_DAQ_LIST_RES_PACKET_SIZE_IN_BYTES;
        XCP_INPUT_PKT_ERROR_IF(createPoolError != XCP_SUCCESS, XCP_PROTO_MEMORY_OVERFLOW,
                ("START_STOP_DAQ_LIST: cannot allocate reserved memory pool for the DAQ list\n"));
    }

    /* Error if the command cannot be executed as the processing of
       at least one event was in progress */
    XCP_INPUT_PKT_ERROR_IF(!locked, XCP_PROTO_BUSY,
        ("START_STOP_DAQ_LIST: cannot be carried out as an event was processed\n"));

    return protoErrorCode;
}

static void startStopDaqListOutputPacketHandler(XcpProtoErrorCode inputCode, void *packet, size_t packetSize)
{
    if (inputCode == XCP_PROTO_SUCCESS) {
        XcpStartStopDaqListResPacketFrame *frame = (XcpStartStopDaqListResPacketFrame *)packet;

        /* Fill the response */
        frame->PID      = XCP_PID_RES;
        frame->firstPid = xcpDynamicDaqLists.daq[startStopDaqListId].firstPid;
    }
    else {
        genericOutputPacketHandler(inputCode, packet, packetSize);
    }
}

/*****************************************************************************
    XCP START_STOP_SYNCH
******************************************************************************/
static XcpProtoErrorCode startStopSynchInputPacketHandler(void   *msgBuffer,
                                                          size_t  xcpPacketOffset,
                                                          size_t *outputPacketSize)
{
    XcpProtoErrorCode protoErrorCode = XCP_PROTO_SUCCESS;
    uint8_T *packet = (uint8_T *)msgBuffer + xcpPacketOffset;
    XcpStartStopSynchCmdPacketFrame *frame = (XcpStartStopSynchCmdPacketFrame *)packet;
    boolean_T ok = false;
    boolean_T locked = false;
    int32_T i = 0;

    /* Check if requested mode is valid */
    ok = isValidStartStopSynchMode((uint8_T)frame->mode);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_MODE_NOT_VALID,
        ("START_STOP_SYNCH: invalid mode %d selected \n", frame->mode));

    /* Check if the DAQ Lists are available */
    ok = (xcpDynamicDaqLists.daq != NULL) && (xcpDynamicDaqLists.daqCount > 0);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE,
        ("START_STOP_SYNCH: DAQ lists not available\n"));

    /* Everything is fine -> proceed and try to change the DAQ list status */

    /* Need to get the lock to guarantee that they are all done in the same cycle */
    locked = xcpDaqLock();

    if (locked) {
        XcpErrorCode createPoolError = XCP_SUCCESS;
        uint16_T daqListId = 0;

        switch (frame->mode) {
        case XCP_DAQ_LIST_STOP_ALL:
            for (i = 0; i < xcpDynamicDaqLists.daqCount; i++) {
                if (xcpDynamicDaqLists.daq[i].status != XCP_DAQ_INIT) {
                    XCP_PRINTF("START_STOP_SYNCH: stopping DAQ list %d\n", i);
                    xcpDynamicDaqLists.daq[i].status = XCP_DAQ_STOPPED;
                }
            }

            /* update status as there are no more active DAQ list */
            xcpStatusSet(XCP_CONNECTED);
            xcpSessionStatusClearMask(XCP_SESSION_DAQ_RUNNING_MASK);
            break;

        case XCP_DAQ_LIST_START_SELECTED:
            /* xcpDynamicDaqLists.daq[i].poolId is initialized with
               XCP_INVALID_POOL_ID in allocDaqInputPacketHandler.

               Allocate a reserved memory pool for the DAQ list, if needed. */
            for (i = 0; (i < xcpDynamicDaqLists.daqCount) && (createPoolError == XCP_SUCCESS); i++) {
                if ((xcpDynamicDaqLists.daq[i].status == XCP_DAQ_SELECTED) && 
                    (xcpDynamicDaqLists.daq[i].poolId == XCP_INVALID_POOL_ID)) {
                    if (IS_USING_CUSTOM_POOL(xcpDynamicDaqLists.daq[i].eventId)) {
                        /* Allocate custom pool if custom memory functions are
                           provided for this eventId */
                        createPoolError = xcpMemCustomPoolCreate(
                                              xcpEventCustomMemoryManager.allocHandler,
                                              xcpEventCustomMemoryManager.freeHandler,
                                              &xcpDynamicDaqLists.daq[i].poolId);
                    } else if (i < XCP_MEM_DAQ_RESERVED_POOLS_NUMBER
                        && xcpDynamicDaqLists.daq[i].eventId < XCP_MIN_EVENT_NO_RESERVED_POOL) {
                        /* Create a reserved memory pool, if not already created */
                        createPoolError = createDaqReservedPool(&xcpDynamicDaqLists.daq[i]);
                        xcpDynamicDaqLists.daq[i].currentSample = 0;
                        daqListId = (uint16_T) i;
                    }
                }
            }

            if (createPoolError == XCP_SUCCESS) {
                /* If the allocation went well for all the DAQ lists, start them */
                for (i = 0; (i < xcpDynamicDaqLists.daqCount); i++) {
                    if (xcpDynamicDaqLists.daq[i].status == XCP_DAQ_SELECTED) {
                        XCP_PRINTF("START_STOP_SYNCH: starting DAQ list %d\n", i);

                        xcpDynamicDaqLists.daq[i].status = XCP_DAQ_STARTED;
                        xcpStatusSet(XCP_SYNC_DATA_TRANSFER);
                        xcpSessionStatusSetMask(XCP_SESSION_DAQ_RUNNING_MASK);
                    }
                }
            }
            break;

        case XCP_DAQ_LIST_STOP_SELECTED:
            for (i = 0; i < xcpDynamicDaqLists.daqCount; i++) {
                if (xcpDynamicDaqLists.daq[i].status == XCP_DAQ_SELECTED) {
                    XCP_PRINTF("START_STOP_SYNCH: stopping DAQ list %d\n", i);
                    xcpDynamicDaqLists.daq[i].status = XCP_DAQ_STOPPED;
                }
            }

            if (noActiveDaqLists()) {
                /* update status if this was the only active DAQ list */
                xcpStatusSet(XCP_CONNECTED);
                xcpSessionStatusClearMask(XCP_SESSION_DAQ_RUNNING_MASK);
            }
            break;
        }

        /* Unlock all the events */
        xcpDaqUnlock();

        (void)daqListId; /* to suppress unused-but-set-variable when XCP_PRINTF is empty */
        XCP_INPUT_PKT_ERROR_IF(createPoolError != XCP_SUCCESS, XCP_PROTO_MEMORY_OVERFLOW,
                ("START_STOP_SYNCH: cannot allocate reserved memory pool for the DAQ list %d\n", daqListId));

        *outputPacketSize = XCP_GENERIC_RES_PACKET_SIZE_IN_BYTES;
    }

    /* Error if the command cannot be executed as the processing of
       at least one event was in progress */
    XCP_INPUT_PKT_ERROR_IF(!locked, XCP_PROTO_BUSY,
        ("START_STOP_SYNCH: cannot be carried out as an event was processed\n"));

    return protoErrorCode;
}

/*****************************************************************************
    XCP GET DAQ PROCESSOR INFO
******************************************************************************/
static XcpProtoErrorCode getDaqProcessorInfoInputPacketHandler(void   *msgBuffer,
                                                               size_t  xcpPacketOffset,
                                                               size_t *outputPacketSize)
{
    XCP_UNUSED_PARAM(msgBuffer);
    XCP_UNUSED_PARAM(xcpPacketOffset);

    XCP_PRINTF("GET DAQ PROCESSOR INFO\n");

    *outputPacketSize =  XCP_GET_DAQ_PROCESSOR_INFO_RES_PACKET_SIZE_IN_BYTES;

    return XCP_PROTO_SUCCESS;
}

static void getDaqProcessorInfoOutputPacketHandler(XcpProtoErrorCode inputCode, void *packet, size_t packetSize)
{
    XcpGetDaqProcessorInfoResPacketFrame *frame = (XcpGetDaqProcessorInfoResPacketFrame *)packet;

    XCP_UNUSED_PARAM(inputCode);
    XCP_UNUSED_PARAM(packetSize);

    /* Fill Get Daq Processor Info response */
    XCP_MEMSET(frame, 0, sizeof(*frame));

    frame->PID = XCP_PID_RES;
    frame->daqProperties = XCP_DAQ_PROPERTIES_VALUE;
    frame->maxDaq = XCP_MAX_DAQ;
    frame->maxEventChannel = XCP_MAX_EVENT_CHANNEL;
    frame->minDaq = XCP_MIN_DAQ;
    frame->daqKeyByte = XCP_DAQ_KEY_VALUE;

    XCP_PRINTF("* DAQ Properties:    %xH\n", frame->daqProperties);
    XCP_PRINTF("* MAX_DAQ:           %d\n", frame->maxDaq);
    XCP_PRINTF("* MAX_EVENT_CHANNEL: %d\n", frame->maxEventChannel);
    XCP_PRINTF("* MIN_DAQ:           %d\n", frame->minDaq);
    XCP_PRINTF("* Daq Key Byte:      %xH\n", frame->daqKeyByte);
}

/*****************************************************************************
    XCP GET DAQ RESOLUTION INFO
******************************************************************************/
static XcpProtoErrorCode getDaqResolutionInfoInputPacketHandler(void   *msgBuffer,
    size_t  xcpPacketOffset,
    size_t *outputPacketSize)
{
    XCP_UNUSED_PARAM(msgBuffer);
    XCP_UNUSED_PARAM(xcpPacketOffset);

    XCP_PRINTF("GET DAQ RESOLUTION INFO\n");

    *outputPacketSize = XCP_GET_DAQ_RESOLUTION_INFO_RES_PACKET_SIZE_IN_BYTES;

    return XCP_PROTO_SUCCESS;
}

static void getDaqResolutionInfoOutputPacketHandler(XcpProtoErrorCode inputCode, void *packet, size_t packetSize)
{
    XcpGetDaqResolutionInfoResPacketFrame *frame = (XcpGetDaqResolutionInfoResPacketFrame *)packet;

    XCP_UNUSED_PARAM(inputCode);
    XCP_UNUSED_PARAM(packetSize);

    /* Fill Get Daq Resolution Info response */
    XCP_MEMSET(frame, 0, sizeof(*frame));

    frame->PID = XCP_PID_RES;

    /* @note in the current version the same limits apply for DAQ and STIM */
    frame->daqOdtEntrySizeGranularity = XCP_ODT_ENTRY_SIZE_GRANULARITY;
    frame->maxDaqOdtEntrySize = XCP_MAX_ODT_ENTRY_SIZE;
    frame->stimOdtEntrySizeGranularity = XCP_ODT_ENTRY_SIZE_GRANULARITY;
    frame->maxStimOdtEntrySize = XCP_MAX_ODT_ENTRY_SIZE;
    frame->timestampMode = XCP_TIMESTAMP_MODE_VALUE;
    frame->timestampTicks = XCP_TIMESTAMP_TICKS;

    XCP_PRINTF("* DAQ ODT Entry size granularity:   %d\n",  frame->daqOdtEntrySizeGranularity);
    XCP_PRINTF("* Max DAQ ODT Entry size:           %d\n",  frame->maxDaqOdtEntrySize);
    XCP_PRINTF("* STIM ODT Entry size granularity:  %d\n",  frame->stimOdtEntrySizeGranularity);
    XCP_PRINTF("* Max STIM ODT Entry size:          %d\n",  frame->maxStimOdtEntrySize);
    XCP_PRINTF("* Timestamp Mode =                  %xH\n", frame->timestampMode);
    XCP_PRINTF("* Timestamp Ticks =                 %d\n",  frame->timestampTicks);
}

/*****************************************************************************
    XCP FREE_DAQ
******************************************************************************/
static XcpProtoErrorCode freeDaqInputPacketHandler(void   *msgBuffer,
                                                   size_t  xcpPacketOffset,
                                                   size_t *outputPacketSize)
{
    XcpProtoErrorCode protoErrorCode = XCP_PROTO_SUCCESS;
    boolean_T ok = false;

    XCP_UNUSED_PARAM(msgBuffer);
    XCP_UNUSED_PARAM(xcpPacketOffset);

    XCP_PRINTF("FREE_DAQ: clearing existing dynamic DAQ lists\n");
    ok = xcpResetDaqListStatus();

    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_BUSY,
        ("FREE_DAQ: cannot be carried out as an event was processed\n"));

    /* No DAQ lists are running -> XCP_SYNC_DATA_TRANSFER cannot be active anymore */
    xcpStatusSet(XCP_CONNECTED);

    /* Everything is fine -> send positive response */
    *outputPacketSize = XCP_GENERIC_RES_PACKET_SIZE_IN_BYTES;

    return protoErrorCode;
}

/*****************************************************************************
    XCP ALLOC_DAQ
******************************************************************************/
static XcpProtoErrorCode allocDaqInputPacketHandler(void   *msgBuffer,
                                                    size_t  xcpPacketOffset,
                                                    size_t *outputPacketSize)
{
    XcpProtoErrorCode protoErrorCode = XCP_PROTO_SUCCESS;
    uint8_T *packet = (uint8_T *)msgBuffer + xcpPacketOffset;
    XcpAllocDaqCmdPacketFrame *frame = (XcpAllocDaqCmdPacketFrame *) packet;
    int32_T daqCount = (uint16_T)frame->daqCount;
    XcpDaq *daq = NULL;
    boolean_T ok = false;
    int i = 0;

    /* Check if DAQ lists have already been allocated */
    ok = xcpDynamicDaqLists.daq == NULL;
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_SEQUENCE_ERROR,
        ("ALLOC_DAQ: sequence error detected, DAQ lists already allocated\n"));

    /* Check if the number of DAQ lists requested is supported */
    ok = (daqCount > 0) && (daqCount <= (XCP_MAX_DAQ - XCP_MIN_DAQ));
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE,
        ("ALLOC_DAQ: invalid number of DAQ lists (%d)\n", daqCount));

    XCP_PRINTF("ALLOC_DAQ: allocating %d DAQ lists (%lu B)\n", daqCount, (unsigned long) ((size_t)daqCount * sizeof(XcpDaq)));

    /* Proceed and allocate the DAQ lists */
    daq = (XcpDaq *)xcpMemAlloc((size_t) daqCount * sizeof(XcpDaq));
    XCP_INPUT_PKT_ERROR_IF(daq == NULL, XCP_PROTO_MEMORY_OVERFLOW,
        ("ALLOC_DAQ: not enough memory to allocate %d DAQ lists\n", daqCount));

    XCP_MEMSET(daq, 0, (size_t)daqCount * sizeof(XcpDaq));

    /* Make sure that the DAQ lists are available for use */
    xcpDynamicDaqLists.daq = daq;
    xcpDynamicDaqLists.daqCount = (uint16_T) daqCount;

    for (i = 0; i < daqCount; i++) {
        daq[i].poolId        = XCP_INVALID_POOL_ID;
        daq[i].packedMode    = XCP_DAQ_DATA_NOT_PACKED;
        daq[i].timestampMode = XCP_DAQ_SINGLE_TIMESTAMP_FIRST_SAMPLE;
        daq[i].sampleCount   = 1;
        daq[i].currentSample = 0;
    }

    *outputPacketSize = XCP_GENERIC_RES_PACKET_SIZE_IN_BYTES;

    return protoErrorCode;
}

/*****************************************************************************
    XCP ALLOC_ODT
******************************************************************************/
static XcpProtoErrorCode allocOdtInputPacketHandler(void   *msgBuffer,
                                                    size_t  xcpPacketOffset,
                                                    size_t *outputPacketSize)
{
    XcpProtoErrorCode protoErrorCode = XCP_PROTO_SUCCESS;
    uint8_T *packet = (uint8_T *)msgBuffer + xcpPacketOffset;
    XcpAllocOdtCmdPacketFrame *frame = (XcpAllocOdtCmdPacketFrame *)packet;
    uint16_T daqListId = (uint16_T)frame->daqListId;
    uint8_T  odtCount  = (uint8_T)frame->odtCount;
    boolean_T ok = false;

    /* Check if DAQ lists have already been allocated */
    ok = (xcpDynamicDaqLists.daq != NULL);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_SEQUENCE_ERROR,
        ("ALLOC_ODT: sequence error detected, DAQ lists not allocated\n"));

    /* Check if the DAQ List Id is valid */
    ok = (daqListId < xcpDynamicDaqLists.daqCount);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE,
        ("ALLOC_ODT: DAQ list id %d is out of range\n", daqListId));

    /* Check if the number of ODTs is valid */
    ok = (odtCount > 0) && (odtCount <= XCP_MAX_DAQ_ODT_NUMBER);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE,
        ("ALLOC_ODT: invalid number of ODT (%d) for DAQ list id %d\n", odtCount, daqListId));

    /* Check if the ODTs have already been allocated */
    ok = (xcpDynamicDaqLists.daq[daqListId].odt == NULL);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_SEQUENCE_ERROR,
        ("ALLOC_ODT: sequence error detected, ODT already allocated\n"));

    XCP_PRINTF("ALLOC_ODT: allocating %d ODTs in DAQ list %d (%lu B)\n",
        odtCount, daqListId, (unsigned long)(odtCount * sizeof(XcpOdt)));

    /* Everything is fine -> proceed and allocate the ODTs */
    xcpDynamicDaqLists.daq[daqListId].odt = (XcpOdt *)xcpMemAlloc(odtCount * sizeof(XcpOdt));
    XCP_INPUT_PKT_ERROR_IF(xcpDynamicDaqLists.daq[daqListId].odt == NULL, XCP_PROTO_MEMORY_OVERFLOW,
        ("ALLOC_ODT: not enough memory to allocate %d ODT in DAQ list %d\n", odtCount, daqListId));

    XCP_MEMSET(xcpDynamicDaqLists.daq[daqListId].odt, 0, odtCount * sizeof(XcpOdt));

    xcpDynamicDaqLists.daq[daqListId].odtCount = odtCount;

    *outputPacketSize = XCP_GENERIC_RES_PACKET_SIZE_IN_BYTES;

    return protoErrorCode;
}

/*****************************************************************************
    XCP ALLOC_ODT_ENTRY
******************************************************************************/
static XcpProtoErrorCode allocOdtEntryInputPacketHandler(void   *msgBuffer,
    size_t  xcpPacketOffset,
    size_t *outputPacketSize)
{
    XcpProtoErrorCode protoErrorCode = XCP_PROTO_SUCCESS;
    uint8_T *packet = (uint8_T *)msgBuffer + xcpPacketOffset;
    XcpAllocOdtEntryCmdPacketFrame *frame = (XcpAllocOdtEntryCmdPacketFrame *)packet;
    uint16_T daqListId       = (uint16_T)frame->daqListId;
    uint8_T  odtId           = (uint8_T)frame->odtId;
    uint8_T  odtEntriesCount = (uint8_T)frame->odtEntriesCount;
    boolean_T ok = false;

    /* Check if the DAQ lists have already been allocated */
    ok = (xcpDynamicDaqLists.daq != NULL);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_SEQUENCE_ERROR,
        ("ALLOC_ODT_ENTRY: sequence error detected, DAQ lists not allocated\n"));

    /* Check if the DAQ List Id is valid */
    ok = (daqListId < xcpDynamicDaqLists.daqCount);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE,
        ("ALLOC_ODT_ENTRY: DAQ list id %d is out of range\n", daqListId));

    /* Check if the ODTs have already been allocated */
    ok = (xcpDynamicDaqLists.daq[daqListId].odt != NULL);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_SEQUENCE_ERROR,
        ("ALLOC_ODT_ENTRY: sequence error detected, ODTs not allocated\n"));

    /* Check if the ODT Id is valid */
    ok = (odtId < xcpDynamicDaqLists.daq[daqListId].odtCount);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE,
        ("ALLOC_ODT_ENTRY: ODT id %d is out of range\n", odtId));

    /* Check if the ODT entries have already been allocated */
    ok = (xcpDynamicDaqLists.daq[daqListId].odt[odtId].entry == NULL);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_SEQUENCE_ERROR,
        ("ALLOC_ODT_ENTRY: sequence error detected, ODT Entries already allocated\n"));

    /* Check if the number of ODT Entries is valid */
    ok = (odtEntriesCount > 0);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE,
        ("ALLOC_ODT_ENTRY: invalid number of ODT entries (%d) for ODT %d in DAQ list %d\n", odtEntriesCount, odtId, daqListId));

    XCP_PRINTF("ALLOC_ODT_ENTRY: allocating %d ODT Entries in DAQ list %d ODT %d (%lu B)\n", 
        odtEntriesCount, daqListId, odtId, (unsigned long)(odtEntriesCount * sizeof(XcpOdtEntry)));

    /* Everything is fine -> proceed and allocate the ODT entries */
    xcpDynamicDaqLists.daq[daqListId].odt[odtId].entry = (XcpOdtEntry *)xcpMemAlloc(odtEntriesCount * sizeof(XcpOdtEntry));

    XCP_INPUT_PKT_ERROR_IF(xcpDynamicDaqLists.daq[daqListId].odt[odtId].entry == NULL, XCP_PROTO_MEMORY_OVERFLOW,
        ("ALLOC_ODT_ENTRY: not enough memory to allocate %d ODT in DAQ list %d ODT %d\n", odtEntriesCount, daqListId, odtId));

    XCP_MEMSET(xcpDynamicDaqLists.daq[daqListId].odt[odtId].entry, 0, odtEntriesCount * sizeof(XcpOdtEntry));

    xcpDynamicDaqLists.daq[daqListId].odt[odtId].entriesCount = odtEntriesCount;

    *outputPacketSize = XCP_GENERIC_RES_PACKET_SIZE_IN_BYTES;

    return protoErrorCode;
}

/*****************************************************************************
    XCP GET DAQ CLOCK
******************************************************************************/

static XcpProtoErrorCode getDaqClockInputPacketHandler(void *msgBuffer,
                                                       size_t xcpPacketOffset,
                                                       size_t *outputPacketSize)
{
    XCP_UNUSED_PARAM(msgBuffer);
    XCP_UNUSED_PARAM(xcpPacketOffset);
    *outputPacketSize = XCP_ERROR_PACKET_SIZE_IN_BYTES;
    return XCP_PROTO_RESOURCE_NOT_ACCESSIBLE;
}

/*****************************************************************************
    XCP LEVEL1 COMMAND
******************************************************************************/

/* The only supported Level 1 command is SET DAQ PACKED MODE */
static XcpProtoErrorCode setDaqPackedModeInputPacketHandler(void   *msgBuffer,
                                                            size_t  xcpPacketOffset,
                                                            size_t *outputPacketSize)
{
    XcpProtoErrorCode protoErrorCode = XCP_PROTO_SUCCESS;
    uint8_T *packet = (uint8_T *)msgBuffer + xcpPacketOffset;
    XcpSetDaqPackedModeCmdPacketFrame *frame = (XcpSetDaqPackedModeCmdPacketFrame *)packet;
    uint16_T daqListId        = (uint16_T) frame->daqListId;
    uint8_T  daqPackedMode    = (uint8_T)  frame->daqPackedMode;
    uint8_T  dpmTimestampMode = (uint8_T)  frame->dpmTimestampMode;
    uint16_T dpmSampleCount   = (uint16_T) frame->dpmSampleCount;
    boolean_T ok = true;

    /* Check if the DAQ lists have already been allocated */
    ok = (xcpDynamicDaqLists.daq != NULL);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_SEQUENCE_ERROR,
        ("SET_DAQ_PACKED_MODE: sequence error detected, DAQ lists not allocated\n"));

    /* Check if the DAQ List Id is valid */
    ok = (daqListId < xcpDynamicDaqLists.daqCount);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE,
        ("SET_DAQ_PACKED_MODE: DAQ list id %d is out of range\n", daqListId));

#if XCP_MEM_DAQ_RESERVED_POOLS_NUMBER > 0
    /* DAQ Packed Mode is only supported for DAQ lists with a reserved memory pool */
    ok = (daqListId < XCP_MEM_DAQ_RESERVED_POOLS_NUMBER);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_MODE_NOT_VALID,
        ("SET_DAQ_PACKED_MODE: packed mode is not supported for DAQ list id %d, with no reserved memory pool\n", daqListId));
#else
    ok = false;
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_MODE_NOT_VALID,
        ("SET_DAQ_PACKED_MODE: packed mode is not supported for DAQ list id %d, with no reserved memory pool\n", daqListId));
#endif

    /* Check if the selected DAQ packed mode is valid. 
       Currently we only support event-grouped data packing */
    ok = ((daqPackedMode == XCP_DAQ_DATA_NOT_PACKED) || 
          (daqPackedMode == XCP_DAQ_EVENT_GROUPED_PACKING));
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_MODE_NOT_VALID,
        ("SET_DAQ_PACKED_MODE: DAQ Packed mode %d is not supported\n", daqPackedMode));

    /* Check if the selected DAQ Timestamp mode is valid. 
       Currently we only support single timestamp of the first sample */
    ok = (dpmTimestampMode == XCP_DAQ_SINGLE_TIMESTAMP_FIRST_SAMPLE);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_MODE_NOT_VALID,
        ("SET_DAQ_PACKED_MODE: DAQ Packed timestamp mode %d is not supported\n", dpmTimestampMode));

    /* Check if the selected DAQ Packed sample count is valid. */
    ok = ((daqPackedMode == XCP_DAQ_DATA_NOT_PACKED && (dpmSampleCount == 1)) ||
         ((daqPackedMode != XCP_DAQ_DATA_NOT_PACKED) && (dpmSampleCount >= XCP_DAQ_PACKED_SAMPLE_COUNT_MIN)));
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_MODE_NOT_VALID,
        ("SET_DAQ_PACKED_MODE: DAQ Packed sample count %d is not supported.\n", dpmSampleCount));

    /* Check that the DAQ list is not already running */
    ok = !isActiveDaqList(daqListId);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_DAQ_ACTIVE,
        ("SET_DAQ_PACKED_MODE: packed mode configuration cannot be changed if the DAQ is running.\n"));

    /* Finally: select the new configuration */
    XCP_PRINTF("SET_DAQ_PACKED_MODE: setting packed mode %d timestamp mode %d sample count %d\n", 
               daqPackedMode, dpmTimestampMode, dpmSampleCount);

    xcpDynamicDaqLists.daq[daqListId].packedMode    = daqPackedMode;
    xcpDynamicDaqLists.daq[daqListId].timestampMode = dpmTimestampMode;
    xcpDynamicDaqLists.daq[daqListId].sampleCount   = dpmSampleCount;
    xcpDynamicDaqLists.daq[daqListId].currentSample = 0;

    *outputPacketSize = XCP_GENERIC_RES_PACKET_SIZE_IN_BYTES;

    return protoErrorCode;
}

static XcpProtoErrorCode level1CommandInputPacketHandler(void  *msgBuffer,
                                                        size_t  xcpPacketOffset,
                                                        size_t *outputPacketSize)
{
    XcpProtoErrorCode protoErrorCode = XCP_PROTO_SUCCESS;
    XcpLevel1CommandPacketFrame* command = (XcpLevel1CommandPacketFrame*) ((uint8_T *)msgBuffer + xcpPacketOffset);
    boolean_T ok = (command->level1Code == XCP_DAQ_LEVEL1_CODE_SET_DAQ_PACKED_MODE);

    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_CMD_UNKNOWN,
        ("DAQ LEVEL1 COMMAND: command code %d is not supported\n", command->level1Code));

    /* The only supported Level 1 command is SET_DAQ_PACKED_MODE */
    protoErrorCode = setDaqPackedModeInputPacketHandler(msgBuffer, xcpPacketOffset, outputPacketSize);

    return protoErrorCode;
}


/** This table contains the list of supported Rx packets and the corresponding handlers */
static const XcpPacketHandlers daqSupportedRxPacket[] =
{
    { XCP_PID_START_STOP_DAQ_LIST,     startStopDaqListInputPacketHandler,     startStopDaqListOutputPacketHandler },
    { XCP_PID_START_STOP_SYNCH,        startStopSynchInputPacketHandler,       genericOutputPacketHandler },
    { XCP_PID_GET_DAQ_PROCESSOR_INFO,  getDaqProcessorInfoInputPacketHandler,  getDaqProcessorInfoOutputPacketHandler},
    { XCP_PID_GET_DAQ_RESOLUTION_INFO, getDaqResolutionInfoInputPacketHandler, getDaqResolutionInfoOutputPacketHandler },
    { XCP_PID_SET_DAQ_PTR,             setDaqPtrInputPacketHandler,            genericOutputPacketHandler },
    { XCP_PID_WRITE_DAQ,               writeDaqInputPacketHandler,             genericOutputPacketHandler },
    { XCP_PID_SET_DAQ_LIST_MODE,       setDaqListModeInputPacketHandler,       genericOutputPacketHandler },
    { XCP_PID_FREE_DAQ,                freeDaqInputPacketHandler,              genericOutputPacketHandler },
    { XCP_PID_ALLOC_DAQ,               allocDaqInputPacketHandler,             genericOutputPacketHandler },
    { XCP_PID_ALLOC_ODT,               allocOdtInputPacketHandler,             genericOutputPacketHandler },
    { XCP_PID_ALLOC_ODT_ENTRY,         allocOdtEntryInputPacketHandler,        genericOutputPacketHandler },
    { XCP_PID_GET_DAQ_CLOCK,           getDaqClockInputPacketHandler,          genericOutputPacketHandler },
    { XCP_PID_LEVEL1_COMMAND,          level1CommandInputPacketHandler,        genericOutputPacketHandler }
};



/*****************************************************************************
    XCP Packet Lookup Function for basic DAQ commands
******************************************************************************/

/* Default DAQ Packet Lookup function, supporting only basic commands
   listed in the table above */
static const XcpPacketHandlers* getPacket(XcpRxPidCode pid)
{
    return xcpFindPacket(pid, daqSupportedRxPacket,
                         XCP_ELEMENTS_NUMBER(daqSupportedRxPacket));
}

static XcpPacketLookupFunction packetLookup = NULL;



/*****************************************************************************
    Public Functions (invoked within the Protocol Layer)
******************************************************************************/
void xcpDaqInit(void)
{
#if !defined(XCP_DAQ_LIST_INDEPENDENT_EVENT_LOCK) || (XCP_DAQ_LIST_INDEPENDENT_EVENT_LOCK == 0)
    XCP_MUTEX_INIT(daqLock);
#else
    uint16_T i = 0;

    /* Initialize Event locks */
    for (i = 0; i < XCP_MAX_EVENT_DATA_NUMBER; i++) {
        XCP_MUTEX_INIT(xcpEventData[i].lock);
    }
#endif

    xcpRunningEventCounter = 0;
    XCP_MEMSET(&xcpDynamicDaqLists, 0, sizeof(xcpDynamicDaqLists));

    /* Initialize the packet lookup function to support only basic
       DAQ commands */
    xcpDaqSetPacketLookup(getPacket);

    /* Initialize support for the extended list of DAQ commands
       @note this may override the default Packet lookup function
             by adding support for more (optional) commands */
    xcpDaqExtendedInit();
}


XcpPacketLookupFunction xcpDaqGetPacketLookup(void)
{
    return packetLookup;
}


void xcpDaqSetPacketLookup(XcpPacketLookupFunction getPacketFcn)
{
    packetLookup = getPacketFcn;
}

/**
 Reset a DAQ list, deleting any partial packet associated with it. This can only happen if the DAQ
 list is using packed mode.
*/
static XcpErrorCode xcpResetDaq(uint16_T daqIndex) {
    int i = 0;
    XcpDaq *daq = NULL;

    XCP_ERROR_IF(
        xcpDynamicDaqLists.daq == NULL || xcpDynamicDaqLists.daqCount <= daqIndex,
        XCP_INV_ARG,
        "xcpProcessDaq: invalid daqIndex");

    daq = &xcpDynamicDaqLists.daq[daqIndex];

    /* Resetting only needs to do something if a DAQ list is using packed mode and has started
        writing the packet */
    if (!isDaqPackedModeEnabled(daqIndex) || daq->currentSample == 0) {
        return XCP_SUCCESS;
    }

    for (i = 0; i < daq->odtCount; ++i) {
        xcpMemFree(daq->odt[i].msgBuffer);
    }

    daq->currentSample = 0;

    return XCP_SUCCESS;
}

/**
 Writes xcp data needed at the beginning of an XCP packet for the given ODT. `currentValuePtr` and
 `offsetBytes` are updated accordingly.
 @note The current implementation only supports absolute ODT number.
*/
static void daqWriteXCPDataForOdt(XcpDaq* daq, uint8_T odtNumber, uint32_T timestamp) {
    XcpOdt *odt = &daq->odt[odtNumber];
    uint8_T *packet = odt->currentValuePtr;

#ifdef XCP_EMULATE_BYTE_ADDRESSABLE_TARGET
    odt->offsetBytes = (XCP_IN_AG(XCP_ID_FIELD_SIZE)) % XCP_HARDWARE_ADDRESS_GRANULARITY_BYTES_NUMBER;
#else
    /* Dummy variable used when we are not emulating as BYTE addressable target */
    odt->offsetBytes = 0;
#endif

    /* Fill the Identification Field first
    @note this is hard-coded to support absolute ODT number at the moment */
    *packet = (uint8_T)(daq->firstPid + odtNumber);
    packet = XCP_HW_PTR_INCREMENT(packet, XCP_IN_AG(XCP_ID_FIELD_SIZE), odt->offsetBytes);

    /* Add the timestamp for the first DTO
    @note this is hard-coded to support timestamp size of 4 bytes */
    if (IS_TIMESTAMP_REQUIRED(daq->mode, odtNumber)) {
    #ifdef XCP_EMULATE_BYTE_ADDRESSABLE_TARGET
        size_t srcOffsetBytes = 0;
        xcpMemcpyByte(packet, odt->offsetBytes, &timestamp, srcOffsetBytes, XCP_IN_HW_BYTES(sizeof(timestamp)));
    #else
        XCP_MEMCPY(packet, &timestamp, sizeof(timestamp));
    #endif

        packet = XCP_HW_PTR_INCREMENT(packet, XCP_IN_AG(XCP_TIMESTAMP_SIZE + odt->offsetBytes), odt->offsetBytes);
        odt->offsetBytes = XCP_HW_BYTE_OFFSET_UPDATE(odt->offsetBytes, XCP_IN_AG(XCP_TIMESTAMP_SIZE));
    }
    odt->currentValuePtr = packet;
}

/**
 Writes the value of the entries for the given ODT, at the memory pointed by `currentValuePtr` and
 `offsetBytes`, which are updated accordingly.
*/
static void odtWriteEntries(XcpOdt *odt) {
    uint8_T entryIndex = 0;
    uint8_T *packet = odt->currentValuePtr;

    for (entryIndex = 0; entryIndex < odt->entriesCount; ++entryIndex) {
        XcpOdtEntry *entry = &(odt->entry[entryIndex]);
        uint8_T const* address = XCP_ADDRESS_GET_READ(entry->addressExtension, entry->address);

        if (entry->bitOffset != 0xFF) {
            /* Bitwise access has been selected */
        #ifdef XCP_EMULATE_BYTE_ADDRESSABLE_TARGET
            uint32_T dword;
            uint8_T bitValue;
            size_t srcOffsetBytes = XCP_BYTE_OFFSET_GET(entry->address);
            xcpMemcpyByte(&dword, 0, address, srcOffsetBytes, 4);
            bitValue = (uint8_T) ((dword & ((uint32_T)(1 << entry->bitOffset))) != 0);
            xcpMemcpyByte(packet, odt->offsetBytes, &bitValue, 0, entry->size);
        #else
            uint32_T const* dword = (uint32_T const*)address;
            *packet = (uint8_T) ((*dword & ((uint32_T)(1 << entry->bitOffset))) != 0);
        #endif
        }
        else {
            /* 'Normal' (non-bitwise) access has been selected */
        #ifdef XCP_EMULATE_BYTE_ADDRESSABLE_TARGET
            size_t srcOffsetBytes = XCP_BYTE_OFFSET_GET(entry->address);
            xcpMemcpyByte(packet, odt->offsetBytes, address, srcOffsetBytes, entry->size);
        #else
            XCP_MEMCPY(packet, address, entry->size);
        #endif
        }

        packet = XCP_HW_PTR_INCREMENT(packet, entry->size + odt->offsetBytes, odt->offsetBytes);
        odt->offsetBytes = XCP_HW_BYTE_OFFSET_UPDATE(odt->offsetBytes, entry->size);
    }

    odt->currentValuePtr = packet;
}

/**
 Process a DAQ, allocate memory, write pid and timestamp if needed, and copies variables values in
 the XCP packet.
 */
static XcpErrorCode xcpProcessDaq(uint16_T daqIndex, uint32_T timestamp, XcpTransportQueueType_T txQueue) {
    XcpErrorCode errorCode = XCP_SUCCESS;
    boolean_T error = false;
    XcpDaq *daq = NULL;
    uint8_T odtIndex = 0;
    boolean_T daqPackedModeEnabled = false;
    boolean_T transmitRequest = false;

    XCP_ERROR_IF(
        xcpDynamicDaqLists.daq == NULL || xcpDynamicDaqLists.daqCount <= daqIndex,
        XCP_INV_ARG,
        "xcpProcessDaq: invalid daqIndex");

    daq = &xcpDynamicDaqLists.daq[daqIndex];

    daqPackedModeEnabled = isDaqPackedModeEnabled(daqIndex);
    /* In packed Mode we only want to transmit when the expected number of 
        samples has been reached */
    transmitRequest = !daqPackedModeEnabled ||
                                ((daq->currentSample + 1) == daq->sampleCount);

    /* @note prescaler is not supported at the moment */
    for (odtIndex = 0; (odtIndex < daq->odtCount) && !error; odtIndex++) {
        XcpOdt *odt = &daq->odt[odtIndex];
        size_t packetOffset = xcpTransportPacketOffset();
        size_t msgBufferSize = 0;

        /* when packed mode is active the allocation occurs at the first sample */
        boolean_T requestNewMessageBuffer = !daqPackedModeEnabled || (daq->currentSample == 0);

        if (requestNewMessageBuffer) {
            /* Calculate the required XCP packet size */
            odt->packetSize = odtPacketSize(daq, odtIndex);

            /* Calculate the overall message buffer */
            msgBufferSize = xcpTransportMsgBufferSize(odt->packetSize, XCP_DTO);

            if (daq->poolId != XCP_INVALID_POOL_ID) {
                /* A dedicated memory pool is available */
                odt->msgBuffer = (uint8_T *)xcpMemAllocFromPool(daq->poolId, msgBufferSize);
            } else {
                /* No memory pool available, use the Main memory instead */
                odt->msgBuffer = (uint8_T *)xcpMemAlloc(msgBufferSize);
            }

            if (odt->msgBuffer != NULL) {
                /* Allocation went well, set current pointer to the beginning of the XCP packet */
                odt->currentValuePtr = odt->msgBuffer + packetOffset;

                /* let's fill the PID and timestamp field if required */
                daqWriteXCPDataForOdt(daq, odtIndex, timestamp);

            } else {
                /* An issue occurred during the memory allocation */
                error = true;
                errorCode = XCP_NO_MEMORY;
            }
        }

        if (!error && (odt->msgBuffer != NULL)) {
            if (!IS_USING_CUSTOM_POOL(daq->eventId)) {
                odtWriteEntries(odt);
            }

            if (transmitRequest) {
                /* Send the response back to the Transport Layer */
                /* Calculate the overall message buffer */
                msgBufferSize = xcpTransportMsgBufferSize(odt->packetSize, XCP_DTO);

                errorCode = xcpTransportTxPacketSet(txQueue,
                                                    daqIndex,
                                                    odt->msgBuffer, msgBufferSize,
                                                    packetOffset, odt->packetSize);
                if (errorCode != XCP_SUCCESS) {
                    xcpMemFree(odt->msgBuffer);
                }
            }
        }
    }

    daq->currentSample = (daq->currentSample + 1) % daq->sampleCount;

    return errorCode;
}

XcpErrorCode xcpDaqEvent(XcpEventIdType eventId, uint32_T timestamp, boolean_T resetSamples)
{
    XcpErrorCode errorCode = XCP_SUCCESS;

    /* The lock is used to protect against the concurrent execution of XCP commands
    (within xcpRun()) that can modify the DAQ list data structures */
    xcpDaqReadLock(eventId);

    if ((xcpDynamicDaqLists.daq != NULL) && (xcpDynamicDaqLists.daqCount > 0)) {
        uint16_T i = 0;
        XcpDaq *daq = xcpDynamicDaqLists.daq;

        for (i = 0; (i < xcpDynamicDaqLists.daqCount) && errorCode == XCP_SUCCESS; i++) {
            if (!XCP_READ_BIT_VALUE(daq[i].mode, XCP_DAQ_MODE_DIRECTION_MASK) && /* it's a DAQ list */
                (daq[i].status == XCP_DAQ_STARTED) &&                            /* it's running */
                (daq[i].eventId == eventId)) {                                   /* and associated to this event */

                boolean_T txReady = xcpTransportTxReady(XCP_TRANSPORT_TX_DEFAULT_QUEUE, i);
                XcpTransportQueueType_T txQueue = txReady ? XCP_TRANSPORT_TX_DEFAULT_QUEUE :
                                                            XCP_TRANSPORT_DEFERRED_TX_QUEUE;

            #ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
                if (txReady &&
                    (daq[i].poolId != XCP_INVALID_POOL_ID)) {
                    /* If the Tx queue is ready, it is safe to flush the free memory,
                        so that the deferred free blocks become available for subsequent allocations */
                    xcpMemReservedPoolFlushFreeMem(daq[i].poolId);
                }
            #endif

                if (resetSamples) {
                    errorCode = xcpResetDaq(i);
                } else {
                    errorCode = xcpProcessDaq(i, timestamp, txQueue);
                }

                if (txReady) {
                /* Trigger the transmission of the packets that have been enqueued
                    for this DAQ list */
                    xcpTransportTxTrigger(XCP_TRANSPORT_TX_DEFAULT_QUEUE, i);
                }

            }
        }
    }

    xcpDaqReadUnlock(eventId);

    return errorCode;
}

XcpErrorCode xcpDaqEventsFlush(uint32_T timestamp)
{
    XcpErrorCode errorCode = XCP_SUCCESS;

    XCP_UNUSED_PARAM(timestamp);

    if (xcpDynamicDaqLists.daq != NULL) {
        boolean_T locked = xcpDaqLock();

        if (locked) {
            XcpDaq  *daq = xcpDynamicDaqLists.daq;
            int32_T  daqCount = xcpDynamicDaqLists.daqCount;
            uint16_T i = 0;

            for (i = 0; i < daqCount; i++) {
                boolean_T daqPackedModeEnabled = isDaqPackedModeEnabled(i);

                if (daqPackedModeEnabled &&
                    (daq[i].status == XCP_DAQ_STARTED) &&
                    (daq[i].currentSample > 0)) {
                    
                    boolean_T txReady = xcpTransportTxReady(XCP_TRANSPORT_TX_DEFAULT_QUEUE, i);
                    XcpTransportQueueType_T txQueue = txReady ? XCP_TRANSPORT_TX_DEFAULT_QUEUE :
                                                            XCP_TRANSPORT_DEFERRED_TX_QUEUE;
                    uint16_T j = 0;

                    for (j = 0; j < daq[i].odtCount; j++) {
                        if (daq[i].odt[j].msgBuffer != NULL) {
                            size_t msgBufferSize = xcpTransportMsgBufferSize(daq[i].odt[j].packetSize, XCP_DTO);
                            size_t packetOffset = xcpTransportPacketOffset();
                            uint8_T *packet = daq[i].odt[j].msgBuffer + packetOffset;
                            size_t notInitializedDataSize = XCP_IN_HW_AG(daq[i].odt[j].packetSize) - 
                                                             ((size_t) (daq[i].odt[j].currentValuePtr - packet));

                            XCP_MEMSET(daq[i].odt[j].currentValuePtr, 0, notInitializedDataSize);

                            errorCode = xcpTransportTxPacketSet(txQueue,
                                                                i,
                                                                daq[i].odt[j].msgBuffer, msgBufferSize,
                                                                packetOffset, daq[i].odt[j].packetSize);
                            if (errorCode != XCP_SUCCESS) {
                                xcpMemFree(daq[i].odt[j].msgBuffer);
                            }

                        }
                    }

                    daq[i].currentSample = 0;
                }
            }

            xcpDaqUnlock();
        } else {
            errorCode = XCP_BUSY;
        }
    }

    return errorCode;
}



boolean_T xcpDaqLock(void)
{
    boolean_T locked = true;
#if !defined(XCP_DAQ_LIST_INDEPENDENT_EVENT_LOCK) || (XCP_DAQ_LIST_INDEPENDENT_EVENT_LOCK == 0)
    locked = xcpDaqWriteLock(0);
#else
    XcpEventIdType i = 0;

    for(i = 0; i < XCP_MAX_EVENT_DATA_NUMBER; i++) {
        xcpDaqWriteLock(i);
    }
#endif
    return locked;
}


/* Unlock DAQ list transmission */
void xcpDaqUnlock(void)
{
#if !defined(XCP_DAQ_LIST_INDEPENDENT_EVENT_LOCK) || (XCP_DAQ_LIST_INDEPENDENT_EVENT_LOCK == 0)
    xcpDaqWriteUnlock(0);
#else
    XcpEventIdType i = 0;

    for(i = 0; i < XCP_MAX_EVENT_DATA_NUMBER; i++) {
        /* Unlock in reverse order, to prevent deadlock */
    	xcpDaqWriteUnlock(XCP_MAX_EVENT_DATA_NUMBER - 1 - i);
    }
#endif
}


boolean_T xcpResetDaqListStatus(void)
{
    boolean_T locked = true;

    if (xcpDynamicDaqLists.daq != NULL) {
        XcpDaq *daq = xcpDynamicDaqLists.daq;
        int32_T  daqCount = xcpDynamicDaqLists.daqCount;

        locked = xcpDaqLock();

        if (locked) {
            /* No events were processed, it's safe to remove the DAQ lists */
            int32_T i = 0;
            int32_T j = 0;

            /* No events are currently running,
               it's safe to remove the DAQ lists */
            xcpDynamicDaqLists.daq = NULL;
            xcpDynamicDaqLists.daqCount = 0;
            xcpDynamicDaqLists.firstAvailableDaqPid = 0;
            xcpDynamicDaqLists.firstAvailableStimPid = 0;
            
            /* Reset ID of the DAQ List selected by START_STOP_DAQ_LIST command */
            startStopDaqListId = XCP_INVALID_DAQ_LIST_ID;

            /* Let's release the locks */
            xcpDaqUnlock();

            /* Prepare the Transport Layer for subsequent connections
               (by resetting the Frame Handler counters and deleting the
               existing packets in TX/RX fifos) */
            xcpTransportRestart();

            /* Delete all the Dynamic DAQ data structures and
               reserved  memory pools */
            for (i = 0; i < daqCount; i++) {
                if (IS_USING_CUSTOM_POOL(daq[i].eventId)) {
                    xcpMemCustomPoolDestroy(daq[i].poolId);
                } else if (i < XCP_MEM_DAQ_RESERVED_POOLS_NUMBER
                 && daq[i].eventId < XCP_MIN_EVENT_NO_RESERVED_POOL) {
                    destroyDaqReservedPool(&daq[i]);
                }

                if (daq[i].odt != NULL) {
                    for (j = 0; j < daq[i].odtCount; j++) {
                        if (daq[i].odt[j].entry != NULL) {
                            xcpMemFree(daq[i].odt[j].entry);
                        }
                    }

                    xcpMemFree(daq[i].odt);
                }
            }

            xcpMemFree(daq);

            xcpSessionStatusClearMask(XCP_SESSION_DAQ_RUNNING_MASK);
        }
    }

    return locked;
}


void xcpDaqReset(void)
{
    xcpResetDaqListStatus();

    /* reset support for the extended list of DAQ commands */
    xcpDaqExtendedReset();

    /* Restore the original value for the lookup function */
    xcpDaqSetPacketLookup(NULL);

#if !defined(XCP_DAQ_LIST_INDEPENDENT_EVENT_LOCK) || (XCP_DAQ_LIST_INDEPENDENT_EVENT_LOCK == 0)
    XCP_MUTEX_INIT(daqLock);
#else
    {
        uint16_T i = 0;

        for (i = 0; i < XCP_MAX_EVENT_DATA_NUMBER; i++) {
            XCP_MUTEX_INIT(xcpEventData[i].lock);
        }
    }
#endif
}


XcpErrorCode xcpDaqSetCustomPoolMemoryManager(XcpEventIdType eventId, 
                                              XcpCustomAllocHandler allocHandler, 
                                              XcpCustomFreeHandler freeHandler)
{
    if (allocHandler == NULL || freeHandler == NULL) {
        return XCP_INV_ARG;
    }

    xcpEventCustomMemoryManager.eventId = eventId;
    xcpEventCustomMemoryManager.allocHandler = allocHandler;
    xcpEventCustomMemoryManager.freeHandler = freeHandler;

    return XCP_SUCCESS;
}


XcpErrorCode xcpDaqGetCustomPoolMemoryManager(XcpEventIdType *eventId,
                                              XcpCustomAllocHandler *allocHandler,
                                              XcpCustomFreeHandler *freeHandler)
{
    if (xcpEventCustomMemoryManager.eventId == XCP_DAQ_CUSTOM_MEMORY_INVALID_EVENT_ID) {
        return XCP_NOT_INITIALIZED;
    }
    if (allocHandler == NULL || freeHandler == NULL) {
        return XCP_INV_ARG;
    }

    *eventId = xcpEventCustomMemoryManager.eventId;
    *allocHandler = xcpEventCustomMemoryManager.allocHandler;
    *freeHandler = xcpEventCustomMemoryManager.freeHandler;

    return XCP_SUCCESS;
}


#ifdef XCP_INTERNAL_DAQ_CONFIG_ACCESS_SUPPORT

XcpErrorCode xcpDaqGetOdtEntries(XcpEventIdType eventId, 
                                 XcpDaqDirection direction, 
                                 XcpOdtEntry* entries,
                                 size_t* entriesNumber,
                                 size_t maxEntriesNumber)
{
    XcpErrorCode errorCode = XCP_SUCCESS;
    XcpOdtEntry* currentEntry = entries;
    size_t entriesCount = 0;
    boolean_T memOverflow = false;

    XCP_ERROR_IF(eventId >= XCP_MAX_EVENT_CHANNEL, XCP_INV_ARG, "xcpDaqGetOdtEntries: invalid eventId\n");
    XCP_ERROR_IF(direction != XCP_DIRECTION_DAQ, XCP_NOT_SUPPORTED, "xcpDaqGetOdtEntries: direction not supported\n");
    XCP_ERROR_IF(entries == NULL, XCP_INV_ARG, "xcpDaqGetOdtEntries: invalid entries\n");
    XCP_ERROR_IF(entriesNumber == NULL, XCP_INV_ARG, "xcpDaqGetOdtEntries: invalid entriesNumber\n");
    XCP_ERROR_IF(maxEntriesNumber == 0, XCP_INV_ARG, "xcpDaqGetOdtEntries: invalid maxEntriesNumber\n");

    /* Extract the ODT entries by navigating the whole DAQ List hierarchy */
    if (xcpDynamicDaqLists.daq != NULL) {
        uint16_T daqId = 0;
        for (daqId = 0; (daqId < xcpDynamicDaqLists.daqCount) && !memOverflow; daqId++) {
            XcpDaq *daq = &xcpDynamicDaqLists.daq[daqId];

            if ((daq->eventId == eventId) &&
                ((direction == XCP_DIRECTION_DAQ) &&
                 !XCP_READ_BIT_VALUE(daq[daqId].mode, XCP_DAQ_MODE_DIRECTION_MASK)) &&
                 isActiveDaqList(daqId)) {
                /* only active DAQ lists are considered */
                if (daq->odt != NULL) {
                    uint8_T odtId = 0;

                    for (odtId = 0; (odtId < daq->odtCount) && !memOverflow; odtId++) {
                        XcpOdt *odt = &daq->odt[odtId];
  
                        if (odt->entry != NULL) {
                            uint8_T entryId = 0;

                            for (entryId = 0; (entryId < odt->entriesCount) && !memOverflow; entryId++) {
                                XcpOdtEntry *odtEntry = &odt->entry[entryId];
                                
                                if (entriesCount < maxEntriesNumber) {
                                    XCP_MEMCPY(currentEntry, odtEntry, sizeof(XcpOdtEntry));
                                    currentEntry++;
                                    entriesCount++;
                                } else {
                                    memOverflow = true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    /* update number of entries copied and error code */
    *entriesNumber = entriesCount;
    if (memOverflow) {
        errorCode = XCP_NO_MEMORY;
    }

    return errorCode;
}

#endif /* XCP_INTERNAL_DAQ_CONFIG_ACCESS_SUPPORT */


#ifndef XCP_DAQ_EXTENDED_SUPPORT

void xcpDaqExtendedInit(void) {}
void xcpDaqExtendedReset(void) {}

#endif

#endif
