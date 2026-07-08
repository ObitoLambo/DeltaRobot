/*
* Copyright 2016-2023 The MathWorks, Inc.
*
* File: xcp.c
*
* Abstract:
*  Implementation of XCP Protocol Layer
*/

#include "xcp_common.h"
#include "xcp.h"
#include "xcp_internal.h"
#include "xcp_transport_internal.h"
#include "xcp_cfg.h"
#include "xcp_types.h"
#include "xcp_standard.h"
#include "xcp_daq.h"
#include "xcp_calibration.h"
#include "xcp_mem.h"


/*****************************************************************************
    Internal Global variables
******************************************************************************/
/* Initialization status */
static boolean_T initialized = false;

/** Current Status of the XCP Server according to the Protocol Layer */
static XcpStatus xcpStatus = XCP_DISCONNECTED;

/** Current Session Status of the XCP Server according to the Protocol Layer
(see XCP Session Status bit masks) */
static uint8_T xcpSessionStatus = 0;

/** XCP Server Resource Protection Status (see XCP Resource bit masks) */
static uint8_T xcpResourceProtectionStatus = 0;

/** XCP Session configuration Id
@note xcpSessionConfigurationId has to be set with an XCP SET_REQUEST
      before a STORE_DAQ_REQ set. This allows the client device to verify that
      automatically started DAQ lists contain the expected data transfer
      configuration.
      However the (optional) SET_REQUEST command is not supported at the
      moment, so this value is hard-coded to 0 and never changed. */
static uint16_T xcpSessionConfigurationId = 0;

/** This table contains the list of XcpPacketLookupFunction to be used
    (for each packets group) to get access to the corresponding
    Packet Input and Output handlers */
static XcpPacketLookupFunction groupPacketLookup[XCP_PACKETS_GROUP_NUMBER];

/*****************************************************************************
    Internal Functions
******************************************************************************/
/** Free the msgBuffer and allocate a new buffer to host a XCP packet of newPacketSize */
static XcpErrorCode msgBufferRealloc(void **msgBuffer, size_t *msgBufferSize, size_t *packetOffset, size_t newPacketSize)
{
    XcpErrorCode errorCode = XCP_SUCCESS;
    xcpPoolId_T reservedPoolId = xcpTransportCtoReservedMemPoolId();

    /* Free the original buffer */
    xcpMemFree(*msgBuffer);

    /* Allocate a new buffer with the proper size */
    *msgBufferSize = xcpTransportMsgBufferSize(newPacketSize, XCP_CTO);
    *packetOffset = xcpTransportPacketOffset();

    if (*msgBufferSize > 0) {
        /* Using a reserved memory pool, to guarantee that the reply
           gets always delivered to the XCP Client */
        *msgBuffer = xcpMemAllocFromPool(reservedPoolId, *msgBufferSize);
        if (*msgBuffer == NULL) {
            errorCode = XCP_NO_MEMORY;
        }
    }
    else {
        errorCode = XCP_INV_SIZE;
    }

    return errorCode;
}

/** Input Packet Handler to be used for packets that need to be discarded */
static XcpProtoErrorCode discardInputPacketHandler(void   *msgBuffer,
                                                   size_t  xcpPacketOffset,
                                                   size_t *outputPacketSize)
{
    XCP_UNUSED_PARAM(xcpPacketOffset);

    XCP_PRINTF("Detected attempt to send packet ID %xH when the XCP connection has not been established\n", 
               *((uint8_T *) msgBuffer + xcpPacketOffset));

    if (msgBuffer != NULL) {
        xcpMemFree(msgBuffer);
    }

    *outputPacketSize = 0;

    return XCP_PROTO_SUCCESS;
}

/** Input Packet Handler to be used for unknown commands */
static XcpProtoErrorCode unknownInputPacketHandler(void   *msgBuffer,
                                                   size_t  xcpPacketOffset,
                                                   size_t *outputPacketSize)
{
    XCP_UNUSED_PARAM(msgBuffer);
    XCP_UNUSED_PARAM(xcpPacketOffset);

    XCP_PRINTF("Packet ID  %xH is not supported at the moment\n",
    *((uint8_T *) msgBuffer + xcpPacketOffset));

    *outputPacketSize = XCP_ERROR_PACKET_SIZE_IN_BYTES;

    return XCP_PROTO_CMD_UNKNOWN;
}

/** The function returns the Group Id corresponding to a given Packet ID */
static XcpPacketsGroupIdType xcpGetPacketGroupID(uint8_T pid, uint8_T level1Code)
{
    XcpPacketsGroupIdType groupId = XCP_UNKNOWN_PACKET_GROUP_ID;

    if ((pid >= XCP_STANDARD_PACKETS_ID_MIN) /* && (pid <= XCP_STANDARD_PACKETS_ID_MAX) */) {
        groupId = XCP_STANDARD_PACKETS_ID;
    }
    else if ((pid >= XCP_CALIBRATION_PACKETS_ID_MIN) && (pid <= XCP_CALIBRATION_PACKETS_ID_MAX)) {
        groupId = XCP_CALIBRATION_PACKETS_ID;
    }
    else if (((pid >= XCP_DAQ_PACKETS_ID_MIN) && (pid <= XCP_DAQ_PACKETS_ID_MAX)) ||
             ((pid == XCP_PID_LEVEL1_COMMAND) && 
             (level1Code >= XCP_DAQ_LEVEL1_CODE_MIN) && (level1Code <= XCP_DAQ_LEVEL1_CODE_MAX))) {
        groupId = XCP_DAQ_PACKETS_ID;
    }
    else if ((pid >= XCP_PROGRAM_PACKETS_ID_MIN) && (pid <= XCP_PROGRAM_PACKETS_ID_MAX)) {
        groupId = XCP_PROGRAM_PACKETS_ID;
    }

    return groupId;
}

/** Identify the internal packet handler functions responsible for processing a specific XCP packet.
    @note If the status is XCP_DISCONNECTED, no packets are accepted except XCP_PID_CONNECT
          If the command is not supported, the XCP server has to reply with a ERR response packet
          and ERR_CMD_UNKNOWN error code */
static void xcpGetPacketHandlers(const uint8_T *packet,
                                 size_t packetSize,
                                 XcpInputPacketHandler  *inputHandler,
                                 XcpOutputPacketHandler *outputHandler)
{
    /* uint8_T gets promoted to a higher data-type. On C2000, it is 
     * unsigned int. Since data is packed, a non-zero MSB in packet[0]
     * can then make PID an invalid identifier. Explicitly mask to
     * remove the upper byte if any.
     */
     const XcpLevel1CommandPacketFrame* command = (const XcpLevel1CommandPacketFrame*) packet;

    uint8_T PID = command->PID; /* Retrieve Packet ID */
    uint8_T level1CommandCode = 0;
    if ((PID == XCP_PID_LEVEL1_COMMAND) && (packetSize > 1)) {
        level1CommandCode = command->level1Code;
    }

    if (xcpStatus != XCP_DISCONNECTED) {
        /* Check if the received XCP packet is supported and
           retrieve the corresponding handler */
        XcpPacketsGroupIdType groupId = xcpGetPacketGroupID(PID, level1CommandCode);
        boolean_T found = false;

        if (groupId != XCP_UNKNOWN_PACKET_GROUP_ID) {
            XcpPacketLookupFunction getPacket = groupPacketLookup[groupId];
            const XcpPacketHandlers *packetInfo = NULL;

            if (getPacket != NULL) {
                packetInfo = getPacket(PID);
                found = (packetInfo != NULL);
                if (found) {
                    *inputHandler = packetInfo->inputHandler;
                    *outputHandler = packetInfo->outputHandler;
                }
            }
        }

        if (!found) {
            /* Packet is not supported*/
            *inputHandler = unknownInputPacketHandler;
            *outputHandler = genericOutputPacketHandler;
        }
    }
    else {
        /* If the XCP Server is in the XCP_DISCONNECTED status,
        * no other XCP packets are accepted except XCP_PID_CONNECT */
        if (PID == XCP_PID_CONNECT) {
            XcpPacketLookupFunction getPacket = groupPacketLookup[XCP_STANDARD_PACKETS_ID];

            *inputHandler = NULL;
            *outputHandler = NULL;

            if (getPacket != NULL) {
                const XcpPacketHandlers *connect = getPacket(XCP_PID_CONNECT);
                if (connect != NULL) {
                    /* Note: if the Protocol layer is correctly configured,
                       the CONNECT command should be always implemented */
                    *inputHandler = connect->inputHandler;
                    *outputHandler = connect->outputHandler;
                }
            }
        }
        else {
            /* Input packet must be ignored and discarded */
            *inputHandler = discardInputPacketHandler;
            *outputHandler = NULL;
        }
    }
}

/*****************************************************************************
    Public Functions (shared across different XCP commands)
******************************************************************************/
XcpStatus xcpStatusGet(void)
{
    return xcpStatus;
}

void xcpStatusSet(XcpStatus status)
{
    xcpStatus = status;
}

uint8_T xcpSessionStatusGet(void)
{
    return xcpSessionStatus;
}

void xcpSessionStatusSet(uint8_T status)
{
    xcpSessionStatus = status;
}

void xcpSessionStatusSetMask(uint8_T mask)
{
    XCP_SET_MASK(xcpSessionStatus, mask);
}

void xcpSessionStatusClearMask(uint8_T mask)
{
    XCP_CLEAR_MASK(xcpSessionStatus, mask);
}

uint8_T xcpResourceProtectionStatusGet(void)
{
    return xcpResourceProtectionStatus;
}

void xcpResourceProtectionStatusSet(uint8_T status)
{
    xcpResourceProtectionStatus = status;
}

void xcpResourceProtectionSetMask(uint8_T mask)
{
    XCP_SET_MASK(xcpResourceProtectionStatus, mask);
}

void xcpResourceProtectionClearMask(uint8_T mask)
{
    XCP_CLEAR_MASK(xcpResourceProtectionStatus, mask);
}

uint16_T xcpSessionConfigurationIdGet(void)
{
    return xcpSessionConfigurationId;
}

void xcpSessionConfigurationIdSet(uint16_T id)
{
    xcpSessionConfigurationId = id;
}

void genericOutputPacketHandler(XcpProtoErrorCode inputCode, void *packet, size_t packetSize)
{
    XCP_UNUSED_PARAM(packetSize);

    if (inputCode == XCP_PROTO_SUCCESS) {
        /* Send back a generic RES packet */
        XcpGenericResPacketFrame *frame = (XcpGenericResPacketFrame *)packet;
        frame->PID = XCP_PID_RES;
    }
    else {
        XcpErrorPacketFrame *frame = (XcpErrorPacketFrame *)packet;
        frame->PID = XCP_PID_ERR;

        switch (inputCode) {
        case XCP_PROTO_MEMORY_OVERFLOW:
            frame->errorCode = XCP_ERR_MEMORY_OVERFLOW;
            break;
        case XCP_PROTO_SEQUENCE_ERROR:
            frame->errorCode = XCP_ERR_SEQUENCE;
            break;
        case XCP_PROTO_OUT_OF_RANGE:
            frame->errorCode = XCP_ERR_OUT_OF_RANGE;
            break;
        case XCP_PROTO_CMD_UNKNOWN:
            frame->errorCode = XCP_ERR_CMD_UNKNOWN;
            break;
        case XCP_PROTO_SYNCH:
            frame->errorCode = XCP_ERR_CMD_SYNC;
            break;
        case XCP_PROTO_WRITE_PROTECTED:
            frame->errorCode = XCP_ERR_WRITE_PROTECTED;
            break;
        case XCP_PROTO_CMD_SYNTAX:
            frame->errorCode = XCP_ERR_CMD_SYNTAX;
            break;
        case XCP_PROTO_DAQ_ACTIVE:
            frame->errorCode = XCP_ERR_DAQ_ACTIVE;
            break;
        case XCP_PROTO_MODE_NOT_VALID:
            frame->errorCode = XCP_ERR_MODE_NOT_VALID;
            break;
        case XCP_PROTO_DAQ_CONFIG_ERROR:
            frame->errorCode = XCP_ERR_DAQ_CONFIG;
            break;
        case XCP_PROTO_BUSY:
            frame->errorCode = XCP_ERR_CMD_BUSY;
            break;
        case XCP_PROTO_ACCESS_DENIED:
            frame->errorCode = XCP_ERR_ACCESS_DENIED;
            break;
        case XCP_PROTO_SEGMENT_UNKNOWN:
            frame->errorCode = XCP_ERR_SEGMENT_NOT_VALID;
            break;
        case XCP_PROTO_PAGE_UNKNOWN:
            frame->errorCode = XCP_ERR_PAGE_NOT_VALID;
            break;
        case XCP_PROTO_GENERIC_ERROR:
            frame->errorCode = XCP_ERR_GENERIC;
            break;
        case XCP_PROTO_RESOURCE_NOT_ACCESSIBLE:
            frame->errorCode = XCP_ERR_RESOURCE_NOT_ACCESSIBLE;
            break;
        default:
            /* This should never happen */
            XCP_PRINTF("genericOutputPacketHandler error: %d input code is not supported", inputCode);
            frame->errorCode = XCP_ERR_CMD_UNKNOWN;
        }
    }
}


const XcpPacketHandlers* xcpFindPacket(XcpRxPidCode pid, const XcpPacketHandlers* packets, size_t packetsNumber)
{
    const XcpPacketHandlers* packet = NULL;

    if (packets != NULL) {
        size_t i = 0;
        for (i = 0; (i < packetsNumber) && (packet == NULL); i++) {
            if (pid == packets[i].PID) {
                packet = &packets[i];
            }
        }
    }

    return packet;
}

/*****************************************************************************
    Public Functions (implementing the Protocol Layer interface)
******************************************************************************/
XcpErrorCode xcpInit(
    int   argc,   /**< [in] number of init parameters              */
    void *argv[]  /**< [in] array of parameters values (C strings) */
    )
{
    XCP_UNUSED_PARAM(argc);
    XCP_UNUSED_PARAM(argv);

    XCP_ERROR_IF(initialized, XCP_ERROR, "xcpInit: protocol layer already initialized\n");

    /* Initialize Standard Commands support */
    xcpStandardInit();
    groupPacketLookup[XCP_STANDARD_PACKETS_ID] = xcpStandardGetPacketLookup();

    /* Initialize DAQ lists (and STIM) support */
    xcpDaqInit();
    groupPacketLookup[XCP_DAQ_PACKETS_ID] = xcpDaqGetPacketLookup();

    /* Initialize Calibration support */
    xcpCalibrationInit();
    groupPacketLookup[XCP_CALIBRATION_PACKETS_ID] = xcpCalibrationGetPacketLookup();

    initialized = true;

    return XCP_SUCCESS;
}

XcpErrorCode xcpRun(void)
{
    XcpErrorCode errorCode = XCP_SUCCESS;
    XcpProtoErrorCode protoErrorCode = XCP_PROTO_SUCCESS;
    uint8_T *msgBuffer = NULL;
    size_t   msgBufferSize = 0;
    size_t   packetOffset = 0;
    size_t   packetSize = 0;
    size_t   outputPacketSize = 0;
    uint8_T *packet = NULL;
    XcpInputPacketHandler  inputHandler  = NULL;
    XcpOutputPacketHandler outputHandler = NULL;

    XCP_ERROR_IF(!initialized, XCP_NOT_INITIALIZED, "xcpRun: protocol layer not initialized\n");

    /* Extract a packet from the transport layer */
    errorCode = xcpTransportRxPacketGet(XCP_TRANSPORT_RX_DEFAULT_QUEUE,
                                       (void**)&msgBuffer, &msgBufferSize,
                                       &packetOffset, &packetSize);
    if ((errorCode != XCP_SUCCESS) &&
        (errorCode != XCP_PKT_OUT_OF_SEQUENCE) && /* If a wrong packet counter is detected,       */
        (errorCode != XCP_PKT_LOST)) {            /* the packet content is still considered valid */
        goto error;
    }

    /* Check buffer sizes and retrieve the pointer to the XCP packet area */
    if ((msgBuffer == NULL) || ((packetOffset + XCP_IN_HW_AG(packetSize)) > msgBufferSize)) { 
        errorCode = XCP_INV_SIZE;
        goto error;
    }

    /* Identify the handler suitable for processing the given XCP packet */
    packet = msgBuffer + packetOffset;
    xcpGetPacketHandlers(packet, packetSize, &inputHandler, &outputHandler);

    /* An input handler needs to be always present */
    if (inputHandler == NULL) {
        XCP_PRINTF("xcpRun: missing input handler for packet ID %xH", packet[0]);
        errorCode = XCP_ERROR;
        goto error;
    }

    /* Process input packet */
    protoErrorCode = inputHandler(msgBuffer, packetOffset, &outputPacketSize);

    if ((outputHandler != NULL) && (outputPacketSize > 0)) {
        errorCode = XCP_SUCCESS;

        /* An Output Packet needs to be generated -> check if current message buffer is big enough for the response packet */
        if (packetSize < outputPacketSize) {
            errorCode = msgBufferRealloc((void **)&msgBuffer, &msgBufferSize, &packetOffset, outputPacketSize);
        }

        XCP_MEMSET(packet, 0, XCP_IN_HW_AG(packetSize));

        if (errorCode == XCP_SUCCESS) {
            /* Fill the response packet */
            packet = msgBuffer + packetOffset;
            outputHandler(protoErrorCode, packet, outputPacketSize);

            /* Send the response back to the Transport Layer
               @note to guarantee the maximum responsiveness even when the TX traffic
                     is maxed out, the highest priority queue is used */
            errorCode = xcpTransportTxPacketSet(XCP_TRANSPORT_TX_HIGHEST_PRIO_QUEUE,
                                                XCP_TRANSPORT_NOT_A_DAQ,
                                                msgBuffer, msgBufferSize,
                                                packetOffset, outputPacketSize);
            if (errorCode != XCP_SUCCESS) {
                goto error;
            }
        }
    }

    return errorCode;

error:
    if (msgBuffer != NULL) {
        xcpMemFree(msgBuffer);
    }

    return errorCode;
}

XcpErrorCode xcpEvent(XcpEventIdType eventId)
{
    XcpErrorCode errorCode = XCP_SUCCESS;
    uint32_T timestamp = 0;

    XCP_ERROR_IF(!initialized, XCP_NOT_INITIALIZED, "xcpEvent: protocol layer not initialized\n");

    timestamp = (uint32_T)XCP_TIMESTAMP_GET();
    
    /* Process DAQ Lists associated to the event */
    errorCode = xcpDaqEvent(eventId, timestamp, false);

    return errorCode;
}

XcpErrorCode xcpEventExternalTimestamp(XcpEventIdType eventId, uint32_T timestamp)
{
    XcpErrorCode errorCode = XCP_SUCCESS;

    XCP_ERROR_IF(!initialized, XCP_NOT_INITIALIZED, "xcpEvent: protocol layer not initialized\n");

    /* Process DAQ Lists associated to the event */
    errorCode = xcpDaqEvent(eventId, timestamp, false);

    return errorCode;
}

XcpErrorCode xcpEventNotificationReq(XcpEventCode eventCode, const void *eventData, size_t eventDataSize)
{
    XcpErrorCode errorCode = XCP_SUCCESS;
    size_t packetOffset = xcpTransportPacketOffset();
    size_t maxEventDataSize = xcpTransportMaxCtoSize() - XCP_EVENT_PACKET_SIZE_IN_BYTES;
    size_t packetSize = 0;
    uint8_T *msgBuffer = NULL;
    size_t msgBufferSize = 0;

    /* Validate input parameters */
    XCP_ERROR_IF((eventDataSize > 0) && (eventData == NULL), XCP_INV_ARG, "xcpEventNotificationReq: invalid eventData buffer\n");
    XCP_ERROR_IF(eventDataSize > maxEventDataSize, XCP_INV_ARG, "xcpEventNotificationReq: invalid eventDataSize\n");

    /* Calculate the required message buffer size */
    packetSize = eventDataSize + XCP_EVENT_PACKET_SIZE_IN_BYTES;

    /* Allocate the message buffer */
    msgBufferSize = xcpTransportMsgBufferSize(packetSize, XCP_CTO);
    msgBuffer = (uint8_T *)xcpMemAlloc(msgBufferSize);

    if (msgBuffer != NULL) {
        XcpEventPacketFrame *frame = (XcpEventPacketFrame *) (msgBuffer + packetOffset);

        /* Fill the EV packet information */
        frame->PID = XCP_PID_EV;
        frame->eventCode = eventCode;

        if (eventDataSize > 0) {
            uint8_T *packetEventData = msgBuffer + packetOffset + XCP_IN_HW_AG(XCP_EVENT_PACKET_SIZE_IN_BYTES);

            /* Copy the optional event data */
            XCP_MEMCPY(packetEventData, eventData, XCP_IN_HW_AG(eventDataSize));
        }

        /* Send the EV packet to the Transport Layer
           @note to guarantee that this packet gets delivered as soon as possible
                 to the XCP Client, the highest priority queue is used */
        errorCode = xcpTransportTxPacketSet(XCP_TRANSPORT_TX_HIGHEST_PRIO_QUEUE,
                                            XCP_TRANSPORT_NOT_A_DAQ,
                                            msgBuffer, msgBufferSize,
                                            packetOffset, packetSize);
        if (errorCode != XCP_SUCCESS) {
            xcpMemFree(msgBuffer);
        }
    }
    else
    {
        errorCode = XCP_NO_MEMORY;
    }

    return errorCode;
}

XcpErrorCode xcpRemoteServiceReq(XcpReqServiceCode serviceReqCode, const void *serviceReqData, size_t serviceReqDataSize)
{
    XcpErrorCode errorCode = XCP_SUCCESS;
    size_t packetOffset = xcpTransportPacketOffset();
    size_t maxServiceReqDataSize = xcpTransportMaxCtoSize() - XCP_SERVICE_REQ_PACKET_SIZE_IN_BYTES;
    size_t packetSize = 0;
    uint8_T *msgBuffer = NULL;
    size_t msgBufferSize = 0;

    /* Validate input parameters */
    XCP_ERROR_IF((serviceReqDataSize > 0) && (serviceReqData == NULL), XCP_INV_ARG, "xcpRemoteServiceReq: invalid serviceReqData buffer\n");
    XCP_ERROR_IF(serviceReqDataSize > maxServiceReqDataSize, XCP_INV_ARG, "xcpRemoteServiceReq: invalid serviceReqDataSize\n");

    /* Calculate the required message buffer size */
    packetSize = serviceReqDataSize + XCP_SERVICE_REQ_PACKET_SIZE_IN_BYTES;

    /* Allocate the message buffer */
    msgBufferSize = xcpTransportMsgBufferSize(packetSize, XCP_CTO);
    msgBuffer = (uint8_T *)xcpMemAlloc(msgBufferSize);

    if (msgBuffer != NULL) {
        XcpServiceReqPacketFrame *frame = (XcpServiceReqPacketFrame *) (msgBuffer + packetOffset);

        /* Fill the SERV packet information */
        frame->PID = XCP_PID_SERV;
        frame->serviceReqCode = serviceReqCode;

        if (serviceReqDataSize > 0) {
            uint8_T *packetServiceReqData = msgBuffer + packetOffset + XCP_IN_HW_AG(XCP_SERVICE_REQ_PACKET_SIZE_IN_BYTES);

            /* Copy the optional event data */
            XCP_MEMCPY(packetServiceReqData, serviceReqData, XCP_IN_HW_AG(serviceReqDataSize));
        }

        /* Send the SERV packet to the Transport Layer
           @note to guarantee that this packet gets delivered as soon as possible
                 to the XCP Client, the highest priority queue is used */
        errorCode = xcpTransportTxPacketSet(XCP_TRANSPORT_TX_HIGHEST_PRIO_QUEUE,
                                            XCP_TRANSPORT_NOT_A_DAQ,
                                            msgBuffer, msgBufferSize,
                                            packetOffset, packetSize);
        if (errorCode != XCP_SUCCESS) {
            xcpMemFree(msgBuffer);
        }
    }
    else
    {
        errorCode = XCP_NO_MEMORY;
    }

    return errorCode;
}

XcpStatus xcpGetStatus(void)
{
    return xcpStatus;
}

XcpErrorCode xcpReset(void)
{
    if (!initialized) {
        XCP_PRINTF("xcpReset: protocol layer already reset\n");
        /* Nothing to do: just printing out a warning message*/
    }

    /* Reset Calibration support */
    xcpCalibrationReset();

    /* Reset DAQ lists (and STIM) support */
    xcpDaqReset();

    /* Reset Standard Commands support */
    xcpStandardReset();

    XCP_MEMSET((void*)groupPacketLookup, 0, sizeof(groupPacketLookup));

    initialized = false;

    return XCP_SUCCESS;
}

XcpErrorCode xcpSetCustomPoolMemoryManager(XcpEventIdType eventId, XcpCustomAllocHandler allocHandler, XcpCustomFreeHandler freeHandler)
{
    XcpErrorCode errorCode;

    /* Forward registration to DAQ list */
    errorCode = xcpDaqSetCustomPoolMemoryManager(eventId, allocHandler, freeHandler);

    return errorCode;
}

XcpErrorCode xcpGetCustomPoolMemoryManager(XcpEventIdType *eventId, XcpCustomAllocHandler *allocHandler, XcpCustomFreeHandler *freeHandler)
{
    XcpErrorCode errorCode;

    /* Forward query to DAQ list */
    errorCode = xcpDaqGetCustomPoolMemoryManager(eventId, allocHandler, freeHandler);

    return errorCode;
}

XcpErrorCode xcpPackedModeEventsFlush(uint32_T timestamp)
{
    XcpErrorCode errorCode;

    errorCode = xcpDaqEventsFlush(timestamp);

    return errorCode;
}

XcpErrorCode xcpPackedModeEventReset(XcpEventIdType eventId)
{
    XcpErrorCode errorCode;
    boolean_T resetSamples = true;

    /* Reset the status of pending packed DAQ lists,
       discarding incomplete packets. */
    errorCode = xcpDaqEvent(eventId, 0, resetSamples);

    return errorCode;
}

#ifdef XCP_INTERNAL_DAQ_CONFIG_ACCESS_SUPPORT

XcpErrorCode xcpGetDaqOdtEntries(XcpEventIdType eventId,
                                 XcpOdtEntry* entries,
                                 size_t* entriesNumber,
                                 size_t maxEntriesNumber)
{
    XcpErrorCode errorCode = XCP_SUCCESS;

    errorCode = xcpDaqGetOdtEntries(eventId, XCP_DIRECTION_DAQ,
                                 entries, entriesNumber,
                                 maxEntriesNumber);
    return errorCode;
}

#endif /* XCP_INTERNAL_DAQ_CONFIG_ACCESS_SUPPORT */


#ifndef XCP_DAQ_SUPPORT

void xcpDaqInit(void) {}
XcpPacketLookupFunction xcpDaqGetPacketLookup(void) {return NULL;}
void xcpDaqSetPacketLookup(XcpPacketLookupFunction getPacket) {XCP_UNUSED_PARAM(getPacket);}
XcpErrorCode xcpDaqEvent(XcpEventIdType eventId, uint32_T timestamp, boolean_T resetSamples) {XCP_UNUSED_PARAM(eventId); XCP_UNUSED_PARAM(timestamp); XCP_UNUSED_PARAM(resetSamples); return XCP_SUCCESS;}
boolean_T xcpResetDaqListStatus(void) {return true;}
XcpErrorCode xcpDaqEventsFlush(uint32_T timestamp) {XCP_UNUSED_PARAM(timestamp); return XCP_SUCCESS;}
boolean_T xcpDaqLock(void) {return true;}
void xcpDaqUnlock(void) {}
void xcpDaqReset(void) {}
XcpErrorCode xcpDaqSetCustomPoolMemoryManager(XcpEventIdType eventId, XcpCustomAllocHandler allocHandler, XcpCustomFreeHandler freeHandler) {
    XCP_UNUSED_PARAM(eventId); XCP_UNUSED_PARAM(allocHandler); XCP_UNUSED_PARAM(freeHandler); return XCP_SUCCESS;}
XcpErrorCode xcpDaqGetCustomPoolMemoryManager(XcpEventIdType *eventId, XcpCustomAllocHandler *allocHandler, XcpCustomFreeHandler *freeHandler) {
    XCP_UNUSED_PARAM(eventId); XCP_UNUSED_PARAM(allocHandler); XCP_UNUSED_PARAM(freeHandler); return XCP_SUCCESS;}

#ifdef XCP_INTERNAL_DAQ_CONFIG_ACCESS_SUPPORT
XcpErrorCode xcpGetDaqOdtEntries(XcpEventIdType eventId, XcpOdtEntry* entries, size_t* entriesNumber, size_t maxEntriesNumber){
    XCP_UNUSED_PARAM(eventId); XCP_UNUSED_PARAM(entries); XCP_UNUSED_PARAM(entriesNumber); XCP_UNUSED_PARAM(maxEntriesNumber); return XCP_SUCCESS;}
#endif /* XCP_INTERNAL_DAQ_CONFIG_ACCESS_SUPPORT */

#endif


#ifndef XCP_CALIBRATION_SUPPORT

void xcpCalibrationInit(void) {}
XcpPacketLookupFunction xcpCalibrationGetPacketLookup(void) {return NULL;}
void xcpCalibrationSetPacketLookup(XcpPacketLookupFunction getPacket) {XCP_UNUSED_PARAM(getPacket);}
void xcpCalibrationReset(void) {}

#endif
