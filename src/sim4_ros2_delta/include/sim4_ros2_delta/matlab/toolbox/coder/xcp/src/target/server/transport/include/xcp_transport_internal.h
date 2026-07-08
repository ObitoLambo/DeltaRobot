/*
* Copyright 2016-2020 The MathWorks, Inc.
*
* File: xcp_transport_internal.h
*
* Abstract:
*  Internal interface for XCP Transport Layer
*  The APIs are intended to be invoked by the XCP Protocol layer only
*/

#ifndef XCP_TRANSPORT_INTERNAL_H
#define XCP_TRANSPORT_INTERNAL_H

#include "xcp.h"
#include "xcp_transport.h"
#include "xcp_mem.h"

typedef int  XcpTransportQueueType_T;

/** Supported TX queues */
#define XCP_TRANSPORT_DEFERRED_TX_QUEUE      -1
#define XCP_TRANSPORT_TX_DEFAULT_QUEUE        0
#define XCP_TRANSPORT_TX_HIGHEST_PRIO_QUEUE   1

/** Supported RX queues */
#define XCP_TRANSPORT_RX_DEFAULT_QUEUE  0

#define XCP_TRANSPORT_NOT_A_DAQ      0xFFFF

typedef enum _XcpPacketType{
    XCP_CTO,
    XCP_DTO
} XcpPacketType;

/** Send a new XCP message to the transport layer
    @note the Protocol layer is responsible for 'filling' the XCP packet part (starting at offset xcpTransportPacketOffset()),
          whereas it is Transport layer responsibility to add the remaining info (e.g. XCP Header and XCP Tail etc.)
          If successfully executed, the ownership of the whole buffer containing the packet will be transferred
          to the Transport Layer, that will then be responsible for freeing it once the message has been transmitted */
XcpErrorCode xcpTransportTxPacketSet(
    XcpTransportQueueType_T queueType, /**< [in] type of the TX queue the packet has to be sent to */
    uint16_T daqId,       /**< [in] id of the DAQ list the packet refers to */
    void  *msgBuffer,     /**< [in] pointer to the buffer containing the full XCP message */
    size_t msgBufferSize, /**< [in] size (in AG units) of the buffer */
    size_t xcpPktOffset,  /**< [in] offset (in AG units) where the actual XCP packet content has been copied (within xcpMsg buffer)  */
    size_t xcpPktSize     /**< [in] size (in bytes) of the XCP packet */
    );


/** Returns true if the transport layer is ready to transmit new packets
    on a given queue and for a specific DAQ list */
boolean_T xcpTransportTxReady(
    XcpTransportQueueType_T queueType, /**< [in] type of the TX queue the packet has to be sent to */
    uint16_T daqId                     /**< [in] id of the DAQ list the packet refers to */
);


/** If lockless streaming is enabled, trigger the transmission of
    packets previously enqueued (by invoking xcpTransportTxPacketSet). */
XcpErrorCode  xcpTransportTxTrigger(
    XcpTransportQueueType_T queueType, /**< [in] type of the TX queue the packet has to be sent to */
    uint16_T daqId                     /**< [in] id of the DAQ list the packet refers to */
);

/** Extract a new XCP message from the transport layer
    @note the Transport layer is also responsible for checking the XCP message integrity,
          returning an error code if an error condition has been detected.
          After the invocation of the function, the Transport layer will not own xcpMsg memory area anymore.
          Therefore the memory needs to be explicitly freed (by the Protocol layer) using xcpMemFree(),
          or re-used for subsequent transmissions.
          The only case where the Transport Layer still owns the memory is when the function is invoked
          using invalid parameters (and returns XCP_INV_ARG) */
XcpErrorCode xcpTransportRxPacketGet(
    XcpTransportQueueType_T queueType, /**< [in] type of the RX queue the packet has to be received from */
    void  **msgBuffer,     /**< [out] pointer to the buffer containing the full XCP message content that has been extracted */
    size_t *msgBufferSize, /**< [out] size (in AG units) of the buffer */
    size_t *xcpPktOffset,  /**< [out] offset (in AG units) of the XCP packet within the message buffer */
    size_t *xcpPktSize     /**< [out] size (in bytes) of the XCP packet */
    );


/** Offset (in AG units) of the XCP packet within the full XCP message buffer
    @note the offset is a constant value and it is implementation dependent.
          In particular it must be equal to the size of the
          XCP Header field (according to XCP Transport Layer specs),
          plus the size of any extra data structure (e.g. Linked List entry)
          that the Transport layer may need for the handling of the RX and
          TX message FIFOs */
size_t xcpTransportPacketOffset(void);


/** Return the size (in AG units) of the XCP message buffer required to store given XCP packet
    or zero if the packet size is invalid.
    The first argument specifies the packet size in BYTEs
    The second argument specifies if this packet is destined to be a CTO or a DTO.
    @note the value is implementation dependent.
    In particular it must be equal to
    sizeof(XCP Header) + packetSize + sizeof(XCP Tail) + sizeof(Transport Layer "internal data structures")
    for the given Transport Layer implementation (see previous comments) */
size_t xcpTransportMsgBufferSize(size_t xcpPacketSize, XcpPacketType pktType);


/** Return the Max DTO (Data Transfer Object) Size (in bytes) */
size_t xcpTransportMaxDtoSize(void);


/** Return the Max CTO (Command Transfer Object) Size (in bytes) */
size_t xcpTransportMaxCtoSize(void);


/** Restart XCP Transport Layer by deleting all the packets (both RX and TX)
    and restarting the Frame Handler */
XcpErrorCode xcpTransportRestart(void);


/** Re-synchronize XCP Transport Layer by deleting all the TX packets
    and restarting the Frame Handler */
XcpErrorCode xcpTransportResynch(void);


/** Retrieve the Id of the memory pool reserved for the allocation of CTO XCP Packets, 
    or -1 if the Id cannot be retrieved */
xcpPoolId_T xcpTransportCtoReservedMemPoolId(void);


#endif /* XCP_TRANSPORT_INTERNAL_H */
