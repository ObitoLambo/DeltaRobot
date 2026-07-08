/*
* Copyright 2016-2021 The MathWorks, Inc.
*
* File: xcp_transport.c
*
* Abstract:
*  Implementation of XCP Transport Layer.
*  The file contains the common part that is responsible for the handling
*  of Tx/Rx buffers and relies on the Xcp Driver and Xcp Frame Handler layers
*  for sending and receiving data to/from the supported transport layer specification
*/

#include "xcp_common.h"
#include "xcp.h"
#include "xcp_transport.h"
#include "xcp_transport_types.h"
#include "xcp_transport_internal.h"
#include "xcp_frame.h"
#include "xcp_fifo.h"
#include "xcp_mem.h"
#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
#include "xcp_daq_types.h"
#endif

/** Number of supported Tx queues */

#define XCP_TRANSPORT_TX_QUEUES_TYPES_NUMBER     2

/** Number of supported Rx queues */
#define XCP_TRANSPORT_RX_QUEUES_TYPES_NUMBER  1


/* In this Transport Layer implementation, the memory buffer associated with a generic
   XCP message has the following format:

   --------------      +---->  --------------
  | xcpFifoEntry | ----+      | xcpFifoEntry |
   --------------              --------------
  | XCP Header   |            | XCP Header   |
   --------------              --------------
  | XCP Packet   |            | XCP Packet   |
   --------------              --------------

  xcpFifoEntry is an internal data structure used to implement a FIFO through a single linked list
*/



/* Transmit FIFOs */
static struct xcpFifo txFifo[XCP_TRANSPORT_TX_QUEUES_TYPES_NUMBER];

static XCP_MUTEX_DEFINE(txLock);

/* Receive FIFOs */
static struct xcpFifo rxFifo[XCP_TRANSPORT_RX_QUEUES_TYPES_NUMBER];

static XCP_MUTEX_DEFINE(rxLock);


#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT

#if XCP_MEM_DAQ_RESERVED_POOLS_NUMBER <= 0
#error  "XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT requires a XCP_MEM_DAQ_RESERVED_POOLS_NUMBER value greater than 0"
#endif

static struct xcpFifo daqDeferredTxFifo[XCP_MEM_DAQ_RESERVED_POOLS_NUMBER];
static struct xcpFifo daqTxFifo[XCP_MEM_DAQ_RESERVED_POOLS_NUMBER];
static boolean_T      daqTxRequest[XCP_MEM_DAQ_RESERVED_POOLS_NUMBER];
static uint16_T       lastDaqIdTx = 0xFFFF;

#endif


/* true if the Transport Layer has been successfully initialized */
static boolean_T  initialized = false;

/* ID of the memory pool reserved for the allocation of
    CTO XCP Packets */
static xcpPoolId_T xcpCtoReservedMemPoolId = -1;


#ifdef XCP_DEBUG_SUPPORT
static const char *xcpTransportDiagDataString[XCP_TRANSPORT_COUNTERS_NUMBER] = {
    "XCP_TRANSPORT_INIT_COUNTER               ",
    "XCP_TRANSPORT_TX_COUNTER                 ",
    "XCP_TRANSPORT_TX_PACKET_SET_COUNTER      ",
    "XCP_TRANSPORT_RX_PACKET_GET_COUNTER      ",
    "XCP_TRANSPORT_RX_COUNTER                 ",
    "XCP_TRANSPORT_RESTART_COUNTER            ",
    "XCP_TRANSPORT_RESYNCH_COUNTER            ",
    "XCP_TRANSPORT_RESET_COUNTER              ",
    "XCP_TRANSPORT_MEM_INIT_ERROR             ",
    "XCP_TRANSPORT_MEM_RESET_ERROR            ",
    "XCP_TRANSPORT_FRAME_INIT_ERROR           ",
    "XCP_TRANSPORT_FRAME_CREATE_MSG_ERROR     ",
    "XCP_TRANSPORT_FRAME_SEND_ERROR           ",
    "XCP_TRANSPORT_FRAME_RECV_ERROR           ",
    "XCP_TRANSPORT_FRAME_EXTRACT_PACKET_ERROR ",
    "XCP_TRANSPORT_FRAME_RESTART_ERROR        ",
    "XCP_TRANSPORT_FRAME_RESET_ERROR          ",
    "XCP_TRANSPORT_TX_PACKET_IN_COUNTER       ",
    "XCP_TRANSPORT_TX_PACKET_OUT_COUNTER      ",
    "XCP_TRANSPORT_TX_PACKET_FREED_COUNTER    ",
    "XCP_TRANSPORT_TX_IS_EMPTY_COUNTER        ",
    "XCP_TRANSPORT_RX_PACKET_IN_COUNTER       ",
    "XCP_TRANSPORT_RX_PACKET_OUT_COUNTER      ",
    "XCP_TRANSPORT_RX_PACKET_FREED_COUNTER    ",
    "XCP_TRANSPORT_RX_IS_EMPTY_COUNTER        "
};

static XcpTransportDiagnostic xcpTransportDiagnostic;


void xcpTransportDiagnosticReset(void)
{
    XCP_MEMSET(&xcpTransportDiagnostic, 0, sizeof(xcpTransportDiagnostic));
}


void xcpTransportDiagnosticGet(XcpTransportDiagnostic *diag)
{
    XCP_MEMCPY(diag, &xcpTransportDiagnostic, sizeof(xcpTransportDiagnostic));
}


void xcpTransportDiagnosticPrint(const XcpTransportDiagnostic *diag)
{
    if (diag != NULL) {
        int i = 0;
        for (i = 0; i < XCP_TRANSPORT_COUNTERS_NUMBER; i++) {
            XCP_PRINTF("%s = %lu\n", xcpTransportDiagDataString[i], (unsigned long) xcpTransportDiagnostic.data[i]);
        }
    }
}

#define XCP_DIAG_UPDATE(counterId)     xcpTransportDiagnostic.data[counterId]++

#else

#define XCP_DIAG_UPDATE(counterId) 

#endif

/* Empty the TX FIFO by deleting the existing packets */
static void xcpTransportEmptyTxFifo(void)
{
    struct xcpFifoEntry *msgBufferHeader = NULL;
    uint16_T i = 0;

    XCP_MUTEX_LOCK(txLock);

    for (i = 0; i < XCP_TRANSPORT_TX_QUEUES_TYPES_NUMBER; i++) {
        do {
            xcpFifoDequeue(&txFifo[i], &msgBufferHeader);

            if (msgBufferHeader != NULL) {
                xcpMemFree(msgBufferHeader);
                XCP_DIAG_UPDATE(XCP_TRANSPORT_TX_PACKET_FREED_COUNTER);
            }
        } while (msgBufferHeader != NULL);
    }

    XCP_MUTEX_UNLOCK(txLock);

#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
    for (i = 0; i < XCP_MEM_DAQ_RESERVED_POOLS_NUMBER; i++) {
        do {
            xcpFifoDequeue(&daqTxFifo[i], &msgBufferHeader);

            if (msgBufferHeader != NULL) {
                xcpMemFree(msgBufferHeader);
                XCP_DIAG_UPDATE(XCP_TRANSPORT_TX_PACKET_FREED_COUNTER);
            }
        } while (msgBufferHeader != NULL);

        do {
            xcpFifoDequeue(&daqDeferredTxFifo[i], &msgBufferHeader);

            if (msgBufferHeader != NULL) {
                xcpMemFree(msgBufferHeader);
                XCP_DIAG_UPDATE(XCP_TRANSPORT_TX_PACKET_FREED_COUNTER);
            }
        } while (msgBufferHeader != NULL);

        daqTxRequest[i] = false;
    }
    lastDaqIdTx = 0xFFFF;
#endif
}


/* Empty the RX FIFO by deleting the existing packets */
static void xcpTransportEmptyRxFifo(void)
{
    struct xcpFifoEntry *msgBufferHeader = NULL;
    XcpTransportQueueType_T i = 0;

    XCP_MUTEX_LOCK(rxLock);

    for (i = 0; i < XCP_TRANSPORT_RX_QUEUES_TYPES_NUMBER; i++) {
        do {
            xcpFifoDequeue(&rxFifo[i], &msgBufferHeader);

            if (msgBufferHeader != NULL) {
                xcpMemFree(msgBufferHeader);
                XCP_DIAG_UPDATE(XCP_TRANSPORT_RX_PACKET_FREED_COUNTER);
            }
        } while (msgBufferHeader != NULL);
    }

    XCP_MUTEX_UNLOCK(rxLock);
}


XcpErrorCode xcpTransportInit(int argc, void * argv[])
{
    XcpErrorCode errorCode = XCP_SUCCESS;
    uint16_T i = 0;

    XCP_ERROR_IF(initialized, XCP_ERROR, "xcpTransportInit: transport layer already initialized\n");

    XCP_DIAG_UPDATE(XCP_TRANSPORT_INIT_COUNTER);

    /* Initialize Tx and Rx data structures*/
    for (i = 0; i < XCP_TRANSPORT_TX_QUEUES_TYPES_NUMBER; i++) {
        xcpFifoInit(&txFifo[i]);
    }
    XCP_MUTEX_INIT(txLock);

#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
    for (i = 0; i < XCP_MEM_DAQ_RESERVED_POOLS_NUMBER; i++) {
        xcpFifoInit(&daqTxFifo[i]);
        xcpFifoInit(&daqDeferredTxFifo[i]);
        daqTxRequest[i] = false;
    }
    lastDaqIdTx = 0xFFFF;
#endif

    for (i = 0; i < XCP_TRANSPORT_RX_QUEUES_TYPES_NUMBER; i++) {
        xcpFifoInit(&rxFifo[i]);
    }
    XCP_MUTEX_INIT(rxLock);

    /* Initialize XCP memory allocator */
    errorCode = xcpMemInit();
    if (errorCode != XCP_SUCCESS) {
        XCP_PRINTF("xcpTransportInit: error (%d) detected during memory allocator initialization\n", errorCode);
        XCP_DIAG_UPDATE(XCP_TRANSPORT_MEM_INIT_ERROR);
        return errorCode;
    }

    {
        /* Allocate the memory pool reserved for the allocation
           of CTO, EV and SERV XCP Packets */
        size_t maxCtoSize = xcpTransportMaxCtoSize();
        size_t maxBufferSize = xcpTransportMsgBufferSize(maxCtoSize, XCP_CTO);

        errorCode = xcpMemReservedPoolCreate(maxBufferSize, XCP_MEM_CTO_RESERVED_POOL_BLOCKS_NUMBER,
                                             &xcpCtoReservedMemPoolId);

        if (errorCode != XCP_SUCCESS) {
            XCP_PRINTF("xcpTransportInit: error (%d) detected during allocation of reserved memory pool\n", errorCode);
            XCP_DIAG_UPDATE(XCP_TRANSPORT_MEM_INIT_ERROR);
            return errorCode;
        }

        xcpFrameSetCtoReservedMemPoolId(xcpCtoReservedMemPoolId);
    }

    /* Initialize XCP Frame handler */
    errorCode = xcpFrameInit(argc, argv);
    if (errorCode != XCP_SUCCESS) {
        XCP_PRINTF("xcpTransportInit: error (%d) detected during frame handler initialization\n", errorCode);
        XCP_DIAG_UPDATE(XCP_TRANSPORT_FRAME_INIT_ERROR);
        return errorCode;
    }

    initialized = true;

    return errorCode;
}


#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT

boolean_T xcpTransportTxReady(
    XcpTransportQueueType_T queueType, /**< [in] type of the TX queue the packet has to be sent to */
	uint16_T daqId                     /**< [in] id of the DAQ list the packet refers to */
)
{
    boolean_T ready = true;
    /* In the lockless implementation we enqueue data only
       if the previous transmission has been completed */
    if ((queueType == XCP_TRANSPORT_TX_DEFAULT_QUEUE) &&
        (daqId <  XCP_MEM_DAQ_RESERVED_POOLS_NUMBER)) {
        ready = !daqTxRequest[daqId];
    }

    return ready;
}


/** If lockless streaming is enabled, trigger the transmission of
    packets previously enqueued (by invoking xcpTransportTxPacketSet.
    The function returns XCP_BUSY if the TX request is already
    in progress */
XcpErrorCode  xcpTransportTxTrigger(
    XcpTransportQueueType_T queueType, /**< [in] type of the TX queue the packet has to be sent to */
    uint16_T daqId                     /**< [in] id of the DAQ list the packet refers to */
)
{
    XcpErrorCode errorCode = XCP_SUCCESS;

    if ((queueType != XCP_TRANSPORT_TX_DEFAULT_QUEUE) ||
        (daqId >= XCP_MEM_DAQ_RESERVED_POOLS_NUMBER)) {
        return XCP_INV_ARG;
    }

    if (!daqTxRequest[daqId]) {
        if (!xcpFifoEmpty(daqDeferredTxFifo)) {
            /* If the transmission of any packets was deferred 
               from previous run, we need to enqueue them at the
               head of the daqTxFifo, as they need to be sent first */
            xcpFifoSpliceHead(daqTxFifo, daqDeferredTxFifo);
        }

        XCP_MEM_BARRIER();
        daqTxRequest[daqId] = true;
    } else {
        errorCode = XCP_BUSY;
    }

    return errorCode;
}

#else

boolean_T xcpTransportTxReady(
    XcpTransportQueueType_T queueType, /**< [in] type of the TX queue the packet has to be sent to */
    uint16_T daqId                     /**< [in] id of the DAQ list the packet refers to */
)
{
    XCP_UNUSED_PARAM(queueType);
    XCP_UNUSED_PARAM(daqId);

    /* In the standard (mutex-based) implementation we can always enqueue
       new packets and therefore TX is always ready*/
    return true;
}


XcpErrorCode  xcpTransportTxTrigger(
    XcpTransportQueueType_T queueType, /**< [in] type of the TX queue the packet has to be sent to */
    uint16_T daqId                     /**< [in] id of the DAQ list the packet refers to */
)
{
    /* In the standard (mutex-based) implementation the transmission
       is always in progress, as long as there are samples in the TX fifo.
       For this reasons this method has no effect in this case */
    XCP_UNUSED_PARAM(queueType);
    XCP_UNUSED_PARAM(daqId);

    return XCP_SUCCESS;
}

#endif


XcpErrorCode xcpTransportTxPacketSet(
    XcpTransportQueueType_T queueType, /**< [in] id of the TX queue the packet has to be sent to */
    uint16_T daqId,       /**< [in] id of the DAQ list the packet refers to */
    void  *msgBuffer,       /* [in] pointer to the buffer containing the full XCP message */
    size_t msgBufferSize,   /* [in] size (in AG units) of the buffer */
    size_t xcpPktOffset,    /* [in] offset (in AG units) where the actual XCP packet content has been copied (within xcpMsg buffer)  */
    size_t xcpPktSize       /* [in] size (in BYTEs) of the XCP packet */
    )
{
    struct xcpFifoEntry *msgBufferHeader = NULL;
    uint8_T *msgFrame = NULL;
    size_t msgFrameMaxSize = 0;
    size_t msgFrameSize = 0;
    XcpErrorCode errorCode = XCP_SUCCESS;
#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
    /* Lock must be excluded for DAQ lists associated to reserved memory pools */
    boolean_T locklessTx = ((queueType == XCP_TRANSPORT_TX_DEFAULT_QUEUE)   ||
                            (queueType == XCP_TRANSPORT_DEFERRED_TX_QUEUE)) &&
                            (daqId < XCP_MEM_DAQ_RESERVED_POOLS_NUMBER);
#else
    XCP_UNUSED_PARAM(daqId);
#endif

    /* Input parameters validation */
    XCP_ERROR_IF((queueType != XCP_TRANSPORT_DEFERRED_TX_QUEUE) && 
                 ((queueType < 0) || (queueType >= XCP_TRANSPORT_TX_QUEUES_TYPES_NUMBER)), 
                 XCP_INV_ARG, "xcpTransportTxPacketSet: invalid queueType\n");
    XCP_ERROR_IF(msgBuffer == NULL, XCP_INV_ARG, "xcpTransportTxPacketSet: invalid msgBuffer\n");
    XCP_ERROR_IF(xcpPktOffset != xcpTransportPacketOffset(), XCP_INV_ARG, "xcpTransportTxPacketSet: invalid packet offset\n");
    XCP_ERROR_IF(xcpPktSize == 0, XCP_INV_ARG, "xcpTransportTxPacketSet: invalid packet size\n");
    XCP_ERROR_IF(msgBufferSize < (xcpPktOffset + XCP_IN_HW_AG(xcpPktSize)), XCP_INV_ARG, "xcpTransportTxPacketSet: invalid msgBufferSize\n");
    XCP_ERROR_IF(!initialized, XCP_NOT_INITIALIZED, "xcpTransportTxPacketSet: transport layer not initialized\n");

    XCP_DIAG_UPDATE(XCP_TRANSPORT_TX_PACKET_SET_COUNTER);

    msgBufferHeader = (struct xcpFifoEntry *) msgBuffer;
    msgFrame = (uint8_T *)msgBuffer + sizeof(struct xcpFifoEntry);
    msgFrameMaxSize = msgBufferSize - sizeof(struct xcpFifoEntry);

#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
    if (!locklessTx) {
#endif
    /* Now fill the XCP frame content 
       @note the lock is used to protect the txFifo data structures from concurrent execution */
    XCP_MUTEX_LOCK(txLock);

#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
    }
#endif

    errorCode = xcpFrameCreateMsg(msgFrame, msgFrameMaxSize, &msgFrameSize, xcpPktSize);
    if (errorCode != XCP_SUCCESS) {
        XCP_PRINTF("xcpTransportTxPacketSet: xcpFrameCreateMsg failure detected (%d)\n", errorCode);
        XCP_DIAG_UPDATE(XCP_TRANSPORT_FRAME_CREATE_MSG_ERROR);
        goto error;
    }

    msgBufferHeader->msgFrameSize = msgFrameSize;

#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
    if (!locklessTx) {
#endif
        /* Now that the packet has been created, we can add it to the txFifo for transmission */
        xcpFifoEnqueue(&txFifo[queueType], msgBufferHeader);

#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
    } else {
        if (queueType == XCP_TRANSPORT_DEFERRED_TX_QUEUE) {
            xcpFifoEnqueue(&daqDeferredTxFifo[daqId], msgBufferHeader);
        } else {
         xcpFifoEnqueue(&daqTxFifo[daqId], msgBufferHeader);
        }
    }
#endif

    XCP_DIAG_UPDATE(XCP_TRANSPORT_TX_PACKET_IN_COUNTER);

error:
#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
    if (!locklessTx) {
#endif
    XCP_MUTEX_UNLOCK(txLock);
#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
    }
#endif

    return errorCode;
}


XcpErrorCode xcpTransportTx(void)
{
    XcpErrorCode errorCode = XCP_SUCCESS;
    size_t msgFrameSize = 0;
    struct xcpFifoEntry *msgBufferHeader = NULL;
    XcpTransportQueueType_T queueType = XCP_TRANSPORT_TX_HIGHEST_PRIO_QUEUE;
#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
    /* Lock must be excluded for DAQ lists associated to reserved memory pools */
    boolean_T locklessTx = false;
#endif

    XCP_ERROR_IF(!initialized, XCP_NOT_INITIALIZED, "xcpTransportTx: transport layer not initialized\n");

    XCP_DIAG_UPDATE(XCP_TRANSPORT_TX_COUNTER);

    XCP_MUTEX_LOCK(txLock);
    /* Extract the message from the Fifos, starting from the highest priority */
    xcpFifoDequeue(&txFifo[queueType], &msgBufferHeader);

    XCP_MUTEX_UNLOCK(txLock);

#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
    if (msgBufferHeader == NULL) {
        uint16_T i = 0;
        /* If the highest priority is empty, check if one of the DAQ-specific ones
           has a pending request */
        for (i = 0; (i < XCP_MEM_DAQ_RESERVED_POOLS_NUMBER) && !locklessTx; i++) {
            lastDaqIdTx = (lastDaqIdTx + 1) %  XCP_MEM_DAQ_RESERVED_POOLS_NUMBER;
            if (daqTxRequest[lastDaqIdTx]) {
                xcpFifoDequeue(&daqTxFifo[lastDaqIdTx], &msgBufferHeader);
                if (msgBufferHeader == NULL) {
                    /* if no data are available, we can acknowledge the request
                       straight away */
                    daqTxRequest[lastDaqIdTx] = false;
                } else {
                    locklessTx = true;
                }
            }
        }
    }
#endif
    if (msgBufferHeader == NULL) {
        /* If the highest priority is empty, and no DAQ packets are available
           try the default queue */
        XCP_MUTEX_LOCK(txLock);

        queueType = XCP_TRANSPORT_TX_DEFAULT_QUEUE;
        xcpFifoDequeue(&txFifo[queueType], &msgBufferHeader);
        XCP_MUTEX_UNLOCK(txLock);
    }

    if (msgBufferHeader != NULL) {
        msgFrameSize = msgBufferHeader->msgFrameSize;

        /* Trying to send one XCP message via the frame handler
           @note The frame handler will be responsible for releasing
                 the message buffer if the data has been successfully sent
                 (return code XCP_SUCCESS).
                 If not, the memory area won't be freed and the message
                 will need to go back to the FIFO. */
        errorCode = xcpFrameMsgSend(msgBufferHeader, sizeof(struct xcpFifoEntry), msgFrameSize);

        if (errorCode != XCP_SUCCESS) {
            XCP_DIAG_UPDATE(XCP_TRANSPORT_FRAME_SEND_ERROR);

#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
            if (!locklessTx) {
#endif
            XCP_MUTEX_LOCK(txLock);

            xcpFifoEnqueueHead(&txFifo[queueType], msgBufferHeader);

            XCP_MUTEX_UNLOCK(txLock);
#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
            } else {
                xcpFifoEnqueueHead(&daqTxFifo[lastDaqIdTx], msgBufferHeader);
            }
#endif
        } else {
            XCP_DIAG_UPDATE(XCP_TRANSPORT_TX_PACKET_OUT_COUNTER);

#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
            if (locklessTx && xcpFifoEmpty(&daqTxFifo[lastDaqIdTx])) {
                /* All packets in the queue have been sent,
                   the request can be acknowledged */
                XCP_MEM_BARRIER();
                daqTxRequest[lastDaqIdTx] = false;
            }
#endif
        }
    } else {
        errorCode = XCP_EMPTY;
        XCP_DIAG_UPDATE(XCP_TRANSPORT_TX_IS_EMPTY_COUNTER);
    }

    return errorCode;
}


XcpErrorCode xcpTransportRx(void)
{
    XcpErrorCode errorCode = XCP_SUCCESS;
    size_t msgFrameSize = 0;
    struct xcpFifoEntry *msgBufferHeader = NULL;

    XCP_ERROR_IF(!initialized, XCP_NOT_INITIALIZED, "xcpTransportRx: transport layer not initialized\n");

    XCP_DIAG_UPDATE(XCP_TRANSPORT_RX_COUNTER);

#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
    /* Make sure that the blocks associated to the CTO reserved memory
       pool are made available for subsequent allocations */
    xcpMemReservedPoolFlushFreeMem(xcpCtoReservedMemPoolId);
#endif

    /* Trying to receive one XCP message via the frame handler
       @note The frame handler is also responsible for allocating the
             memory area and it is no longer responsible for the ownership
             of the memory if the function has been successfully executed
             (error code XCP_SUCCESS)
             If there are no messages available, the return code is XCP_EMPTY */
    errorCode = xcpFrameMsgRecv((void**) &msgBufferHeader, sizeof(struct xcpFifoEntry), &msgFrameSize);

    if (errorCode == XCP_SUCCESS) {
        /* Add the message to the Fifo */
        msgBufferHeader->msgFrameSize = msgFrameSize;

        XCP_MUTEX_LOCK(rxLock);

        /* Only one RX queue available */
        xcpFifoEnqueue(&rxFifo[XCP_TRANSPORT_RX_DEFAULT_QUEUE], msgBufferHeader);
        XCP_DIAG_UPDATE(XCP_TRANSPORT_RX_PACKET_IN_COUNTER);

        XCP_MUTEX_UNLOCK(rxLock);
    } else if (errorCode == XCP_EMPTY) {
        XCP_DIAG_UPDATE(XCP_TRANSPORT_RX_IS_EMPTY_COUNTER);
    } else {
        XCP_DIAG_UPDATE(XCP_TRANSPORT_FRAME_RECV_ERROR);
    }

    return errorCode;
}


XcpErrorCode xcpTransportRxPacketGet(
    XcpTransportQueueType_T queueType, /**< [in] id of the RX queue the packet has to be received from */
    void  **msgBuffer,     /* [out] pointer to the buffer containing the full XCP message that has been extracted */
    size_t *msgBufferSize, /* [out] size (in AG units) of the buffer */
    size_t *xcpPktOffset,  /* [out] offset (in AG units) of the XCP packet within the message buffer */
    size_t *xcpPktSize     /* [out] size (in BYTEs) of the XCP packet */
    )
{
    struct xcpFifoEntry *msgBufferHeader = NULL;
    uint8_T *msgFrame = NULL;
    size_t msgFrameSize = 0;
    XcpErrorCode errorCode = XCP_SUCCESS;

    /* Input parameters validation */
    XCP_ERROR_IF((queueType < 0) || (queueType >= XCP_TRANSPORT_RX_QUEUES_TYPES_NUMBER), XCP_INV_ARG, "xcpTransportRxPacketGet: invalid queueType\n");
    XCP_ERROR_IF(msgBuffer == NULL, XCP_INV_ARG, "xcpTransportRxPacketGet: invalid msgBuffer pointer\n");
    XCP_ERROR_IF(msgBufferSize == NULL, XCP_INV_ARG, "xcpTransportRxPacketGet: invalid msgBufferSize pointer\n");
    XCP_ERROR_IF(xcpPktOffset == NULL, XCP_INV_ARG, "xcpTransportRxPacketGet: invalid xcpPktOffset pointer\n");
    XCP_ERROR_IF(xcpPktSize == NULL, XCP_INV_ARG, "xcpTransportRxPacketGet: invalid xcpPktSize pointer\n");
    XCP_ERROR_IF(!initialized, XCP_NOT_INITIALIZED, "xcpTransportRxPacketGet: transport layer not initialized\n");

    XCP_DIAG_UPDATE(XCP_TRANSPORT_RX_PACKET_GET_COUNTER);

    XCP_MUTEX_LOCK(rxLock);

    xcpFifoDequeue(&rxFifo[queueType], &msgBufferHeader);

    XCP_MUTEX_UNLOCK(rxLock);

    if (msgBufferHeader == NULL) {
        return XCP_EMPTY; /* No messages in the FIFO */
    }
    else {
        XCP_DIAG_UPDATE(XCP_TRANSPORT_RX_PACKET_OUT_COUNTER);
    }

    msgFrame = (uint8_T *) msgBufferHeader + sizeof(struct xcpFifoEntry);
    msgFrameSize = msgBufferHeader->msgFrameSize;

    /* Extract a new XCP packet from the XCP message, and carry out all the relevant checks in the frame format */
    errorCode = xcpFrameExtractPacket(msgFrame, msgFrameSize, xcpPktSize);
    if (errorCode != XCP_SUCCESS) {
        XCP_DIAG_UPDATE(XCP_TRANSPORT_FRAME_EXTRACT_PACKET_ERROR);
    }

    *msgBuffer = msgBufferHeader;
    *msgBufferSize = msgBufferHeader->msgFrameSize + sizeof(struct xcpFifoEntry);
    *xcpPktOffset = xcpTransportPacketOffset();

    return errorCode;
}


size_t xcpTransportPacketOffset(void)
{
    size_t size = sizeof(struct xcpFifoEntry);

    size += XCP_IN_HW_AG(xcpFrameHeaderSize());

    return size;
}

/* xcpPacketSize is the number of octets in the XCP Packet
 * Return value the buffer size in terms of target memory granularity needed to contain this packet.
 * On 32-bit address granularity processor, a packet of 6 contiguous BYTEs, we require 2 memory
 *  locations to contain it.
 */
size_t xcpTransportMsgBufferSize(size_t xcpPacketSize, XcpPacketType pktType)
{
    size_t size = 0;
    size_t maxPacketSize = 0;
    
    maxPacketSize = (pktType == XCP_CTO) ? xcpFrameMaxCtoSize() : xcpFrameMaxDtoSize();

    if (xcpPacketSize > maxPacketSize) {
        XCP_PRINTF("xcpTransportMsgSize: invalid xcpPacketSize\n");
        return 0;
    }

    size = sizeof(struct xcpFifoEntry);

    size += XCP_IN_HW_AG(xcpFrameHeaderSize());
    size += XCP_IN_HW_AG(xcpPacketSize);
    size += XCP_IN_HW_AG(xcpFrameTailSize());

    return size;
}


size_t xcpTransportMaxDtoSize(void)
{
    return xcpFrameMaxDtoSize();
}


size_t xcpTransportMaxCtoSize(void)
{
    return xcpFrameMaxCtoSize();
}


XcpErrorCode xcpTransportRestart(void)
{
    XcpErrorCode errorCode = XCP_SUCCESS;

    XCP_DIAG_UPDATE(XCP_TRANSPORT_RESTART_COUNTER);

    /* Restart Frame Handler */
    errorCode = xcpFrameRestart();
    if (errorCode != XCP_SUCCESS) {
        XCP_PRINTF("xcpTransportRestart: error (%d) detected during frame handler restart\n", errorCode);
        XCP_DIAG_UPDATE(XCP_TRANSPORT_FRAME_RESTART_ERROR);
        return errorCode;
    }

    /* Empty TX and RX fifos */
    xcpTransportEmptyTxFifo();
    xcpTransportEmptyRxFifo();

    return errorCode;
}


XcpErrorCode xcpTransportResynch(void)
{
    XcpErrorCode errorCode = XCP_SUCCESS;

    XCP_DIAG_UPDATE(XCP_TRANSPORT_RESYNCH_COUNTER);

    /* Restart Frame Handler */
    errorCode = xcpFrameRestart();
    if (errorCode != XCP_SUCCESS) {
        XCP_PRINTF("xcpTransportRestart: error (%d) detected during frame handler restart\n", errorCode);
        XCP_DIAG_UPDATE(XCP_TRANSPORT_FRAME_RESTART_ERROR);
        return errorCode;
    }

    /* Empty TX FIFO */
    xcpTransportEmptyTxFifo();

    return errorCode;
}


xcpPoolId_T xcpTransportCtoReservedMemPoolId(void)
{
    return xcpCtoReservedMemPoolId;
}


XcpErrorCode xcpTransportReset(void)
{
    XcpErrorCode errorCode = XCP_SUCCESS;
    XcpTransportQueueType_T i = 0;

    if (!initialized) {
        XCP_PRINTF("xcpTransportReset: transport layer already reset\n");
        return XCP_SUCCESS; /* Nothing to do: just printing out a warning message*/
    }

    XCP_DIAG_UPDATE(XCP_TRANSPORT_RESET_COUNTER);

    /* Reset XCP Frame handler */
    errorCode = xcpFrameReset();
    if (errorCode != XCP_SUCCESS) {
        XCP_PRINTF("xcpTransportReset: error (%d) detected during frame handler reset\n", errorCode);
        XCP_DIAG_UPDATE(XCP_TRANSPORT_FRAME_RESET_ERROR);
        return errorCode;
    }

    /* Before resetting the txFifo and rxFifo, all the message buffers still enqueued need to be freed
    as the ownership of the memory area was explicitly given to the Transport Layer */
    xcpTransportEmptyTxFifo();
    xcpTransportEmptyRxFifo();

    /* Destroy the reserved memory pool */
    xcpMemReservedPoolDestroy(xcpCtoReservedMemPoolId);

    /* Reset XCP memory allocator */
    errorCode = xcpMemReset();
    if (errorCode != XCP_SUCCESS) {
        XCP_PRINTF("xcpTransportReset: error (%d) detected during memory allocator reset\n", errorCode);
        XCP_DIAG_UPDATE(XCP_TRANSPORT_MEM_RESET_ERROR);
        return errorCode;
    }

    /* Now we can reset Tx and Rx Fifos */
    /* Initialize Tx and Rx data structures*/
    for (i = 0; i < XCP_TRANSPORT_TX_QUEUES_TYPES_NUMBER; i++) {
        xcpFifoReset(&txFifo[i]);
    }
    for (i = 0; i < XCP_TRANSPORT_RX_QUEUES_TYPES_NUMBER; i++) {
        xcpFifoReset(&rxFifo[i]);
    }

#ifdef XCP_LOCKLESS_SYNC_DATA_TRANSFER_SUPPORT
    for (i = 0; i < XCP_MEM_DAQ_RESERVED_POOLS_NUMBER; i++) {
        xcpFifoReset(&daqTxFifo[i]);
        xcpFifoReset(&daqDeferredTxFifo[i]);
        daqTxRequest[i] = false;
    }
    lastDaqIdTx = 0xFFFF;
#endif

    initialized = false;

    return errorCode;
}


