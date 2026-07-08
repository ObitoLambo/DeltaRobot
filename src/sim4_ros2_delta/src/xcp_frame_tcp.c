/*
* Copyright 2016-2023 The MathWorks, Inc.
*
* File: xcp_frame_tcp.c
*
* Abstract:
*  XCP Frame Handler implementation for TCP/IP transport layer.
*/

#include "xcp_common.h"
#include "xcp.h"
#include "xcp_frame.h"
#include "xcp_frame_tcp.h"
#include "xcp_drv.h"
#include "xcp_mem.h"
#include "xcp_types.h"

/* Frame Handler internal counters */
static uint16_T txCounter = 0;
static uint16_T rxCounter = 0;
static int      firstRxCounterReceived = 0;

/* true if the Frame Handler has been successfully initialized */
static boolean_T initialized = false;

/* ID for of the memory pool reserved for the allocation of
    CTO XCP Packets */
static xcpPoolId_T xcpCtoReservedMemPoolId = -1;

#if XCP_HARDWARE_ADDRESS_GRANULARITY != XCP_ADDRESS_GRANULARITY_BYTE
    #error "TCP/IP transport is not supported on 16/32-bit word targets"
#endif

uint16_T xcpTcpHtons(uint16_T hostShort)
{
#ifndef XCP_BIG_ENDIAN
    return hostShort;
#else
    return ((hostShort & 0xFF00) >> 8) | ((hostShort & 0x00FF) << 8);
#endif
}


uint16_T xcpTcpNtohs(uint16_T networkShort)
{
    return xcpTcpHtons(networkShort);
}


XcpErrorCode xcpFrameInit(
    int   argc,   /**< [in] number of init parameters              */
    void *argv[]  /**< [in] array of parameters values (C strings) */
    )
{
    XcpErrorCode errorCode = XCP_SUCCESS;

    XCP_ERROR_IF(initialized, XCP_ERROR, "xcpFrameInit: frame handler already initialized\n");

    /* Open the XCP driver */
    errorCode = xcpDrvOpen(argc, argv);
    if (errorCode == XCP_SUCCESS) {
        txCounter = 0;
        rxCounter = 0;
        firstRxCounterReceived = 0;

        initialized = true;
    }

    return errorCode;
}


XcpErrorCode xcpFrameMsgSend(
    void    *msgBuffer,  /**< [in] pointer to the base address of the buffer where the message is stored */
    size_t   msgOffset,  /**< [in] offset (from base address) where the XCP message frame is stored */
    size_t   msgSize     /**< [in] size (in bytes) of the XCP message frame */
    )
{
    uint8_T *msg = NULL;
    struct XcpHeader *header = NULL;
    uint16_T length = 0;
    XcpErrorCode errorCode = XCP_SUCCESS;

    XCP_ERROR_IF(msgBuffer == NULL, XCP_INV_ARG, "xcpFrameMsgSend: invalid msgBuffer\n");
    XCP_ERROR_IF(msgSize < sizeof(struct XcpHeader), XCP_INV_ARG, "xcpFrameMsgSend: invalid msgSize\n");
    XCP_ERROR_IF(!initialized, XCP_NOT_INITIALIZED, "xcpFrameMsgSend: frame handler not initialized\n");

    msg = (uint8_T *) msgBuffer + msgOffset;

    header = (struct XcpHeader *) msg;
    length = xcpTcpNtohs(header->length);

    if ((length == 0) || (msgSize < (sizeof(*header) + length))) {
        XCP_PRINTF("xcpFrameMsgSend: invalid message format detected\n");
        return XCP_INV_MSG_FORMAT;
    }

    /* Set the value of the transport layer txCounter immediately before
     * sending the message so that it reflects the correct order for the
     * sequence of frames that are sent rather than the sequence in which
     * they are constructed (which could be different in the case where
     * there are multiple prioritized queues). */
    header->counter = xcpTcpHtons(txCounter);
    
    /* Send the full message or nothing
     * If the xcpDrvSend() is blocking, the return code should be XCP_SUCCESS
     * If the xcpDrvSend() is non-blocking, the return code could be XCP_SUCCESS or XCP_BUSY.
     * If something went wrong we should get XCP_ERROR or XCP_PKT_TX_TIMEOUT_ERROR */
    errorCode = xcpDrvSend(msg, msgSize);

    /* If successfully sent, the buffer can be freed */
    if (errorCode == XCP_SUCCESS) {
        xcpMemFree(msgBuffer);        
    }
    
    /* Don't increment the counter if the transfer of the packet is delayed
     * due to the transport layer being busy. */
    if (errorCode != XCP_BUSY) {
        txCounter++;
    }

    return errorCode;
}


XcpErrorCode xcpFrameMsgRecv(
    void  **msgBuffer,  /**< [out] pointer to the base address of the buffer where the message is stored */
    size_t  msgOffset,  /**< [in] offset (from base address) where the XCP message frame is stored */
    size_t *msgSize     /**< [out] size (in bytes) of the XCP message frame that has been received */
    )
{
    XcpErrorCode errorCode = XCP_SUCCESS;
    struct RxXcpHeader rxHeader;
    uint16_T length = 0;
    uint8_T *buffer;
    uint8_T *bufferPtr;

    XCP_ERROR_IF(msgBuffer == NULL, XCP_INV_ARG, "xcpFrameMsgRecv: invalid msgBuffer pointer\n");
    XCP_ERROR_IF(msgSize == NULL, XCP_INV_ARG, "xcpFrameMsgRecv: invalid msgSize pointer\n");
    XCP_ERROR_IF(!initialized, XCP_NOT_INITIALIZED, "xcpFrameMsgRecv: frame handler not initialized\n");

    /* Receive message header first.
     * If the xcpDrvRecv() is blocking, the return code should be XCP_SUCCESS
     * If the xcpDrvRecv() is non-blocking, the return code could be XCP_SUCCESS or XCP_EMPTY
     * If something went wrong we should get XCP_ERROR or XCP_PKT_RX_TIMEOUT_ERROR */
    errorCode = xcpDrvRecv(&rxHeader, sizeof(rxHeader));
    if (errorCode != XCP_SUCCESS) {
        return errorCode;
    }

    length = xcpTcpNtohs(rxHeader.header.length);

    if (length == 0) {
        XCP_PRINTF("xcpFrameMsgRecv: invalid message format detected\n");
        errorCode = XCP_INV_MSG_FORMAT;
        return errorCode;
    }

    /* Request to the XCP allocator the memory for the message to be received */
    if ((rxHeader.PID >= XCP_COMMAND_PACKETS_ID_MIN) &&
        (xcpCtoReservedMemPoolId >= 0)) {
        buffer = xcpMemAllocFromPool(xcpCtoReservedMemPoolId, msgOffset + sizeof(rxHeader.header) + length);
    } else {
        buffer = xcpMemAlloc(msgOffset + sizeof(rxHeader.header) + length);
    }

    if (buffer == NULL) {
        errorCode = XCP_NO_MEMORY;
        return errorCode;
    }

    bufferPtr = buffer;

    /* Initialize the header part */
    XCP_MEMSET(bufferPtr, 0, msgOffset);
    bufferPtr += msgOffset;

    XCP_MEMCPY(bufferPtr, &rxHeader, sizeof(rxHeader));
    bufferPtr += sizeof(rxHeader);

    if (length > sizeof(rxHeader.PID)) {
        /* Receive the remaining part of the XCP packet
         * Since have already started receiving the packet, we should wait
         * until the remaining part is received, or a XCP_PKT_RX_TIMEOUT_ERROR has occurred.
         * For this reason we force the subsequent xcpDrvRecv to be blocking */
        xcpDrvIoctl(XCP_DRV_FORCE_BLOCKING);

        errorCode = xcpDrvRecv(bufferPtr, length - sizeof(rxHeader.PID));

        /* The packet has been received, or an error occurred.
         * In any case we can restore the default blocking/non-blocking behavior */
        xcpDrvIoctl(XCP_DRV_RESTORE_DEFAULT_BLOCKING_SETUP);

        if (errorCode != XCP_SUCCESS){
            /* XCP Header and XCP Packet should be in the same TCP/IP packet */
            XCP_PRINTF("xcpFrameMsgRecv: received XCP header, but XCP packet is not available or packet size is incorrect\n");
            errorCode = XCP_INV_MSG_FORMAT;
            goto rxError;
        }
    }

    *msgBuffer = buffer;
    *msgSize = sizeof(rxHeader.header) + length;

    return errorCode;

rxError:
    *msgBuffer = NULL;
    *msgSize = 0;
    xcpMemFree(buffer);

    return errorCode;
}


XcpErrorCode xcpFrameCreateMsg(
    void        *msgFrame,           /**< [out] pointer to the buffer containing the full message frame */
    size_t       msgFrameBufferSize, /**< [in]  max size (in bytes) of the buffer where the message frame is copied */
    size_t      *msgFrameSize,       /**< [out] size (in bytes) of the generated msg frame */
    size_t       xcpPacketSize       /**< [in]  size (in bytes) of the XCP packet */
    )
{
    struct XcpHeader *header = (struct XcpHeader *) msgFrame;

    /* Input parameters validation */
    XCP_ERROR_IF(msgFrame == NULL, XCP_INV_ARG, "xcpFrameCreateMsg: invalid msgFrame buffer\n");
    XCP_ERROR_IF(msgFrameSize == NULL, XCP_INV_ARG, "xcpFrameCreateMsg: invalid msgFrameSize pointer\n");
    XCP_ERROR_IF(xcpPacketSize == 0, XCP_INV_ARG, "xcpFrameCreateMsg: invalid xcpPacketSize\n");
    XCP_ERROR_IF(msgFrameBufferSize < (xcpPacketSize + sizeof(struct XcpHeader)), XCP_INV_ARG, "xcpFrameCreateMsg: invalid msgFrameBufferSize\n");

    /* update XCP header */
    header->length = xcpTcpHtons((uint16_T)xcpPacketSize);
    
    *msgFrameSize = xcpPacketSize + sizeof(struct XcpHeader);

    return XCP_SUCCESS;
}


XcpErrorCode xcpFrameExtractPacket(
    const void  *msgFrame,      /**< [in]  pointer to the buffer containing full message frame content */
    size_t       msgFrameSize,  /**< [in]  size (in bytes) of the full message frame */
    size_t      *xcpPacketSize  /**< [out] size (in bytes) of the XCP packet that has been processed */
    )
{
    const struct XcpHeader *header = (const struct XcpHeader *) msgFrame;
    uint16_T length = 0;
    uint16_T counter = 0;
    XcpErrorCode errorCode = XCP_SUCCESS;

    /* Input parameters validation */
    XCP_ERROR_IF(msgFrame == NULL, XCP_INV_ARG, "xcpFrameExtractPacket: invalid msgFrame\n");
    XCP_ERROR_IF(xcpPacketSize == NULL, XCP_INV_ARG, "xcpFrameExtractPacket: invalid xcpPacketSize pointer\n");
    XCP_ERROR_IF(msgFrameSize <= sizeof(struct XcpHeader), XCP_INV_ARG, "xcpFrameExtractPacket: invalid msgFrameSize\n");
    XCP_ERROR_IF(!initialized, XCP_NOT_INITIALIZED, "xcpFrameExtractPacket: frame handler not initialized\n");

    /* Extract info from the received message */
    length = xcpTcpNtohs(header->length);
    counter = xcpTcpNtohs(header->counter);

    /* Check message length */
    if (msgFrameSize != (length + sizeof(struct XcpHeader))) {
        XCP_PRINTF("xcpFrameExtractPacket: Xcp packet has an invalid size\n");
        return XCP_INV_MSG_FORMAT;
    }

    /* Check message counter */
    if (!firstRxCounterReceived) {
        /* initialize the rxCounter with the first value received from the Client */
        rxCounter = counter;
        firstRxCounterReceived = 1;
    }
    else {
        if (counter < (rxCounter + 1)) {
            errorCode = XCP_PKT_OUT_OF_SEQUENCE;
        }
        else if (counter >(rxCounter + 1)) {
            errorCode = XCP_PKT_LOST;
        }

        rxCounter = counter;
    }

    *xcpPacketSize = length;

    return errorCode;
}


size_t xcpFrameHeaderSize(void)
{
    return sizeof(struct XcpHeader);
}


size_t xcpFrameTailSize(void)
{
    return 0;
}


size_t xcpFrameMaxDtoSize(void)
{
    return XCP_MAX_DTO_SIZE;
}


size_t xcpFrameMaxCtoSize(void)
{
    return XCP_MAX_CTO_SIZE;
}


XcpErrorCode xcpFrameRestart(void)
{
    txCounter = 0;
    rxCounter = 0;
    firstRxCounterReceived = 0;

    return XCP_SUCCESS;
}


void xcpFrameSetCtoReservedMemPoolId(xcpPoolId_T poolId)
{
    xcpCtoReservedMemPoolId = poolId;
}


XcpErrorCode xcpFrameReset(void)
{
    XcpErrorCode errorCode = XCP_SUCCESS;

    if (!initialized) {
        XCP_PRINTF("xcpFrameReset: frame handler already reset\n");
        return XCP_SUCCESS; /* Nothing to do: just printing out a warning message*/
    }

    /* Close the actual communication channel */
    errorCode = xcpDrvClose();
    if (errorCode == XCP_SUCCESS) {
        txCounter = 0;
        rxCounter = 0;
        firstRxCounterReceived = 0;

        initialized = false;
    } else {
        XCP_PRINTF("xcpFrameReset: xcpDrvClose error\n");
    }

    return errorCode;
}
