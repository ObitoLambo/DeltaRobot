/*
* Copyright 2016-2023 The MathWorks, Inc.
*
* File: xcp_drv_rtiostream.c
*
* Abstract:
*  Implementation of XCP driver based on rtIOStream
*/

#include "rtiostream.h"
#include "xcp_common.h"
#include "xcp.h"
#include "xcp_drv.h"

#define INVALID_DRV_ID  -1

/* Timeout expected for the reception of a packet, once the reception has started */
static const uint32_T XCP_RECEIVE_PACKET_TIMEOUT_IN_MICROSECONDS  = 1000000L;  /* 1s */

/* Delay before attempting the reception of new data */
static const uint32_T XCP_RECEIVE_RETRY_TIME_IN_MICROSECONDS = 10L;  /* 10us */

/* Timeout expected for the transmission of a packet, once the transmission has started */
static const uint32_T XCP_SEND_PACKET_TIMEOUT_IN_MICROSECONDS = 2000000L;   /* 2s */

/* Delay before attempting the transmission of new data */
static const uint32_T XCP_SEND_RETRY_TIME_IN_MICROSECONDS = 10L;  /* 10us */


/* According to the XCP standard, the XCP server only supports the connection
   with one XCP Client at a time. The active connection is identified by
   a specific drvID */
static int drvID = INVALID_DRV_ID;


/* If set to true, force the xcpDrvSend and xcpDrvRecv APIs to be blocking */
static boolean_T  forceBlocking = false;


XcpErrorCode xcpDrvOpen(
    int   argc,
    void *argv[]
    )
{
    XcpErrorCode errorCode = XCP_SUCCESS;

    if (drvID != INVALID_DRV_ID) {
        XCP_PRINTF("xcpDrvOpen: XCP driver already initialized\n");
        return XCP_ERROR;
    }

    /* Open the actual communication channel */
    drvID = rtIOStreamOpen(argc, argv);
    if (drvID < 0) {
        XCP_PRINTF("xcpDrvOpen: unable to open communication channel\n");
        drvID = INVALID_DRV_ID;
        errorCode = XCP_ERROR;
    }

    return errorCode;
}


XcpErrorCode xcpDrvIoctl(XcpDrvIoctlCommand cmd)
{
    XcpErrorCode errorCode = XCP_SUCCESS;

    XCP_ERROR_IF((cmd != XCP_DRV_RESTORE_DEFAULT_BLOCKING_SETUP) &&
                 (cmd != XCP_DRV_FORCE_BLOCKING), XCP_INV_ARG,
                 "xcpDrvIoctl: invalid command\n");

    forceBlocking = (cmd == XCP_DRV_FORCE_BLOCKING);

    return errorCode;
}


XcpErrorCode xcpDrvSend(const void *src, size_t size)
{
    int ret = RTIOSTREAM_NO_ERROR;
    XcpErrorCode errorCode = XCP_SUCCESS;
    size_t   sent = 0;
    size_t sentBytesCounter = 0;
    uint32_T elapsedTime = 0;
    const uint8_T *dataPtr = NULL;
    boolean_T done = false;

    XCP_ERROR_IF(src == NULL, XCP_INV_ARG, "xcpDrvSend: invalid src buffer\n");
    XCP_ERROR_IF(size == 0, XCP_INV_ARG, "xcpDrvSend: invalid size\n");
    XCP_ERROR_IF(drvID == INVALID_DRV_ID, XCP_NOT_INITIALIZED, "xcpDrvSend: XCP driver not initialized\n");

    while (!done) {
        dataPtr = (const uint8_T *)src + sentBytesCounter;
        ret = rtIOStreamSend(drvID, dataPtr, (size - sentBytesCounter), &sent);

        if (ret == RTIOSTREAM_NO_ERROR) {
            if ((sent == 0) && (sentBytesCounter == 0) && !forceBlocking) {
                /* This will only happen if the rtIOStreamSend function
                   is non-blocking and we haven't started sending data yet */
                errorCode = XCP_BUSY;
            } else {
                /* Send packet in progress, we need to complete it */
                sentBytesCounter += sent;

                done = (sentBytesCounter >= size) ;

                if (!done) {
                    if (elapsedTime >= XCP_SEND_PACKET_TIMEOUT_IN_MICROSECONDS) {
                        errorCode = XCP_PKT_TX_TIMEOUT_ERROR;
                    } else {
                        XCP_SLEEP(0, XCP_SEND_RETRY_TIME_IN_MICROSECONDS);
                        elapsedTime += XCP_SEND_RETRY_TIME_IN_MICROSECONDS;
                    }
                }
            }
        } else {
            errorCode = XCP_ERROR;
        }

        done = done || (errorCode == XCP_BUSY) || (errorCode == XCP_ERROR) ||
               (errorCode == XCP_PKT_TX_TIMEOUT_ERROR);
    }

    return errorCode;
}


XcpErrorCode xcpDrvRecv(void *dst, size_t size)
{
    int ret = RTIOSTREAM_NO_ERROR;
    XcpErrorCode errorCode = XCP_SUCCESS;
    size_t receivedBytesCounter = 0;
    size_t received = 0;
    uint32_T elapsedTime = 0;
    uint8_T *dataPtr = NULL;
    boolean_T done = false;

    XCP_ERROR_IF(dst == NULL, XCP_INV_ARG, "xcpDrvRecv: invalid src buffer\n");
    XCP_ERROR_IF(size == 0, XCP_INV_ARG, "xcpDrvRecv: invalid size\n");
    XCP_ERROR_IF(drvID == INVALID_DRV_ID, XCP_NOT_INITIALIZED, "xcpDrvRecv: XCP driver not initialized\n");

    while (!done) {
        dataPtr = (uint8_T *)dst + receivedBytesCounter;

        ret = rtIOStreamRecv(drvID, dataPtr, (size - receivedBytesCounter), &received);
        if (ret == RTIOSTREAM_NO_ERROR) {
            if ((received == 0) && (receivedBytesCounter == 0) && !forceBlocking) {
                /* This will only happen if the rtIOStreamRecv function
                   is non-blocking and we haven't started receiving data yet */
                errorCode = XCP_EMPTY;
            } else {
                /* Receive packet in progress, we need to complete it */
                receivedBytesCounter += received;
                done = (receivedBytesCounter >= size);

                if (!done) {
                    if (elapsedTime >= XCP_RECEIVE_PACKET_TIMEOUT_IN_MICROSECONDS) {
                        errorCode = XCP_PKT_RX_TIMEOUT_ERROR;
                    } else {
                        XCP_SLEEP(0, XCP_RECEIVE_RETRY_TIME_IN_MICROSECONDS);
                        elapsedTime += XCP_RECEIVE_RETRY_TIME_IN_MICROSECONDS;
                    }
                }
            }
        }
        else {
            errorCode = XCP_ERROR;
        }
        
        done = done || (errorCode == XCP_EMPTY) || (errorCode == XCP_ERROR) ||
               (errorCode ==  XCP_PKT_RX_TIMEOUT_ERROR);
    }

    return errorCode;
}


XcpErrorCode xcpDrvRecvUnknownSize(void *dst, size_t *size, size_t maxSize)
{
    int ret = RTIOSTREAM_NO_ERROR;
    XcpErrorCode errorCode = XCP_SUCCESS;
    size_t receivedBytes = 0;
    uint8_T *dataPtr = NULL;

    XCP_ERROR_IF(dst == NULL, XCP_INV_ARG, "xcpDrvRecvUnknownSize: invalid dst buffer\n");
    XCP_ERROR_IF(size == NULL, XCP_INV_ARG, "xcpDrvRecvUnknownSize: invalid size pointer\n");
    XCP_ERROR_IF(maxSize == 0, XCP_INV_ARG, "xcpDrvRecvUnknownSize: invalid maxSize\n");
    XCP_ERROR_IF(drvID == INVALID_DRV_ID, XCP_NOT_INITIALIZED, "xcpDrvRecvUnknownSize: XCP driver not initialized\n");

    dataPtr = (uint8_T *)dst;

    ret = rtIOStreamRecv(drvID, dataPtr, maxSize, &receivedBytes);
    if (ret == RTIOSTREAM_NO_ERROR) {
        if (receivedBytes == 0) {
            /* This will only happen if we haven't yet started receiving data */
            errorCode = XCP_EMPTY;
        }
        else if (receivedBytes <= maxSize) {
            /* Packet received. Hence, update the actual size */
            *size = receivedBytes;
            errorCode = XCP_SUCCESS;
        }
        else {
            errorCode = XCP_ERROR;
        }
    }
    else {
        errorCode = XCP_ERROR;
    }

    return errorCode;
}


XcpErrorCode xcpDrvClose(void)
{
    int ret = RTIOSTREAM_NO_ERROR;
    XcpErrorCode errorCode = XCP_SUCCESS;

    if (drvID == INVALID_DRV_ID) {
        XCP_PRINTF("xcpDrvClose: XCP driver already closed\n");
        return XCP_SUCCESS; /* Nothing to do: just printing out a warning message*/
    }

    ret = rtIOStreamClose(drvID);

    if (ret == RTIOSTREAM_NO_ERROR) {
        drvID = INVALID_DRV_ID;
        errorCode = XCP_SUCCESS;
    }
    else {
        errorCode = XCP_ERROR;
    }

    return errorCode;
}

