/*
* Copyright 2016-2020 The MathWorks, Inc.
*
* File: xcp_drv.h
*
* Abstract:
*  XCP Driver interface.
*  The interface provides methods to send or receive full XCP message frames
*  over the supported transport layer
*/

#ifndef XCP_DRV_H
#define XCP_DRV_H

#include "xcp.h"

/** I/O Control commands to dynamically modify the XCP Driver behavior */
typedef enum {
    XCP_DRV_RESTORE_DEFAULT_BLOCKING_SETUP, /**< Restore the default driver
                                                 blocking behavior */
    XCP_DRV_FORCE_BLOCKING                  /**< Force all the subsequent xcpDrvSend
                                                 and xcpDrvRecv APIs to be blocking */
} XcpDrvIoctlCommand;


/** Open the communication channel
    @note all the input parameters are implementation dependent
          and they are obtained during the Transport Layer initialization */
XcpErrorCode xcpDrvOpen(int argc, void *argv[]);


/** Modify the XCP Driver behavior  */
XcpErrorCode xcpDrvIoctl(XcpDrvIoctlCommand cmd);


/** Send the raw data to the communication channel

    If the xcpTransportTx() is expected to be non blocking,
    the xcpDrvSend() should either
    - send ALL the data (and return XCP_SUCCESS)
    - or not send ANY data and return XCP_BUSY

    If the xcpTransportTx() is expected to be blocking,
    the xcpDrvSend() should send ALL the data
    (and return XCP_SUCCESS)

    If something wrong happens, then XCP_ERROR should be returned.
    If the transmission takes too long a XCP_PKT_TX_TIMEOUT_ERROR
    should be returned

    @note the driver doesn't need to have any knowledge on the
          actual XCP frame format (handled by the Frame Handler) */
XcpErrorCode xcpDrvSend(const void *src, size_t size);


/** Receive the raw data from the communication channel

    If the xcpTransportRx() is expected to be non blocking,
    the xcpDrvRecv() should either
    - receive ALL the data (and return XCP_SUCCESS)
    - or not receive ANY data and return XCP_EMPTY

    If the xcpTransportRx() is expected to be blocking,
    the xcpDrvRecv() should receive ALL the data
    (and return XCP_SUCCESS)

    If something wrong happens, then XCP_ERROR should be returned.
    If the receive takes too long a XCP_PKT_RX_TIMEOUT_ERROR
    should be returned

    @note the driver doesn't need to have any knowledge on the
          actual XCP frame format (handled by the Frame Handler) */
XcpErrorCode xcpDrvRecv(void *dst, size_t size);

/** Receive the raw data from the communication channels where the
    size of the packet being received is not known upfront.
    (e.g. transport layers where there is no XCP header, such as XCP on CAN)
    The actual size of the received packet is the output value 'size'.
    'maxSize' is an input and it represents the maximum size of the dst buffer.

    If the xcpTransportRx() is expected to be non blocking, 
    the xcpDrvRecvUnknownSize() should either
    - receive ALL the data (and return XCP_SUCCESS)
    - or not receive ANY data and return XCP_EMPTY

    If the xcpTransportRx() is expected to be blocking, 
    the xcpDrvRecvUnknownSize() should receive ALL the data 
    (and return XCP_SUCCESS).

    If something wrong happens, then XCP_ERROR should be returned. */
XcpErrorCode xcpDrvRecvUnknownSize(void *dst, size_t *size, size_t maxSize);

/** Close the communication channel */
XcpErrorCode xcpDrvClose(void);


#endif /* XCP_DRV_H */
