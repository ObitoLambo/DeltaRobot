/*
* Copyright 2016-2021 The MathWorks, Inc.
*
* File: xcp_transport.h
*
* Abstract:
*  XCP Transport Layer interface
*/

#ifndef XCP_TRANSPORT_H
#define XCP_TRANSPORT_H

#include "xcp.h"


/** Initialize XCP transport layer
    @note all the input parameters are intended to be used for the initialization
          of the underlying XCP Driver and are implementation dependent */
XcpErrorCode xcpTransportInit(
    int   argc,   /**< [in] number of init parameters              */
    void *argv[]  /**< [in] array of parameters values (C strings) */
    );


/** Trigger transmission of one of the pending XCP packets
    @note the blocking or non-blocking behaviour (and the corresponding
          error codes) depends on the underlying xcpDrvSend() implementation */
XcpErrorCode xcpTransportTx(void);


/** Receive one of the available XCP packets (if any)
    @note the blocking or non-blocking behaviour (and the corresponding
          error codes) depends on the underlying xcpDrvRecv() implementation */
XcpErrorCode xcpTransportRx(void);


/** Reset XCP transport layer to the default initial state  */
XcpErrorCode xcpTransportReset(void);


#ifdef XCP_DEBUG_SUPPORT

/** Transport layer diagnostic counters */
typedef enum XcpTransportCountersId {
    XCP_TRANSPORT_INIT_COUNTER,
    XCP_TRANSPORT_TX_COUNTER,
    XCP_TRANSPORT_TX_PACKET_SET_COUNTER,
    XCP_TRANSPORT_RX_PACKET_GET_COUNTER,
    XCP_TRANSPORT_RX_COUNTER,
    XCP_TRANSPORT_RESTART_COUNTER,
    XCP_TRANSPORT_RESYNCH_COUNTER,
    XCP_TRANSPORT_RESET_COUNTER,
    XCP_TRANSPORT_MEM_INIT_ERROR,
    XCP_TRANSPORT_MEM_RESET_ERROR,
    XCP_TRANSPORT_FRAME_INIT_ERROR,
    XCP_TRANSPORT_FRAME_CREATE_MSG_ERROR,
    XCP_TRANSPORT_FRAME_SEND_ERROR,
    XCP_TRANSPORT_FRAME_RECV_ERROR,
    XCP_TRANSPORT_FRAME_EXTRACT_PACKET_ERROR,
    XCP_TRANSPORT_FRAME_RESTART_ERROR,
    XCP_TRANSPORT_FRAME_RESET_ERROR,
    XCP_TRANSPORT_TX_PACKET_IN_COUNTER,
    XCP_TRANSPORT_TX_PACKET_OUT_COUNTER,
    XCP_TRANSPORT_TX_PACKET_FREED_COUNTER,
    XCP_TRANSPORT_TX_IS_EMPTY_COUNTER,
    XCP_TRANSPORT_RX_PACKET_IN_COUNTER,
    XCP_TRANSPORT_RX_PACKET_OUT_COUNTER,
    XCP_TRANSPORT_RX_PACKET_FREED_COUNTER,
    XCP_TRANSPORT_RX_IS_EMPTY_COUNTER,
    XCP_TRANSPORT_COUNTERS_NUMBER
} XcpTransportCounterId;


typedef struct XcpTransportDiagnostic {
    uint32_T data[XCP_TRANSPORT_COUNTERS_NUMBER];
} XcpTransportDiagnostic;


/** Reset Transport Layer Diagnostic Data */
void xcpTransportDiagnosticReset(void);

/** Retrieve Transport Layer Diagnostic Data */
void xcpTransportDiagnosticGet(XcpTransportDiagnostic *diag);

/** Print out Transport Layer Diagnostic Data */
void xcpTransportDiagnosticPrint(const XcpTransportDiagnostic *diag);

#endif

#endif /* XCP_TRANSPORT_H */
