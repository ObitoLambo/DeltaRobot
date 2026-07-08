/*
* Copyright 2016-2023 The MathWorks, Inc.
*
* File: xcp.h
*
* Abstract:
*  XCP Protocol layer interface
*/

#ifndef XCP_H
#define XCP_H

#include "xcp_common.h"

/** XCP Error codes */
typedef enum {
    XCP_SUCCESS              =  0,
    XCP_INV_ARG              = -1,
    XCP_EMPTY                = -2,
    XCP_FULL                 = -3,
    XCP_PKT_OUT_OF_SEQUENCE  = -4,
    XCP_PKT_LOST             = -5,
    XCP_BUSY                 = -6,
    XCP_INV_MSG_FORMAT       = -7,
    XCP_INV_SIZE             = -8,
    XCP_NOT_INITIALIZED      = -9,
    XCP_NO_MEMORY            = -10,
    XCP_NOT_SUPPORTED        = -11,
    XCP_ERROR                = -12,
    XCP_PKT_CHECKSUM_ERROR   = -13,
    XCP_PKT_RX_TIMEOUT_ERROR = -14,
    XCP_PKT_TX_TIMEOUT_ERROR = -15
} XcpErrorCode;


/** XCP event Id type */
typedef uint16_T XcpEventIdType;


/** XCP Event code
    @note the values reflect the ASAM Protocol Layer specification */
typedef enum {
    XCP_EV_RESUME_MODE        = 0x00, /**< Server starting in RESUME mode */
    XCP_EV_CLEAR_DAQ          = 0x01, /**< The DAQ configuration in non-volatile memory has been cleared */
    XCP_EV_STORE_DAQ          = 0x02, /**< The DAQ configuration has been stored into non-volatile memory */
    XCP_EV_STORE_CAL          = 0x03, /**< The calibration data has been stored into non-volatile memory */
    XCP_EV_CMD_PENDING        = 0x05, /**< Server requesting to restart timeout */
    XCP_EV_DAQ_OVERLOAD       = 0x06, /**< DAQ processor overload */
    XCP_EV_SESSION_TERMINATED = 0x07, /**< Session terminated by server device */
    XCP_EV_TIME_SYNC          = 0x08, /**< Transfer of externally triggered timestamp */
    XCP_EV_STIM_TIMEOUT       = 0x09, /**< Indication of a STIM timeout */
    XCP_EV_SLEEP              = 0x0A, /**< Server entering SLEEP mode */
    XCP_EV_WAKE_UP            = 0x0B, /**< Server leaving SLEEP mode */
    XCP_EV_USER               = 0xFE, /**< user-defined event */
    XCP_EV_TRANSPORT          = 0xFF  /**< Transport layer specific event */
} XcpEventCode;


/** XCP Service code
@note the values reflect the ASAM Protocol Layer specification */
typedef enum {
    XCP_SERV_RESET = 0x00, /**< Server requesting to be reset */
    XCP_SERV_TEXT  = 0x01  /**< Server transferring a byte stream of plain ASCII text.
                                The line separator is LF or CR/LF. The text can be transferred in consecutive packets.
                                The end of the overall text is indicated by the last packet containing a NULL terminated string */
} XcpReqServiceCode;


/** XCP status */
typedef enum {
    XCP_DISCONNECTED = 0,        /**< Server disconnected from the Client, no XCP communication active */
    XCP_CONNECTED = 1,           /**< Established a logical point to point XCP connection with the Client */
    XCP_SYNC_DATA_TRANSFER = 2   /**< Server has started the synchronous TX/RX of XCP DAQ lists and STIM */
} XcpStatus;


/** Initialize XCP protocol layer
    @note in the current implementation all the input parameters are ignored,
          but future versions might support different configuration options */
XcpErrorCode xcpInit(
    int   argc,   /**< [in] number of init parameters              */
    void *argv[]  /**< [in] array of parameters values (C strings) */
    );


/** Process one XCP packet (retrieved from the XCP transport layer)
    and generate packet response (automatically sent back to the XCP transport layer)
    @note the function returns XCP_EMPTY when there are no more packets
          available for processing.
          The packet is processed only if no errors have occurred */
XcpErrorCode xcpRun(void);


/** Inform the XCP Protocol layer about the fact that a specific event has occurred */
XcpErrorCode xcpEvent(XcpEventIdType eventId);


/** Inform the XCP Protocol layer about the fact that a specific event has occurred,
    and pass the timestamp value associated to the DAQ list as an input parameter */
XcpErrorCode xcpEventExternalTimestamp(XcpEventIdType eventId, uint32_T timestamp);


/** Generate a request to the XCP Protocol stack to send a XCP EV packet,
    in order to notify the XCP Client about a specific condition
    @note XCP EV packets are not acknowledged, therefore the transmission is not guaranteed
          The Protocol Layer does NOT send any EV packets on its own.
          The request must be explicitly triggered by the user and it is NOT mandatory. */
XcpErrorCode xcpEventNotificationReq(XcpEventCode eventCode, const void *eventData, size_t eventDataSize);


/** Generate a request to the XCP Protocol stack to send a XCP SERV packet,
    in order to request the XCP Client the execution if a specific service.
    @note XCP SERV packets are not acknowledged, therefore the transmission is not guaranteed
          The Protocol Layer does NOT send any SERV packets on its own.
          The request must be explicitly triggered by the user and it is NOT mandatory. */
XcpErrorCode xcpRemoteServiceReq(XcpReqServiceCode serviceReqCode, const void *serviceReqData, size_t serviceReqDataSize);


/** Get the current XCP protocol layer status */
XcpStatus xcpGetStatus(void);


/** Reset XCP protocol layer to the default initial state */
XcpErrorCode xcpReset(void);

#endif /* XCP_H */

