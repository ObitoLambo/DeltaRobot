/*
* Copyright 2016-2023 The MathWorks, Inc.
*
* File: xcp_frame_tcp.h
*
* Abstract:
*  TCP/IP Frame Handler definitions
*  The file contains data types and utility functions specific
*  to the XCP over TCP/IP transport layer
*/

#ifndef XCP_FRAME_TCP_H
#define XCP_FRAME_TCP_H

#include "xcp_common.h"
#include "xcp_cfg.h"

/** Max Data Transfer Object size
    @note this can be set up to 0xFFFF for TCP/IP 
          0xFFFC is the max value that is always multiple of 
          XCP_ADDRESS_GRANULARITY_BYTES_NUMBER */
#ifndef XCP_MAX_DTO_SIZE
#define XCP_MAX_DTO_SIZE   0xFFFC
#endif

#if XCP_MAX_DTO_SIZE > 0xFFFF || XCP_MAX_DTO_SIZE < 0x08
#error "Invalid XCP_MAX_DTO_SIZE"
#endif


/** Max Command Transfer Object size
    @note this can be set up to 0xFF */
#ifndef XCP_MAX_CTO_SIZE
#define XCP_MAX_CTO_SIZE   0xFF
#endif

#if XCP_MAX_CTO_SIZE > 0xFF || XCP_MAX_CTO_SIZE < 0x08
#error "Invalid XCP_MAX_CTO_SIZE"
#endif


/** Max size of the generic message that can be received */
#define XCP_MAX_MSG_SIZE   XCP_MAX_DTO_SIZE


/** XCP Header (according to Transport Layer Specification XCP on Ethernet
(TCP_IP and UDP_IP))
@note counter and length must always be in Little Endian format
*/
XCP_PRAGMA_PACK_BEGIN(1)
struct XCP_ATTRIBUTE_PACKED XcpHeader {
    uint16_T length;   /**< number of bytes in the original XCP packet */
    uint16_T counter;  /**< packet counter */
};

/* Internal structure used to detect length and type of the received XCP message */
struct XCP_ATTRIBUTE_PACKED RxXcpHeader {
    struct XcpHeader header; /**< XCP Header structure */
    uint8_T PID;             /**< XCP Packet identifier */
};
XCP_PRAGMA_PACK_END()


/** Convert from host byte order to network byte order (always Little Endian
according to Transport Layer Specification XCP on Ethernet) */
uint16_T xcpTcpHtons(uint16_T hostShort);


/** Convert from network byte order (always Little Endian
according to Transport Layer Specification XCP on Ethernet)
to host byte order */
static uint16_T xcpTcpNtohs(uint16_T networkShort);


#endif /* XCP_FRAME_TCP_H */
