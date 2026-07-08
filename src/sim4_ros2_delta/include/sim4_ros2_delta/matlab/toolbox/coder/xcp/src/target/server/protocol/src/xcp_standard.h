/*
* Copyright 2016-2020 The MathWorks, Inc.
*
* File: xcp_standard.h
*
* Abstract:
*  XCP Protocol layer Standard commands support internal interface
*/

#ifndef XCP_STANDARD_H
#define XCP_STANDARD_H

#include "xcp_types.h"

#define XCP_CONNECT_PACKET_ID     0

/** Initialize Standard Commands support */
void xcpStandardInit(void);

/** Return the XcpPacketLookupFunction to be invoked
    to get access to the Input and Output Packet
    handlers for the supported Standard Commands */
XcpPacketLookupFunction xcpStandardGetPacketLookup(void);

/** Override the XcpPacketLookupFunction to be invoked
    to get access to the Input and Output Packet
    handlers for the supported Standard Commands */
void xcpStandardSetPacketLookup(XcpPacketLookupFunction getPacket);

/** Reset Standard Commands support to the initial state */
void xcpStandardReset(void);

/* Get/Set MTA and memory address extension value */
uint8_T* xcpStandardGetAddressFromMta(void);
boolean_T xcpStandardSetMta(uint32_T address, uint8_T addressExtension);
void xcpStandardGetMta(uint32_T *address, uint8_T *addressExtension);
void xcpStandardIncrementMta(uint8_T incr);

#endif /* XCP_STANDARD_H */

