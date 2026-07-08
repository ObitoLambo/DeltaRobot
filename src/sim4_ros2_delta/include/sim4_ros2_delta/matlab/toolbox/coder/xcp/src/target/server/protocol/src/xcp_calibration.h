/*
* Copyright 2017-2019 The MathWorks, Inc.
*
* File: xcp_calibration.h
*
* Abstract:
*  XCP Protocol layer Calibration support internal interface
*/

#ifndef XCP_CALIBRATION_H
#define XCP_CALIBRATION_H

#include "xcp_types.h"

/** Initialize Calibration support */
void xcpCalibrationInit(void);

/** Return the XcpPacketLookupFunction to be invoked
    to get access to the Input and Output Packet
    handlers for the supported Calibration Commands */
XcpPacketLookupFunction xcpCalibrationGetPacketLookup(void);

/** Override the XcpPacketLookupFunction to be invoked
    to get access to the Input and Output Packet
    handlers for the supported Calibration Commands */
void xcpCalibrationSetPacketLookup(XcpPacketLookupFunction getPacketFcn);

/** Reset Calibration support to the initial status */
void xcpCalibrationReset(void);

#endif /* XCP_CALIBRATION_H */

