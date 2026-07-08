/*
* Copyright 2019 The MathWorks, Inc.
*
* File: xcp_daq_ext.h
*
* Abstract:
*  XCP Protocol layer Extended DAQ lists (and STIM) internal interface
*/

#ifndef XCP_DAQ_EXT_H
#define XCP_DAQ_EXT_H

/** Initialize Extended DAQ lists (and STIM) support */
void xcpDaqExtendedInit(void);

/** Reset Extended DAQ lists (and STIM) support to the initial status */
void xcpDaqExtendedReset(void);

#endif /* XCP_DAQ_EXT_H */

