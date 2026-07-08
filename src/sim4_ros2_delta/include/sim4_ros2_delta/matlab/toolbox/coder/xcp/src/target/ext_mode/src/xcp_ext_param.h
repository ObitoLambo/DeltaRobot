/*
* Copyright 2017-2018 The MathWorks, Inc.
*
* File: xcp_ext_param.h
*
* Abstract:
*  This file contains the APIs required to extract the XCP parameters
*  list from the user-defined command line options
*/

#ifndef XCP_EXT_PARAM_H
#define XCP_EXT_PARAM_H

#include "xcp_common.h"

/** Parse the command line parameters list, in order to extract
    the actual XCP init parameters. Once extracted, the input parameters 
    are set to NULL. */
void xcpExtModeParseArgs(int_T argc, const char_T *argv[]);

/** Get the XCP Transport Layer initialization parameters */
void xcpTransportGetInitParameters(int_T *parNumber, void **parList[]);

/** Get the XCP Protocol Layer initialization parameters */
void xcpGetInitParameters(int_T *parNumber,  void **parList[]);

#endif /* XCP_EXT_PARAM_H */
