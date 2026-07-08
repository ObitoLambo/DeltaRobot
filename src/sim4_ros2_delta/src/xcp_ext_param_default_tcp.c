/*
 * Copyright 2017-2022 The MathWorks, Inc.
 *
 * File: xcp_ext_param_default_tcp.c     
 *
 * Abstract:
 *  The file provides the implementation of the xcp_ext_param.h 
 *  interface specific for the XCP Default Platform Abstraction Layer
 *  for the TCP/IP transport layer.
 */

#include "rtwtypes.h"

#include "rtw_extmode.h"

#include "xcp_common.h"
#include "xcp_ext_param.h"

#ifndef EXTMODE_DISABLE_ARGS_PROCESSING

/* This IDs need to match with the xcpTransportLayerParams below */
typedef enum {
    XCP_PARAM_PORT_ID,
    XCP_PARAM_PORT_VALUE_ID,
    XCP_PARAM_BLOCKING_ID,
    XCP_PARAM_BLOCKING_VALUE_ID,
    XCP_PARAM_PROTOCOL_ID,
    XCP_PARAM_PROTOCOL_VALUE_ID,
    XCP_PARAM_CLIENT_ID,
    XCP_PARAM_CLIENT_VALUE_ID,
    XCP_PARAM_VERBOSE_ID,
    XCP_PARAM_VERBOSE_VALUE_ID
} XcpTransportLayerParams;
 
 
 /** Transport Layer initialization parameters 
     @note the default values can be overwritten when parsing the
           External Mode command line arguments */
static const void* xcpTransportLayerParams[] =
{
    "-port", "17725",
    "-blocking", "0",
    "-protocol", "TCP",
    "-client", "0",     /* this is the only setting that cannot be overwritten */
    "-verbose", "0",
};

#endif /* EXTMODE_DISABLE_ARGS_PROCESSING */

void xcpExtModeParseArgs(int_T argc, const char_T *argv[])
{
#ifdef  EXTMODE_DISABLE_ARGS_PROCESSING
    /* Some targets do not support command line args */
    UNUSED_PARAMETER(argc);
    UNUSED_PARAMETER(argv);
#else
    if ((argv != NULL) && (argc > 0)) {
        int_T optionId = 1;

        while (optionId < argc) {
            const char_T *option = argv[optionId];

            optionId++;

            if ((option != NULL) && (optionId != argc)) {
                boolean_T isXcpOption = false;

                if (strcmp(option, xcpTransportLayerParams[XCP_PARAM_PORT_ID]) == 0) {
                    /* Overwrite "-port" value, if specified */
                    xcpTransportLayerParams[XCP_PARAM_PORT_VALUE_ID] = argv[optionId];
                    isXcpOption = true;
                }
                else if (strcmp(option, xcpTransportLayerParams[XCP_PARAM_BLOCKING_ID]) == 0) {
                    /* Overwrite "-blocking" value, if specified */
                    xcpTransportLayerParams[XCP_PARAM_BLOCKING_VALUE_ID] = argv[optionId];
                    isXcpOption = true;
                }
                else if (strcmp(option, xcpTransportLayerParams[XCP_PARAM_PROTOCOL_ID]) == 0) {
                    /* Overwrite "-protocol" value, if specified */
                    xcpTransportLayerParams[XCP_PARAM_PROTOCOL_VALUE_ID] = argv[optionId];
                    isXcpOption = true;
                }
                else if (strcmp(option, xcpTransportLayerParams[XCP_PARAM_VERBOSE_ID]) == 0) {
                    /* Overwrite "-verbose" value, if specified */
                    xcpTransportLayerParams[XCP_PARAM_VERBOSE_VALUE_ID] = argv[optionId];
                    isXcpOption = true;
                }

                if (isXcpOption) {
                    /* Mark arguments as "processed" */
                    argv[optionId - 1] = NULL;
                    argv[optionId] = NULL;
                }
                
                optionId++;
            }
        }
    }
#endif
}

void xcpTransportGetInitParameters(int_T *parNumber, void **parList[])
{
#ifdef  EXTMODE_DISABLE_ARGS_PROCESSING
    if ((parNumber != NULL) && (parList != NULL)) {
        /* No Transport Layer parameters defined (assuming rtiostream default) */
        *parNumber = 0;
        *parList = NULL;
    }
#else
    if ((parNumber != NULL) && (parList != NULL)) {
        *parNumber = XCP_ELEMENTS_NUMBER(xcpTransportLayerParams);
        *parList = (void **) &xcpTransportLayerParams;
    }
#endif
}

void xcpGetInitParameters(int_T *parNumber, void **parList[])
{
    if ((parNumber != NULL) && (parList != NULL)) {
        /* No Protocol Layer parameters available at the moment */
        *parNumber = 0;
        *parList = NULL;
    }
}


