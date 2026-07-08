/*
* Copyright 2016-2022 The MathWorks, Inc.
*
* File: xcp_common.h
*
* Abstract:
*  XCP common definitions
*/

#ifndef XCP_COMMON_H
#define XCP_COMMON_H

#include <stddef.h> /* It contains size_t definition */

#include "rtwtypes.h"

#include "xcp_platform.h"

#define XCP_ERROR_IF(cond,errCode,msg) if(cond) {XCP_PRINTF(msg); return (errCode);}

#define XCP_UNUSED_PARAM(par) (void)(par)

#define XCP_ELEMENTS_NUMBER(x) sizeof(x)/sizeof((x)[0])

#if !defined(UINTPTR_MAX)
    typedef unsigned long uintptr_t;
#endif

#if defined(XCP_ADDRESS_GET_READ) || defined(XCP_ADDRESS_GET_WRITE)
#error "XCP_ADDRESS_GET_READ and XCP_ADDRESS_GET_WRITE should not be defined externally"
#endif

#ifdef XCP_CUSTOM_ADDRESS_TRANSLATION
extern uint8_T const* xcpCustomAddressGetRead(uint8_T, uint32_T);
#define XCP_ADDRESS_GET_READ xcpCustomAddressGetRead
#define XCP_ADDRESS_GET_WRITE XCP_ADDRESS_GET
#else
#define XCP_ADDRESS_GET_READ XCP_ADDRESS_GET
#define XCP_ADDRESS_GET_WRITE XCP_ADDRESS_GET
#endif

#endif /* XCP_COMMON_H */
