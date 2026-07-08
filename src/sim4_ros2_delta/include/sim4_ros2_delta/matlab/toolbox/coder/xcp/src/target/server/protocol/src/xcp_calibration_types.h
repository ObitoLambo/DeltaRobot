/*
* Copyright 2017-2021 The MathWorks, Inc.
*
* File: xcp_calibration_types.h
*
* Abstract:
*  XCP Protocol layer data types and definitions specific to Calibration support
*/

#ifndef XCP_CALIBRATION_TYPES_H
#define XCP_CALIBRATION_TYPES_H

#include "xcp_common.h"
#include "xcp_cfg.h"

/*****************************************************************************
    XCP SHORT DOWNLOAD
******************************************************************************/
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
   XCP_PRAGMA_PACK_BEGIN(1)
   typedef struct XCP_ATTRIBUTE_PACKED XcpShortDownloadCmdPacketFrame {
        uint8_T  PID;              /**< Packet identifier, always XCP_PID_SHORT_DOWNLOAD */
        uint8_T  size;             /**< number of data elements, it must be in the range
                                        [0..(XCP_MAX_CTO_SIZE - 8)/XCP_ADDRESS_GRANULARITY] */
        uint8_T  reserved;         /**< reserved byte */
        uint8_T  addressExtension; /**< address extension of the parameter to download */
        uint32_T address;          /**< address of the parameter to download */

        /* The remaining bytes of the packet represent the actual content
        of the parameter to be downloaded */
    }   XcpShortDownloadCmdPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpShortDownloadCmdPacketFrame {
        uint16_T  PID               :8;
        uint16_T  size              :8;
        uint16_T  reserved          :8;
        uint16_T  addressExtension  :8;
        uint32_T address;
    }   XcpShortDownloadCmdPacketFrame;
#endif

/*****************************************************************************
    XCP DOWNLOAD
******************************************************************************/
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpDownloadCmdPacketFrame {
        uint8_T  PID;              /**< Packet identifier, always XCP_PID_DOWNLOAD */
        uint8_T  size;             /**< number of data elements, it must be in the range
                                        [0..(XCP_MAX_CTO_SIZE - 8)/XCP_ADDRESS_GRANULARITY] */        
                                        
        /* The remaining bytes of the packet represent the actual content 
        of the parameter to be downloaded */
    }   XcpDownloadCmdPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpDownloadCmdPacketFrame {
        uint16_T PID                :8;
        uint16_T size               :8;
        /* The remaining bytes of the packet represent the actual content 
        of the parameter to be downloaded */
    } XcpDownloadCmdPacketFrame;
#endif

#endif /* XCP_CALIBRATION_TYPES_H */

