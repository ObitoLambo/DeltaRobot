/*
* Copyright 2016-2023 The MathWorks, Inc.
*
* File: xcp_standard_types.h
*
* Abstract:
*  XCP Protocol layer data types and definitions specific to Standard commands support
*/

#ifndef XCP_STANDARD_TYPES_H
#define XCP_STANDARD_TYPES_H

#include "xcp_common.h"
#include "xcp_cfg.h"

#define XCP_MAJOR_NUMBER(ver) (((ver) >> 8) & 0xFF)
#define XCP_MINOR_NUMBER(ver) ((ver) & 0xFF)

/*****************************************************************************
    XCP Basic Communication Mode bit masks
******************************************************************************/
/** Byte order (for multi-byte parameters in a XCP packet) bit: 0 -> Little Endian, 1 -> Big Endian */
#define XCP_COMM_MODE_BYTE_ORDER_MASK   0x01

/** Size of an element contained in a single address (2 bits). See XCP_ADDRESS_GRANULARITY_XXXX */
#define XCP_COMM_MODE_ADDRESS_GRANULARITY_MASK         0x06
#define XCP_COMM_MODE_ADDRESS_GRANULARITY_OFFSET       1

/** Server Block Mode bit: 0 -> not available, 1 -> available */
#define XCP_COMM_MODE_SLAVE_BLOCK_MODE_MASK   0x40

/** Optional additional information bit on supported types of Communication Mode
(retrievable via GET_COMM_MODE_INFO): 0 -> not available, 1 -> available*/
#define XCP_COMM_MODE_OPTIONAL_MASK           0x80


/*****************************************************************************
    XCP address representation
******************************************************************************/
typedef struct XcpAddress {
     uint32_T address;
     uint8_T addressExtension;
} XcpAddress;

/*****************************************************************************
    XCP CONNECT
******************************************************************************/
#define XCP_CONNECT_MODE_NORMAL        0
#define XCP_CONNECT_MODE_USER_DEFINED  1

#define XCP_CONNECT_RES_PACKET_SIZE_IN_BYTES 8
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpConnectCmdPacketFrame {
        uint8_T PID;     /**< Packet identifier, always XCP_PID_CONNECT */
        uint8_T mode;    /**< 00 -> Normal mode, 01 -> user defined  */
    }   XcpConnectCmdPacketFrame;

    typedef struct XCP_ATTRIBUTE_PACKED XcpConnectResPacketFrame {
        uint8_T          PID;                 /**< Packet identifier, always XCP_PID_RES if positive response */
        uint8_T          resource;            /**< Resource (see XCP Resource bit masks) */
        uint8_T          commModeBasic;       /**< Basic Communication Mode (see XCP Basic Communication Mode bit masks) */
        uint8_T          maxCtoSize;          /**< max size in byte for CTO packets */
        uint16_T         maxDtoSize;          /**< max size in byte for DTO packets */
        uint8_T          xcpProtocolVersion;  /**< XCP Protocol Layer Version Number */
        uint8_T          xcpTransportVersion; /**< XCP Transport Layer Version Number */
    }   XcpConnectResPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpConnectCmdPacketFrame {
        uint16_T PID    :8;
        uint16_T mode   :8;
    }   XcpConnectCmdPacketFrame;

    typedef struct XcpConnectResPacketFrame {
        uint16_T PID                    :8;
        uint16_T resource               :8;
        uint16_T commModeBasic          :8;
        uint16_T maxCtoSize             :8;
        uint16_T maxDtoSize             :16;
        uint16_T xcpProtocolVersion     :8;
        uint16_T xcpTransportVersion    :8;
    }   XcpConnectResPacketFrame;
#endif

/*****************************************************************************
    XCP GET STATUS
******************************************************************************/
#define XCP_GET_STATUS_RES_PACKET_SIZE_IN_BYTES 6
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpGetStatusCmdPacketFrame {
        uint8_T PID;     /**< Packet identifier, always XCP_PID_GET_STATUS */
    }   XcpGetStatusCmdPacketFrame;

    typedef struct XCP_ATTRIBUTE_PACKED XcpGetStatusResPacketFrame {
        uint8_T   PID;                      /**< Packet identifier, always XCP_PID_RES */
        uint8_T   sessionStatus;            /**< Session Status (see XCP Session Status bit masks) */
        uint8_T   resourceProtectionStatus; /**< Resource Protection status (see XCP Resource bit masks)
                                                 @note If protected, all the commands allocated
                                                       to a respective resource will return an ERR_ACCESS_LOCKED
                                                       upon an attempt to execute a command without a previous
                                                       successful GET_SEED/UNLOCK sequence */
        uint8_T   reserved;                 /**< reserved */
        uint16_T  sessionConfigurationId;   /**< Session Configuration Id */
    }   XcpGetStatusResPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpGetStatusCmdPacketFrame {
        uint16_T PID :8;
    }   XcpGetStatusCmdPacketFrame;

    typedef struct XcpGetStatusResPacketFrame {
        uint16_T PID                       :8;
        uint16_T sessionStatus             :8;
        uint16_T resourceProtectionStatus  :8;
        uint16_T reserved                  :8;
        uint16_T sessionConfigurationId    :16;
    }   XcpGetStatusResPacketFrame;
#endif

/*****************************************************************************
    XCP SET_MTA
******************************************************************************/
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpSetMTACmdPacketFrame {
        uint8_T  PID;              /**< Packet identifier, always XCP_PID_SET_MTA */
        uint16_T reserved;         /**< reserved word */
        uint8_T  addressExtension; /**< address extension of the parameter to upload */
        uint32_T address;          /**< address of the parameter to upload */
    }   XcpSetMTACmdPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpSetMTACmdPacketFrame {
        uint16_T PID                    :8;
        uint16_T reserved1              :8;        /**< first part of reserved non-aligned word */
        uint16_T reserved2              :8;        /**< second part of reserved non-aligned word */
        uint16_T addressExtension       :8;
        uint32_T address;
    }   XcpSetMTACmdPacketFrame;
#endif

/*****************************************************************************
    XCP UPLOAD
******************************************************************************/
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpUploadCmdPacketFrame {
        uint8_T  PID;              /**< Packet identifier, always XCP_PID_UPLOAD */
        uint8_T  size;             /**< number of data elements, it must be in the range
                                        [0..(XCP_MAX_CTO_SIZE)/XCP_ADDRESS_GRANULARITY] */
    }   XcpUploadCmdPacketFrame;

    #define XCP_UPLOAD_RES_PACKET_SIZE_IN_BYTES 1
    typedef struct XCP_ATTRIBUTE_PACKED XcpUploadResPacketFrame {
        uint8_T   PID;             /**< Packet identifier, always XCP_PID_RES if positive response */
        /* The remaining bytes of the packet represent the actual content
           of the parameter to be uploaded */
    }   XcpUploadResPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpUploadCmdPacketFrame {
        uint16_T  PID       :8;
        uint16_T  size      :8;
    }   XcpUploadCmdPacketFrame;
    
#if (XCP_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE)
    #define XCP_UPLOAD_RES_PACKET_SIZE_IN_BYTES 1
#else
    #define XCP_UPLOAD_RES_PACKET_SIZE_IN_BYTES 2
#endif
    typedef struct XcpUploadResPacketFrame {
        uint16_T   PID      :8;
        uint16_T   padding  :8;
        /* The remaining bytes of the packet represent the actual content
           of the parameter to be uploaded */
    }   XcpUploadResPacketFrame;
#endif

/*****************************************************************************
    XCP SHORT UPLOAD
******************************************************************************/
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpShortUploadCmdPacketFrame {
        uint8_T  PID;              /**< Packet identifier, always XCP_PID_SHORT_UPLOAD */
        uint8_T  size;             /**< number of data elements, it must be in the range
                                        [0..(XCP_MAX_CTO_SIZE)/XCP_ADDRESS_GRANULARITY] */
        uint8_T  reserved;         /**< reserved byte */
        uint8_T  addressExtension; /**< address extension of the parameter to upload */
        uint32_T address;          /**< address of the parameter to upload */
    }   XcpShortUploadCmdPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpShortUploadCmdPacketFrame {
        uint16_T  PID               :8;
        uint16_T  size              :8;
        uint16_T  reserved          :8;
        uint16_T  addressExtension  :8;
        uint32_T  address;
    }   XcpShortUploadCmdPacketFrame;
#endif

/* Short upload response packet has same form as upload response packet */
#define XCP_SHORT_UPLOAD_RES_PACKET_SIZE_IN_BYTES XCP_UPLOAD_RES_PACKET_SIZE_IN_BYTES
typedef struct XcpUploadResPacketFrame XcpShortUploadResPacketFrame;

#endif /* XCP_STANDARD_TYPES_H */
