/*
* Copyright 2016-2023 The MathWorks, Inc.
*
* File: xcp_types.h
*
* Abstract:
*  XCP Protocol layer data types and definitions
*/

#ifndef XCP_TYPES_H
#define XCP_TYPES_H

#include "xcp_common.h"
#include "xcp_cfg.h"

/** XCP PID (Packet IDentifier) Code for Rx packets (received from the XCP Client) */
typedef enum {
    XCP_PID_CONNECT = 0xFF,
    XCP_PID_DISCONNECT = 0xFE,
    XCP_PID_GET_STATUS = 0xFD,
    XCP_PID_SYNCH = 0xFC,

    XCP_PID_SET_MTA = 0xF6,
    XCP_PID_UPLOAD = 0xF5,
    XCP_PID_SHORT_UPLOAD = 0xF4,

    XCP_PID_DOWNLOAD = 0xF0,
    XCP_PID_SHORT_DOWNLOAD = 0xED,

    /* Extended Calibration PIDs */
    XCP_PID_SET_CAL_PAGE = 0xEB,
    XCP_PID_GET_CAL_PAGE = 0xEA,
    XCP_PID_COPY_CAL_PAGE = 0xE4,

    XCP_PID_SET_DAQ_PTR = 0xE2,
    XCP_PID_WRITE_DAQ   = 0xE1,
    XCP_PID_SET_DAQ_LIST_MODE = 0xE0,

    XCP_PID_GET_DAQ_CLOCK = 0xDC,

    XCP_PID_START_STOP_DAQ_LIST = 0xDE,
    XCP_PID_START_STOP_SYNCH = 0xDD,

    XCP_PID_GET_DAQ_PROCESSOR_INFO = 0xDA,
    XCP_PID_GET_DAQ_RESOLUTION_INFO = 0xD9,

    XCP_PID_FREE_DAQ = 0xD6,
    XCP_PID_ALLOC_DAQ = 0xD5,
    XCP_PID_ALLOC_ODT = 0xD4,
    XCP_PID_ALLOC_ODT_ENTRY = 0xD3,

    XCP_PID_LEVEL1_COMMAND = 0xC0
} XcpRxPidCode;

/** XCP PID (Packet IDentifier) Code for Tx packets (sent to the XCP Client) */
typedef enum {
    XCP_PID_RES = 0xFF, /**< Command Response Packet (RES) */
    XCP_PID_ERR = 0xFE, /**< Error Packet (ERR) */
    XCP_PID_EV = 0xFD, /**< Event Packet (EV) */
    XCP_PID_SERV = 0xFC /**< Service Request Packet (EV) */
} XcpTxPidCode;

/** XCP Error Packet's error codes */
typedef enum {
    XCP_ERR_CMD_SYNC = 0x00, /**< Command processor synchronization */

    XCP_ERR_CMD_BUSY = 0x10, /**< Command was not executed */
    XCP_ERR_DAQ_ACTIVE = 0x11, /**< Command rejected because DAQ is running */
    XCP_ERR_PGM_ACTIVE = 0x12, /**< Command rejected because PGM is running */

    XCP_ERR_CMD_UNKNOWN = 0x20, /**< Unknown command or not implemented optional command */
    XCP_ERR_CMD_SYNTAX = 0x21, /**< Command syntax invalid */
    XCP_ERR_OUT_OF_RANGE = 0x22, /**< Command syntax valid, but command parameter(s) out of range */
    XCP_ERR_WRITE_PROTECTED = 0x23, /**< The memory location is write protected */
    XCP_ERR_ACCESS_DENIED = 0x24, /**< The memory location is not accessible */
    XCP_ERR_ACCESS_LOCKED = 0x25, /**< Access denied, Seed & Key is required */
    XCP_ERR_PAGE_NOT_VALID = 0x26, /**< Selected page not available */
    XCP_ERR_MODE_NOT_VALID = 0x27, /**< Selected page mode not valid */
    XCP_ERR_SEGMENT_NOT_VALID = 0x28, /**< Selected segment not valid */
    XCP_ERR_SEQUENCE = 0x29, /**< Sequence error */
    XCP_ERR_DAQ_CONFIG = 0x2A, /**< DAQ configuration not valid */

    XCP_ERR_MEMORY_OVERFLOW = 0x30, /**< Memory overflow error */
    XCP_ERR_GENERIC = 0x31, /**< Generic error */
    XCP_ERR_VERIFY = 0x32, /**< The server internal program verify routine detects an error */
    XCP_ERR_RESOURCE_NOT_ACCESSIBLE = 0x33  /**< Access to the requested resource is temporary not possible */
} XcpErrorPacketCode;

/** XCP Protocol Layer Internal Error codes */
typedef enum {
    XCP_PROTO_SUCCESS          =  0,
    XCP_PROTO_CMD_UNKNOWN      = -1,
    XCP_PROTO_OUT_OF_RANGE     = -2,
    XCP_PROTO_MEMORY_OVERFLOW  = -3,
    XCP_PROTO_SEQUENCE_ERROR   = -4,
    XCP_PROTO_SYNCH            = -5,
    XCP_PROTO_WRITE_PROTECTED  = -6,
    XCP_PROTO_CMD_SYNTAX       = -7,
    XCP_PROTO_DAQ_ACTIVE       = -8,
    XCP_PROTO_DAQ_CONFIG_ERROR = -9,
    XCP_PROTO_MODE_NOT_VALID   = -10,
    XCP_PROTO_BUSY             = -11,
    XCP_PROTO_ACCESS_DENIED    = -12,
    XCP_PROTO_SEGMENT_UNKNOWN  = -13,
    XCP_PROTO_PAGE_UNKNOWN     = -14,
    XCP_PROTO_GENERIC_ERROR    = -15,
    XCP_PROTO_RESOURCE_NOT_ACCESSIBLE = -16
} XcpProtoErrorCode;

/** XCP Input Packet Handler
    The function is responsible for processing the input packet content,
    returning and internal code and the size of the subsequent output packet
    (0 if no response is required) */
typedef XcpProtoErrorCode (*XcpInputPacketHandler) (void   *msgBuffer,
                                                    size_t  xcpPacketOffset,
                                                    size_t *outputPacketSize);

/** XCP Output Packet Handler
    The function is responsible for updating the output packet content,
    based on the XcpProtoErrorCode from the previous input packet processing */
typedef void (*XcpOutputPacketHandler) (XcpProtoErrorCode inputCode,
                                        void *packet,
                                        size_t packetSize);

/** Packet Handlers for XCP Rx packets */
typedef struct XcpPacketHandlers {
    XcpRxPidCode           PID;            /**< Packet identifier     */
    XcpInputPacketHandler  inputHandler;   /**< Input Packet handler  */
    XcpOutputPacketHandler outputHandler;  /**< Output Packet handler */
} XcpPacketHandlers;


/** XCP Packet Lookup Function
    If the PID is supported the function is responsible for returning the
    corresponding Input and Output packet handlers.
    If the PID is NOT supported, the NULL value is returned instead */
typedef const XcpPacketHandlers* (*XcpPacketLookupFunction) (XcpRxPidCode PID);


/** Packet Handlers Group ID data type */
typedef enum XcpPacketsGroupIdType {
    XCP_STANDARD_PACKETS_ID,
    XCP_CALIBRATION_PACKETS_ID,
    XCP_DAQ_PACKETS_ID,
    XCP_PROGRAM_PACKETS_ID,
    XCP_PACKETS_GROUP_NUMBER,
    XCP_UNKNOWN_PACKET_GROUP_ID
} XcpPacketsGroupIdType;


/** Generic Output Packet handler */
void genericOutputPacketHandler(XcpProtoErrorCode inputCode, void *packet, size_t packetSize);

/*****************************************************************************
    Internal MACROS and utility functions
******************************************************************************/
/** This is a simplified version of Read/Write semaphore based on one mutex only.
The mutex is used by the readers just to update the readersCounter value.
The same mutex is locked by the Writer and released if at least one reader
is using the resource.
By checking the XCP_WRITE_TRY_LOCK() return value, the Writer can understand
if the access to the resource has been granted, or if the status of the resource
is busy and it's not possible to proceed with the status update.
*/
#define XCP_READ_LOCK(lock,readersCounter) do {                           \
                                                  XCP_MUTEX_LOCK(lock);   \
                                                  (readersCounter)++;     \
                                                  XCP_MUTEX_UNLOCK(lock); \
                                           } while(0)

#define XCP_READ_UNLOCK(lock,readersCounter) do {                           \
                                                    XCP_MUTEX_LOCK(lock);   \
                                                    (readersCounter)--;     \
                                                    XCP_MUTEX_UNLOCK(lock); \
                                             } while (0)

#define XCP_WRITE_TRY_LOCK(lock, readersCounter, locked) do {                       \
                                                    XCP_MUTEX_LOCK(lock);           \
                                                    locked = (readersCounter == 0); \
                                                    if (!locked) {                  \
                                                        XCP_MUTEX_UNLOCK(lock);     \
                                                    }                               \
                                                } while (0)

#define XCP_WRITE_UNLOCK(lock)  XCP_MUTEX_UNLOCK(lock)

/** Error check macro used by various Input Packet Handlers */
#define XCP_INPUT_PKT_ERROR_IF(cond, protoCode, printfMsgWithParentheses)           \
                               if (cond) {                                          \
                                   XCP_PRINTF printfMsgWithParentheses;             \
                                   *outputPacketSize = XCP_ERROR_PACKET_SIZE_IN_BYTES; \
                                   return protoCode;                                \
                               }

/** Error macro used by various Input Packet Handlers */
#define XCP_INPUT_PKT_ERROR(protoCode, printfMsgWithParentheses)  do {          \
                               XCP_PRINTF printfMsgWithParentheses;             \
                               *outputPacketSize = XCP_ERROR_PACKET_SIZE_IN_BYTES; \
                               return protoCode;                                \
                           } while(0)

/** Macros used for bitwise access */
#define XCP_SET_MASK(var,mask)   (var) |= (var) | (mask)
#define XCP_CLEAR_MASK(var,mask) (var) &= ~(mask)

#define XCP_WRITE_BIT_VALUE(var,mask,value)   if (value) { XCP_SET_MASK(var,mask); } \
                                              else {XCP_CLEAR_MASK(var,mask);}

#define XCP_READ_BIT_VALUE(var,mask)  (((var) & (mask)) != 0)

/** Search for packet pid in the packets list.
    The function returns NULL if the packet is not found */
const XcpPacketHandlers* xcpFindPacket(XcpRxPidCode pid, const XcpPacketHandlers* packets, size_t packetsNumber);

/*****************************************************************************
    XCP Packets ranges
******************************************************************************/
#define XCP_STANDARD_PACKETS_ID_MAX      0xFF
#define XCP_STANDARD_PACKETS_ID_MIN      0xF1

#define XCP_CALIBRATION_PACKETS_ID_MAX   0xF0
#define XCP_CALIBRATION_PACKETS_ID_MIN   0xE4

#define XCP_DAQ_PACKETS_ID_MAX           0xE2
#define XCP_DAQ_PACKETS_ID_MIN           0xD3

#define XCP_PROGRAM_PACKETS_ID_MAX       0xD2
#define XCP_PROGRAM_PACKETS_ID_MIN       0xC8

#define XCP_COMMAND_PACKETS_ID_MIN       0xC0

#define XCP_DAQ_LEVEL1_CODE_MAX          0x02
#define XCP_DAQ_LEVEL1_CODE_MIN          0x01


/*****************************************************************************
    XCP Status
******************************************************************************/
/** Get Current Status of the XCP Server */
XcpStatus xcpStatusGet(void);

/** Set Current Status of the XCP Server */
void xcpStatusSet(XcpStatus status);

/*****************************************************************************
    XCP Session Status
******************************************************************************/
/** Request to store calibration data bit: 0 -> reset, 1 -> set */
#define XCP_SESSION_STORE_CAL_REQ_MASK   0x01

/** Request to store DAQ list bit: 0 -> reset, 1 -> set */
#define XCP_SESSION_STORE_DAQ_REQ_MASK   0x04

/** Request to clear DAQ configuration bit: 0 -> reset, 1 -> set */
#define XCP_SESSION_CLEAR_DAQ_REQ_MASK   0x08

/** Data transfer status bit: 0 -> not running, 1 -> running */
#define XCP_SESSION_DAQ_RUNNING_MASK     0x40

/** Server resume mode bit: 0 -> not active, 1 -> active */
#define XCP_SESSION_RESUME_MASK          0x80

/** Get Current Session Status */
uint8_T xcpSessionStatusGet(void);

/** Set Current Session Status */
void xcpSessionStatusSet(uint8_T status);

/** Set Current Session Status Bits */
void xcpSessionStatusSetMask(uint8_T mask);

/** Clear Current Session Status Bits */
void xcpSessionStatusClearMask(uint8_T mask);

/** Get Current Session Configuration Id */
uint16_T xcpSessionConfigurationIdGet(void);

/** Set Current Session Configuration Id */
void xcpSessionConfigurationIdSet(uint16_T id);

/*****************************************************************************
    XCP Resource bit masks
******************************************************************************/
/** CALibration and PAGing bit: 0 -> not available, 1 -> available */
#define XCP_RESOURCE_CAL_PAG_MASK   0x01

/** DAQ lists bit: 0 -> not available, 1 -> available */
#define XCP_RESOURCE_DAQ_MASK       0x04

/** STIMulation mode bit: 0 -> not available, 1 -> available */
#define XCP_RESOURCE_STIM_MASK      0x08

/** (Flash) ProGraMming bit: 0 -> not available, 1 -> available */
#define XCP_RESOURCE_PGM_MASK       0x10

/** Get Current Resource Protection Status */
uint8_T xcpResourceProtectionStatusGet(void);

/** Set Current Resource Protection Status */
void xcpResourceProtectionStatusSet(uint8_T status);

/** Set Current Resource Protection Status Bits */
void xcpResourceProtectionSetMask(uint8_T mask);

/** Clear Current Resource Protection Status Bits */
void xcpResourceProtectionClearMask(uint8_T mask);

/*****************************************************************************
    XCP Generic Response
******************************************************************************/
#define XCP_GENERIC_RES_PACKET_SIZE_IN_BYTES 1
XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpGenericResPacketFrame {
    uint8_T PID;       /**< Packet identifier, always XCP_PID_RES */
}   XcpGenericResPacketFrame;
XCP_PRAGMA_PACK_END()

/*****************************************************************************
    XCP Error Packet
******************************************************************************/
#define XCP_ERROR_PACKET_SIZE_IN_BYTES 2
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpErrorPacketFrame {
        uint8_T PID;       /**< Packet identifier, always XCP_PID_ERR */
        uint8_T errorCode; /**< Error Code -> XcpErrorPacketCode  */
    }   XcpErrorPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpErrorPacketFrame {
        uint16_T PID        :8;
        uint16_T errorCode  :8;
    }   XcpErrorPacketFrame;
#endif

/*****************************************************************************
    XCP Event Packet
******************************************************************************/
#define XCP_EVENT_PACKET_SIZE_IN_BYTES 2
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpEventPacketFrame {
        uint8_T PID;       /**< Packet identifier, always XCP_PID_EV */
        uint8_T eventCode; /**< Event Code -> XcpEventCode  */
        /* The remaining part of the frame contains optional event information data */
    }   XcpEventPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpEventPacketFrame {
        uint16_T PID        :8;
        uint16_T eventCode  :8;
    }   XcpEventPacketFrame;
#endif

/*****************************************************************************
    XCP Service Request Packet
******************************************************************************/
#define XCP_SERVICE_REQ_PACKET_SIZE_IN_BYTES 2
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpServiceReqPacketFrame {
        uint8_T PID;            /**< Packet identifier, always XCP_PID_SERV */
        uint8_T serviceReqCode; /**< Service Request Code -> XcpReqServiceCode */
        /* The remaining part of the frame contains optional service request data */
    }   XcpServiceReqPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpServiceReqPacketFrame {
        uint16_T PID             :8;
        uint16_T serviceReqCode  :8;
    }   XcpServiceReqPacketFrame;
#endif

/*****************************************************************************
    XCP Level 1 Command
******************************************************************************/
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpLevel1CommandPacketFrame {
        uint8_T PID;            /**< Packet identifier, always 0xC0 */
        uint8_T level1Code;     /**< Level 1 command code */
        /* The remaining part of the frame is command specific */
    }   XcpLevel1CommandPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpLevel1CommandPacketFrame {
        uint16_T PID        :8;
        uint16_T level1Code :8;
    }   XcpLevel1CommandPacketFrame;
#endif

#endif /* XCP_TYPES_H */

