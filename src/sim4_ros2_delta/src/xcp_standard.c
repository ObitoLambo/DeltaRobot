/*
* Copyright 2016-2022 The MathWorks, Inc.
*
* File: xcp_standard.c
*
* Abstract:
*  Implementation of XCP Protocol Layer standard commands
*/

#include "xcp_common.h"
#include "xcp.h"
#include "xcp_cfg.h"
#include "xcp_types.h"
#include "xcp_transport_internal.h"
#include "xcp_daq.h"
#include "xcp_standard.h"
#include "xcp_standard_ext.h"
#include "xcp_standard_types.h"


/*****************************************************************************
    Internal Variables
******************************************************************************/
static XcpAddress xcpMta = {0,0};   /* MTA and MTA extension in same format as in SET_MTA packet, used by (SHORT)_UPLOAD, (SHORT)_DOWNLOAD */
static uint8_T xcpUploadSize = 0;


/*****************************************************************************
    Internal Utility Functions
******************************************************************************/
static boolean_T isValidUploadSize(uint8_T size)
{
    size_t addressGranularity = XCP_ADDRESS_GRANULARITY_BYTES_NUMBER;
    size_t maxCtoSize = xcpTransportMaxCtoSize();

    return (size < ((maxCtoSize) / addressGranularity));
}

/** memcpy function used to copy data from MTA address to the packet */
static void xcpMemcpyFromMTA(uint8_T *pktData, uint8_T dstOffsetBytes)
{
    /* Retrieve memory address */
    uint8_T *xcpMTARawPointer = xcpStandardGetAddressFromMta();

    XCP_PRINTF("reading at address %p\n", xcpMTARawPointer);

#ifdef XCP_EMULATE_BYTE_ADDRESSABLE_TARGET
{
    uint32_T address;
    uint8_T addressExtension;
    uint8_T srcOffsetBytes;
    xcpStandardGetMta(&address, &addressExtension);
    srcOffsetBytes = XCP_BYTE_OFFSET_GET(address);
    if (dstOffsetBytes) { pktData--; }
    xcpMemcpyByte(pktData, dstOffsetBytes, xcpMTARawPointer, srcOffsetBytes, xcpUploadSize);
}
#else
    XCP_UNUSED_PARAM(dstOffsetBytes);
    XCP_MEMCPY(pktData, xcpMTARawPointer, xcpUploadSize);
#endif

}

/*****************************************************************************
    XCP CONNECT
******************************************************************************/
#define XCP_NO_RESOURCES  0x00

static const uint8_T xcpResourceValue = (
    XCP_NO_RESOURCES
#ifdef XCP_DAQ_SUPPORT
    | XCP_RESOURCE_DAQ_MASK
#endif
#ifdef XCP_STIM_SUPPORT
    | XCP_RESOURCE_STIM_MASK
#endif
#ifdef XCP_PGM_SUPPORT
    | XCP_RESOURCE_PGM_MASK
#endif
#ifdef XCP_CALIBRATION_SUPPORT
    | XCP_RESOURCE_CAL_PAG_MASK
#endif
    );

static const uint8_T xcpCommModeBaseValue = (
    (XCP_ADDRESS_GRANULARITY << XCP_COMM_MODE_ADDRESS_GRANULARITY_OFFSET)
#if (XCP_BYTE_ORDER != 0)
    | XCP_COMM_MODE_BYTE_ORDER_MASK
#endif
#ifdef XCP_BLOCK_MODE_SUPPORT
    | XCP_COMM_MODE_SLAVE_BLOCK_MODE_MASK
#endif
#ifdef XCP_COMM_MODE_INFO_SUPPORT
    | XCP_COMM_MODE_OPTIONAL_MASK
#endif
    );

static XcpProtoErrorCode connectInputPacketHandler(void   *msgBuffer,
                                                   size_t  xcpPacketOffset,
                                                   size_t *outputPacketSize)
{
    XcpProtoErrorCode protoErrorCode = XCP_PROTO_SUCCESS;
    uint8_T *packet = (uint8_T *)msgBuffer + xcpPacketOffset;
    XcpConnectCmdPacketFrame *frame = (XcpConnectCmdPacketFrame *) packet;
    boolean_T ok = false;

    /* If DAQ List support is enabled, reset the status of dynamic DAQ list data structures */
    ok = xcpResetDaqListStatus();

    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_BUSY,
        ("CONNECT: DAQ lists reset cannot be carried out, as an event was processed\n"));

    /* Validate command inputs */
    ok = (frame->mode == XCP_CONNECT_MODE_NORMAL) || (frame->mode == XCP_CONNECT_MODE_USER_DEFINED);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE,
                          ("CONNECT: invalid mode detected %xH\n", frame->mode));

    /* Everything is fine -> updating the Protocol Layer status */
    XCP_PRINTF("CONNECT: entering XCP_CONNECTED status\n");
    xcpStatusSet(XCP_CONNECTED);

    protoErrorCode = XCP_PROTO_SUCCESS;
    *outputPacketSize = XCP_CONNECT_RES_PACKET_SIZE_IN_BYTES;

    return protoErrorCode;
}

static void connectOutputPacketHandler(XcpProtoErrorCode inputCode, void *packet, size_t packetSize)
{
    if (inputCode == XCP_PROTO_SUCCESS) {
        /* Fill connect response */
        XcpConnectResPacketFrame *frame = (XcpConnectResPacketFrame *)packet;
        XCP_MEMSET(frame, 0, sizeof(*frame));

        frame->PID = XCP_PID_RES;

        /* Update resource field */
        frame->resource = xcpResourceValue;

        /* Update commModeBasic field */
        frame->commModeBasic = xcpCommModeBaseValue;

        /* Update Max CTO and DTO fields */
        frame->maxCtoSize = (uint8_T)xcpTransportMaxCtoSize();
        frame->maxDtoSize = (uint16_T)xcpTransportMaxDtoSize();

        /* Update Protocol and Transport Version fields */
        frame->xcpProtocolVersion  = XCP_MAJOR_NUMBER(XCP_PROTOCOL_LAYER_VERSION);
        frame->xcpTransportVersion = XCP_MAJOR_NUMBER(XCP_TRANSPORT_LAYER_VERSION);

        XCP_PRINTF("* Resource:          %xH\n", frame->resource);
        XCP_PRINTF("* Comm Mode Basic:   %xH\n", frame->commModeBasic);
        XCP_PRINTF("* Max CTO size:      %d\n",  frame->maxCtoSize);
        XCP_PRINTF("* Max DTO size:      %d\n",  frame->maxDtoSize);
        XCP_PRINTF("* Protocol Version:  %d\n",  frame->xcpProtocolVersion);
        XCP_PRINTF("* Transport Version: %d\n",  frame->xcpTransportVersion);
    }
    else {
        genericOutputPacketHandler(inputCode, packet, packetSize);
    }
}

/*****************************************************************************
    XCP DISCONNECT
******************************************************************************/
static XcpProtoErrorCode disconnectInputPacketHandler(void   *msgBuffer,
                                                      size_t  xcpPacketOffset,
                                                      size_t *outputPacketSize)
{
    boolean_T ok = true;

    XCP_UNUSED_PARAM(msgBuffer);
    XCP_UNUSED_PARAM(xcpPacketOffset);

    /* If DAQ List support is enabled, reset the status of dynamic DAQ list data structures */
    ok = xcpResetDaqListStatus();

    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_BUSY,
        ("DISCONNECT: DAQ lists reset cannot be carried out, as an event was processed\n"));

    /* Everything is fine -> proceed with the status update */
    XCP_PRINTF("DISCONNECT: entering XCP_DISCONNECTED status\n");
    xcpStatusSet(XCP_DISCONNECTED);

    *outputPacketSize = XCP_GENERIC_RES_PACKET_SIZE_IN_BYTES;

    return XCP_PROTO_SUCCESS;
}

/*****************************************************************************
    XCP GET_STATUS
******************************************************************************/
static XcpProtoErrorCode getStatusInputPacketHandler(void   *msgBuffer,
                                                     size_t  xcpPacketOffset,
                                                     size_t *outputPacketSize)
{
    XCP_UNUSED_PARAM(msgBuffer);
    XCP_UNUSED_PARAM(xcpPacketOffset);

    XCP_PRINTF("GET STATUS\n");

    *outputPacketSize = XCP_GET_STATUS_RES_PACKET_SIZE_IN_BYTES;

    return XCP_PROTO_SUCCESS;
}

static void getStatusOutputPacketHandler(XcpProtoErrorCode inputCode, void *packet, size_t packetSize)
{
    XcpGetStatusResPacketFrame *frame = (XcpGetStatusResPacketFrame *)packet;

    XCP_UNUSED_PARAM(inputCode);
    XCP_UNUSED_PARAM(packetSize);

    /* Fill Get Status response */
    XCP_MEMSET(frame, 0, sizeof(*frame));

    frame->PID                      = XCP_PID_RES;
    frame->sessionStatus            = xcpSessionStatusGet();
    frame->resourceProtectionStatus = xcpResourceProtectionStatusGet();
    frame->sessionConfigurationId   = xcpSessionConfigurationIdGet();

    XCP_PRINTF("* Session Status:              %xH\n", frame->sessionStatus);
    XCP_PRINTF("* Resource Protection Status:  %xH\n", frame->resourceProtectionStatus);
    XCP_PRINTF("* Session Config Id:           %d\n", frame->sessionConfigurationId);
}

/*****************************************************************************
    XCP SYNCH
******************************************************************************/
#define XCP_SYNC_RES_PACKET_SIZE_IN_BYTES 2

static XcpProtoErrorCode synchInputPacketHandler(void   *msgBuffer,
                                                 size_t  xcpPacketOffset,
                                                 size_t *outputPacketSize)
{
    boolean_T locked = false;

    XCP_UNUSED_PARAM(msgBuffer);
    XCP_UNUSED_PARAM(xcpPacketOffset);

    XCP_PRINTF("SYNCH\n");

    /* Make sure that no DAQ list is currently
       in the process of sending packets */
    locked = xcpDaqLock();

    if (locked) {
        /* Re-synchronizing the transport layer, by flushing the TX packets currently
           in the TX FIFO and restarting the frame handler */
        xcpTransportResynch();

        xcpDaqUnlock();
    }

    *outputPacketSize = XCP_SYNC_RES_PACKET_SIZE_IN_BYTES;

    return XCP_PROTO_SYNCH;
}


#if XCP_SET_MTA_ENABLE == 1
/*****************************************************************************
    XCP SET_MTA
******************************************************************************/
    static XcpProtoErrorCode setMTAInputPacketHandler(void  *msgBuffer, 
        size_t  xcpPacketOffset, 
        size_t *outputPacketSize) {

        XcpProtoErrorCode protoErrorCode = XCP_PROTO_SUCCESS;
        uint8_T *packet = (uint8_T *)msgBuffer + xcpPacketOffset;
        XcpSetMTACmdPacketFrame *frame = (XcpSetMTACmdPacketFrame *)packet;
        boolean_T ok = false;

        /* Check if the memory address is valid and set the MTA pointer */
        ok = xcpStandardSetMta(frame->address, (uint8_T)frame->addressExtension);
        XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_ACCESS_DENIED, ("SET_MTA: invalid address\n"));
        
        XCP_PRINTF("SET_MTA: setting MTA value to %x and extension %x\n", frame->address, frame->addressExtension);    

        *outputPacketSize = sizeof(XcpGenericResPacketFrame);

        return protoErrorCode;
    }

/*****************************************************************************
    XCP UPLOAD
******************************************************************************/
    static XcpProtoErrorCode uploadInputPacketHandler(void   *msgBuffer,
        size_t  xcpPacketOffset,
        size_t *outputPacketSize) {

        XcpProtoErrorCode protoErrorCode = XCP_PROTO_SUCCESS;
        uint8_T *packet = (uint8_T *)msgBuffer + xcpPacketOffset;
        XcpUploadCmdPacketFrame *frame = (XcpUploadCmdPacketFrame *)packet;
        boolean_T ok = false;
        uint8_T *xcpMTARawPointer = xcpStandardGetAddressFromMta();

        /* Check if the number of data elements is valid */
        ok = isValidUploadSize((uint8_T)frame->size);
        XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE,
            ("UPLOAD: invalid number of data elements (%d)\n", (uint8_T)frame->size));
        
        /* Check if a valid MTA address is available */
        XCP_INPUT_PKT_ERROR_IF(xcpMTARawPointer == 0, XCP_PROTO_ACCESS_DENIED,
            ("UPLOAD: invalid address\n"));
        
        xcpUploadSize = (uint8_T)frame->size;
        *outputPacketSize = XCP_UPLOAD_RES_PACKET_SIZE_IN_BYTES + XCP_IN_BYTES(xcpUploadSize);

        return protoErrorCode;
    }

    
    static void uploadOutputPacketHandler(XcpProtoErrorCode inputCode, void *packet, size_t packetSize) {
        
        if (inputCode == XCP_PROTO_SUCCESS) {

            /* Fill Upload response */
            uint8_T *xcpMTARawPointer = xcpStandardGetAddressFromMta();
            XcpUploadResPacketFrame *frame = (XcpUploadResPacketFrame *)packet;
            XCP_MEMSET(frame, 0, sizeof(*frame));
            frame->PID = XCP_PID_RES;

            if ((xcpMTARawPointer != NULL) &&
                (packetSize == (XCP_UPLOAD_RES_PACKET_SIZE_IN_BYTES + XCP_IN_BYTES(xcpUploadSize)))
                ) {
                uint8_T *pktData = (uint8_T *)packet + XCP_IN_AG(XCP_UPLOAD_RES_PACKET_SIZE_IN_BYTES);
                uint8_T dstOffsetBytes = (XCP_UPLOAD_RES_PACKET_SIZE_IN_BYTES % XCP_HARDWARE_ADDRESS_GRANULARITY_BYTES_NUMBER);

                XCP_PRINTF("UPLOAD: ");
                
                /* Copy the memory content to the packet data area */
                xcpMemcpyFromMTA(pktData, dstOffsetBytes);

                /* Post-increment MTA by the frame size in address granularity units */
                xcpStandardIncrementMta(xcpUploadSize);
                xcpUploadSize = 0;
            }
            else {
                /* This is a software error and it should never happen */
                XCP_PRINTF("UPLOAD: invalid data detected\n");
            }
        }
        else {
            genericOutputPacketHandler(inputCode, packet, packetSize);
        }
    }
#endif  /* XCP_SET_MTA_ENABLE == 1  */


/*****************************************************************************
    XCP SHORT_UPLOAD
******************************************************************************/
static XcpProtoErrorCode shortUploadInputPacketHandler(void   *msgBuffer,
    size_t  xcpPacketOffset,
    size_t *outputPacketSize)
{
    XcpProtoErrorCode protoErrorCode = XCP_PROTO_SUCCESS;
    uint8_T *packet = (uint8_T *)msgBuffer + xcpPacketOffset;
    XcpShortUploadCmdPacketFrame *frame = (XcpShortUploadCmdPacketFrame *)packet;
    boolean_T ok = false;

    /* Check if the number of data elements is valid */
    ok = isValidUploadSize((uint8_T)frame->size);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE,
        ("SHORT_UPLOAD: invalid number of data elements (%d)\n", (uint8_T)frame->size));

    /* Check if the memory address is valid and set the MTA pointer */
    ok = xcpStandardSetMta(frame->address, (uint8_T)frame->addressExtension);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_ACCESS_DENIED, ("SHORT_UPLOAD: invalid address\n"));

    xcpUploadSize = (uint8_T)frame->size;

    *outputPacketSize = XCP_SHORT_UPLOAD_RES_PACKET_SIZE_IN_BYTES + XCP_IN_BYTES(xcpUploadSize);

    return protoErrorCode;
}

static void shortUploadOutputPacketHandler(XcpProtoErrorCode inputCode, void *packet, size_t packetSize)
{    
    if (inputCode == XCP_PROTO_SUCCESS) {

        /* Fill Short Upload response */
        uint8_T *xcpMTARawPointer = xcpStandardGetAddressFromMta();
        XcpShortUploadResPacketFrame *frame = (XcpShortUploadResPacketFrame *)packet;
        XCP_MEMSET(frame, 0, sizeof(*frame));

        frame->PID = XCP_PID_RES;

        if ((xcpMTARawPointer != NULL) &&
            (packetSize == XCP_SHORT_UPLOAD_RES_PACKET_SIZE_IN_BYTES + XCP_IN_BYTES(xcpUploadSize) )
           ) {
            uint8_T *pktData = (uint8_T *)packet + XCP_IN_AG(XCP_SHORT_UPLOAD_RES_PACKET_SIZE_IN_BYTES);
            uint8_T dstOffsetBytes = (XCP_SHORT_UPLOAD_RES_PACKET_SIZE_IN_BYTES % XCP_HARDWARE_ADDRESS_GRANULARITY_BYTES_NUMBER);

            XCP_PRINTF("SHORT_UPLOAD: ");

            /* Copy the memory content to the packet data area */
            xcpMemcpyFromMTA(pktData, dstOffsetBytes);

            /* Post-increment MTA by the frame size in address granularity units */
            xcpStandardIncrementMta(xcpUploadSize);

            xcpUploadSize = 0;
        }
        else {
            /* This is a software error and it should never happen */
            XCP_PRINTF("SHORT_UPLOAD: invalid data detected\n");
        }
    }
    else {
        genericOutputPacketHandler(inputCode, packet, packetSize);
    }
}


/** This table contains the list of supported Rx packets and the corresponding handlers */
static const XcpPacketHandlers standardSupportedRxPacket[] =
{
    { XCP_PID_CONNECT,      connectInputPacketHandler,     connectOutputPacketHandler },
    { XCP_PID_DISCONNECT,   disconnectInputPacketHandler,  genericOutputPacketHandler },
    { XCP_PID_GET_STATUS,   getStatusInputPacketHandler,   getStatusOutputPacketHandler },
    { XCP_PID_SYNCH,        synchInputPacketHandler,       genericOutputPacketHandler },
#if XCP_SET_MTA_ENABLE == 1  
    { XCP_PID_SET_MTA,      setMTAInputPacketHandler,      genericOutputPacketHandler },
    { XCP_PID_UPLOAD,       uploadInputPacketHandler,      uploadOutputPacketHandler},
#endif
    { XCP_PID_SHORT_UPLOAD, shortUploadInputPacketHandler, shortUploadOutputPacketHandler }
};



/*****************************************************************************
    XCP Packet Lookup Function for basic standard commands
******************************************************************************/

/* Default Standard Packet Lookup function, supporting only basic commands
   listed in the table above */
static const XcpPacketHandlers* getPacket(XcpRxPidCode pid)
{
    return xcpFindPacket(pid, standardSupportedRxPacket,
                         XCP_ELEMENTS_NUMBER(standardSupportedRxPacket));
}

static XcpPacketLookupFunction packetLookup = NULL;



/*****************************************************************************
    Public Functions (invoked within the Protocol Layer)
******************************************************************************/
void xcpStandardInit(void)
{
    /* Initialize the packet lookup function to support only basic
       standard commands */
    xcpStandardSetPacketLookup(getPacket);

    /* Initialize support for the extended list of Standard commands
       @note this may override the default Packet lookup function
             by adding support for more (optional) commands */
    xcpStandardExtendedInit();
}


XcpPacketLookupFunction xcpStandardGetPacketLookup(void)
{
    return packetLookup;
}


void xcpStandardSetPacketLookup(XcpPacketLookupFunction getPacketFcn)
{
    packetLookup = getPacketFcn;
}


void xcpStandardReset(void)
{
    /* reset support for the extended list of Standard commands */
    xcpStandardExtendedReset();

    /* Restore the original value for the lookup function */
    xcpStandardSetPacketLookup(NULL);
}


uint8_T* xcpStandardGetAddressFromMta(void) {
    return XCP_ADDRESS_GET_WRITE(xcpMta.addressExtension, xcpMta.address);
}


boolean_T xcpStandardSetMta(uint32_T address, uint8_T addressExtension) {
    if ((addressExtension == 0) && (address == 0)) {
        return false;
    } else {
        xcpMta.address = address;
        xcpMta.addressExtension = addressExtension;
        return true;
    }
}


void xcpStandardGetMta(uint32_T *address, uint8_T *addressExtension) {
    *address = xcpMta.address;
    *addressExtension = xcpMta.addressExtension;
}


void xcpStandardIncrementMta(uint8_T incr) {
    xcpMta.address += incr;
}


#ifndef XCP_STANDARD_EXTENDED_SUPPORT

void xcpStandardExtendedInit(void) {}
void xcpStandardExtendedReset(void) {}

#endif
