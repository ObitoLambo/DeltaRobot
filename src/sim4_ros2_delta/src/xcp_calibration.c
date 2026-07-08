/*
* Copyright 2017-2020 The MathWorks, Inc.
*
* File: xcp_calibration.c
*
* Abstract:
*  Implementation of XCP Protocol Layer Calibration support
*/

#include "xcp_common.h"
#include "xcp.h"
#include "xcp_cfg.h"
#include "xcp_calibration.h"

#ifdef XCP_CALIBRATION_SUPPORT

#ifdef XCP_CALIBRATION_EXTENDED_SUPPORT
#include "xcp_calibration_ext.h"
#endif

#include "xcp_types.h"
#include "xcp_calibration_types.h"
#include "xcp_standard.h"
#include "xcp_transport_internal.h"

/*****************************************************************************
    Internal Functions specific to Calibration support
******************************************************************************/
static boolean_T isValidShortDownloadSize(uint8_T size)
{
    size_t addressGranularity = XCP_ADDRESS_GRANULARITY_BYTES_NUMBER;
    size_t maxCtoSize = xcpTransportMaxCtoSize();

    return (size <= ((maxCtoSize - 8) / addressGranularity));
}

#if XCP_SET_MTA_ENABLE == 1
    static boolean_T isValidDownloadSize(uint8_T size)
    {
        size_t addressGranularity = XCP_ADDRESS_GRANULARITY_BYTES_NUMBER;
        size_t maxCtoSize = xcpTransportMaxCtoSize();
        
        return ( (size >= 1)  && (size <= (maxCtoSize-2) / addressGranularity ) );
    }
#endif  /* XCP_SET_MTA_ENABLE == 1 */

/** memcpy function used to copy data from packet to MTA address */
static void xcpMemcpyToMTA(uint8_T *packet, uint8_T size)
{
    /* Retrieve memory address */
    uint8_T *xcpMTARawPointer = xcpStandardGetAddressFromMta();

    XCP_PRINTF("writing at address %p\n", xcpMTARawPointer);

#ifdef XCP_EMULATE_BYTE_ADDRESSABLE_TARGET
{
    uint32_T address;
    uint8_T addressExtension;
    uint8_T dstOffsetBytes;
    xcpStandardGetMta(&address, &addressExtension);
    dstOffsetBytes = XCP_BYTE_OFFSET_GET(address);
    xcpMemcpyByte(xcpMTARawPointer, dstOffsetBytes, packet, 0, size);
}
#else
    XCP_MEMCPY(xcpMTARawPointer, packet, size);
#endif
}

/*****************************************************************************
    XCP DOWNLOAD
******************************************************************************/
#if XCP_SET_MTA_ENABLE == 1
    static XcpProtoErrorCode downloadInputPacketHandler(void   *msgBuffer,
        size_t  xcpPacketOffset,
        size_t *outputPacketSize)
    {
        XcpProtoErrorCode protoErrorCode = XCP_PROTO_SUCCESS;
        uint8_T *packet = (uint8_T *)msgBuffer + xcpPacketOffset;
        XcpDownloadCmdPacketFrame *frame = (XcpDownloadCmdPacketFrame *)packet;
        boolean_T ok = false;
        uint8_T *xcpMTARawPointer = xcpStandardGetAddressFromMta();

        /* Check if the number of data elements is valid */
        ok = isValidDownloadSize((uint8_T)frame->size);
        XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE,
            ("DOWNLOAD: invalid number of data elements (%d)\n", frame->size));
            
        /* Check if a valid MTA address is available */
        XCP_INPUT_PKT_ERROR_IF((xcpMTARawPointer == 0) || (xcpMTARawPointer == NULL), 
            XCP_PROTO_ACCESS_DENIED, ("DOWNLOAD: invalid address\n"));

        XCP_PRINTF("DOWNLOAD: ");
    
        /* Update the value of the parameter with the data element content */
        xcpMemcpyToMTA(packet + sizeof(XcpDownloadCmdPacketFrame), (uint8_T)frame->size);
    
        /* Post-increment MTA by the frame size in address granularity units */
        xcpStandardIncrementMta((uint8_T)frame->size);
        
        *outputPacketSize = XCP_GENERIC_RES_PACKET_SIZE_IN_BYTES;

        return protoErrorCode;
    }
#endif  /* XCP_SET_MTA_ENABLE == 1 */

/*****************************************************************************
    XCP SHORT_DOWNLOAD
******************************************************************************/
static XcpProtoErrorCode shortDownloadInputPacketHandler(void   *msgBuffer,
    size_t  xcpPacketOffset,
    size_t *outputPacketSize)
{
    XcpProtoErrorCode protoErrorCode = XCP_PROTO_SUCCESS;
    uint8_T *packet = (uint8_T *)msgBuffer + xcpPacketOffset;
    XcpShortDownloadCmdPacketFrame *frame = (XcpShortDownloadCmdPacketFrame *)packet;
    boolean_T ok = false;

    /* Check if the number of data elements is valid */
    ok = isValidShortDownloadSize((uint8_T)frame->size);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_OUT_OF_RANGE,
        ("SHORT_DOWNLOAD: invalid number of data elements (%d)\n", frame->size));

    /* Check if the memory address is valid and set the MTA pointer */
    ok = xcpStandardSetMta(frame->address, (uint8_T)frame->addressExtension);
    XCP_INPUT_PKT_ERROR_IF(!ok, XCP_PROTO_ACCESS_DENIED, ("SHORT_DOWNLOAD: invalid address\n"));

    XCP_PRINTF("SHORT_DOWNLOAD: ");

    /* Update the value of the parameter with the data element content */
    xcpMemcpyToMTA(packet + sizeof(XcpShortDownloadCmdPacketFrame), (uint8_T)frame->size);

    /* Post-increment MTA by the frame size in address granularity units */
    xcpStandardIncrementMta((uint8_T)frame->size);

    *outputPacketSize = XCP_GENERIC_RES_PACKET_SIZE_IN_BYTES;

    return protoErrorCode;
}


/** This table contains the list of supported Rx packets and the corresponding handlers */
static const XcpPacketHandlers calibrationSupportedRxPacket[] =
{
#if XCP_SET_MTA_ENABLE == 1
    { XCP_PID_DOWNLOAD,       downloadInputPacketHandler,     genericOutputPacketHandler },
#endif
    { XCP_PID_SHORT_DOWNLOAD, shortDownloadInputPacketHandler, genericOutputPacketHandler }
};


/*****************************************************************************
    XCP Packet Lookup Function for basic calibration commands
******************************************************************************/

/* Default Calibration Packet Lookup function, supporting only basic commands
   listed in the table above */
static const XcpPacketHandlers* getPacket(XcpRxPidCode pid)
{
    return xcpFindPacket(pid, calibrationSupportedRxPacket,
                         XCP_ELEMENTS_NUMBER(calibrationSupportedRxPacket));
}

static XcpPacketLookupFunction packetLookup = NULL;



/*****************************************************************************
    Public Functions (invoked within the Protocol Layer)
******************************************************************************/
void xcpCalibrationInit(void)
{
    /* Initialize the packet lookup function to support only basic
       calibration commands */
    xcpCalibrationSetPacketLookup(getPacket);

#ifdef XCP_CALIBRATION_EXTENDED_SUPPORT
    /* Initialize support for the extended list of calibration commands */
    xcpCalibrationExtendedInit();
#endif
}


XcpPacketLookupFunction xcpCalibrationGetPacketLookup(void)
{
    return packetLookup;
}


void xcpCalibrationSetPacketLookup(XcpPacketLookupFunction getPacketFcn)
{
    packetLookup = getPacketFcn;
}

void xcpCalibrationReset(void)
{
#ifdef XCP_CALIBRATION_EXTENDED_SUPPORT
    /* Reset support for the extended list of calibration commands */
    xcpCalibrationExtendedReset();
#endif
}

#endif
