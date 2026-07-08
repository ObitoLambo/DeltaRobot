/*
* Copyright 2016-2023 The MathWorks, Inc.
*
* File: xcp_frame.h
*
* Abstract:
*  Frame Handler interface
*  The interface defines the methods that need to be implemented in order to
*  be able to add and extract XCP Header and XCP Tail according to the supported
*  XCP Transport Layer specification.
*  The component is also responsible for initializing the underlying
*  XCP driver used to transfer the packets over the network, and to
*  carry out the extra checks related to the frame format when the messages
*  are actually sent and received by the driver.
*/

#ifndef XCP_FRAME_H
#define XCP_FRAME_H

#include "xcp.h"
#include "xcp_mem.h"


/** Initialize XCP Frame Handler */
XcpErrorCode xcpFrameInit(
    int   argc,   /**< [in] number of init parameters              */
    void *argv[]  /**< [in] array of parameters values (C strings) */
    );


/** Send the XCP frame data to the communication channel
    @note the data need to have the 'granularity' of one full XCP message frame
          (XCP Header + XCP Packet + XCP Tail)
          The actual msg content is available at offset msgOffset from the msgBuffer
          memory area.
          If the transmission is successful (return code XCP_SUCCESS),
          the frame handler is also responsible to release the buffer memory
          area, that will be no longer available */
XcpErrorCode xcpFrameMsgSend(
    void    *msgBuffer,  /**< [in] pointer to the base address of the buffer where the message is stored */
    size_t   msgOffset,  /**< [in] offset (from base address) where the XCP message frame is stored */
    size_t   msgSize     /**< [in] size (in bytes) of the XCP message frame */
    );


/** Receive the XCP frame data from the communication channel
    @note the data has the 'granularity' of one full XCP message frame
          (XCP Header + XCP Packet + XCP Tail)
          The frame handler is also responsible for allocating the
          memory area and it is no longer responsible for the ownership
          of the memory if the function has been successfully executed
          (error code XCP_SUCCESS) */
XcpErrorCode xcpFrameMsgRecv(
    void  **msgBuffer,  /**< [out] pointer to the base address of the buffer where the message is stored */
    size_t  msgOffset,  /**< [in] offset (from base address) where the XCP message frame is stored */
    size_t *msgSize     /**< [out] size (in bytes) of the XCP message frame that has been received */
    );


/** Craft a complete message frame, by adding XCP Header and XCP tail
    (Transport Layer dependent)
    @note the memory area where the XCP packet content is stored is not modified */
XcpErrorCode xcpFrameCreateMsg(
    void        *msgFrame,           /**< [out] pointer to the buffer containing the full message frame */
    size_t       msgFrameBufferSize, /**< [in]  max size (in bytes) of the buffer where the message frame is copied */
    size_t      *msgFrameSize,       /**< [out] size (in bytes) of the generated msg frame */
    size_t       xcpPacketSize       /**< [in]  size (in bytes) of the XCP packet */
    );


/** Extract the XCP Packet information from a complete message frame, and carry out all the relevant checks in the frame format */
XcpErrorCode xcpFrameExtractPacket(
    const void  *msgFrame,      /**< [in]  pointer to the buffer containing full message frame content */
    size_t       msgFrameSize,  /**< [in]  size (in bytes) of the full message frame */
    size_t      *xcpPacketSize  /**< [out] size (in bytes) of the XCP packet that has been processed */
    );


/** Return the XCP Header size (in bytes) */
size_t xcpFrameHeaderSize(void);


/** Return the XCP Tail size (in bytes) */
size_t xcpFrameTailSize(void);


/** Return the Max DTO (Data Transfer Object) Size (in bytes) */
size_t xcpFrameMaxDtoSize(void);


/** Return the Max CTO (Command Transfer Object) Size (in bytes) */
size_t xcpFrameMaxCtoSize(void);


/** Restart XCP Frame Handler (resetting all the packet counters checks) */
XcpErrorCode xcpFrameRestart(void);


/** Set the reserved memory pool to be used when a XCP CTO packet is received from the client */
void xcpFrameSetCtoReservedMemPoolId(xcpPoolId_T poolId);


/** Reset XCP Frame Handler to the default initial state */
XcpErrorCode xcpFrameReset(void);


#endif /* XCP_FRAME_H */
