/*
* Copyright 2016-2023 The MathWorks, Inc.
*
* File: xcp_cfg.h
*
* Abstract:
*  XCP Protocol layer configuration file
*/

#ifndef XCP_CFG_H
#define XCP_CFG_H

#include <limits.h>

/** Major revision of the supported Protocol Layer specification */
#define XCP_PROTOCOL_LAYER_VERSION  0x0103

/** Major revision of the supported Transport Layer specification */
#define XCP_TRANSPORT_LAYER_VERSION 0x0103

/** Support for Calibration and Paging */
#ifdef XCP_CALIBRATION_SUPPORT
#define XCP_CALIBRATION_ENABLE 1
#else
#define XCP_CALIBRATION_ENABLE 0
#endif

/** Support for DAQ lists */
#ifdef XCP_DAQ_SUPPORT
#define XCP_DAQ_ENABLE 1
#else
#define XCP_DAQ_ENABLE 0
#endif

/** Support for STIM */
#ifdef XCP_STIM_SUPPORT
#error "XCP STIM features not supported yet"
#define XCP_STIM_ENABLE 1
#else
#define XCP_STIM_ENABLE 0
#endif

/** Support for Flash Programming
   @note this is not planned at the moment */
#ifdef XCP_PGM_SUPPORT
#error "XCP Flash Programming features not supported yet"
#define XCP_PGM_ENABLE 1
#else
#define XCP_PGM_ENABLE 0
#endif

/** Support for Resource Protection based on Seed/Key mechanism */
#ifdef XCP_RESOURCE_PROTECTION_SUPPORT
#error "XCP Resource Protection features not supported yet"
#define XCP_RESOURCE_PROTECTION_ENABLE 1
#else
#define XCP_RESOURCE_PROTECTION_ENABLE 0
#endif

/** Support for SET_MTA, UPLOAD (and DOWNLOAD, if calibration enabled) commands */
#ifdef XCP_SET_MTA_SUPPORT
#define XCP_SET_MTA_ENABLE 1
#else
#define XCP_SET_MTA_ENABLE 0
#endif

/** Byte Order */
#ifdef XCP_BIG_ENDIAN
#define XCP_BYTE_ORDER   1
#else
#define XCP_BYTE_ORDER   0
#endif

/** Support for Block Mode
   @note this is not planned at the moment */
#ifdef XCP_BLOCK_MODE_SUPPORT
#error "XCP Block Mode not supported yet"
#define XCP_BLOCK_MODE_ENABLE   1
#else
#define XCP_BLOCK_MODE_ENABLE   0
#endif

/** Support for Optional Communication Mode Info */
#ifdef XCP_COMM_MODE_INFO_SUPPORT
#error "XCP Communication Mode Info not supported yet"
#define XCP_COMM_MODE_INFO_ENABLE   1
#else
#define XCP_COMM_MODE_INFO_ENABLE   0
#endif

/** Granularity of the XCP Address */
#define XCP_ADDRESS_GRANULARITY_BYTE      0x00
#define XCP_ADDRESS_GRANULARITY_WORD      0x01
#define XCP_ADDRESS_GRANULARITY_DWORD     0x02
#define XCP_ADDRESS_GRANULARITY_RESERVED  0x03

/* Try to verify server address granularity */
#ifndef XCP_ADDRESS_GRANULARITY
    #if !defined(CHAR_BIT)
        #error "XCP_ADDRESS_GRANULARITY definition is missing and automatic granularity deduction failed."
    #elif CHAR_BIT == 8
            #define XCP_ADDRESS_GRANULARITY XCP_ADDRESS_GRANULARITY_BYTE
    #elif CHAR_BIT == 16
            #define XCP_ADDRESS_GRANULARITY XCP_ADDRESS_GRANULARITY_WORD
    #elif CHAR_BIT == 32
        #error "XCP Server is not supported on 32-bit address granularity targets."
    #else
        #error "XCP Server is supported only on 8/16-bit address granularity targets."
    #endif
#endif

#if !( (XCP_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE) || \
       (XCP_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD) )
    #error "XCP Server is supported only on 8/16-bit address granularity targets."\
        " Check definition of XCP_ADDRESS_GRANULARITY."
#endif

#if XCP_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
#define XCP_ADDRESS_GRANULARITY_BYTES_NUMBER 1
#elif XCP_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
#define XCP_ADDRESS_GRANULARITY_BYTES_NUMBER 2
#elif XCP_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_DWORD
#define XCP_ADDRESS_GRANULARITY_BYTES_NUMBER 4
#else
#error "XCP_ADDRESS_GRANULARITY value is not valid"
#endif

/** Granularity of the ODT entry (in bytes)
   @note this is assumed to be the same for DAQ and STIM */
#ifndef XCP_ODT_ENTRY_SIZE_GRANULARITY
#define XCP_ODT_ENTRY_SIZE_GRANULARITY XCP_ADDRESS_GRANULARITY_BYTES_NUMBER
#endif

/** Granularity of the ODT entry (in bytes) must be >=
     Granularity of the XCP Address */
#if (XCP_ODT_ENTRY_SIZE_GRANULARITY) < (XCP_ADDRESS_GRANULARITY_BYTES_NUMBER)
#error "XCP_ODT_ENTRY_SIZE_GRANULARITY must not be less than XCP_ADDRESS_GRANULARITY"
#endif

/** Granularity of the ODT entry (in bytes) must be a multiple of
     Granularity of the XCP Address */
#if (XCP_ODT_ENTRY_SIZE_GRANULARITY)%(XCP_ADDRESS_GRANULARITY_BYTES_NUMBER) != 0
#error "XCP_ODT_ENTRY_SIZE_GRANULARITY must be a multiple of XCP_ADDRESS_GRANULARITY"
#endif

/** Max size of the ODT entry
   @note this is assumed to be the same for DAQ and STIM */
#ifndef XCP_MAX_ODT_ENTRY_SIZE
#define XCP_MAX_ODT_ENTRY_SIZE 0xFF
#endif
#if XCP_MAX_ODT_ENTRY_SIZE > 0xFF
#error "invalid XCP_MAX_ODT_ENTRY_SIZE"
#endif

/** Max number of Event Channels */
#ifndef XCP_MAX_EVENT_CHANNEL
#define XCP_MAX_EVENT_CHANNEL 128
#endif
#if XCP_MAX_EVENT_CHANNEL > 0xFFFF
#error "invalid XCP_MAX_EVENT_CHANNEL"
#endif

/** Max number of ODTs in a given DAQ list
   @note it cannot be greater than 0xFC */
#ifndef XCP_MAX_DAQ_ODT_NUMBER
#define XCP_MAX_DAQ_ODT_NUMBER 0xFC
#endif
#if XCP_MAX_DAQ_ODT_NUMBER > 0xFC
#error "invalid XCP_MAX_DAQ_ODT_NUMBER"
#endif

/** Max number of ODTs in a given STIM list
   @note it cannot be greater than 0xC0 */
#ifndef XCP_MAX_STIM_ODT_NUMBER
#define XCP_MAX_STIM_ODT_NUMBER 128
#endif
#if XCP_MAX_STIM_ODT_NUMBER > 0xC0
#error "invalid XCP_MAX_STIM_ODT_NUMBER"
#endif

/** Max number of DAQ lists */
#ifndef XCP_MAX_DAQ
#define XCP_MAX_DAQ 0xFFFF
#endif
#if XCP_MAX_DAQ > 0xFFFF
#error "invalid XCP_MAX_DAQ"
#endif

/** Number of pre-defined DAQ lists */
#ifndef XCP_MIN_DAQ
#define XCP_MIN_DAQ 0
#endif
#if (XCP_MIN_DAQ != 0)
#error "XCP_MIN_DAQ not supported"
#endif

/** XCP Identification Field Type
   @note only XCP_ID_ABSOLUTE_ODT_NUMBER is currently supported */
#define XCP_ID_ABSOLUTE_ODT_NUMBER               0x00
#define XCP_ID_REL_ODT_ABS_DAQ_NUMBER_BYTE       0x01
#define XCP_ID_REL_ODT_ABS_DAQ_NUMBER_WORD       0x02
#define XCP_ID_REL_ODT_ABS_DAQ_NUMBER_WORD_ALIGN 0x03

#ifndef XCP_ID_FIELD_TYPE
#define XCP_ID_FIELD_TYPE  XCP_ID_ABSOLUTE_ODT_NUMBER
#endif

#if  (XCP_ID_FIELD_TYPE == XCP_ID_ABSOLUTE_ODT_NUMBER)
#define XCP_ID_FIELD_SIZE 1
#endif

#if (XCP_ID_FIELD_TYPE == XCP_ID_REL_ODT_ABS_DAQ_NUMBER_BYTE)
#error "XCP_ID_REL_ODT_ABS_DAQ_NUMBER_BYTE identification field not supported yet"
#define XCP_ID_FIELD_SIZE 2
#endif

#if (XCP_ID_FIELD_TYPE == XCP_ID_REL_ODT_ABS_DAQ_NUMBER_WORD)
#error "XCP_ID_REL_ODT_ABS_DAQ_NUMBER_WORD identification field not supported yet"
#define XCP_ID_FIELD_SIZE 3
#endif

#if (XCP_ID_FIELD_TYPE == XCP_ID_REL_ODT_ABS_DAQ_NUMBER_WORD_ALIGN)
#error "XCP_ID_REL_ODT_ABS_DAQ_NUMBER_WORD_ALIGN identification field not supported yet"
#define XCP_ID_FIELD_SIZE 4
#endif

/** XCP Timestamp unit */
#define XCP_TIMESTAMP_UNIT_1NS     0x00
#define XCP_TIMESTAMP_UNIT_10NS    0x01
#define XCP_TIMESTAMP_UNIT_100NS   0x02
#define XCP_TIMESTAMP_UNIT_1US     0x03
#define XCP_TIMESTAMP_UNIT_10US    0x04
#define XCP_TIMESTAMP_UNIT_100US   0x05
#define XCP_TIMESTAMP_UNIT_1MS     0x06
#define XCP_TIMESTAMP_UNIT_10MS    0x07
#define XCP_TIMESTAMP_UNIT_100MS   0x08
#define XCP_TIMESTAMP_UNIT_1S      0x09
#define XCP_TIMESTAMP_UNIT_1PS     0x0A
#define XCP_TIMESTAMP_UNIT_10PS    0x0B
#define XCP_TIMESTAMP_UNIT_100PS   0x0C

#ifndef XCP_TIMESTAMP_UNIT
#define XCP_TIMESTAMP_UNIT XCP_TIMESTAMP_UNIT_1US
#endif

#if (XCP_TIMESTAMP_UNIT < 0) || (XCP_TIMESTAMP_UNIT > 0x0C)
#error "invalid XCP_TIMESTAMP_UNIT"
#endif

/** XCP Timestamp fixed. If set to true, the server will always 
   send DTO packets in time-stamped mode*/
#ifndef XCP_TIMESTAMP_FIXED
#define XCP_TIMESTAMP_FIXED 0
#endif

/** XCP Timestamp size (in bytes)
   @note valid values are 1, 2 or 4, but the current implementation
         just supports 4 */
#ifndef XCP_TIMESTAMP_SIZE
#define XCP_TIMESTAMP_SIZE 4
#endif

#if (XCP_TIMESTAMP_SIZE != 4)
#error "XCP_TIMESTAMP_SIZE value not currently supported"
#endif

#if (XCP_TIMESTAMP_SIZE > 0)
#define XCP_TIMESTAMP_ENABLE 1
#else
#define XCP_TIMESTAMP_ENABLE 0
#endif

/** XCP Timestamp ticks (number of ticks per unit)
   @note the only supported value is 1 at the moment */
#ifndef XCP_TIMESTAMP_TICKS
#define XCP_TIMESTAMP_TICKS 1
#endif

#if (XCP_TIMESTAMP_TICKS != 1)
#error "XCP_TIMESTAMP_TICKS value not currently supported"
#endif

/* Some (case sensitive !) naming conventions (per the XCP standard v1.5) :
 * BYTE  :  8-bits
 * WORD  : 16-bits
 * DWORD : 32-bits
 * DLONG : 64-bits
 *
 * 'word' refers one of the above depending on the width of each memory location
 * for the given processor.
 */

 /* returns number of BYTEs (via XCP_ADDRESS_GRANULARITY) contained in x words */
#define XCP_IN_BYTES(x)    (((size_t)x)*(XCP_ADDRESS_GRANULARITY_BYTES_NUMBER))

/* returns number of words (via XCP_ADDRESS_GRANULARITY) needed to hold x bytes */
#define XCP_IN_AG(x)     ((((size_t)x)+((XCP_ADDRESS_GRANULARITY_BYTES_NUMBER)-1))/(XCP_ADDRESS_GRANULARITY_BYTES_NUMBER))


#ifndef XCP_HARDWARE_ADDRESS_GRANULARITY
    /* If XCP_HARDWARE_ADDRESS_GRANULARITY is not explicitly defined, 
       we assume that it matches with XCP_ADDRESS_GRANULARITY   */
    #define XCP_HARDWARE_ADDRESS_GRANULARITY XCP_ADDRESS_GRANULARITY
    #define XCP_HARDWARE_ADDRESS_GRANULARITY_BYTES_NUMBER XCP_ADDRESS_GRANULARITY_BYTES_NUMBER
#else
    /* If XCP_ADDRESS_GRANULARITY is BYTE and XCP_HARDWARE_ADDRESS_GRANULARITY is WORD, 
       then it means that we are emulating a 16-bit addressable target as an 8-bit 
       addressable target. Currently, only the combination of XCP_ADDRESS_GRANULARITY_BYTE 
       and XCP_HARDWARE_ADDRESS_GRANULARITY_WORD is supported. But, this can be extended for 
       other combinations.   */
    #if ((XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD) && (XCP_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE))
        #define XCP_EMULATE_BYTE_ADDRESSABLE_TARGET
        #define XCP_HARDWARE_ADDRESS_GRANULARITY_BYTES_NUMBER 2
        #define XCP_BYTE_OFFSET_GET(address)  (uint8_T)(address % 2)
    #else
        #error "XCP Server supports 8-bit emulation only on 16-bit address granularity targets."
    #endif
#endif

 /* returns number of BYTEs (via XCP_ADDRESS_GRANULARITY) contained in x processor words */
#define XCP_IN_HW_BYTES(x)    (((size_t)x)*(XCP_HARDWARE_ADDRESS_GRANULARITY_BYTES_NUMBER))

/* returns number of processor words (via XCP_HARDWARE_ADDRESS_GRANULARITY) needed to hold x bytes */
#define XCP_IN_HW_AG(x)  ((((size_t)x)+((XCP_HARDWARE_ADDRESS_GRANULARITY_BYTES_NUMBER)-1))/(XCP_HARDWARE_ADDRESS_GRANULARITY_BYTES_NUMBER))

#endif /* XCP_CFG_H */

