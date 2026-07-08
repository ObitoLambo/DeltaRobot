/*
* Copyright 2016-2021 The MathWorks, Inc.
*
* File: xcp_daq_types.h
*
* Abstract:
*  XCP Protocol layer data types and definitions specific to DAQ lists support
*/

#ifndef XCP_DAQ_TYPES_H
#define XCP_DAQ_TYPES_H

#include "xcp_common.h"
#include "xcp_cfg.h"
#include "xcp_mem.h"

/* Number of DAQ lists that require a reserved memory pool.
   If the DAQ list has a reserved memory pool, the pool memory
   is used for the allocation of DTO XCP Packets in the DAQ list */
#ifndef XCP_MEM_DAQ_RESERVED_POOLS_NUMBER
#define XCP_MEM_DAQ_RESERVED_POOLS_NUMBER   0
#endif


/* Number of blocks associated to the reserved memory pool
   used for the allocation of DTO XCP Packets of a DAQ list 
   @note this number represents the number of "copies"
         of DAQ data that are allowed in the reserved pool
         so that xcpEvent() can be invoked multiple times
         without losing any samples. The real number of blocks
         in the XCP allocator pool is in reality:
         XCP_MEM_DAQ_RESERVED_POOL_BLOCKS_NUMBER * daq->odtCount
         so that it is guaranteed that we have enough memory also 
         in the case where a DAQ list is split into multiple ODTs */
#ifndef XCP_MEM_DAQ_RESERVED_POOL_BLOCKS_NUMBER
#define XCP_MEM_DAQ_RESERVED_POOL_BLOCKS_NUMBER   3
#endif


/*****************************************************************************
    XCP DAQ Mode bit masks
******************************************************************************/
/** Alternating Display Mode bit: 0 -> disabled, 1 -> enabled */
#define XCP_DAQ_MODE_ALTERNATING_MASK   0x01

/** Direction bit: 0 -> DAQ, 1 -> STIM  */
#define XCP_DAQ_MODE_DIRECTION_MASK     0x02

/** Timestamp bit: 0 -> disabled, 1 -> enabled  */
#define XCP_DAQ_MODE_TIMESTAMP_MASK     0x10

/** Disable transmission of Identification Field bit: 0 -> with PID, 1 -> without PID  */
#define XCP_DAQ_MODE_PID_OFF_MASK       0x20

/*****************************************************************************
    XCP DAQ Properties bit masks
******************************************************************************/
/** DAQ list configuration type: 0 -> static DAQ list, 1 -> dynamic DAQ list */
#define XCP_DYNAMIC_DAQ_SUPPORT_MASK        0x01

/** DAQ list prescaler support: 0 -> not supported, 1 -> supported */
#define XCP_DAQ_PRESCALER_SUPPORT_MASK      0x02

/** DAQ Resume mode support: 0 -> not supported, 1 -> supported */
#define XCP_DAQ_RESUME_SUPPORT_MASK         0x04

/** DAQ bitwise data stimulation support: 0 -> not supported, 1 -> supported */
#define XCP_DAQ_BIT_STIM_SUPPORT_MASK       0x08

/** DAQ time-stamped mode support: 0 -> not supported, 1 -> supported */
#define XCP_DAQ_TIMESTAMP_SUPPORT_MASK      0x10

/** Support for switching off the identification field: 0 -> PID cannot be switched off, 1 -> PID can be switched off */
#define XCP_DAQ_PID_OFF_SUPPORT_MASK        0x20

/** Support for DAQ overload indication in MSB of PID: 0 -> not supported, 1 -> supported
    @note it cannot be active at the same time as XCP_DAQ_OVERLOAD_EVENT_SUPPORT_MASK */
#define XCP_DAQ_OVERLOAD_MSB_SUPPORT_MASK   0x40

/** Support for DAQ overload indication by Event Packet: 0 -> not supported, 1 -> supported
    @note it cannot be active at the same time as XCP_DAQ_OVERLOAD_MSB_SUPPORT_MASK */
#define XCP_DAQ_OVERLOAD_EVENT_SUPPORT_MASK 0x80

/*****************************************************************************
    XCP DAQ Key bit masks and values
******************************************************************************/
/** DAQ list Optimization */
#define XCP_OPTIMIZATION_OM_DEFAULT            0x00
#define XCP_OPTIMIZATION_OM_ODT_TYPE_16        0x01
#define XCP_OPTIMIZATION_OM_ODT_TYPE_32        0x02
#define XCP_OPTIMIZATION_OM_ODT_TYPE_64        0x03
#define XCP_OPTIMIZATION_OM_ODT_TYPE_ALIGNMENT 0x04
#define XCP_OPTIMIZATION_OM_MAX_ENTRY_SIZE     0x05

#define XCP_OPTIMIZATION_TYPE_MASK             0x0F
#define XCP_OPTIMIZATION_TYPE_OFFSET           0x00

/** DAQ list Address Extension */
#define XCP_ADDRESS_EXT_DIFFERENT              0x00 /* address extension can be different within one and the same ODT */
#define XCP_ADDRESS_EXT_SAME_WITHIN_ODT        0x01 /* address extension to be the same for all entries within one ODT */
#define XCP_ADDRESS_EXT_RESERVED               0x02 /* not allowed */
#define XCP_ADDRESS_EXT_SAME_WITHIN_DAQ        0x03 /* address extension to be the same for all entries within one DAQ */

#define XCP_ADDRESS_EXT_MASK                   0x30
#define XCP_ADDRESS_EXT_OFFSET                 0x04

/** DAQ list Identification field
    @note the value is defined by XCP_ID_FIELD_TYPE */
#define XCP_ID_FIELD_TYPE_MASK                 0xC0
#define XCP_ID_FIELD_TYPE_OFFSET               0x06

/*****************************************************************************
    XCP Timestamp mode bit masks and values
******************************************************************************/
/** Timestamp size */
#define XCP_TIMESTAMP_SIZE_MASK             0x07
#define XCP_TIMESTAMP_SIZE_OFFSET           0x00

/** Timestamp Fixed bit */
#define XCP_TIMESTAMP_FIXED_MASK            0x08

/** Timestamp unit */
#define XCP_TIMESTAMP_UNIT_MASK             0xF0
#define XCP_TIMESTAMP_UNIT_OFFSET           0x04

/*****************************************************************************
    DAQ Data Types
******************************************************************************/

/** XCP ODT */
typedef struct XcpOdt {
    XcpOdtEntry *entry;            /**< Array of ODT entries associated with the ODT */
    uint8_T      entriesCount;     /**< Number of ODT entries */
    size_t       size;             /**< total size (in bytes) of all the ODT entries */
    uint8_T     *msgBuffer;        /**< pointer to the memory area of the XCP DTO message */
    size_t       packetSize;       /**< overall XCP packet size */
    uint8_T     *currentValuePtr;  /**< pointer to the next value to write within the DTO message */
    size_t       offsetBytes;      /**< offset in bytes to use when writing to currentValuePtr */
}   XcpOdt;

/** XCP DAQ list status */
typedef enum {
    XCP_DAQ_INIT = 0,
    XCP_DAQ_STOPPED = 1,
    XCP_DAQ_SELECTED = 2,
    XCP_DAQ_STARTED = 3
} XcpDaqStatus;

/** XCP DAQ */
typedef struct XcpDaq {
    XcpOdt      *odt;           /**< Array of ODTs associated with the DAQ list */
    uint8_T      odtCount;      /**< Number of ODTs */
    XcpDaqStatus status;        /**< DAQ list status (see XCP DAQ Mode bit masks) */
    uint16_T     eventId;       /**< Event channel number */
    uint8_T      mode;          /**< DAQ list mode */
    uint8_T      prescaler;     /**< Transmission rate prescaler */
    uint8_T      priority;      /**< DAQ list priority (0xFF -> highest) */
    uint8_T      firstPid;      /**< If DTOs have 'absolute ODT number' identification field type,
                                     this is the absolute ODT number in the DTO packet of the
                                     first ODT transferred by this DAQ list */
    uint8_T      packedMode;    /**< DAQ Packed Mode: 0 = not packed, 1 = element-grouped data packing, 
                                        2 = event-grouped data packing.  All other values are reserved for 
                                        future extensions */
    uint8_T      timestampMode; /**< DAQ Packed Mode Timestamp Mode: 0x00 single timestamp of last sample,
                                     0x01 single timestamp of first sample. */
    uint16_T     sampleCount;   /**< DAQ Packed Mode Sample count: number of samples packed in the DAQ list */
    uint16_T     currentSample; /**< Current sample ID (from 0 to sampleCount - 1) */
    xcpPoolId_T  poolId;        /**< ID of the reserved memory pool to be used to store the packets
                                     associated to this DAQ list. If poolId = XCP_INVALID_POOL_ID
                                     the "Main" memory needs to be used */
}   XcpDaq;

/** XCP DAQ Lists */
typedef struct XcpDaqLists {
    XcpDaq    *daq;                  /**< Array of DAQ lists  */
    uint16_T   daqCount;             /**< Number of DAQ lists */
    uint8_T    firstAvailableDaqPid; /**< If DTOs have XCP_ID_ABSOLUTE_ODT_NUMBER,
                                          this is the first PID available for the firstPid
                                          of the next DAQ list. The value gets updated after
                                          the first START_STOP_DAQ_LIST command of a given DAQ list */
    uint8_T    firstAvailableStimPid; /**< If DTOs have XCP_ID_ABSOLUTE_ODT_NUMBER,
                                           this is the first PID available for the firstPid
                                           of the next STIM list. The value gets updated after
                                           the first START_STOP_DAQ_LIST command of a given STIM list */
}   XcpDaqLists;

/** Pointer to XCP ODT Entry */
typedef struct XcpDaqPtr {
    uint16_T daqListId;  /**< DAQ list Id */
    uint8_T  odtId;      /**< ODT Id within the DAQ list */
    uint8_T  odtEntryId; /**< ODT Entry Id within ODT */
}   XcpDaqPtr;

/*****************************************************************************
    XCP SET DAQ PTR
******************************************************************************/
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpSetDaqPtrCmdPacketFrame {
        uint8_T  PID;        /**< Packet identifier, always XCP_PID_SET_DAQ_PTR */
        uint8_T  reserved;   /**< reserved */
        uint16_T daqListId;  /**< DAQ list Id */
        uint8_T  odtId;      /**< ODT Id within the DAQ list */
        uint8_T  odtEntryId; /**< ODT Entry Id within the ODT */
    }   XcpSetDaqPtrCmdPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpSetDaqPtrCmdPacketFrame {
        uint16_T PID        :8;
        uint16_T reserved   :8;
        uint16_T daqListId  :16;
        uint16_T odtId      :8;
        uint16_T odtEntryId :8;
    }   XcpSetDaqPtrCmdPacketFrame;
#endif

/*****************************************************************************
    XCP WRITE DAQ
******************************************************************************/
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpWriteDaqCmdPacketFrame {
        uint8_T  PID;              /**< Packet identifier, always XCP_PID_WRITE_DAQ */
        uint8_T  bitOffset;        /**< position of bit in 32 bit variable referenced
                                        by the address and extension below [0..31]
                                        0xFF if the ODT entry is not a bit */
        uint8_T  size;             /**< size of the DAQ element (in XCP_ODT_ENTRY_SIZE_GRANULARITY units) */
        uint8_T  addressExtension; /**< address extension of the DAQ element */
        uint32_T address;          /**< address of the DAQ element */
    }   XcpWriteDaqCmdPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpWriteDaqCmdPacketFrame {
        uint16_T PID               :8;
        uint16_T bitOffset         :8;
        uint16_T size              :8;
        uint16_T addressExtension  :8;
        uint32_T address;
    }   XcpWriteDaqCmdPacketFrame;
#endif

/*****************************************************************************
    XCP SET DAQ LIST MODE
******************************************************************************/
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpSetDaqListModeCmdPacketFrame {
        uint8_T    PID;       /**< Packet identifier, always XCP_PID_SET_DAQ_LIST_MODE */
        uint8_T    mode;      /**< DAQ list mode (XCP DAQ Mode bit masks) */
        uint16_T   daqListId; /**< DAQ list Id */
        uint16_T   eventId;   /**< Event Channel Id */
        uint8_T    prescaler; /**< Transmission rate prescaler */
        uint8_T    priority;  /**< DAQ list priority (0xFF -> highest) */
    }   XcpSetDaqListModeCmdPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpSetDaqListModeCmdPacketFrame {
        uint16_T PID          :8;
        uint16_T mode         :8;
        uint16_T daqListId    :16;
        uint16_T eventId      :16;
        uint16_T prescaler    :8;
        uint16_T priority     :8;
    }   XcpSetDaqListModeCmdPacketFrame;
#endif

/*****************************************************************************
    XCP START STOP DAQ LIST
******************************************************************************/
#define XCP_DAQ_LIST_STOP    0x00
#define XCP_DAQ_LIST_START   0x01
#define XCP_DAQ_LIST_SELECT  0x02

#define XCP_START_STOP_DAQ_LIST_RES_PACKET_SIZE_IN_BYTES 2
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpStartStopDaqListCmdPacketFrame {
        uint8_T  PID;       /**< Packet identifier, always XCP_PID_START_STOP_DAQ_LIST */
        uint8_T  mode;      /**< Mode (00 = stop, 01 = start, 02 = select) */
        uint16_T daqListId; /**< DAQ list Id */
    }   XcpStartStopDaqListCmdPacketFrame;

    typedef struct XCP_ATTRIBUTE_PACKED XcpStartStopDaqListResPacketFrame {
        uint8_T  PID;       /**< Packet identifier, always XCP_PID_RES */
        uint8_T  firstPid;  /**< If DTOs have 'absolute ODT number' identification field type,
                                 this is the absolute ODT number in the DTO packet of the
                                 first ODT transferred by this DAQ list */
    }   XcpStartStopDaqListResPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpStartStopDaqListCmdPacketFrame {
        uint16_T  PID       :8;
        uint16_T  mode      :8;
        uint16_T daqListId  :16;
    }   XcpStartStopDaqListCmdPacketFrame;

    typedef struct XcpStartStopDaqListResPacketFrame {
        uint16_T PID       :8;
        uint16_T firstPid  :8;
    }   XcpStartStopDaqListResPacketFrame;

#endif

/*****************************************************************************
    XCP START STOP SYNCH
******************************************************************************/
#define XCP_DAQ_LIST_STOP_ALL       0x00
#define XCP_DAQ_LIST_START_SELECTED 0x01
#define XCP_DAQ_LIST_STOP_SELECTED  0x02

#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpStartStopSynchCmdPacketFrame {
        uint8_T  PID;       /**< Packet identifier, always XCP_PID_START_STOP_SYNCH */
        uint8_T  mode;      /**< Mode (00 = stop all, 01 = start selected, 02 = stop selected) */
    }   XcpStartStopSynchCmdPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpStartStopSynchCmdPacketFrame {
        uint16_T PID    :8;
        uint16_T mode   :8;
    }   XcpStartStopSynchCmdPacketFrame;
#endif

/*****************************************************************************
    XCP GET DAQ PROCESSOR INFO
******************************************************************************/
#define XCP_DAQ_PROPERTIES_VALUE   (XCP_DYNAMIC_DAQ_SUPPORT_MASK | XCP_DAQ_TIMESTAMP_SUPPORT_MASK)

#define XCP_DAQ_KEY_VALUE (((XCP_OPTIMIZATION_TYPE_MASK & \
                            (XCP_OPTIMIZATION_OM_DEFAULT << XCP_OPTIMIZATION_TYPE_OFFSET))) | \
                           ((XCP_ADDRESS_EXT_MASK & \
                            (XCP_ADDRESS_EXT_DIFFERENT << XCP_OPTIMIZATION_TYPE_OFFSET))) | \
                           ((XCP_ID_FIELD_TYPE_MASK) & \
                            (XCP_ID_FIELD_TYPE << XCP_ID_FIELD_TYPE_OFFSET)))

#define XCP_GET_DAQ_PROCESSOR_INFO_RES_PACKET_SIZE_IN_BYTES 8
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpGetDaqProcessorInfoResPacketFrame {
        uint8_T  PID;                 /**< Packet identifier, always XCP_PID_RES */
        uint8_T  daqProperties;       /**< General properties of the DAQ lists (see DAQ Properties bit masks) */
        uint16_T maxDaq;              /**< Total number of available DAQ lists */
        uint16_T maxEventChannel;     /**< Total number of available Event channels */
        uint8_T  minDaq;              /**< Total number of predefined DAQ lists */
        uint8_T  daqKeyByte;          /**< DAQ key byte (see DAQ Key bit masks and values) */
    }   XcpGetDaqProcessorInfoResPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpGetDaqProcessorInfoResPacketFrame {
        uint16_T PID                :8;
        uint16_T daqProperties      :8;
        uint16_T maxDaq             :16;
        uint16_T maxEventChannel    :16;
        uint16_T minDaq             :8;
        uint16_T daqKeyByte         :8;
    }   XcpGetDaqProcessorInfoResPacketFrame;
#endif

/*****************************************************************************
    XCP GET DAQ RESOLUTION INFO
******************************************************************************/

#define XCP_TIMESTAMP_SIZE_VALUE ((XCP_TIMESTAMP_SIZE_MASK) & \
                                  (XCP_TIMESTAMP_SIZE << XCP_TIMESTAMP_SIZE_OFFSET))

#if XCP_TIMESTAMP_FIXED == 0
#define XCP_TIMESTAMP_FIXED_VALUE 0
#else
#define XCP_TIMESTAMP_FIXED_VALUE XCP_TIMESTAMP_FIXED_MASK
#endif

#define XCP_TIMESTAMP_UNIT_VALUE ((XCP_TIMESTAMP_UNIT_MASK) & \
                                    (XCP_TIMESTAMP_UNIT << XCP_TIMESTAMP_UNIT_OFFSET))

#define XCP_TIMESTAMP_MODE_VALUE  XCP_TIMESTAMP_SIZE_VALUE | \
                                   XCP_TIMESTAMP_FIXED_VALUE | \
                                   XCP_TIMESTAMP_UNIT_VALUE

#define XCP_GET_DAQ_RESOLUTION_INFO_RES_PACKET_SIZE_IN_BYTES 8
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpGetDaqResolutionInfoResPacketFrame {
        uint8_T  PID;                         /**< Packet identifier, always XCP_PID_RES */
        uint8_T  daqOdtEntrySizeGranularity;  /**< Granularity for size of ODT entry (DAQ direction) */
        uint8_T  maxDaqOdtEntrySize;          /**< Maximum size of ODT entry (DAQ direction) */
        uint8_T  stimOdtEntrySizeGranularity; /**< Granularity for size of ODT entry (STIM direction) */
        uint8_T  maxStimOdtEntrySize;         /**< Maximum size of ODT entry (STIM direction) */
        uint8_T  timestampMode;               /**< Timestamp mode (see Timestamp Mode bit masks and values) */
        uint16_T timestampTicks;              /**< Timestamp ticks per unit */
    }   XcpGetDaqResolutionInfoResPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpGetDaqResolutionInfoResPacketFrame {
        uint16_T PID                            :8;
        uint16_T daqOdtEntrySizeGranularity     :8;
        uint16_T maxDaqOdtEntrySize             :8;
        uint16_T stimOdtEntrySizeGranularity    :8;
        uint16_T maxStimOdtEntrySize            :8;
        uint16_T timestampMode                  :8;
        uint16_T timestampTicks                 :16;
    }   XcpGetDaqResolutionInfoResPacketFrame;
#endif
                                  
/*****************************************************************************
    XCP ALLOC DAQ
******************************************************************************/
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpAllocDaqCmdPacketFrame {
        uint8_T  PID;      /**< Packet identifier, always XCP_PID_ALLOC_DAQ */
        uint8_T  reserved; /**< reserved */
        uint16_T daqCount; /**< number of DAQ lists to be allocated */
    }   XcpAllocDaqCmdPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpAllocDaqCmdPacketFrame {
        uint16_T PID       :8;
        uint16_T reserved  :8;
        uint16_T daqCount  :16;
    }   XcpAllocDaqCmdPacketFrame;
#endif

/*****************************************************************************
    XCP ALLOC ODT
******************************************************************************/
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpAllocOdtCmdPacketFrame {
        uint8_T  PID;           /**< Packet identifier, always XCP_PID_ALLOC_ODT */
        uint8_T  reserved;      /**< reserved */
        uint16_T daqListId;     /**< DAQ list Id */
        uint8_T  odtCount;      /**< number of ODT to be assigned to the DAQ list */
    }   XcpAllocOdtCmdPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpAllocOdtCmdPacketFrame {
        uint16_T PID        :8;
        uint16_T reserved   :8;
        uint16_T daqListId  :16;
        uint16_T odtCount   :8;
    }   XcpAllocOdtCmdPacketFrame;
#endif

/*****************************************************************************
    XCP ALLOC ODT ENTRY
******************************************************************************/
#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpAllocOdtEntryCmdPacketFrame {
        uint8_T  PID;             /**< Packet identifier, always XCP_PID_ALLOC_ODT_ENTRY */
        uint8_T  reserved;        /**< reserved */
        uint16_T daqListId;       /**< DAQ list Id */
        uint8_T  odtId;           /**< ODT Id within the DAQ list */
        uint8_T  odtEntriesCount; /**< number of ODT Entries to be assigned to the ODT */
    }   XcpAllocOdtEntryCmdPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpAllocOdtEntryCmdPacketFrame {
        uint16_T PID                :8;
        uint16_T reserved           :8;
        uint16_T daqListId          :16;
        uint16_T odtId              :8;
        uint16_T odtEntriesCount    :8;
    }   XcpAllocOdtEntryCmdPacketFrame;
#endif

/*****************************************************************************
    XCP SET DAQ PACKED MODE
******************************************************************************/
#define XCP_DAQ_LEVEL1_CODE_SET_DAQ_PACKED_MODE 0x01

/* DAQ Packed Mode */
#define XCP_DAQ_DATA_NOT_PACKED         0x00
#define XCP_DAQ_ELEMENT_GROUPED_PACKING 0x01
#define XCP_DAQ_EVENT_GROUPED_PACKING   0x02

/* DAQ Packed Timestamp Mode */
#define XCP_DAQ_SINGLE_TIMESTAMP_LAST_SAMPLE  0x00
#define XCP_DAQ_SINGLE_TIMESTAMP_FIRST_SAMPLE 0x01

/* DAQ Packed Sample count */
#define XCP_DAQ_PACKED_SAMPLE_COUNT_MIN       0x0002
#define XCP_DAQ_PACKED_SAMPLE_COUNT_MAX       0xFFFF


#if XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_BYTE
    XCP_PRAGMA_PACK_BEGIN(1)
    typedef struct XCP_ATTRIBUTE_PACKED XcpSetDaqPackedModeCmdPacketFrame {
        uint8_T  PID;              /**< Packet identifier, always XCP_SET_DAQ_PACKED_MODE */
        uint8_T  level1Code;       /**< Level 1 code, always 0x01 */
        uint16_T daqListId;        /**< DAQ list Id */
        uint8_T  daqPackedMode;    /**< DAQ Packed Mode: 0 = not packed, 1 = element-grouped data packing, 
                                        2 = event-grouped data packing. 
                                        All other values are reserved for future extensions */
        uint8_T  dpmTimestampMode; /**< Timestamp Mode: 0x00 single timestamp of last sample,
                                        0x01 single timestamp of first sample. */
        uint16_T dpmSampleCount;   /**< Sample count: number of samples packed in the DAQ list
                                        transmission (must be between 2 and 0xFFFF) */
    }   XcpSetDaqPackedModeCmdPacketFrame;
    XCP_PRAGMA_PACK_END()
#elif XCP_HARDWARE_ADDRESS_GRANULARITY == XCP_ADDRESS_GRANULARITY_WORD
    typedef struct XcpSetDaqPackedModeCmdPacketFrame {
        uint16_T  PID              :8;
        uint16_T  level1Code       :8;
        uint16_T  daqListId        :16;
        uint16_T  daqPackedMode    :8;
        uint16_T  dpmTimestampMode :8;
        uint16_T  dpmSampleCount   :16;
    }   XcpSetDaqPackedModeCmdPacketFrame;
#endif


/*****************************************************************************
    XCP CUSTOM MEMORY MANAGEMENT
******************************************************************************/
#define XCP_DAQ_CUSTOM_MEMORY_INVALID_EVENT_ID ((uint16_T) - 1)

/** Use the provided handlers to allocate/free memory for a specific event */
typedef struct XcpEventCustomMemoryManager {
    uint16_T eventId;
    XcpCustomAllocHandler allocHandler;
    XcpCustomFreeHandler freeHandler;
} XcpEventCustomMemoryManager;

#endif /* XCP_DAQ_TYPES_H */

