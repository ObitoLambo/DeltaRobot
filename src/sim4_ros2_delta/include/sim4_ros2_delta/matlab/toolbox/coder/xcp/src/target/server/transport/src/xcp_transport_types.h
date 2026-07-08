/*
* Copyright 2018 The MathWorks, Inc.
*
* File: xcp_transport_types.h
*
* Abstract:
*  XCP Transport layer data types and definitions
*/

#ifndef XCP_TRANSPORT_TYPES_H
#define XCP_TRANSPORT_TYPES_H

#include "xcp_cfg.h"

/* Number of blocks associated to the reserved memory pool
   for the allocation of CTO XCP Packets
   @note the default number allows the storage of 2 CTO packets
   (1 RX and 1 TX) */
#ifndef XCP_MEM_CTO_RESERVED_POOL_BLOCKS_NUMBER
#define XCP_MEM_CTO_RESERVED_POOL_BLOCKS_NUMBER   2
#endif


#endif /* XCP_TRANSPORT_TYPES_H */

