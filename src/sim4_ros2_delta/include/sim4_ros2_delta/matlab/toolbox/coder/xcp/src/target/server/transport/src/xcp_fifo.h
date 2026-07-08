/*
* Copyright 2016-2021 The MathWorks, Inc.
*
* File: xcp_fifo.h
*
* Abstract:
*  Interface of FIFO data structure based on single linked list
*/

#ifndef XCP_FIFO_H
#define XCP_FIFO_H

#include <stddef.h>

#include "xcp_common.h"
#include "xcp_mem_config.h"

/** FIFO entry data structure */
XCP_PRAGMA_PACK_BEGIN(XCP_MEM_ALIGNMENT)
struct XCP_ATTRIBUTE_ALIGNED(XCP_MEM_ALIGNMENT) xcpFifoEntry {
    struct xcpFifoEntry *next;
    size_t msgFrameSize;    /* in AG units */
};
XCP_PRAGMA_PACK_END()


/** FIFO data structure */
struct xcpFifo {
    struct xcpFifoEntry  *first;
    struct xcpFifoEntry **last;
};


/** Initialize FIFO data structure */
void xcpFifoInit(struct xcpFifo * fifo);


/** Add an entry at the tail of the fifo */
void xcpFifoEnqueue(struct xcpFifo *fifo, struct xcpFifoEntry *element);


/** Get an entry from the head of the fifo
   (NULL if no elements are present) */
void xcpFifoDequeue(struct xcpFifo *fifo, struct xcpFifoEntry **element);


/** Restore the entry at the head of the fifo */
void xcpFifoEnqueueHead(struct xcpFifo *fifo, struct xcpFifoEntry *element);


/** Transfer elements removing them from srcFifo and inserting them at 
    the head of dstFifo */
void xcpFifoSpliceHead(struct xcpFifo *dstFifo, struct xcpFifo *srcFifo);


/** Return true if the fifo is currently empty */
boolean_T xcpFifoEmpty(struct xcpFifo *fifo);


/** Reset fifo data structure to the default initial state  */
void xcpFifoReset(struct xcpFifo * fifo);


#endif /* XCP_FIFO_H */
