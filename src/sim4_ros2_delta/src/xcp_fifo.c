/*
* Copyright 2016-2020 The MathWorks, Inc.
*
* File: xcp_fifo.c
*
* Abstract:
*  Implementation of FIFO data structure based on single linked list
*/

#include "xcp_common.h"
#include "xcp_fifo.h"


void xcpFifoInit(struct xcpFifo * fifo)
{
    fifo->first = NULL;
    fifo->last = &(fifo)->first;
}


void xcpFifoEnqueue(struct xcpFifo *fifo, struct xcpFifoEntry *element)
{
    element->next = NULL;
    *(fifo)->last = element;
    fifo->last = &(element->next);
}


void xcpFifoDequeue(struct xcpFifo *fifo, struct xcpFifoEntry **element)
{
    *element = fifo->first;

    if (fifo->first != NULL) {
        fifo->first = fifo->first->next;

        if (fifo->first == NULL)
            fifo->last = &(fifo)->first;
    }
}


void xcpFifoEnqueueHead(struct xcpFifo *fifo, struct xcpFifoEntry *element)
{
    element->next = (fifo)->first;

    if ((fifo)->first == NULL) {
        (fifo)->last = &(element->next);
    }

    (fifo)->first = element;
}


void xcpFifoSpliceHead(struct xcpFifo *dstFifo, struct xcpFifo *srcFifo)
{
    if (srcFifo->first != NULL) {
        *(srcFifo->last) = dstFifo->first;
        dstFifo->first = srcFifo->first;
         
        srcFifo->first = NULL;
        srcFifo->last = &(srcFifo)->first;
    }
}


boolean_T xcpFifoEmpty(struct xcpFifo *fifo)
{
    return ((fifo)->first == NULL);
}


void xcpFifoReset(struct xcpFifo * fifo)
{
    xcpFifoInit(fifo);
}
