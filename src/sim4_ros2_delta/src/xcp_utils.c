/* Copyright 2022 The MathWorks, Inc. */

#include "xcp_utils.h"

/**
 * Sort elements in `toSort` in ascending order, keeping the correspondence with elements of
 * `toKeepAligned`.
 **/
void xcpSortArray(size_t *toSort, size_t *toKeepAligned, size_t size)
{
    unsigned char sorted = size == 0; /* an empty array is already sorted */
    size_t idx = 0;

    while (!sorted) {
        sorted = 1;
        for (idx = 0; idx < size - 1; ++idx) {
            if (toSort[idx + 1] < toSort[idx]) {
                size_t tempToSort = toSort[idx];
                size_t tempToKeepAligned = toKeepAligned[idx];

                sorted = 0;

                toKeepAligned[idx] = toKeepAligned[idx + 1];
                toKeepAligned[idx + 1] = tempToKeepAligned;

                toSort[idx] = toSort[idx + 1];
                toSort[idx + 1] =  tempToSort;
            }
        }
    }
}
