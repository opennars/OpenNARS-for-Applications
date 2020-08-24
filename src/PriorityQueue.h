/* 
 * The MIT License
 *
 * Copyright 2020 The OpenNARS authors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef PRIORITYQUEUE_H
#define PRIORITYQUEUE_H

/////////////////////
//  Priority queue //
/////////////////////
//The priority queue for concepts and tasks
//Related publication: Atkinson, M. D., Sack, J. R., Santoro, N., & Strothotte, T. (1986). Min-max heaps and generalized priority queues. Communications of the ACM, 29(10), 996-1000.
//Ported from https://github.com/quxiaofeng/python-stl/blob/master/meshlab/MeshLabSrc_AllInc_v132/meshlab/src/plugins_experimental/edit_ocme/src/cache/old/mmheap.h

//References//
//-----------//
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

//Data structure//
//--------------//
typedef struct
{
    double priority;
    void *address;
} Item;

typedef struct
{
    Item *items;
    int32_t itemsAmount;
    int32_t maxElements;
} PriorityQueue;

typedef struct
{
    bool added;
    Item addedItem;
    bool evicted;
    Item evictedItem;
} PriorityQueue_Push_Feedback;

//Methods//
//-------//
//Resets the priority queue
void PriorityQueue_RESET(PriorityQueue *queue, Item *items, int32_t maxElements);
//Push element of a certain priority into the queue.
//If successful, addedItem will point to the item in the data structure, with address of the evicted item, if eviction happened
PriorityQueue_Push_Feedback PriorityQueue_Push(PriorityQueue *queue, double priority);
//use this function and add again if maybe lower!
bool PriorityQueue_PopAt(PriorityQueue *queue, int32_t i, void** returnItemAddress);
//Rebuilds the data structure by re-inserting all elements:
void PriorityQueue_Rebuild(PriorityQueue *queue);
//Pops minimum element
bool PriorityQueue_PopMin(PriorityQueue *queue, void** returnItemAddress, double* returnItemPriority);
//Pops maximum element
bool PriorityQueue_PopMax(PriorityQueue *queue, void** returnItemAddress, double* returnItemPriority);

#endif

