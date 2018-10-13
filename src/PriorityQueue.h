#ifndef PRIORITYQUEUE_H
#define PRIORITYQUEUE_H

///////////////////////////
//  ANSNA priority queue //
///////////////////////////
//The priority queue for concepts and tasks
//Ported from https://github.com/quxiaofeng/python-stl/blob/master/meshlab/MeshLabSrc_AllInc_v132/meshlab/src/plugins_experimental/edit_ocme/src/cache/old/mmheap.h

//References//
//-----------//
#include <stdlib.h>
#include <stdbool.h>

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
    int itemsAmount;
    int maxElements;
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
void PriorityQueue_RESET(PriorityQueue *queue, Item *items, int maxElements);
//Push element of a certain priority into the queue.
//If successful, addedItem will point to the item in the data structure, with address of the evicted item, if eviction happened
PriorityQueue_Push_Feedback PriorityQueue_Push(PriorityQueue *queue, double priority);
//Retrieve first item of the queue
Item PriorityQueue_PopMin(PriorityQueue *queue);
Item PriorityQueue_PopMax(PriorityQueue *queue);
//make sure that the new priority is really higher!
void PriorityQueue_IncreasePriority(PriorityQueue *queue, int i, double newPriority);

#endif

