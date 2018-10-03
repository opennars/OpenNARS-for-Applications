#ifndef PRIORITYQUEUE_H
#define PRIORITYQUEUE_H

///////////////////////////
//  ANSNA priority queue //
///////////////////////////
//The priority queue for concepts and tasks
//Idea from https://stackoverflow.com/a/2935995

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
    int items_amount;
} PriorityQueue;

//Methods//
//-------//
typedef struct
{
    bool added;
    Item addedItem;
    bool evicted;
    Item evictedItem;
} PriorityQueue_Push_Feedback;
//Push element of a certain priority into the queue.
//If successful, addedItem will point to the item in the data structure, with address of the evicted item
PriorityQueue_Push_Feedback PriorityQueue_Push(PriorityQueue *queue, double priority, int maxElements);
Item PriorityQueue_Pop(PriorityQueue *queue);

#endif

