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
#include "Event.h"
#include "Concept.h"

//Parameters//
//----------//
#define CONCEPTS_MAX 10000
#define EVENTS_MAX 64

//Data structure//
//--------------//
#define PriorityQueue_Header(QueueType, MaxSize, ItemType)                                                                  \
static int QueueType##_MaxSize = MaxSize;                                                                                   \
typedef struct                                                                                                              \
{                                                                                                                           \
    int items_amount;                                                                                                       \
    ItemType items[MaxSize];                                                                                                \
} QueueType;                                                                                                                \
                                                                                                                            \
typedef struct                                                                                                              \
{                                                                                                                           \
    bool added;                                                                                                             \
    bool evicted;                                                                                                           \
    ItemType evicted_item;                                                                                                  \
} QueueType##_Push_Feedback;                                                                                                \
                                                                                                                            \
/*Methods*/                                                                                                                 \
/*-------*/                                                                                                                 \
/*Push element into the queue*/                                                                                             \
QueueType##_Push_Feedback QueueType##_Push(QueueType *queue, ItemType item);                                                \
/*Pop first element from the queue*/                                                                                        \
ItemType QueueType##_Pop(QueueType *queue);

PriorityQueue_Header(ConceptQueue, CONCEPTS_MAX, Concept);  
PriorityQueue_Header(EventQueue, EVENTS_MAX, Event);                                                                 

#endif

