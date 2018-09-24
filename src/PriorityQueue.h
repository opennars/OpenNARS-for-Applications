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
#define DefinePriorityQueue(QueueName, n, Type, itemsname) \
    typedef struct \
    { \
        int itemsname##_amount; \
        Type itemsname[n]; \
        } \
        QueueName;
typedef struct
{
    double priority;
} Prioritized;
DefinePriorityQueue(PriorityQueue, 0, Prioritized, items)

//Methods//
//-------//

//Push element into the queue. Note that while data struct surely starts with Prioritized (so has priority), it contains the entire data of the item
//so might be a Task or Concept, so make sure to pass sizeof(Task) or sizeof(Concept) as datasize
//returns if the object as rejected
typedef struct
{
    bool added;
    bool evicted;
} PriorityQueue_Push_Feedback;
PriorityQueue_Push_Feedback PriorityQueue_Push(PriorityQueue *queue, Prioritized *item, int itemsize, int maxElements, Prioritized *evicted_item);
void PriorityQueue_Pop(PriorityQueue *queue, Prioritized *returnedItem, int itemsize);

#endif

