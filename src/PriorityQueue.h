#ifndef PRIORITYQUEUE_H
#define PRIORITYQUEUE_H

///////////////////////////
//  ANSNA priority queue //
///////////////////////////
//The priority queue for concepts and tasks
//Idea from https://stackoverflow.com/a/2935995

//References//
//-----------//
#include "AttentionValue.h"

//Parameters//
//----------//
#define MAX_ELEMENTS 1000000

//Data structure//
//--------------//
#define DefinePriorityQueue(QueueName, n, itemsname) \
	typedef struct \
	{ \
		int itemsname##_amount; \
		AttentionValue itemsname[n]; \
		} \
		QueueName;
DefinePriorityQueue(PriorityQueue, 0, items)

//Methods//
//-------//
void PriorityQueue_init(PriorityQueue *queue);

void PriorityQueue_add(PriorityQueue *queue, AttentionValue *element);

void *PriorityQueue_next(PriorityQueue *queue);

#endif

