#ifndef PRIORITYQUEUE_H
#define PRIORITYQUEUE_H

#include "Array.h"

#define PRIORITYQUEUE_LEVELS 100

// lower priorities are favored


typedef struct {
	// -1 : not used
	int priority;
	Array fifo;
} FifoAndPriorityTuple;

typedef struct {
	// priorities are ordered
	FifoAndPriorityTuple levels[PRIORITYQUEUE_LEVELS];
} PriorityQueue;

void PriorityQueue_init(PriorityQueue *queue);

void PriorityQueue_add(PriorityQueue *queue, int priority, void *element);

void *PriorityQueue_next(PriorityQueue *queue);

#endif

