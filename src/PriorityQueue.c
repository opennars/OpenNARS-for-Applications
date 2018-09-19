#include "PriorityQueue.h"

void PriorityQueue_init(PriorityQueue *queue)
{
	// initialize levels and corresponding arrays
	for (int iPriority=0;iPriority<PRIORITYQUEUE_LEVELS;iPriority++) {
		queue->levels[iPriority].priority = iPriority;
		array_init(&queue->levels[iPriority].fifo, sizeof(void*));
	}
}

void PriorityQueue_add(PriorityQueue *queue, int priority, void *element)
{
	// assumption< priorities are indices >
	array_append(&queue->levels[priority].fifo, &element);
}

void *PriorityQueue_next(PriorityQueue *queue)
{
	for (int iPriority=0;iPriority<PRIORITYQUEUE_LEVELS;iPriority++) {
		if (array_length(&queue->levels[iPriority].fifo) == 0) {
			continue;
		}

		void *element;
		memcpy(element, array_at(&queue->levels[iPriority].fifo, 0), sizeof(void*));

		array_removeAt(&queue->levels[iPriority].fifo, 0);
		
		return element;
	}

	return (void*)0;
}
