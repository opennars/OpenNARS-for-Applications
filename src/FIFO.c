#include "FIFO.h"

void FIFO_Add(Event *event, FIFO *fifo)
{
    fifo->array[fifo->currentIndex] = *event;
    fifo->currentIndex = (fifo->currentIndex + 1) % FIFO_SIZE;
}

Event FIFO_AddAndRevise(Event *event, FIFO *fifo)
{
    Event closest = {0};
    int closest_i = -1;
    for(int i=0; i<FIFO_SIZE; i++)
    {
        if(fifo->array[i].type != EVENT_TYPE_DELETED)
        {
            if(closest.type == EVENT_TYPE_DELETED)
            {
                closest = fifo->array[i];
                closest.truth = Truth_Projection(closest.truth, closest.occurrenceTime, event->occurrenceTime);
                closest_i = i;
            }
            else
            {
                Event potential_closest = fifo->array[i];
                potential_closest.truth = Truth_Projection(potential_closest.truth, potential_closest.occurrenceTime, event->occurrenceTime);
                if(potential_closest.truth.confidence > closest.truth.confidence)
                {
                    closest = potential_closest;
                    closest_i = i;
                }
            }
        }
    }
    if(Stamp_checkOverlap(&event->stamp, &closest.stamp))
    {
        FIFO_Add(event, fifo);
        return *event;
    }
    Event revised = Inference_EventRevision(&closest, event);
    fifo->array[closest_i] = (Event) {0};
    FIFO_Add(&revised, fifo);
    return revised;
}
