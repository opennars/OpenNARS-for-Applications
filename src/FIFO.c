#include "FIFO.h"

#define MIN(a,b) (((a)<(b))?(a):(b))

void FIFO_Add(Event *event, FIFO *fifo)
{
    fifo->array[fifo->currentIndex] = *event;
    fifo->currentIndex = (fifo->currentIndex + 1) % FIFO_SIZE;
    fifo->itemsAmount = MIN(fifo->currentIndex + 1, FIFO_SIZE);
}

FIFO_Query_Result FIFO_GetHighestConfidentProjectedTo(FIFO *fifo, long occurrenceTime)
{
    int closest_i = -1;
    Event closest = {0};
    for(int i=0; i<FIFO_SIZE; i++)
    {
        if(fifo->array[i].type != EVENT_TYPE_DELETED)
        {
            if(closest.type == EVENT_TYPE_DELETED)
            {
                closest = fifo->array[i];
                closest.truth = Truth_Projection(closest.truth, closest.occurrenceTime, occurrenceTime);
                closest_i = i;
            }
            else
            {
                Event potential_closest = fifo->array[i];
                potential_closest.truth = Truth_Projection(potential_closest.truth, potential_closest.occurrenceTime, occurrenceTime);
                if(potential_closest.truth.confidence > closest.truth.confidence)
                {
                    closest = potential_closest;
                    closest_i = i;
                }
            }
        }
    }
    return (FIFO_Query_Result) { .index = closest_i, 
                                 .projectedEvent = closest };
}

Event FIFO_AddAndRevise(Event *event, FIFO *fifo)
{
    if(fifo->itemsAmount == 0)
    {
        FIFO_Add(event, fifo);
        return (Event) {0};
    }
    FIFO_Query_Result closest = FIFO_GetHighestConfidentProjectedTo(fifo, event->occurrenceTime);    
    //overlap happened, we can't revise, so just add the event to FIFO
    if(Stamp_checkOverlap(&event->stamp, &closest.projectedEvent.stamp))
    {
        FIFO_Add(event, fifo);
        return (Event) {0};
    }
    Event revised = Inference_EventRevision(&closest.projectedEvent, event);
    //Revision into the middle of both occurrence times leaded to truth lower than the premises, don't revise and add event
    if(revised.truth.confidence < closest.projectedEvent.truth.confidence && revised.truth.confidence < event->truth.confidence)
    {
        FIFO_Add(event, fifo);
        return (Event) {0};
    }
    //Else we add the revised one and set the closest one to be deleted/0
    fifo->array[closest.index] = (Event) {0};
    FIFO_Add(&revised, fifo);
    return revised;
}
