#include "FIFO.h"

void FIFO_RESET(FIFO *fifo)
{
    fifo->itemsAmount = 0;
    fifo->currentIndex = 0;
    for(int i=0; i<FIFO_SIZE; i++)
    {
        fifo->array[i] = (Event) {0};
    }
}

void FIFO_COPY(FIFO *src, FIFO *target)
{
    target->itemsAmount = src->itemsAmount;
    target->currentIndex = src->currentIndex;
    for(int i=0; i<FIFO_SIZE; i++)
    {
        target->array[i] = src->array[i];
    }
}

void FIFO_Add(Event *event, FIFO *fifo)
{
    /*for(int i=0; i<fifo->itemsAmount; i++)
    {
        if(SDR_Equal(&fifo->array[i].sdr, &event->sdr))
        {
            fifo->array[i].deleted = true;
        }
    }*/
    fifo->array[fifo->currentIndex] = *event;
    fifo->currentIndex = (fifo->currentIndex + 1) % FIFO_SIZE;
    fifo->itemsAmount = MIN(fifo->itemsAmount + 1, FIFO_SIZE);
}

FIFO_Query_Result FIFO_GetHighestConfidentProjectedTo(FIFO *fifo, long occurrenceTime, SDR *referenceSdr)
{
    int closest_i = -1;
    Event closest = {0};
    for(int i=0; i<FIFO_SIZE; i++)
    {
        if(fifo->array[i].type != EVENT_TYPE_DELETED && fifo->array[i].operationID == 0)
        {
            if(closest.type == EVENT_TYPE_DELETED)
            {
                closest = fifo->array[i];
                closest.truth = Truth_Analogy(SDR_Similarity(referenceSdr, &closest.sdr), closest.truth);
                closest.sdr = *referenceSdr;
                closest.truth = Truth_Projection(closest.truth, closest.occurrenceTime, occurrenceTime);
                closest_i = i;
            }
            else
            {
                Event potential_closest = fifo->array[i];
                potential_closest.truth = Truth_Projection(potential_closest.truth, potential_closest.occurrenceTime, occurrenceTime);
                potential_closest.occurrenceTime = occurrenceTime;
                if(potential_closest.truth.confidence > closest.truth.confidence)
                {
                    closest = potential_closest;
                    closest_i = i;
                }
            }
        }
    }
    return (FIFO_Query_Result) { .originalEvent = &fifo->array[closest_i], 
                                 .projectedEvent = closest };
}

Event FIFO_AddAndRevise(Event *event, FIFO *fifo)
{
    if(fifo->itemsAmount == 0)
    {
        FIFO_Add(event, fifo);
        return (Event) {0};
    }
    FIFO_Query_Result closest = FIFO_GetHighestConfidentProjectedTo(fifo, event->occurrenceTime, &event->sdr);    
    //overlap happened, we can't revise, so just add the event to FIFO
    if(Stamp_checkOverlap(&event->stamp, &closest.projectedEvent.stamp))
    {
        FIFO_Add(event, fifo);
        return (Event) {0};
    }
    //one is (&/,a,op1) the other (&/,a,op2), they are not compatible
    if(event->operationID != closest.projectedEvent.operationID)
    {
        FIFO_Add(event, fifo);
        return (Event) {0};
    }
    Event revised = Inference_EventRevision(closest.originalEvent, event);
    //Revision into the middle of both occurrence times leaded to truth lower than the premises, don't revise and add event
    if(revised.truth.confidence < closest.projectedEvent.truth.confidence && revised.truth.confidence < event->truth.confidence)
    {
        FIFO_Add(event, fifo);
        return (Event) {0};
    }
    //Else we set the revised one instead of the closest one
    *(closest.originalEvent) = revised;
    return revised;
}

Event* FIFO_GetKthNewestElement(FIFO *fifo, int k)
{
    if(fifo->itemsAmount == 0)
    {
        return NULL;
    }
    int index = fifo->currentIndex - 1 - k;
    if(index < 0)
    {
        index = FIFO_SIZE+index;
    }
    return &fifo->array[index];
}

Event* FIFO_GetNewestElement(FIFO *fifo)
{
    return FIFO_GetKthNewestElement(fifo, 0);
}
