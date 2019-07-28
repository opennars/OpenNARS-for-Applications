#include "FIFO.h"

void FIFO_RESET(FIFO *fifo)
{
    fifo->itemsAmount = 0;
    fifo->currentIndex = 0;
    for(int len=0; len<MAX_SEQUENCE_LEN; len++)
    {
        for(int i=0; i<FIFO_SIZE; i++)
        {
            fifo->array[len][i] = (Event) {0};
        }
    }
}

void FIFO_Add(Event *event, FIFO *fifo)
{
    //build sequence elements:
    for(int len=0; len<MAX_SEQUENCE_LEN; len++)
    {
        Event sequence = *event;
        if(len>0)
        {
            Event *ev = FIFO_GetNewestSequence(fifo, len);
            if(ev == NULL || ev->type == EVENT_TYPE_DELETED)
            {
                break;
            }
            sequence = Inference_BeliefIntersection(&sequence, ev);
        }
        fifo->array[len][fifo->currentIndex] = sequence;
    }
    fifo->currentIndex = (fifo->currentIndex + 1) % FIFO_SIZE;
    fifo->itemsAmount = MIN(fifo->itemsAmount + 1, FIFO_SIZE);
}

Event* FIFO_GetKthNewestSequence(FIFO *fifo, int k, int len)
{
    if(fifo->itemsAmount == 0 || k >= FIFO_SIZE)
    {
        return NULL;
    }
    int index = fifo->currentIndex - 1 - k;
    if(index < 0)
    {
        index = FIFO_SIZE+index;
    }
    return &fifo->array[len][index];
}

Event* FIFO_GetNewestSequence(FIFO *fifo, int len)
{
    return FIFO_GetKthNewestSequence(fifo, 0, len);
}
