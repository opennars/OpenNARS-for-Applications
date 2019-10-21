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
        //len is 0, just add current event to FIFO on len0 level
        if(len == 0)
        {
            fifo->array[len][fifo->currentIndex] = *event;
        }
        else //len>0, so chain previous sequence with length len-1 with new event
        {
            Event *sequence = FIFO_GetNewestSequence(fifo, len-1);
            if(sequence == NULL || sequence->type == EVENT_TYPE_DELETED)
            {
                break;
            }
            //printf("occurrence times a=%d, b=%d", ((int) sequence->occurrenceTime),((int) event->occurrenceTime));
            Event new_sequence = Inference_BeliefIntersection(sequence, event);
            fifo->array[len][fifo->currentIndex] = new_sequence;
        }
        
    }
    fifo->currentIndex = (fifo->currentIndex + 1) % FIFO_SIZE;
    fifo->itemsAmount = MIN(fifo->itemsAmount + 1, FIFO_SIZE);
}

Event* FIFO_GetKthNewestSequence(FIFO *fifo, int k, int len)
{
    if(fifo->itemsAmount == 0 || k >= fifo->itemsAmount)
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
