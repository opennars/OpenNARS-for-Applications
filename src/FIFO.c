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
    //build saturated sequence element:
    Event sequence = *event;
    if(FIFO_PERFORM_SEQUENCING)
    {
        SDR contextSDR = {0};
        for(int k=0; k<MAX_SEQUENCE_LEN; k++)
        {
            if(SDR_CountTrue(&sequence.sdr) >= SDR_MAX_SATURATION)
            {
                break;
            }
            Event *ev = FIFO_GetKthNewestElement(fifo, k);
            if(ev == NULL || ev->type == EVENT_TYPE_DELETED)
            {
                break;
            }
            contextSDR = SDR_Tuple(&contextSDR, &ev->sdr);
            sequence.sdr = SDR_Union(&contextSDR, &event->sdr);
        }
    }
    fifo->sequence_array[fifo->currentIndex] = sequence;
    fifo->array[fifo->currentIndex] = *event;
    fifo->currentIndex = (fifo->currentIndex + 1) % FIFO_SIZE;
    fifo->itemsAmount = MIN(fifo->itemsAmount + 1, FIFO_SIZE);
}

Event* FIFO_GetKthNewestElement(FIFO *fifo, int k)
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
    return &fifo->array[index];
}

Event* FIFO_GetNewestElement(FIFO *fifo)
{
    return FIFO_GetKthNewestElement(fifo, 0);
}

Event* FIFO_GetKthNewestSequence(FIFO *fifo, int k)
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
    return &fifo->sequence_array[index];
}

Event* FIFO_GetNewestSequence(FIFO *fifo)
{
    return FIFO_GetKthNewestSequence(fifo, 0);
}

