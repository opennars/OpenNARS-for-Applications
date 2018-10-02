#include "FIFO.h"

void FIFO_RESET(FIFO *fifo)
{
    for(int i=0; i<FIFO_SIZE; i++)
    {
        fifo->array[i] = (Event) {0};
    }
    fifo->currentIndex = 0;
}

void FIFO_Add(Event *event, FIFO *fifo)
{
    fifo->array[fifo->currentIndex] = *event;
    fifo->currentIndex = (fifo->currentIndex + 1) % FIFO_SIZE;
}
