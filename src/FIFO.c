/* 
 * The MIT License
 *
 * Copyright 2020 The OpenNARS authors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "FIFO.h"

static int FIFO_Index(FIFO *fifo, int k)
{
    assert(k >= 0 && k < FIFO_SIZE, "FIFO shift out of bounds!");
    int index = fifo->currentIndex - 1 - k;
    if(index < 0)
    {
        index = FIFO_SIZE+index;
    }
    return index;
}

void FIFO_Add(Event *event, FIFO *fifo)
{
    //build sequence elements:
    fifo->array[fifo->currentIndex] = *event;
    fifo->currentIndex = (fifo->currentIndex + 1) % FIFO_SIZE;
    fifo->itemsAmount = MIN(fifo->itemsAmount + 1, FIFO_SIZE);
}

Event* FIFO_GetKthNewestSequence(FIFO *fifo, int k)
{
    //an element must exist, rightmost element needs to be included to avoid duplicates, shift needs to be within FIFO array
    if(fifo->itemsAmount == 0)
    {
        return NULL;
    }
    //try to return cached sequence if existing
    return &fifo->array[FIFO_Index(fifo, k)];
}

Event* FIFO_GetNewestSequence(FIFO *fifo)
{
    return FIFO_GetKthNewestSequence(fifo, 0);
}
