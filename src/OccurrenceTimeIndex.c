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

#include "OccurrenceTimeIndex.h"

static int OccurrenceTimeIndex_Index(OccurrenceTimeIndex *fifo, int k)
{
    assert(k >= 0 && k < OCCURRENCE_TIME_INDEX_SIZE, "OccurrenceTimeIndex shift out of bounds!");
    int index = fifo->currentIndex - 1 - k;
    if(index < 0)
    {
        index = OCCURRENCE_TIME_INDEX_SIZE+index;
    }
    return index;
}

void OccurrenceTimeIndex_Add(Concept *concept, OccurrenceTimeIndex *fifo)
{
    //build sequence elements:
    fifo->array[fifo->currentIndex] = concept;
    fifo->currentIndex = (fifo->currentIndex + 1) % OCCURRENCE_TIME_INDEX_SIZE;
    fifo->itemsAmount = MIN(fifo->itemsAmount + 1, OCCURRENCE_TIME_INDEX_SIZE);
}

Concept* OccurrenceTimeIndex_GetKthNewestElement(OccurrenceTimeIndex *fifo, int k)
{
    //an element must exist
    if(fifo->itemsAmount == 0)
    {
        return NULL;
    }
    //try to return cached sequence if existing
    return fifo->array[OccurrenceTimeIndex_Index(fifo, k)];
}

