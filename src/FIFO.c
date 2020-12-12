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
    fifo->array[1-1][fifo->currentIndex] = *event;
    fifo->currentIndex = (fifo->currentIndex + 1) % FIFO_SIZE;
    fifo->itemsAmount = MIN(fifo->itemsAmount + 1, FIFO_SIZE);
    for(int state=3; state<(1 << MAX_SEQUENCE_LEN); state++) //3=11 in binary
    {
        if(!(state & 1))
        {
            continue;
        }
        //query the substate (everything left of the 1 at the end of the bit sequence)
        int substate = state >> 1;
        int shifts = 1;
        while(!(substate & 1))
        {
            assert(substate > 0, "Substate is not supposed to vanish due to shift since we had state > 1");
            substate = substate >> 1;
            shifts++;
        }
        //If yes retrieve sequence and new event
        Event *sequence = FIFO_GetKthNewestSequence(fifo, shifts, substate); //cached sequence
        if(sequence == NULL)
        {
            continue;
        }
        Event *event = FIFO_GetNewestSequence(fifo, 1); //new event
        //and build and cache new sequence
        assert(event != NULL, "Event is not supposed to be NULL since sequence wasn't NULL!");
        bool success;
        Event new_sequence = Inference_BeliefIntersection(sequence, event, &success);
        if(!success || new_sequence.truth.confidence < MIN_CONFIDENCE) //Subsitution success and Apriory criteria
        {
            continue;
        }
        fifo->array[state-1][FIFO_Index(fifo, 0)] = new_sequence;
    }
}

Event* FIFO_GetKthNewestSequence(FIFO *fifo, int k, int state)
{
    assert(state > 0, "No event requested from FIFO!");
    //an element must exist, rightmost element needs to be included to avoid duplicates, shift needs to be within FIFO array
    if(fifo->itemsAmount == 0 || !(state & 1))
    {
        return NULL;
    }
    //try to return cached sequence if existing
    return &fifo->array[state-1][FIFO_Index(fifo, k)];
}

Event* FIFO_GetNewestSequence(FIFO *fifo, int state)
{
    return FIFO_GetKthNewestSequence(fifo, 0, state);
}
