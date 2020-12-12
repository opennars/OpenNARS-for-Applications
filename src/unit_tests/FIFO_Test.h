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

void FIFO_Test()
{
    return; //TODO!!!
    puts(">>FIFO test start");
    FIFO fifo = {0};
    //First, evaluate whether the fifo works, not leading to overflow
    for(int i=FIFO_SIZE*2; i>=1; i--) //"rolling over" once by adding a k*FIFO_Size items
    {
        Event event1 = { .term = Narsese_AtomicTerm("test"), 
                         .type = EVENT_TYPE_BELIEF, 
                         .truth = { .frequency = 1.0, .confidence = 0.9 },
                         .stamp = { .evidentalBase = { i } }, 
                         .occurrenceTime = FIFO_SIZE*2 - i*10 };
        FIFO_Add(&event1, &fifo);
    }
    for(int i=0; i<FIFO_SIZE; i++)
    {
        assert(FIFO_SIZE-i == fifo.array[0][i].stamp.evidentalBase[0], "Item at FIFO position has to be right");
    }
    //now see whether a new item is revised with the correct one:
    int i=3; //revise with item 10, which has occurrence time 10
    int newbase = FIFO_SIZE*2+1;
    Event event2 = { .term = Narsese_AtomicTerm("test"), 
                     .type = EVENT_TYPE_BELIEF, 
                     .truth = { .frequency = 1.0, .confidence = 0.9 },
                     .stamp = { .evidentalBase = { newbase } }, 
                     .occurrenceTime = i*10+3 };
    FIFO fifo2 = {0};
    for(int i=0; i<FIFO_SIZE*2; i++)
    {
        Term zero = (Term) {0};
        FIFO_Add(&event2, &fifo2);
        if(i < FIFO_SIZE && i < MAX_SEQUENCE_LEN)
        {
            char buf[100]; 
            Event *ev = FIFO_GetKthNewestSequence(&fifo2, 0, i);
            sprintf(buf,"This event Term is not allowed to be zero, sequence length=%d\n",i+1);
            assert(!Term_Equal(&zero, &ev->term),buf);
        }
    }
    assert(fifo2.itemsAmount == FIFO_SIZE, "FIFO size differs");
    puts("<<FIFO Test successful");
}
