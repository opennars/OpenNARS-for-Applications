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

void OccurrenceTimeIndex_Test()
{
    puts(">>OccurrenceTimeIndex test start");
    OccurrenceTimeIndex fifo = {0};
    //First, evaluate whether the fifo works, not leading to overflow
    int occurrence = 0;
    for(int i=OCCURRENCE_TIME_INDEX_SIZE*2; i>=1; i--) //"rolling over" once by adding a k*FIFO_Size items
    {
        Concept *c1 = (Concept*) (long) i;
        OccurrenceTimeIndex_Add(c1, &fifo);
    }
    for(int i=0; i<OCCURRENCE_TIME_INDEX_SIZE; i++)
    {
        assert(OCCURRENCE_TIME_INDEX_SIZE-i == (long) fifo.array[i], "Item at OccurrenceTimeIndex position has to be right");
    }
    assert(fifo.itemsAmount == OCCURRENCE_TIME_INDEX_SIZE, "OccurrenceTimeIndex size differs");
    puts("<<OccurrenceTimeIndex Test successful");
}
