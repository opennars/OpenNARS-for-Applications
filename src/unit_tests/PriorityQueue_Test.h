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

void PriorityQueue_Test()
{
    puts(">>PriorityQueue test start");
    PriorityQueue queue;
    int32_t n_items = 10;
    Item items[n_items];
    for(int32_t i=0; i<n_items; i++)
    {
        items[i].address = (void*) ((uint32_t) i+1);
        items[i].priority = 0;
    }
    PriorityQueue_RESET(&queue, items, n_items);
    for(int32_t i=0, evictions=0; i<n_items*2; i++)
    {
        PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&queue, 1.0/((double) (n_items*2-i)));
        if(feedback.added)
        {
            printf("item was added %f %" PRIu32 "\n", feedback.addedItem.priority, (uint32_t)feedback.addedItem.address);
        }
        if(feedback.evicted)
        {
            double epsilon = 0.0001;
            printf("evicted item %f %" PRIu32 "\n", feedback.evictedItem.priority, (uint32_t)feedback.evictedItem.address);
            assert(evictions>0 || feedback.evictedItem.priority-1.0/((double) (n_items*2))<epsilon, "the evicted item has to be the lowest priority one");
            assert(queue.itemsAmount < n_items+1, "eviction should only happen when full!");
            evictions++;
        }
    }
    puts("<<PriorityQueue test successful");
}
