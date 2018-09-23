#include "PriorityQueue.h"

void PriorityQueue_Push(PriorityQueue *queue, Prioritized *item, int itemsize, int maxElements)
{
    int i = queue->items_amount + 1;
    int j = i / 2;
    //"move the item "up" (exchanging child with parent) till element to insert priority is smaller than the node
    while(i > 1 && queue->items[j].priority <= item->priority)
    {
        if(i < maxElements) //we can't put the parent there if it's outside of the array
        {
            //queue->items[i] = queue->items[j];
            memcpy(&(queue->items[i]), &(queue->items[j]), itemsize);
        }
        i = j;
        j = j / 2; //parent
    }
    //now put element at the "free" parent, which was already copied to the child (which itself was copied to its child..):
    if(i < maxElements) //we can't put the element there if it's outside of the array
    {
        //queue->items[i] = item;
        memcpy(&(queue->items[i]), item, itemsize);
    }
    queue->items_amount++;
}

void PriorityQueue_Pop(PriorityQueue *queue, Prioritized *returnedItem, int itemsize)
{
    //No items, we can only return null
    if (!queue->items_amount)
    {
        return NULL;
    }
    //the highest priority item is the first in the array
    //returnedItem = queue->items[1];
    memcpy(returnedItem, &(queue->items[1]), itemsize);
    //queue->items[1] = queue->items[queue->items_amount];
    memcpy(&(queue->items[1]), &(queue->items[queue->items_amount]), itemsize);
    //if we remove it the amount gets reduced
    queue->items_amount--;
    //heapify what is left after the removal (decide new parent, propagate this handling iteratively down the tree)
    int current = 1, left, right, largest;
    while(current != queue->items_amount+1)
    {
        int left = 2*current;
        int right = left+1;
        largest = queue->items_amount+1;
        if (left <= queue->items_amount && queue->items[left].priority >= queue->items[largest].priority)
        {
            largest = left; //left is largest
        }
        if (right <= queue->items_amount && queue->items[right].priority >= queue->items[largest].priority)
        {
            largest = right; //move up right child
        }
        //queue->items[current] = queue->items[largest];
        memcpy(&(queue->items[current]), &(queue->items[largest]), itemsize);
        current = largest;
    }
}
