#include "PriorityQueue.h"

PriorityQueue_Push_Feedback PriorityQueue_Push(PriorityQueue *queue, double priority, int maxElements)
{
    PriorityQueue_Push_Feedback feedback = {0};
    int i = queue->items_amount + 1;
    int j = i / 2;
    //"move the item "up" (exchanging child with parent) till element to insert priority is smaller than the node
    while(i > 1 && queue->items[j].priority <= priority)
    {
        if(i < maxElements) //we can't put the parent there if it's outside of the array
        {
            queue->items[i] = queue->items[j];
        } 
        else //item_j got evicted as there was no space(i) move it to, it gets replaced with item_j/2 now
        {
            feedback.evicted = true;
            feedback.evictedItem = queue->items[j];
        }
        i = j;
        j = j / 2; //parent
    }
    //now put element at the "free" parent, which was already copied to the child (which itself was copied to its child..):
    if(i < maxElements) //we can't put the element there if it's outside of the array
    {
        queue->items[i].priority = priority;
        //if not evicted, item address already points to the related item storage item,
        //else use address of evicted item
        if(feedback.evicted) 
        {
            queue->items[i].address = feedback.evictedItem.address;
        }
        else
        {
            queue->items_amount++;
        }
        feedback.added = true;
        feedback.addedItem = queue->items[i];
        
    }
    return feedback;
}

Item PriorityQueue_Pop(PriorityQueue *queue)
{
    //No items, we can only return null
    if (!queue->items_amount)
    {
        return (Item) {0};
    }
    //the highest priority item is the first in the array
    Item returnedItem = queue->items[1];
    queue->items[1] = queue->items[queue->items_amount];
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
        queue->items[current] = queue->items[largest];
        current = largest;
    }
    return returnedItem;
}
