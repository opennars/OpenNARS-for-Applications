#include "PriorityQueue.h"

#define PriorityQueue_Implementation(QueueType, ItemType, ItemsName)                                                                \
QueueType##_Push_Feedback QueueType##_Push(QueueType *queue, ItemType item)                                                         \
{                                                                                                                                   \
    QueueType##_Push_Feedback feedback = {0};                                                                                       \
    int i = queue->items_amount + 1;                                                                                                \
    int j = i / 2;                                                                                                                  \  
    /*"move the item "up" (exchanging child with parent) till element to insert priority is smaller than the node*/                 \
    while(i > 1 && queue->items[j].attention.priority <= item.attention.priority)                                                   \
    {                                                                                                                               \
        if(i < QueueType##_MaxSize) /*we can't put the parent there if it's outside of the array*/                                  \
        {                                                                                                                           \
            queue->items[i] = queue->items[j];                                                                                      \
        }                                                                                                                           \
        else /*item_j got evicted as there was no space(i) move it to, it gets replaced with item_j/2 now*/                         \
        {                                                                                                                           \
            feedback.evicted = true;                                                                                                \
            feedback.evicted_item = queue->items[j];                                                                                \
        }                                                                                                                           \
        i = j;                                                                                                                      \
        j = j / 2; /*parent*/                                                                                                       \
    }                                                                                                                               \
    /*now put element at the "free" parent, which was already copied to the child (which itself was copied to its child..):*/       \
    if(i < QueueType##_MaxSize) /*we can't put the element there if it's outside of the array*/                                     \
    {                                                                                                                               \
        feedback.added = true;                                                                                                      \
        queue->items[i] = item;                                                                                                     \
    }                                                                                                                               \
    queue->items_amount++;                                                                                                          \
    return feedback;                                                                                                                \
}                                                                                                                                   \
                                                                                                                                    \
                                                                                                                                    \
ItemType QueueType##_Pop(QueueType *queue)                                                                                          \
{                                                                                                                                   \
    /*No items, we can only return null*/                                                                                           \
    if (!queue->items_amount)                                                                                                       \                                                     
    {                                                                                                                               \
        return (ItemType) {0};                                                                                                                \
    }                                                                                                                               \
    /*the highest priority item is the first in the array*/                                                                         \
    ItemType returnedItem = queue->items[1];                                                                                        \
    queue->items[1] = queue->items[queue->items_amount];                                                                            \
    /*if we remove it the amount gets reduced*/                                                                                     \
    queue->items_amount--;                                                                                                          \
    /*heapify what is left after the removal (decide new parent, propagate this handling iteratively down the tree)*/               \
    int current = 1, left, right, largest;                                                                                          \
    while(current != queue->items_amount+1)                                                                                         \
    {                                                                                                                               \
        int left = 2*current;                                                                                                       \
        int right = left+1;                                                                                                         \
        largest = queue->items_amount+1;                                                                                            \
        if (left <= queue->items_amount && queue->items[left].attention.priority >= queue->items[largest].attention.priority)       \
        {                                                                                                                           \
            largest = left; /*left is largest*/                                                                                     \
        }                                                                                                                           \
        if (right <= queue->items_amount && queue->items[right].attention.priority >= queue->items[largest].attention.priority)     \
        {                                                                                                                           \
            largest = right; /*move up right child*/                                                                                \
        }                                                                                                                           \
        queue->items[current] = queue->items[largest];                                                                              \
        current = largest;                                                                                                          \
    }                                                                                                                               \
    return returnedItem;                                                                                                            \
}

PriorityQueue_Implementation(ConceptQueue, Concept, concepts)
PriorityQueue_Implementation(EventQueue, Event, events)
