#include "PriorityQueue.h"

void PriorityQueue_RESET(PriorityQueue *queue, Item *items, int maxElements)
{
    queue->items = items;
    queue->maxElements = maxElements;
    queue->itemsAmount = 0;
}

#define at(i) (queue->items[i])

void swap(PriorityQueue *queue, int index1, int index2)
{
    Item temp = at(index1);
    at(index1) = at(index2);
    at(index2) = temp;
}

bool isOnMaxLevel(int i)
{ 
    int level=-1;
    int n = i+1;
    while(n)
    {
      n = n >> 1;
      level++;
    }
    return level & 1; /*% 2*/;
}

int parent(int i) 
{ 
    return ((i+1)/2)-1;
}

int grandparent(int i) 
{ 
    return ((i+1)/4)-1;
}

int leftChild(int i) 
{ 
    return 2*i + 1;
}

int leftGrandChild(int i)
{ 
    return 4*i + 3;
}

int smallestChild(PriorityQueue *queue, int i, bool invert) 
{ //return smallest of children or self if no children
    int l = leftChild(i);
    if(l >= queue->itemsAmount)
    { 
        return i; //no children, return self
    }
    int r = l+1; //right child
    if(r < queue->itemsAmount)
    {
        Item lv = at(l);
        Item rv = at(r);
        if((rv.priority < lv.priority)^invert)
        {
            return r;
        }
    }
    return l;
}

int smallestGrandChild(PriorityQueue *queue, int i, bool invert)
{//return smallest of grandchildren or self if no children
    int l = leftGrandChild(i);
    if(l >= queue->itemsAmount)
    {
        return i;
    }
    Item lv = at(l);
    int min = l;
    for(int r=l+1; r<queue->itemsAmount && r < l+4; r++) 
    { //iterate on three grandsiblings (they are consecutive)
        Item rv = at(r);
        if((rv.priority < lv.priority)^invert)
        {
            lv = rv;
            min = r;
        }
    }
    return min;
}
  
void trickleDown(PriorityQueue *queue, int i, bool invert)
{   //assert(invert == isOnMaxLevel(i));
    while(1)
    {
        //enforce min-max property on level(i), we need to check children and grandchildren
        int m = smallestChild(queue, i, invert);
        if(m == i)
        {
            break; //no children
        }
        if((at(m).priority < at(i).priority)^invert) //swap children, max property on level(i)+1 automatically enforced
        {
            swap(queue, i, m);
        }
        int j = smallestGrandChild(queue, i, invert);
        if(j == i)
        {
            break; //no grandchildren
        }
        if((at(j).priority < at(i).priority)^invert) 
        {
            swap(queue, i, j);
            i = j; //we need to enforce min-max property on level(j) now.
        } 
        else
        {
            break; //no swap, finish
        }
    }
}

void bubbleUp(PriorityQueue *queue, int i)
{
    int m;
    m = parent(i);
    bool invert = isOnMaxLevel(i);
    if (m>=0 && ((at(i).priority > at(m).priority)^invert))
    {
        swap(queue, i, m);
        i = m;
        invert = !invert;
    }
    m = grandparent(i);
    while (m>=0 && ((at(i).priority < at(m).priority)^invert))
    {
        swap(queue, i,m);
        i = m;
        m = grandparent(i);
    }
}

void PriorityQueue_IncreasePriority(PriorityQueue *queue, int i, double newPriority)
{
    queue->items[i].priority = newPriority;
    bubbleUp(queue, i);
}

PriorityQueue_Push_Feedback PriorityQueue_Push(PriorityQueue *queue, double priority)
{
    PriorityQueue_Push_Feedback feedback = (PriorityQueue_Push_Feedback) {0};
    //first evict if necessary
    if(queue->itemsAmount >= queue->maxElements)
    {
        if(priority < at(0).priority)
        { //smaller than smallest
            return feedback;
        }
        feedback.evicted = true;
        feedback.evictedItem = PriorityQueue_PopMin(queue);
    }
    at(queue->itemsAmount).priority = priority;
    if(feedback.evicted)
    {
        at(queue->itemsAmount).address = feedback.evictedItem.address; 
    }
    feedback.added = true;
    feedback.addedItem = at(queue->itemsAmount);
    queue->itemsAmount++;
    bubbleUp(queue, queue->itemsAmount-1);
    return feedback;
}

Item PriorityQueue_PopMin(PriorityQueue *queue)
{
    Item item = at(0);
    swap(queue, 0, queue->itemsAmount-1);
    queue->itemsAmount--;
    trickleDown(queue, 0, false); //enforce minmax heap property
    return item;
}

Item PriorityQueue_PopMax(PriorityQueue *queue)
{
    int p = smallestChild(queue, 0, true);
    Item item = at(p);
    swap(queue, p, queue->itemsAmount-1); //swap max with last item
    queue->itemsAmount--;
    trickleDown(queue, p, true); //enforce minmax heap property
    return item;
}

Item PriorityQueue_PopAt(PriorityQueue *queue, int i)
{
    Item item = at(i);
    swap(queue, i, queue->itemsAmount-1); 
    queue->itemsAmount--;
    trickleDown(queue, i, true); //enforce minmax heap property
    return item;
}
