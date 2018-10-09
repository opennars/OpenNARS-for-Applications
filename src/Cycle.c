#include "Cycle.h"

int currentTime = 0;
void cycle()
{
    for(int i=0; i<EVENT_SELECTIONS; i++)
    {
        Item item = PriorityQueue_PopMax(&events);
        Event *e = item.address;
        Concept *c = Memory_getClosestConcept(&e);
        FIFO *fifo =  e->type == EVENT_TYPE_BELIEF ? &c->event_beliefs : &c->event_goals;
        FIFO_AddAndRevise(e, fifo);
        e->attention = Attention_forgetTask(&e->attention);
        PriorityQueue_Push_Feedback pushed = PriorityQueue_Push(&events, e->attention.priority);
        Event *cpushed = pushed.addedItem.address;
        *cpushed = *e;
        //todo there is more that needs to happen for event-concept interaction, inference! :)
    }
    for(int i=0; i<concepts.itemsAmount; i++) //as all concepts are forgotten the order won't change
    { //making this operation very cheap, not demanding any heap operation
        Concept *c = concepts.items[i].address;
        c->attention = Attention_forgetConcept(&c->attention, &c->usage, currentTime);
        concepts.items[i].priority = c->attention.priority;
    }
    currentTime++;
}
