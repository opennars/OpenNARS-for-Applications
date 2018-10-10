#include "Cycle.h"

int currentTime = 0;
void composition(Concept *c, Event *e)
{
    //temporal induction and intersection
}

void decomposition(Concept *c, Event *e)
{
    //detachment
}

void cycle()
{
    for(int i=0; i<EVENT_SELECTIONS; i++)
    {
        Item item = PriorityQueue_PopMax(&events);
        Event *e = item.address;
        int closest_concept_i = Memory_getClosestConcept(&e);
        if(closest_concept_i == MEMORY_MATCH_NO_CONCEPT)
        {
            continue;
        }
        Concept *c = concepts.items[closest_concept_i].address;
        decomposition(c, e);
        FIFO *fifo =  e->type == EVENT_TYPE_BELIEF ? &c->event_beliefs : &c->event_goals;
        Event revised = FIFO_AddAndRevise(e, fifo);
        if(revised.type != EVENT_TYPE_DELETED)
        {
            PriorityQueue_Push_Feedback pushed = PriorityQueue_Push(&events, revised.attention.priority);
            Event *toRecycle = pushed.addedItem.address;
            *toRecycle = revised;
        }
        e->attention = Attention_forgetTask(&e->attention);
        PriorityQueue_Push_Feedback pushed = PriorityQueue_Push(&events, e->attention.priority);
        Event *toRecyle = pushed.addedItem.address;
        *toRecyle = *e;
        for(int j=0; j<CONCEPT_SELECTIONS; j++)
        {
            Item item = PriorityQueue_PopMax(&concepts);
            Concept *c = item.address;
            composition(c, e); // deriving a =/> b
        }
        c->attention = Attention_activateConcept(&c->attention, &e->attention); 
        PriorityQueue_bubbleUp(&concepts,closest_concept_i); //priority was increased
        //add concept at the end, because before it needs to be identified with something existing
        Memory_addConcept(&e->sdr, Attention_activateConcept(&c->attention, &e->attention));
    }
    for(int i=0; i<concepts.itemsAmount; i++) //as all concepts are forgotten the order won't change
    { //making this operation very cheap, not demanding any heap operation
        Concept *c = concepts.items[i].address;
        c->attention = Attention_forgetConcept(&c->attention, &c->usage, currentTime);
        concepts.items[i].priority = c->attention.priority;
    }
    currentTime++;
}
