#include "Cycle.h"

void composition(Concept *B, Concept *A, Event *b)
{
    //temporal induction and intersection
    if(b->type == EVENT_TYPE_BELIEF && A->event_beliefs.itemsAmount > 0)
    {
        Event *a = &A->event_beliefs.array[0]; //most recent, highest revised
        if(!Stamp_checkOverlap(&a->stamp, &b->stamp))
        {
            Implication implication = Inference_BeliefInduction(a, b);
            Table_AddAndRevise(&B->precondition_beliefs, &implication);
            Table_AddAndRevise(&A->postcondition_beliefs, &implication);
            Event sequence = b->occurrenceTime > a->occurrenceTime ? Inference_BeliefIntersection(a, b) : Inference_BeliefIntersection(b, a);
            Memory_addEvent(&sequence);
        }
    }
}

void decomposition(Concept *c, Event *e)
{
    //detachment
    if(c->postcondition_beliefs.itemsAmount>0)
    {
        Implication postcon = c->postcondition_beliefs.array[0];
        if(!Stamp_checkOverlap(&e->stamp, &postcon.stamp))
        {
            Event res = e->type == EVENT_TYPE_BELIEF ? Inference_BeliefDeduction(e, &postcon) : Inference_GoalAbduction(e, &postcon);
            res.attention = Attention_deriveEvent(&c->attention, &postcon.truth);
            Memory_addEvent(&res);
            //add negative evidence to the used predictive hypothesis (assumption of failure, for extinction)
            Table_PopHighestTruthExpectationElement(&c->postcondition_beliefs);
            Implication updated = Inference_AssumptionOfFailure(&postcon);
            Table_Add(&c->postcondition_beliefs, &updated);
        }
    }
    if(c->precondition_beliefs.itemsAmount>0)
    {
        Implication precon = c->precondition_beliefs.array[0];
        if(!Stamp_checkOverlap(&e->stamp, &precon.stamp))
        {
            Event res = e->type == EVENT_TYPE_BELIEF ? Inference_BeliefAbduction(e, &precon) : Inference_GoalDeduction(e, &precon);
            res.attention = Attention_deriveEvent(&c->attention, &precon.truth);
            Memory_addEvent(&res);
        }
    }
}

void cycle(long currentTime)
{
    for(int i=0; i<EVENT_SELECTIONS; i++)
    {
        //1. get an event from the event queue
        Item item = PriorityQueue_PopMax(&events);
        Event *e = item.address;
        //determine the concept it is related to
        int closest_concept_i = Memory_getClosestConcept(e);
        if(closest_concept_i != MEMORY_MATCH_NO_CONCEPT)
        {
            Concept *c = concepts.items[closest_concept_i].address;
            c->usage = Usage_use(&c->usage, currentTime);
            Truth matchTruth = SDR_Inheritance(&e->sdr, &c->sdr);
            Event eMatch = *e;
            eMatch.truth = Truth_Revision(e->truth, matchTruth);
            //apply decomposition-based inference: prediction/explanation
            decomposition(c, &eMatch);
            //add event to the FIFO of the concept
            FIFO *fifo =  e->type == EVENT_TYPE_BELIEF ? &c->event_beliefs : &c->event_goals;
            Event revised = FIFO_AddAndRevise(&eMatch, fifo);
            if(revised.type != EVENT_TYPE_DELETED)
            {
                Memory_addEvent(&revised);
            }
            //relatively forget the event, as it was used, and add back to events
            e->attention = Attention_forgetEvent(&e->attention);
            Memory_addEvent(e);
            //trigger composition-based inference hypothesis formation
            Item selectedItem[CONCEPT_SELECTIONS];
            for(int j=0; j<CONCEPT_SELECTIONS; j++)
            {
                selectedItem[j] = PriorityQueue_PopMax(&concepts);
                composition(c, selectedItem[j].address, &eMatch); // deriving a =/> b
            }
            for(int j=0; j<CONCEPT_SELECTIONS; j++)
            {
                PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&concepts, selectedItem[j].priority);
                feedback.addedItem.address = selectedItem[j].address;
            }
            //activate concepts attention with the event's attention
            c->attention = Attention_activateConcept(&c->attention, &e->attention); 
            PriorityQueue_IncreasePriority(&concepts,closest_concept_i, c->attention.priority); //priority was increased
        }
        //add a new concept for e too at the end, just before it needs to be identified with something existing
        Concept *eNativeConcept = Memory_addConcept(&e->sdr, e->attention);
        FIFO_Add(e, (e->type == EVENT_TYPE_BELIEF ? &eNativeConcept->event_beliefs : &eNativeConcept->event_goals));
    }
    //relative forget concepts:
    for(int i=0; i<concepts.itemsAmount; i++) //as all concepts are forgotten the order won't change
    { //making this operation very cheap, not demanding any heap operation, except for these items that fall below USEFULNESS_MAX_PRIORITY_BARRIER
        Concept *c = concepts.items[i].address;
        c->attention = Attention_forgetConcept(&c->attention, &c->usage, currentTime);
        concepts.items[i].priority = c->attention.priority;
        //deal with usefulness changing the order of the else strictly monotonous forgetting
        if(c->attention.priority < USEFULNESS_MAX_PRIORITY_BARRIER)
        {
            Item it = PriorityQueue_PopAt(&concepts, i);
            PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&concepts, c->attention.priority);
            feedback.addedItem.address = c;
        }
    }
}
