#include "Cycle.h"

static Event MatchEventToConcept(Concept *c, Event *e)
{
    Event eMatch = *e;
    eMatch.sdr = c->sdr;
    eMatch.truth = Truth_Deduction(SDR_Inheritance(&e->sdr, &c->sdr), e->truth);
    return eMatch;
}

//doing inference within the matched concept, returning the matched event
static Event LocalInference(Concept *c, Event *e, long currentTime)
{
    Concept_ConfirmAnticipation(c, e);
    //Matched event, see https://github.com/patham9/ANSNA/wiki/SDR:-SDRInheritance-for-matching,-and-its-truth-value
    strcpy(c->debug, e->debug);
    Event eMatch = MatchEventToConcept(c, e);
    if(eMatch.truth.confidence > MIN_CONFIDENCE)
    {
        c->usage = Usage_use(&c->usage, currentTime);          //given its new role it should be doable to add a priorization mechanism to it
        //add event as spike to the concept:
        if(eMatch.type == EVENT_TYPE_BELIEF)
        {
            c->belief_spike = eMatch;
        }
        else
        {
            c->goal_spike = eMatch;
        }
    }
    return eMatch;
}

static Event ProcessEvent(Event *e, long currentTime)
{
    e->processed = true;
    Event_SetSDR(e, e->sdr); // TODO make sure that hash needs to be calculated once instead already
    IN_DEBUG( printf("Event was selected:\n"); Event_Print(e); )
    //determine the concept it is related to
    int closest_concept_i;
    Concept *c = NULL;
    Event eMatch = {0};
    if(Memory_getClosestConcept(&e->sdr, e->sdr_hash, &closest_concept_i))
    {
        c = concepts.items[closest_concept_i].address;
        //perform concept-related inference
        eMatch = LocalInference(c, e, currentTime);
    }
    if(!Memory_FindConceptBySDR(&e->sdr, e->sdr_hash, NULL))
    {   
        //add a new concept for e too at the end, as it does not exist already
        Concept *specialConcept = Memory_Conceptualize(&e->sdr);
        if(specialConcept != NULL && c != NULL)
        {
            //copy over all knowledge
            for(int i=0; i<OPERATIONS_MAX; i++)
            {
                Table_COPY(&c->precondition_beliefs[i],  &specialConcept->precondition_beliefs[i]);
                Table_COPY(&c->postcondition_beliefs[i], &specialConcept->postcondition_beliefs[i]);
            }
        }
    }
    return eMatch;
}

void Cycle_Perform(long currentTime)
{    
    IN_DEBUG
    (
        for(int i=0; i<belief_events.itemsAmount; i++)
        {
            Event *ev = FIFO_GetKthNewestElement(&belief_events, i);
            puts(ev->debug);
            puts("\n");
        }
        printf("items amount: %d",belief_events.itemsAmount);
        getchar();
    )
    //process anticipation
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        Concept_CheckAnticipationDisappointment(concepts.items[i].address, currentTime);
    }
    //1. process newest event
    if(belief_events.itemsAmount > 0)
    {
        Event *toProcess = FIFO_GetNewestElement(&belief_events);
        if(!toProcess->processed)
        {
            //the matched event becomes the postcondition
            Event postcondition = ProcessEvent(toProcess, currentTime);
            //Mine for <(&/,precondition,operation) =/> postcondition> patterns in the FIFO:     
            if(postcondition.operationID != 0)
            {
                return;
            }
            for(int k=1; k<belief_events.itemsAmount; k++)
            {
                Event *precondition = FIFO_GetKthNewestElement(&belief_events, k);
                //if it's an operation find the real precondition and use the current one as action
                int operationID = precondition->operationID;
                if(operationID != 0)
                {
                    for(int j=k+1; j<belief_events.itemsAmount; j++)
                    {
                        precondition = FIFO_GetKthNewestElement(&belief_events, j);
                        if(precondition->operationID == 0)
                        {
                            RuleTable_Composition(precondition, &postcondition, operationID);
                        }
                    }
                }
                else
                {
                    //RuleTable_Composition(currentTime, precondition, &postcondition, operationID);
                }
            }
        }
    }
    //invoke decision making for goals
    if(goal_events.itemsAmount > 0)
    {
        Event *goal = FIFO_GetNewestElement(&goal_events);
        if(!goal->processed)
        {
            ProcessEvent(goal, currentTime);
            Decision_Making(goal, currentTime);
        }
    }
    //Re-sort queue
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        Concept *concept = concepts.items[i].address;
        //usefulness was changed, 
        PriorityQueue_PopAt(&concepts, i, NULL);
        Memory_addConcept(concept, currentTime);
    }
}
