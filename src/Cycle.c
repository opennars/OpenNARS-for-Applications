#include "Cycle.h"

int eventsSelected = 0, eventsDerived = 0;
Event selectedEvents[EVENT_SELECTIONS]; //better to be global
Concept selectedConcepts[CONCEPT_SELECTIONS]; //too large to be a local array
Event derivations[MAX_DERIVATIONS];

//doing inference within the matched concept, returning the matched event
Event localInference(Concept *c, int closest_concept_i, Event *e, long currentTime)
{
    //Matched event, see https://github.com/patham9/ANSNA/wiki/SDR:-SDRInheritance-for-matching,-and-its-truth-value
    Event eMatch = *e;
    eMatch.sdr = c->sdr;
    eMatch.truth = Truth_Deduction(SDR_Inheritance(&e->sdr, &c->sdr), e->truth);
    if(eMatch.truth.confidence > MIN_CONFIDENCE)
    {
        Concept_SDRInterpolation(c, &e->sdr, eMatch.truth); 
        //apply decomposition-based inference: prediction/explanation
        //RuleTable_Decomposition(c, &eMatch, currentTime); <- TODO, how to deal with derived events? I guess FIFO will need to support it
        c->usage = Usage_use(&c->usage, currentTime);          //given its new role it should be doable to add a priorization mechanism to it
        //activate concepts attention with the event's attention
        c->attention = Attention_activateConcept(&c->attention, &eMatch.attention); 
        PriorityQueue_IncreasePriority(&concepts, closest_concept_i, c->attention.priority); //priority was increased
    }
    return eMatch;
}

void Cycle_Perform(long currentTime)
{
    //1. process newest event
    Event e = FIFO_GetNewestElement(&belief_events);
    if(!e.processed)
    {
        e.processed = true;
        Event_SetSDR(&e, e.sdr); // TODO make sure that hash needs to be calculated once instead already
        IN_DEBUG( printf("Event was selected:\n"); Event_Print(&e); )
        //determine the concept it is related to
        int closest_concept_i;
        Concept *c = NULL;
        if(Memory_getClosestConcept(&e, &closest_concept_i))
        {
            c = concepts.items[closest_concept_i].address;
            //perform concept-related inference
            localInference(c, closest_concept_i, &e, currentTime);
        }
        if(!Memory_FindConceptBySDR(&e.sdr, e.sdr_hash, NULL))
        {   
            //add a new concept for e too at the end, as it does not exist already
            Concept *eNativeConcept = Memory_Conceptualize(&e.sdr, e.attention);
            if(eNativeConcept != NULL && c != NULL)
            {
                //copy over all knowledge
                for(int i=0; i<OPERATIONS_MAX; i++)
                {
                    Table_COPY(&c->precondition_beliefs[i],  &eNativeConcept->precondition_beliefs[i]);
                    Table_COPY(&c->postcondition_beliefs[i], &eNativeConcept->postcondition_beliefs[i]);
                }
            }
        }
        //Mine for <(&/,precondition,operation) =/> postcondition> patterns in the FIFO:
        for(int k=0; k<belief_events.itemsAmount; k++)
        {
            Event postcondition = FIFO_GetKthNewestElement(&belief_events, k);     //todo: get something better involving derived events           
            int k2 = k+1;                                                         //to fill in gaps in observations with abduction and also
            if(k2 >= belief_events.itemsAmount || postcondition.operationID != 0) //to support sequences.
            {
                break;
            }
            Event precondition = FIFO_GetKthNewestElement(&belief_events, k2);
            int operationID = 0;
            //if it's an operation find the real precondition and use the current one as action
            if(precondition.operationID != 0)
            {
                int k3 = k+2;
                if(k3>=belief_events.itemsAmount)
                {
                    break;
                }
                operationID = precondition.operationID;
                precondition = FIFO_GetKthNewestElement(&belief_events, k3);
                if(postcondition.operationID != 0)
                {
                    break;
                }
            }
            int preconditionConceptIndex = 0;
            int postconditionConceptIndex = 0;
            if(Memory_getClosestConcept(&precondition,  &preconditionConceptIndex) &&
               Memory_getClosestConcept(&postcondition, &postconditionConceptIndex))
            {
                Concept *preconditionConcept = concepts.items[preconditionConceptIndex].address;
                Concept *postConditionConcept = concepts.items[postconditionConceptIndex].address;
                RuleTable_Composition(preconditionConcept, postConditionConcept, &precondition, &postcondition, operationID, currentTime);
            }
            break; //not yet generalized, we just mine consequent ones so far not overlapping etc. ones
        }
    }
    //invoke decision making for goals
    Event goal = FIFO_GetNewestElement(&belief_events);
    if(!goal.processed)
    {
        goal.processed = true;
        Decision_Making(&e, currentTime);
    }
}
