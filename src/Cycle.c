#include "Cycle.h"

int eventsSelected = 0, eventsDerived = 0;
Event selectedEvents[EVENT_SELECTIONS]; //better to be global
Concept selectedConcepts[CONCEPT_SELECTIONS]; //too large to be a local array
Event derivations[MAX_DERIVATIONS];

//doing inference within the matched concept, returning the matched event
Event localInference(Concept *c, int closest_concept_i, Event *e, long currentTime)
{
    printf("PROCESSING _EVENT1 %ld\n",e->stamp.evidentalBase[0]); //++
    //Matched event, see https://github.com/patham9/ANSNA/wiki/SDR:-SDRInheritance-for-matching,-and-its-truth-value
    Event eMatch = *e;
    if(c == NULL)
    {
        exit(0);
    }
    eMatch.sdr = c->sdr;
    printf("PROCESSING _EVENT1 %ld\n",e->stamp.evidentalBase[0]); //++
    eMatch.truth = Truth_Deduction(SDR_Inheritance(&e->sdr, &c->sdr), e->truth);
    if(eMatch.truth.confidence > MIN_CONFIDENCE)
    {
        printf("PROCESSING _EVENT2 %ld\n",e->stamp.evidentalBase[0]); //++
        Concept_SDRInterpolation(c, &e->sdr, eMatch.truth); 
        //apply decomposition-based inference: prediction/explanation
        //RuleTable_Decomposition(c, &eMatch, currentTime); <- TODO, how to deal with derived events? I guess FIFO will need to support it
        c->usage = Usage_use(&c->usage, currentTime);          //given its new role it should be doable to add a priorization mechanism to it
        //add event to the FIFO of the concept
        printf("PROCESSING _EVENT3 %ld\n",e->stamp.evidentalBase[0]); //++
        FIFO *fifo =  e->type == EVENT_TYPE_BELIEF ? &c->event_beliefs : &c->event_goals;
        FIFO_AddAndRevise(&eMatch, fifo);
        //activate concepts attention with the event's attention
        c->attention = Attention_activateConcept(&c->attention, &eMatch.attention); 
        PriorityQueue_IncreasePriority(&concepts, closest_concept_i, c->attention.priority); //priority was increased
    }
    return eMatch;
}

int u=0; //++
void ProcessEvent(Event *e, long currentTime)
{
    u++; //++
    printf("PROCESSING EVENT %ld\n",e->stamp.evidentalBase[0]); //++
    
    e->processed = true;
    Event_SetSDR(e, e->sdr); // TODO make sure that hash needs to be calculated once instead already
    IN_DEBUG( printf("Event was selected:\n"); Event_Print(e); )
    //determine the concept it is related to
    int closest_concept_i;
    Concept *c = NULL;
    if(Memory_getClosestConcept(&e->sdr, e->sdr_hash, &closest_concept_i))
    {
        printf("PROCESSING EVENT1 %ld\n",e->stamp.evidentalBase[0]); //++
        c = concepts.items[closest_concept_i].address;
        //perform concept-related inference
        localInference(c, closest_concept_i, e, currentTime);
    }
    printf("PROCESSING EVENT_ %ld\n",e->stamp.evidentalBase[0]); //++
    if(!Memory_FindConceptBySDR(&e->sdr, e->sdr_hash, NULL))
    {   
        printf("PROCESSING EVENT2 %ld\n",e->stamp.evidentalBase[0]); //++
        //add a new concept for e too at the end, as it does not exist already
        Concept *eNativeConcept = Memory_Conceptualize(&e->sdr, e->attention);
        if(eNativeConcept != NULL && c != NULL)
        {
            //copy over all knowledge
            for(int i=0; i<OPERATIONS_MAX; i++)
            {
                Table_COPY(&c->precondition_beliefs[i],  &eNativeConcept->precondition_beliefs[i]);
                Table_COPY(&c->postcondition_beliefs[i], &eNativeConcept->postcondition_beliefs[i]);
            }
            FIFO_COPY(&c->event_beliefs, &eNativeConcept->event_beliefs);
            FIFO_COPY(&c->event_goals, &eNativeConcept->event_goals);
        }
    }
}

void Cycle_Perform(long currentTime)
{
    //printf("CYCLE\n"); //++
    //1. process newest event
    if(belief_events.itemsAmount > 0)
    {
        //printf("has beliefs\n"); //++
        Event *e = FIFO_GetNewestElement(&belief_events);
        if(!e->processed)
        {
            printf("has not processed\n"); //++
            ProcessEvent(e, currentTime);
            printf("processed belief\n"); //++
            //Mine for <(&/,precondition,operation) =/> postcondition> patterns in the FIFO:
            for(int k=0; k<belief_events.itemsAmount; k++)
            {
                Event *postcondition = FIFO_GetKthNewestElement(&belief_events, k);     //todo: get something better involving derived events           
                printf("FIRST EVENT %ld\n",postcondition->stamp.evidentalBase[0]); //++
                int k2 = k+1;                                                         //to fill in gaps in observations with abduction and also
                if(k2 >= belief_events.itemsAmount || postcondition->operationID != 0) //to support sequences, use "standard ANSNA approach"
                {
                    break;
                }
                Event *precondition = FIFO_GetKthNewestElement(&belief_events, k2);
                int operationID = 0;
                //if it's an operation find the real precondition and use the current one as action
                if(precondition->operationID != 0)
                {
                    //printf("BREAKP op\n"); //++
                    int k3 = k+2;
                    if(k3>=belief_events.itemsAmount)
                    {
                        break;
                    }
                    operationID = precondition->operationID;
                    precondition = FIFO_GetKthNewestElement(&belief_events, k3);
                    if(precondition->operationID != 0)
                    {
                        break;
                    }
                }
                int preconditionConceptIndex;
                int postconditionConceptIndex;
                if(Memory_getClosestConcept(&precondition->sdr,  precondition->sdr_hash,  &preconditionConceptIndex) &&
                   Memory_getClosestConcept(&postcondition->sdr, postcondition->sdr_hash, &postconditionConceptIndex))
                {
                    Concept *preconditionConcept = concepts.items[preconditionConceptIndex].address;
                    Concept *postConditionConcept = concepts.items[postconditionConceptIndex].address;
                    if(operationID == 0)
                    {
                        break; //mining only (&/,a,op()) =/> b for now!
                    }
                    RuleTable_Composition(preconditionConcept, postConditionConcept, precondition, postcondition, operationID, currentTime);
                }
                break; //not yet generalized, we just mine consequent ones so far not overlapping etc. ones
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
}
