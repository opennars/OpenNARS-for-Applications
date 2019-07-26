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
            c->incoming_goal_spike = eMatch;
        }
    }
    return eMatch;
}

static Event ProcessEvent(Event *e, long currentTime)
{
    e->processed = true;
    Event_SetSDR(e, e->sdr); // TODO make sure that hash needs to be calculated once instead already
    IN_DEBUG( puts("Event was selected:"); Event_Print(e); )
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
        //if different enough
        bool different_enough = true;
        if(c != NULL)
        {
            double novelty = 1.0 - Truth_Expectation(SDR_Similarity(&e->sdr, &eMatch.sdr));
            if(novelty < CONCEPT_FORMATION_NOVELTY)
            {
                different_enough = false;
            }
        }
        if(different_enough)
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
            puts("");
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
                    RuleTable_Composition(precondition, &postcondition, operationID);
                }
            }
        }
    }
    //process goals
    if(goal_events.itemsAmount > 0)
    {
        Event *goal = FIFO_GetNewestElement(&goal_events);
        if(!goal->processed)
        {
            ProcessEvent(goal, currentTime);
            Decision_Making(goal, currentTime);
            goal->processed = true;
        }
    }
    //pass goal spikes on to the next
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        Concept *c = concepts.items[i].address;
        if(c->goal_spike.type != EVENT_TYPE_DELETED)
        {
            for(int opi=0; opi<OPERATIONS_MAX; opi++)
            {
                for(int j=0; j<c->precondition_beliefs[opi].itemsAmount; j++)
                {
                    //todo better handling as spikes could arrive at the same time, overriding each other, maybe same handling as before?
                    SDR *preSDR = &c->precondition_beliefs[opi].array[j].sdr;
                    int closest_concept_i;
                    if(Memory_getClosestConcept(preSDR, SDR_Hash(preSDR), &closest_concept_i)) //todo cache it, maybe in the function
                    {
                       Concept *pre = concepts.items[closest_concept_i].address;
                       pre->incoming_goal_spike = Inference_GoalDeduction(&c->goal_spike, &c->precondition_beliefs[opi].array[j]);
                    }
                }
            }
        }
    }
    //process incoming goal spikes, invoking potential operations
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        Concept *c = concepts.items[i].address;
        if(c->incoming_goal_spike.type != EVENT_TYPE_DELETED)
        {
            bool processGoalSpike = false;
            if(c->goal_spike.type == EVENT_TYPE_DELETED)
            {
                c->goal_spike = c->incoming_goal_spike;
                processGoalSpike = true;
            }
            else
            {
                double expExisting = Truth_Expectation(Truth_Projection(c->goal_spike.truth, c->goal_spike.occurrenceTime, currentTime));
                double expIncoming = Truth_Expectation(Truth_Projection(c->incoming_goal_spike.truth, c->incoming_goal_spike.occurrenceTime, currentTime));
                //check if there is evidental overlap
                bool overlap = Stamp_checkOverlap(&c->incoming_goal_spike.stamp, &c->goal_spike.stamp);
                //if there is, apply choice, keeping the stronger one:
                if(overlap)
                {
                    if(expIncoming > expExisting)
                    {
                        c->goal_spike = c->incoming_goal_spike;
                        processGoalSpike = true;
                    }
                }
                else
                //and else revise, increasing the "activation potential"
                {
                    c->goal_spike = Inference_EventRevision(&c->goal_spike, &c->incoming_goal_spike);
                    processGoalSpike = true;
                }
            }
            if(processGoalSpike && !c->goal_spike.processed)
            {
                c->goal_spike.processed = true;
                if(Decision_Making(&c->goal_spike, currentTime))
                {
                    c->goal_spike = (Event) {0}; //don't propagate further
                }
            }
        }
        c->incoming_goal_spike = (Event) {0};
    }

    //TODO same for belief spikes, but no triggering of decision making, just updating beliefSpike with same policy
    //This will allow it to be better prepared for the future (for instance when observations are missing), though for simple experiments it might not make much difference,
    //but without goal spikes, multistep procedure learning is difficult
    //Re-sort queue
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        Concept *concept = concepts.items[i].address;
        //usefulness was changed, 
        PriorityQueue_PopAt(&concepts, i, NULL);
        Memory_addConcept(concept, currentTime);
    }
}
