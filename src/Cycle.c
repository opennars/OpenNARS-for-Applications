#include "Cycle.h"

static Event Concept_MatchEventToConcept(Concept *c, Event *e)
{
    Event eMatch = *e;
    eMatch.sdr = c->sdr;
    eMatch.truth = Truth_Deduction(SDR_Inheritance(&e->sdr, &c->sdr), e->truth);
    return eMatch;
}

//doing inference within the matched concept, returning the matched event
Event Concept_LocalInference(Concept *c, Event *e, long currentTime)
{
    Concept_ConfirmAnticipation(c, e);
    //Matched event, see https://github.com/patham9/ANSNA/wiki/SDR:-SDRInheritance-for-matching,-and-its-truth-value
    Event eMatch = Concept_MatchEventToConcept(c, e);
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
            //pass spike if the concept doesn't have a satisfying motor command
            if(!Decision_Making(&eMatch, currentTime))
            {
                c->incoming_goal_spike = eMatch;
            }
            else
            {
                e->propagated = true;
            }
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
        eMatch = Concept_LocalInference(c, e, currentTime);
    }
    if(Memory_EventIsNovel(e, c))
    {
        //add a new concept for e too at the end, as it does not exist already
        Concept *specialConcept = Memory_Conceptualize(&e->sdr);
        if(c != NULL && specialConcept != NULL && c != NULL)
        {
            //copy over all knowledge
            for(int i=0; i<OPERATIONS_MAX; i++)
            {
                Table_COPY(&c->precondition_beliefs[i],  &specialConcept->precondition_beliefs[i]);
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
            Event *ev = FIFO_GetKthNewestSequence(&belief_events, i, 0);
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
        //form concepts for the sequences of different length
        for(int len=0; len<MAX_SEQUENCE_LEN; len++)
        {
            Event *toProcess = FIFO_GetNewestSequence(&belief_events, len);
            if(!toProcess->processed)
            {
                //the matched event becomes the postcondition
                Event postcondition = ProcessEvent(toProcess, currentTime);
                //Mine for <(&/,precondition,operation) =/> postcondition> patterns in the FIFO:
                if(len == 0) //postcondition always len1
                {  
                    if(postcondition.operationID != 0)
                    {
                        return;
                    }
                    for(int k=1; k<belief_events.itemsAmount; k++)
                    {
                        for(int len2=0; len2<MAX_SEQUENCE_LEN; len2++)
                        {
                            Event *precondition = FIFO_GetKthNewestSequence(&belief_events, k, len2);
                            //if it's an operation find the real precondition and use the current one as action
                            int operationID = precondition->operationID;
                            if(operationID != 0) //also meaning len2==0
                            {
                                for(int j=k+1; j<belief_events.itemsAmount; j++)
                                {
                                    for(int len3=0; len3<MAX_SEQUENCE_LEN; len3++)
                                    {
                                        precondition = FIFO_GetKthNewestSequence(&belief_events, j, len3);
                                        if(precondition->operationID == 0)
                                        {
                                            RuleTable_Composition(precondition, &postcondition, operationID);
                                        }
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
            }
        }
    }
    //process goals
    if(goal_events.itemsAmount > 0)
    {
        Event *goal = FIFO_GetNewestSequence(&goal_events, 0);
        if(!goal->processed)
        {
            ProcessEvent(goal, currentTime);
        }
    }
    //process spikes
    if(PROPAGATE_GOAL_SPIKES)
    {
        //pass goal spikes on to the next
        for(int i=0; i<concepts.itemsAmount; i++)
        {
            Concept *c = concepts.items[i].address;
            if(c->goal_spike.type != EVENT_TYPE_DELETED && !c->goal_spike.propagated && Truth_Expectation(c->goal_spike.truth) > PROPAGATION_TRUTH_EXPECTATION_THRESHOLD)
            {
                for(int opi=0; opi<OPERATIONS_MAX; opi++)
                {
                    for(int j=0; j<c->precondition_beliefs[opi].itemsAmount; j++)
                    {
                        Implication *imp = &c->precondition_beliefs[opi].array[j];
                        Concept *pre = imp->sourceConcept; //concepts.items[closest_concept_i].address; //TODO check if still the same concept!
                        if((pre->incoming_goal_spike.type == EVENT_TYPE_DELETED || pre->incoming_goal_spike.processed) && SDR_Equal(&pre->sdr, &imp->sourceConceptSDR))
                        {
                            pre->incoming_goal_spike = Inference_GoalDeduction(&c->goal_spike, &c->precondition_beliefs[opi].array[j]);
                        }
                    }
                }
            }
            c->goal_spike.propagated = true;
        }
        //process incoming goal spikes, invoking potential operations
        for(int i=0; i<concepts.itemsAmount; i++)
        {
            Concept *c = concepts.items[i].address;
            if(c->incoming_goal_spike.type != EVENT_TYPE_DELETED)
            {
                c->goal_spike = Inference_IncreasedActionPotential(&c->goal_spike, &c->incoming_goal_spike, currentTime);
                if(c->goal_spike.type != EVENT_TYPE_DELETED && !c->goal_spike.processed && Truth_Expectation(c->goal_spike.truth) > PROPAGATION_TRUTH_EXPECTATION_THRESHOLD)
                {
                    ProcessEvent(&c->goal_spike, currentTime);
                }
            }
            c->incoming_goal_spike = (Event) {0};
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
