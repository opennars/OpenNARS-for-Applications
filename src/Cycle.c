#include "Cycle.h"

//doing inference within the matched concept, returning whether decisionMaking should continue
static bool Cycle_ActivateConcept(int layer, Concept *c, Event *e, long currentTime, bool decisionMaking)
{
    //Matched event, see https://github.com/patham9/ANSNA/wiki/SDR:-SDRInheritance-for-matching,-and-its-truth-value
    Event eMatch = Memory_MatchEventToConcept(c, e);
    if(eMatch.truth.confidence > MIN_CONFIDENCE)
    {
        Concept_SDRInterpolation(c, &e->sdr, eMatch.truth); 
        c->usage = Usage_use(&c->usage, currentTime);          //given its new role it should be doable to add a priorization mechanism to it
        //add event as spike to the concept:
        if(eMatch.type == EVENT_TYPE_BELIEF)
        {
            c->belief_spike = eMatch;
        }
        else
        if(decisionMaking)
        {
            //pass spike if the concept doesn't have a satisfying motor command
            if(!Decision_Making(layer, &eMatch, currentTime))
            {
                c->incoming_goal_spike = eMatch;
            }
            else
            {
                e->propagated = true;
                return false; //already invoked, no need to pass further up the hierarchy
            }
        }
    }
    return true;
}

//Process an event, by creating a concept, or activating an existing
static void Cycle_ProcessEvent(Event *e, long currentTime)
{
    e->processed = true;
    Event_SetSDR(e, e->sdr); // TODO make sure that hash needs to be calculated once instead already
    IN_DEBUG( puts("Event was selected:"); Event_Print(e); )
    bool decisionMaking = true;
    for(int l=0; l<CONCEPT_LAYERS; l++)
    {
        //determine the concept it is related to
        int closest_concept_i;
        Concept *c = NULL;
        if(Memory_getClosestConcept(l, &e->sdr, e->sdr_hash, &closest_concept_i))
        {
            c = concepts[l].items[closest_concept_i].address;
            decisionMaking &= Cycle_ActivateConcept(l, c, e, currentTime, decisionMaking);
        }
    }
    //add a new concept for e too at the end (in all layers)
    Memory_Conceptualize(&e->sdr);
}

//Propagate spikes for subgoal processing, generating anticipations and decisions
static void Cycle_PropagateSpikes(long currentTime)
{
    //process spikes
    if(PROPAGATE_GOAL_SPIKES)
    {
        //pass goal spikes on to the next
        for(int l=0; l<CONCEPT_LAYERS; l++)
        {
            for(int i=0; i<concepts[l].itemsAmount; i++)
            {
                Concept *postc = concepts[l].items[i].address;
                if(postc->goal_spike.type != EVENT_TYPE_DELETED && !postc->goal_spike.propagated && Truth_Expectation(postc->goal_spike.truth) > PROPAGATION_THRESHOLD)
                {
                    for(int opi=0; opi<OPERATIONS_MAX; opi++)
                    {
                        for(int j=0; j<postc->precondition_beliefs[opi].itemsAmount; j++)
                        {
                            Implication *imp = &postc->precondition_beliefs[opi].array[j];
                            Relink_Implication(l, imp);
                            Concept *pre = imp->sourceConcept;
                            if(pre->incoming_goal_spike.type == EVENT_TYPE_DELETED || pre->incoming_goal_spike.processed)
                            {
                                pre->incoming_goal_spike = Inference_GoalDeduction(&postc->goal_spike, &postc->precondition_beliefs[opi].array[j]);
                            }
                        }
                    }
                }
                postc->goal_spike.propagated = true;
            }
        }
        //process incoming goal spikes, invoking potential operations
        for(int l=0; l<CONCEPT_LAYERS; l++)
        {
            for(int i=0; i<concepts[l].itemsAmount; i++)
            {
                Concept *c = concepts[l].items[i].address;
                if(c->incoming_goal_spike.type != EVENT_TYPE_DELETED)
                {
                    c->goal_spike = Inference_IncreasedActionPotential(&c->goal_spike, &c->incoming_goal_spike, currentTime);
                    if(c->goal_spike.type != EVENT_TYPE_DELETED && !c->goal_spike.processed && Truth_Expectation(c->goal_spike.truth) > PROPAGATION_THRESHOLD)
                    {
                        Cycle_ProcessEvent(&c->goal_spike, currentTime);
                    }
                }
                c->incoming_goal_spike = (Event) {0};
            }
        }
    }
}

//Reinforce link between concept a and b (creating it if non-existent)
static void Cycle_ReinforceLink(Event *a, Event *b, int operationID)
{
    if(a->type != EVENT_TYPE_BELIEF || b->type != EVENT_TYPE_BELIEF)
    {
        return;
    }
    for(int l=0; l<CONCEPT_LAYERS; l++)
    {
        int AConceptIndex;
        int BConceptIndex;
        if(Memory_getClosestConcept(l, &a->sdr, a->sdr_hash, &AConceptIndex) &&
           Memory_getClosestConcept(l, &b->sdr, b->sdr_hash, &BConceptIndex))
        {
            Concept *A = concepts[l].items[AConceptIndex].address;
            Concept *B = concepts[l].items[BConceptIndex].address;
            if(A != B)
            {
                //temporal induction
                if(!Stamp_checkOverlap(&a->stamp, &b->stamp))
                {
                    Implication precondition_implication = Inference_BeliefInduction(a, b);
                    precondition_implication.sourceConcept = A;
                    precondition_implication.sourceConceptSDR = A->sdr;
                    if(precondition_implication.truth.confidence >= MIN_CONFIDENCE)
                    {
                        char debug[200];
                        sprintf(debug, "<(&/,%s,^op%d(),+%ld) =/> %s>.",A->debug, operationID,precondition_implication.occurrenceTimeOffset ,B->debug);
                        IN_DEBUG ( if(operationID != 0) { puts(debug); Truth_Print(&precondition_implication.truth); puts("\n"); getchar(); } )
                        IN_OUTPUT( fputs("Formed implication: ", stdout); Implication_Print(&precondition_implication); )
                        Implication *revised_precon = Table_AddAndRevise(&B->precondition_beliefs[operationID], &precondition_implication, debug);
                        if(revised_precon != NULL)
                        {
                            revised_precon->sourceConcept = A;
                            revised_precon->sourceConceptSDR = A->sdr;
                            IN_OUTPUT( if(revised_precon->sdr_hash != 0) { fputs("REVISED pre-condition implication: ", stdout); Implication_Print(revised_precon); } )
                        }
                    }
                }
            }
        }
    }
}

void Cycle_Perform(long currentTime)
{   
    //1. process newest event
    if(belief_events.itemsAmount > 0)
    {
        //form concepts for the sequences of different length
        for(int len=0; len<MAX_SEQUENCE_LEN; len++)
        {
            Event *toProcess = FIFO_GetNewestSequence(&belief_events, len);
            if(!toProcess->processed)
            {
                Cycle_ProcessEvent(toProcess, currentTime);
                Event postcondition = *toProcess;
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
                                            Cycle_ReinforceLink(precondition, &postcondition, operationID);
                                        }
                                    }
                                }
                            }
                            else
                            {
                                Cycle_ReinforceLink(precondition, &postcondition, operationID);
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
            Cycle_ProcessEvent(goal, currentTime);
        }
    }
    Cycle_PropagateSpikes(currentTime);
    //Re-sort queue
    for(int l=0; l<CONCEPT_LAYERS; l++)
    {
        for(int i=0; i<concepts[l].itemsAmount; i++)
        {
            Concept *concept = concepts[l].items[i].address;
            //usefulness was changed, push back if it didn't get void by interpolation
            PriorityQueue_PopAt(&concepts[l], i, NULL);
            if(SDR_CountTrue(&concept->sdr) > 0)
            {
                Memory_addConcept(l, concept, currentTime);
            }
        }
    }
}
