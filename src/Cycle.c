#include "Cycle.h"

//doing inference within the matched concept, returning whether decisionMaking should continue
static Decision Cycle_ActivateConcept(Concept *c, Event *e, long currentTime)
{
    Decision decision = {0};
    Event eMatch = *e;
    if(eMatch.truth.confidence > MIN_CONFIDENCE)
    {
        c->usage = Usage_use(c->usage, currentTime);          //given its new role it should be doable to add a priorization mechanism to it
        //add event as spike to the concept:
        if(eMatch.type == EVENT_TYPE_BELIEF)
        {
            c->belief_spike = eMatch;
        }
        else
        {
            //pass spike if the concept doesn't have a satisfying motor command
            decision = Decision_Suggest(&eMatch, currentTime);
            if(!decision.execute)
            {
                c->incoming_goal_spike = eMatch;
            }
            else
            {
                e->propagated = true;
            }
        }
    }
    return decision;
}

//Process an event, by creating a concept, or activating an existing
static Decision Cycle_ProcessEvent(Event *e, long currentTime)
{
    Decision decision = {0};
    e->processed = true;
    Event_SetTerm(e, e->term); // TODO make sure that hash needs to be calculated once instead already
    IN_DEBUG( puts("Event was selected:"); Event_Print(e); )
    //determine the concept it is related to
    int closest_concept_i;
    Concept *c = NULL;
    if(Memory_FindConceptByTerm(&e->term, /*e->term_hash,*/ &closest_concept_i))
    {
        c = concepts.items[closest_concept_i].address;
        decision = Cycle_ActivateConcept(c, e, currentTime);
    }
    //add a new concept for e too at the end
    Memory_Conceptualize(&e->term);
    return decision;
}

//Propagate spikes for subgoal processing, generating anticipations and decisions
static Decision Cycle_PropagateSpikes(long currentTime)
{
    Decision decision = {0};
    //process spikes
    if(PROPAGATE_GOAL_SPIKES)
    {
        //pass goal spikes on to the next
        for(int i=0; i<concepts.itemsAmount; i++)
        {
            Concept *postc = concepts.items[i].address;
            if(postc->goal_spike.type != EVENT_TYPE_DELETED && !postc->goal_spike.propagated && Truth_Expectation(postc->goal_spike.truth) > PROPAGATION_THRESHOLD)
            {
                for(int opi=0; opi<OPERATIONS_MAX; opi++)
                {
                    for(int j=0; j<postc->precondition_beliefs[opi].itemsAmount; j++)
                    {
                        Implication *imp = &postc->precondition_beliefs[opi].array[j];
                        if(!Memory_ImplicationValid(imp))
                        {
                            Table_Remove(&postc->precondition_beliefs[opi], j);
                            j--;
                            continue;
                        }
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
        //process incoming goal spikes, invoking potential operations
        for(int i=0; i<concepts.itemsAmount; i++)
        {
            Concept *c = concepts.items[i].address;
            if(c->incoming_goal_spike.type != EVENT_TYPE_DELETED)
            {
                c->goal_spike = Inference_IncreasedActionPotential(&c->goal_spike, &c->incoming_goal_spike, currentTime);
                if(c->goal_spike.type != EVENT_TYPE_DELETED && !c->goal_spike.processed && Truth_Expectation(c->goal_spike.truth) > PROPAGATION_THRESHOLD)
                {
                    Decision decision = Cycle_ProcessEvent(&c->goal_spike, currentTime);
                    if(decision.execute)
                    {
                        return decision;
                    }
                }
            }
            c->incoming_goal_spike = (Event) {0};
        }
    }
    return decision;
}

//Reinforce link between concept a and b (creating it if non-existent)
static void Cycle_ReinforceLink(Event *a, Event *b, int operationID)
{
    if(a->type != EVENT_TYPE_BELIEF || b->type != EVENT_TYPE_BELIEF)
    {
        return;
    }
    int AConceptIndex;
    int BConceptIndex;
    if(Memory_FindConceptByTerm(&a->term, /*a->term_hash,*/ &AConceptIndex) &&
       Memory_FindConceptByTerm(&b->term, /*b->term_hash,*/ &BConceptIndex))
    {
        Concept *A = concepts.items[AConceptIndex].address;
        Concept *B = concepts.items[BConceptIndex].address;
        if(A != B)
        {
            //temporal induction
            if(!Stamp_checkOverlap(&a->stamp, &b->stamp))
            {
                Implication precondition_implication = Inference_BeliefInduction(a, b);
                precondition_implication.sourceConcept = A;
                precondition_implication.sourceConceptTerm = A->term;
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
                        revised_precon->sourceConceptTerm = A->term;
                        /*IN_OUTPUT( if(true && revised_precon->term_hash != 0) { fputs("REVISED pre-condition implication: ", stdout); Implication_Print(revised_precon); } ) */
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
            if(toProcess != NULL && !toProcess->processed)
            {
                Cycle_ProcessEvent(toProcess, currentTime);
                Event postcondition = *toProcess;
                Decision_AssumptionOfFailure(postcondition.operationID, currentTime); //collection of negative evidence, new way
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
                            if(precondition != NULL)
                            {
                                //if it's an operation find the real precondition and use the current one as action
                                int operationID = precondition->operationID;
                                if(operationID != 0) //also meaning len2==0
                                {
                                    for(int j=k+1; j<belief_events.itemsAmount; j++)
                                    {
                                        for(int len3=0; len3<MAX_SEQUENCE_LEN; len3++)
                                        {
                                            precondition = FIFO_GetKthNewestSequence(&belief_events, j, len3);
                                            if(precondition != NULL && precondition->operationID == 0)
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
    }
    //process goals
    Decision decision[PROPAGATION_ITERATIONS + 1] = {0};
    if(goal_events.itemsAmount > 0)
    {
        Event *goal = FIFO_GetNewestSequence(&goal_events, 0);
        if(!goal->processed)
        {
            decision[0] = Cycle_ProcessEvent(goal, currentTime);
            //allow reasoning into the future by propagating spikes from goals back to potential current evens
            for(int i=0; i<PROPAGATION_ITERATIONS; i++)
            {
                decision[i+1] = Cycle_PropagateSpikes(currentTime);
            }
        }
    }
    //inject the best action if there was one
    Decision best_decision = {0};
    for(int i=0; i<PROPAGATION_ITERATIONS+1; i++)
    {
        if(decision[i].execute && decision[i].desire >= best_decision.desire)
        {
            best_decision = decision[i];
        }
    }
    if(best_decision.execute && best_decision.operationID > 0)
    {
        Decision_Execute(&best_decision);
    }
    //end of iterations, remove spikes
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        Concept *c = concepts.items[i].address;
        c->incoming_goal_spike = (Event) {0};
        c->goal_spike = (Event) {0};
    }
    //Re-sort queue
    PriorityQueue_Rebuild(&concepts);
}
