#include "Cycle.h"

//doing inference within the matched concept, returning whether decisionMaking should continue
static Decision Cycle_ActivateConcept(Concept *c, Event *e, long currentTime)
{
    Decision decision = {0};
    Event eMatch = *e;
    if(eMatch.truth.confidence > MIN_CONFIDENCE)
    {
        c->usage = Usage_use(c->usage, currentTime);
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
    Decision best_decision = {0};
    //add a new concept for e if not yet existing
    Memory_Conceptualize(&e->term);
    e->processed = true;
    Event_SetTerm(e, e->term); // TODO make sure that hash needs to be calculated once instead already
    IN_DEBUG( puts("Event was selected:"); Event_Print(e); )
    //determine the concept it is related to
    Event ecp = *e;
    for(int concept_i=0; concept_i<concepts.itemsAmount; concept_i++)
    {
        Concept *c = concepts.items[concept_i].address;
        if(!Variable_hasVariable(&e->term, true, true, true))  //concept matched to the event which doesn't have variables
        {
            Substitution subs = Variable_Unify(&c->term, &e->term); //concept with variables, 
            if(subs.success)
            {
                ecp.term = e->term;
                Concept *c = concepts.items[concept_i].address;
                Decision decision = Cycle_ActivateConcept(c, &ecp, currentTime);
                if(decision.execute && decision.desire >= best_decision.desire)
                {
                    best_decision = decision;
                }
            }
        }
        else
        {
            Substitution subs = Variable_Unify(&e->term, &c->term); //event with variable matched to concept
            if(subs.success)
            {
                ecp.term = Variable_ApplySubstitute(e->term, subs);
                Concept *c = concepts.items[concept_i].address;
                Decision decision = Cycle_ActivateConcept(c, &ecp, currentTime);
                if(decision.execute && decision.desire >= best_decision.desire)
                {
                    best_decision = decision;
                }
            }
        }
    }
    return best_decision;
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
                        //no var, just send to source concept
                        if(!Variable_hasVariable(&imp->term, true, true, true))
                        {
                            Concept *pre = imp->sourceConcept;
                            if(pre->incoming_goal_spike.type == EVENT_TYPE_DELETED || pre->incoming_goal_spike.processed)
                            {
                                pre->incoming_goal_spike = Inference_GoalDeduction(&postc->goal_spike, imp);
                            }
                        }
                        //find proper source to send to!
                        else
                        {
                            assert(Narsese_copulaEquals(imp->term.atoms[0], '$'), "Not an implication!");
                            Term right_side = Term_ExtractSubterm(&imp->term, 2);
                            Substitution subs = Variable_Unify(&right_side, &postc->goal_spike.term);
                            assert(subs.success, "Implication and spike needs to be compatible!");
                            Term left_side_with_op = Term_ExtractSubterm(&imp->term, 1);
                            Term left_side = Narsese_GetPreconditionWithoutOp(&left_side_with_op);
                            Term left_side_substituted = Variable_ApplySubstitute(left_side, subs);
                            for(int concept_i=0; concept_i<concepts.itemsAmount; concept_i++)
                            {
                                Concept *pre = concepts.items[concept_i].address;
                                if(Variable_Unify(&pre->term, &left_side_substituted).success) //could be <a --> M>! matching to some <... =/> <$1 --> M>>.
                                {
                                    if(pre->incoming_goal_spike.type == EVENT_TYPE_DELETED || pre->incoming_goal_spike.processed)
                                    {
                                        pre->incoming_goal_spike = Inference_GoalDeduction(&postc->goal_spike, imp);
                                        pre->incoming_goal_spike.term = left_side_substituted; //set term as well, it's a specific goal now as it got specialized!
                                    }
                                }
                            }
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
                c->goal_spike = Inference_IncreasedActionPotential(&c->goal_spike, &c->incoming_goal_spike, currentTime, NULL);
                Memory_printAddedEvent(&c->goal_spike, 1, false, true, false);
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
static void Cycle_ReinforceLink(Event *a, Event *b)
{
    if(a->type != EVENT_TYPE_BELIEF || b->type != EVENT_TYPE_BELIEF)
    {
        return;
    }
    Term a_term_nop = Narsese_GetPreconditionWithoutOp(&a->term);
    int AConceptIndex;
    int BConceptIndex;
    if(Memory_FindConceptByTerm(&a_term_nop, /*a->term_hash,*/ &AConceptIndex) &&
       Memory_FindConceptByTerm(&b->term,    /*b->term_hash,*/ &BConceptIndex))
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
                    Term general_implication_term = IntroduceImplicationVariables(precondition_implication.term);
                    if(Variable_hasVariable(&general_implication_term, true, true, false))
                    {
                        NAL_DerivedEvent(general_implication_term, OCCURRENCE_ETERNAL, precondition_implication.truth, precondition_implication.stamp, currentTime, 1, 1);
                    }
                    int operationID = Narsese_getOperationID(&a->term);
                    IN_DEBUG ( if(operationID != 0) { Narsese_PrintTerm(&precondition_implication.term); Truth_Print(&precondition_implication.truth); puts("\n"); getchar(); } )
                    IN_OUTPUT( fputs("Formed implication: ", stdout); Implication_Print(&precondition_implication); )
                    Implication *revised_precon = Table_AddAndRevise(&B->precondition_beliefs[operationID], &precondition_implication);
                    if(revised_precon != NULL)
                    {
                        revised_precon->creationTime = currentTime; //for evaluation
                        revised_precon->sourceConcept = A;
                        revised_precon->sourceConceptTerm = A->term;
                        /*IN_OUTPUT( if(true && revised_precon->term_hash != 0) { fputs("REVISED pre-condition implication: ", stdout); Implication_Print(revised_precon); } ) */
                        Memory_printAddedImplication(&revised_precon->term, &revised_precon->truth, false, revised_precon->truth.confidence > precondition_implication.truth.confidence);
                    }
                }
            }
        }
    }
}

void popEvents()
{
    for(int i=0; i<EVENT_SELECTIONS; i++)
    {
        Event *e;
        double priority = 0;
        if(!PriorityQueue_PopMax(&cycling_events, (void**) &e, &priority))
        {
            assert(cycling_events.itemsAmount == 0, "No item was popped, only acceptable reason is when it's empty");
            IN_DEBUG( puts("Selecting event failed, maybe there is no event left."); )
            break;
        }
        selectedEventsPriority[eventsSelected] = priority;
        selectedEvents[eventsSelected] = *e; //needs to be copied because will be added in a batch
        eventsSelected++; //that while processing, would make recycled pointers invalid to use
    }
}

void pushEvents(long currentTime)
{
    for(int i=0; i<eventsSelected; i++)
    {
        Memory_addEvent(&selectedEvents[i], currentTime, selectedEventsPriority[i], false, false, true, false);
    }
}

void Cycle_Perform(long currentTime)
{   
    eventsSelected = 0;
    popEvents();
    //1. process newest event
    if(belief_events.itemsAmount > 0)
    {
        //form concepts for the sequences of different length
        for(int len=0; len<MAX_SEQUENCE_LEN; len++)
        {
            Event *toProcess = FIFO_GetNewestSequence(&belief_events, len);
            if(toProcess != NULL && !toProcess->processed && toProcess->type != EVENT_TYPE_DELETED)
            {
                assert(toProcess->type == EVENT_TYPE_BELIEF, "A different event type made it into belief events!");
                Cycle_ProcessEvent(toProcess, currentTime);
                Event postcondition = *toProcess;
                //Mine for <(&/,precondition,operation) =/> postcondition> patterns in the FIFO:
                if(len == 0) //postcondition always len1
                {
                    int op_id = Narsese_getOperationID(&postcondition.term);
                    Decision_AssumptionOfFailure(op_id, currentTime); //collection of negative evidence, new way
                    //build link between internal derivations and external event to explain it:
                    for(int k=0; k<eventsSelected; k++)
                    {
                        if(selectedEvents[k].occurrenceTime < postcondition.occurrenceTime)
                        {
                            Cycle_ReinforceLink(&selectedEvents[k], &postcondition);
                        }
                    }
                    for(int k=1; k<belief_events.itemsAmount; k++)
                    {
                        for(int len2=0; len2<MAX_SEQUENCE_LEN; len2++)
                        {
                            Event *precondition = FIFO_GetKthNewestSequence(&belief_events, k, len2);
                            if(precondition != NULL && precondition->type != EVENT_TYPE_DELETED)
                            {
                                Cycle_ReinforceLink(precondition, &postcondition);
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
        if(!goal->processed && goal->type!=EVENT_TYPE_DELETED)
        {
            assert(goal->type == EVENT_TYPE_GOAL, "A different event type made it into goal events!");
            decision[0] = Cycle_ProcessEvent(goal, currentTime);
            //allow reasoning into the future by propagating spikes from goals back to potential current events
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
    //Inferences
#if STAGE==2
    for(int i=0; i<eventsSelected; i++)
    {
        Event *e = &selectedEvents[i];
        double priority = selectedEventsPriority[i];
        Term dummy_term = {0};
        Truth dummy_truth = {0};
        RuleTable_Apply(e->term, dummy_term, e->truth, dummy_truth, e->occurrenceTime, e->stamp, currentTime, priority, 1, false); 
        IN_DEBUG( puts("Event was selected:"); Event_Print(e); )
        for(int j=0; j<concepts.itemsAmount; j++)
        {
            Concept *c = concepts.items[j].address;
            if(c->belief.type != EVENT_TYPE_DELETED)
            {
                Event project_belief = c->belief_spike;
                Event* belief = &c->belief;
                if(e->occurrenceTime != OCCURRENCE_ETERNAL && project_belief.type != EVENT_TYPE_DELETED) //take event as belief if it's stronger
                {
                    project_belief.truth = Truth_Projection(project_belief.truth, project_belief.occurrenceTime, e->occurrenceTime);
                    project_belief.occurrenceTime = e->occurrenceTime;
                    if(project_belief.truth.confidence > c->belief.truth.confidence)
                    {
                        belief = &project_belief;
                    }
                }
                if(!Stamp_checkOverlap(&e->stamp, &belief->stamp))
                {
                    Stamp stamp = Stamp_make(&e->stamp, &belief->stamp);
                    if(PRINT_CONTROL_INFO)
                    {
                        fputs("Apply rule table on ", stdout);
                        Narsese_PrintTerm(&e->term);
                        printf(" Priority=%f\n", priority);
                        fputs(" and ", stdout);
                        Narsese_PrintTerm(&c->term);
                        puts("");
                    }
                    RuleTable_Apply(e->term, c->term, e->truth, belief->truth, e->occurrenceTime, stamp, currentTime, priority, c->priority, true);
                }
            }
            if(e->type == EVENT_TYPE_BELIEF)
            {
                for(int i=0; i<c->precondition_beliefs[0].itemsAmount; i++)
                {
                    Implication *imp = &c->precondition_beliefs[0].array[i];
                    assert(Narsese_copulaEquals(imp->term.atoms[0],'$'), "Not a valid implication term!");
                    Term precondition_with_op = Term_ExtractSubterm(&imp->term, 1);
                    Term precondition = Narsese_GetPreconditionWithoutOp(&precondition_with_op);
                    Substitution subs = Variable_Unify(&precondition, &e->term);
                    if(subs.success)
                    {
                        Implication updated_imp = *imp;
                        updated_imp.term = Variable_ApplySubstitute(updated_imp.term, subs);
                        Event predicted = Inference_BeliefDeduction(e, &updated_imp);
                        NAL_DerivedEvent(predicted.term, predicted.occurrenceTime, predicted.truth, predicted.stamp, currentTime, priority, 1.0);
                    }
                }
            }
        }
    }
#endif
    //Apply event forgetting:
    for(int i=0; i<cycling_events.itemsAmount; i++)
    {
        cycling_events.items[i].priority *= EVENT_DURABILITY;
    }
    //Apply concept forgetting:
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        Concept *c = concepts.items[i].address;
        c->priority *= CONCEPT_DURABILITY;
    }
    //Re-sort queues
    PriorityQueue_Rebuild(&concepts);
    PriorityQueue_Rebuild(&cycling_events);
    //push selected events back to the queue as well
    pushEvents(currentTime);
}
