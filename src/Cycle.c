/* 
 * The MIT License
 *
 * Copyright 2020 The OpenNARS authors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Cycle.h"

//doing inference within the matched concept, returning whether decisionMaking should continue
static Decision Cycle_ActivateSensorimotorConcept(Concept *c, Event *e, long currentTime)
{
    Decision decision = {0};
    if(e->truth.confidence > MIN_CONFIDENCE)
    {
        c->usage = Usage_use(c->usage, currentTime);
        //add event as spike to the concept:
        if(e->type == EVENT_TYPE_BELIEF)
        {
            c->belief_spike = *e;
        }
        else
        {
            //pass spike if the concept doesn't have a satisfying motor command
            decision = Decision_Suggest(e, currentTime);
            if(!decision.execute)
            {
                c->incoming_goal_spike = *e;
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
static Decision Cycle_ProcessSensorimotorEvent(Event *e, long currentTime)
{
    Decision best_decision = {0};
    //add a new concept for e if not yet existing
    Memory_Conceptualize(&e->term, currentTime);
    e->processed = true;
    Event_SetTerm(e, e->term); // TODO make sure that hash needs to be calculated once instead already
    IN_DEBUG( puts("Event was selected:"); Event_Print(e); )
    //determine the concept it is related to
    bool e_hasVariable = Variable_hasVariable(&e->term, true, true, true);
    #pragma omp parallel for
    for(int concept_i=0; concept_i<concepts.itemsAmount; concept_i++)
    {
        Event ecp = *e;
        Concept *c = concepts.items[concept_i].address;
        if(!e_hasVariable)  //concept matched to the event which doesn't have variables
        {
            Substitution subs = Variable_Unify(&c->term, &e->term); //concept with variables, 
            if(subs.success)
            {
                ecp.term = e->term;
                Decision decision = Cycle_ActivateSensorimotorConcept(c, &ecp, currentTime);
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
                bool success;
                ecp.term = Variable_ApplySubstitute(e->term, subs, &success);
                if(success)
                {
                    Decision decision = Cycle_ActivateSensorimotorConcept(c, &ecp, currentTime);
                    if(decision.execute && decision.desire >= best_decision.desire)
                    {
                        best_decision = decision;
                    }
                }
            }
        }
    }
    return best_decision;
}

//Propagate subgoals, generating anticipations and decisions
static Decision Cycle_PropagateSubgoals(long currentTime)
{
    Decision decision = {0};
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
                        bool success;
                        Term left_side_substituted = Variable_ApplySubstitute(left_side, subs, &success);
                        if(success)
                        {
                            #pragma omp parallel for
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
        }
        postc->goal_spike.propagated = true;
    }
    //process incoming goal spikes, invoking potential operations
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        Concept *c = concepts.items[i].address;
        if(c->incoming_goal_spike.type != EVENT_TYPE_DELETED)
        {
            c->goal_spike = Inference_RevisionAndChoice(&c->goal_spike, &c->incoming_goal_spike, currentTime, NULL);
            Memory_printAddedEvent(&c->goal_spike, 1, false, true, false);
            if(c->goal_spike.type != EVENT_TYPE_DELETED && !c->goal_spike.processed && Truth_Expectation(c->goal_spike.truth) > PROPAGATION_THRESHOLD)
            {
                Decision decision = Cycle_ProcessSensorimotorEvent(&c->goal_spike, currentTime);
                if(decision.execute)
                {
                    return decision;
                }
            }
        }
        c->incoming_goal_spike = (Event) {0};
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
    Concept *A = Memory_FindConceptByTerm(&a_term_nop);
    Concept *B = Memory_FindConceptByTerm(&b->term);
    if(A != NULL && B != NULL && A != B)
    {
        //temporal induction
        if(!Stamp_checkOverlap(&a->stamp, &b->stamp))
        {
            Implication precondition_implication = Inference_BeliefInduction(a, b);
            precondition_implication.sourceConcept = A;
            precondition_implication.sourceConceptId = A->id;
            if(precondition_implication.truth.confidence >= MIN_CONFIDENCE)
            {
                bool success;
                Term general_implication_term = IntroduceImplicationVariables(precondition_implication.term, &success);
                if(success && Variable_hasVariable(&general_implication_term, true, true, false))
                {
                    NAL_DerivedEvent(general_implication_term, OCCURRENCE_ETERNAL, precondition_implication.truth, precondition_implication.stamp, currentTime, 1, 1, precondition_implication.occurrenceTimeOffset, NULL, 0);
                }
                int operationID = Narsese_getOperationID(&a->term);
                IN_DEBUG ( if(operationID != 0) { Narsese_PrintTerm(&precondition_implication.term); Truth_Print(&precondition_implication.truth); puts("\n"); getchar(); } )
                IN_DEBUG( fputs("Formed implication: ", stdout); Implication_Print(&precondition_implication); )
                Implication *revised_precon = Table_AddAndRevise(&B->precondition_beliefs[operationID], &precondition_implication);
                if(revised_precon != NULL)
                {
                    revised_precon->creationTime = currentTime; //for evaluation
                    revised_precon->sourceConcept = A;
                    revised_precon->sourceConceptId = A->id;
                    /*IN_DEBUG( fputs("REVISED pre-condition implication: ", stdout); Implication_Print(revised_precon); )*/
                    Memory_printAddedImplication(&revised_precon->term, &revised_precon->truth, false, revised_precon->truth.confidence > precondition_implication.truth.confidence);
                }
            }
        }
    }
}

void Cycle_PopEvents()
{
    eventsSelected = 0;
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

void Cycle_PushEvents(long currentTime)
{
    for(int i=0; i<eventsSelected; i++)
    {
        Memory_AddEvent(&selectedEvents[i], currentTime, selectedEventsPriority[i], 0, false, false, true, false, false);
    }
}

void Cycle_ProcessInputBeliefEvents(long currentTime)
{
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
                Cycle_ProcessSensorimotorEvent(toProcess, currentTime);
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
                                Term precond = Narsese_GetPreconditionWithoutOp(&precondition->term);  //a or (&/,a,op)
                                for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
                                {
                                    if(Narsese_isOperator(precond.atoms[i]))
                                    {
                                        goto NoReinforce; //if there is an op in a, then a longer sequ has also, try different k
                                    }
                                }
                                Cycle_ReinforceLink(precondition, &postcondition);
                                NoReinforce:;
                            }
                        }
                    }
                }
            }
        }
    }
}

void Cycle_ProcessInputGoalEvents(long currentTime)
{
    //process goals
    bool hadGoal = false;
    Decision decision[PROPAGATION_ITERATIONS + 1] = {0};
    if(goal_events.itemsAmount > 0)
    {
        Event *goal = FIFO_GetNewestSequence(&goal_events, 0);
        if(!goal->processed && goal->type!=EVENT_TYPE_DELETED)
        {
            hadGoal = true;
            assert(goal->type == EVENT_TYPE_GOAL, "A different event type made it into goal events!");
            decision[0] = Cycle_ProcessSensorimotorEvent(goal, currentTime);
            //allow reasoning into the future by propagating spikes from goals back to potential current events
            for(int i=0; i<PROPAGATION_ITERATIONS; i++)
            {
                decision[i+1] = Cycle_PropagateSubgoals(currentTime);
            }
        }
    }
    if(!hadGoal)
    {
        return;
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
}

void Cycle_Inference(long currentTime)
{
    //Inferences
#if STAGE==2
    for(int i=0; i<eventsSelected; i++)
    {
        long countConceptsMatched = 0;
        bool fired[CONCEPTS_MAX] = {0}; //whether a concept already fired
        for(;;)
        {
            long countConceptsMatchedNew = 0;
            //Adjust dynamic firing threshold: (proportional "self"-control)
            double conceptPriorityThresholdCurrent = conceptPriorityThreshold;
            long countConceptsMatchedAverage = Stats_countConceptsMatchedTotal / currentTime;
            double set_point = BELIEF_CONCEPT_MATCH_TARGET;
            double process_value = countConceptsMatchedAverage; 
            double error = process_value - set_point;
            double increment = error*CONCEPT_THRESHOLD_ADAPTATION;
            conceptPriorityThreshold = MIN(1.0, MAX(0.0, conceptPriorityThreshold + increment));
            //IN_DEBUG( printf("conceptPriorityThreshold=%f\n", conceptPriorityThreshold); )
            Event *e = &selectedEvents[i];
            Term subterms_of_e[2] = {0}; //subterms up to level 1
            for(int j=0; j<2; j++)
            {
                subterms_of_e[j] = Term_ExtractSubterm(&e->term, j+1);
            }
            double priority = selectedEventsPriority[i];
            Term dummy_term = {0};
            Truth dummy_truth = {0};
            RuleTable_Apply(e->term, dummy_term, e->truth, dummy_truth, e->occurrenceTime, e->stamp, currentTime, priority, 1, false, NULL, 0); 
            IN_DEBUG( puts("Event was selected:"); Event_Print(e); )
            //Main inference loop:
            #pragma omp parallel for
            for(int j=0; j<concepts.itemsAmount; j++)
            {
                Concept *c = concepts.items[j].address;
                long validation_cid = c->id; //allows for lockfree rule table application (only adding to memory is locked)
                if(fired[j] || c->priority < conceptPriorityThresholdCurrent)
                {
                    continue;
                }
                fired[j] = true;
                //first filter based on common term (semantic relationship)
                bool has_common_term = false;
                for(int k=0; k<2; k++)
                {
                    Term current = Term_ExtractSubterm(&c->term, k+1);
                    for(int h=0; h<2; h++)
                    {
                        if(current.atoms[0] != 0 && subterms_of_e[h].atoms[0] != 0)
                        {
                            if(Term_Equal(&current, &subterms_of_e[h]))
                            {
                                has_common_term = true;
                                goto PROCEED;
                            }
                        }
                    }
                }
                PROCEED:;
                //second  filter based on precondition implication (temporal relationship)
                bool is_temporally_related = false;
                for(int k=0; k<c->precondition_beliefs[0].itemsAmount; k++)
                {
                    Implication imp = c->precondition_beliefs[0].array[k];
                    Term subject = Term_ExtractSubterm(&imp.term, 1);
                    if(Variable_Unify(&subject, &e->term).success)
                    {
                        is_temporally_related = true;
                        break;
                    }
                }
                if(has_common_term)
                {
                    #pragma omp critical(stats)
                    {
                        countConceptsMatchedNew++;
                        countConceptsMatched++;
                        Stats_countConceptsMatchedTotal++;
                    }
                }
                if(has_common_term && c->belief.type != EVENT_TYPE_DELETED)
                {
                    //use eternal belief as belief
                    Event* belief = &c->belief;
                    Event future_belief = c->predicted_belief;
                    //but if there is a predicted one in the event's window, use this one
                    if(e->occurrenceTime != OCCURRENCE_ETERNAL && future_belief.type != EVENT_TYPE_DELETED &&
                       abs(e->occurrenceTime - future_belief.occurrenceTime) < EVENT_BELIEF_DISTANCE) //take event as belief if it's stronger
                    {
                        future_belief.truth = Truth_Projection(future_belief.truth, future_belief.occurrenceTime, e->occurrenceTime);
                        future_belief.occurrenceTime = e->occurrenceTime;
                        belief = &future_belief;
                    }
                    //unless there is an actual belief which falls into the event's window
                    Event project_belief = c->belief_spike;
                    if(e->occurrenceTime != OCCURRENCE_ETERNAL && project_belief.type != EVENT_TYPE_DELETED &&
                       abs(e->occurrenceTime - project_belief.occurrenceTime) < EVENT_BELIEF_DISTANCE) //take event as belief if it's stronger
                    {
                        project_belief.truth = Truth_Projection(project_belief.truth, project_belief.occurrenceTime, e->occurrenceTime);
                        project_belief.occurrenceTime = e->occurrenceTime;
                        belief = &project_belief;
                    }
                    //Check for overlap and apply inference rules
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
                        RuleTable_Apply(e->term, c->term, e->truth, belief->truth, e->occurrenceTime, stamp, currentTime, priority, c->priority, true, c, validation_cid);
                    }
                }
                if(is_temporally_related)
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
                            bool success;
                            updated_imp.term = Variable_ApplySubstitute(updated_imp.term, subs, &success);
                            if(success)
                            {
                                Event predicted = Inference_BeliefDeduction(e, &updated_imp);
                                NAL_DerivedEvent(predicted.term, predicted.occurrenceTime, predicted.truth, predicted.stamp, currentTime, priority, Truth_Expectation(imp->truth), 0, c, validation_cid);
                            }
                        }
                    }
                }
            }
            if(countConceptsMatched > Stats_countConceptsMatchedMax)
            {
                Stats_countConceptsMatchedMax = countConceptsMatched;
            }
            if(countConceptsMatched >= BELIEF_CONCEPT_MATCH_TARGET || countConceptsMatchedNew == 0)
            {
                break;
            }
        }
    }
#endif
}

void Cycle_RelativeForgetting(long currentTime)
{
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
        concepts.items[i].priority = Usage_usefulness(c->usage, currentTime); //how concept memory is sorted by, by concept usefulness
    }
    //BEGIN SPECIAL HANDLING FOR USER KNOWLEDGE
    if(ontology_handling)
    {
        //BEGIN SPECIAL HANDLING FOR USER KNOWLEDGE
        for(int i=0; i<concepts.itemsAmount; i++)
        {
            Concept *c = concepts.items[i].address;
            if(c->hasUserKnowledge)
            {
                c->usage = Usage_use(c->usage, currentTime); //user implication won't be forgotten
            }
        }
    }
    //END SPECIAL HANDLING FOR USER KNOWLEDGE
    //Re-sort queues
    PriorityQueue_Rebuild(&concepts);
    PriorityQueue_Rebuild(&cycling_events);
}

void Cycle_Perform(long currentTime)
{   
    Metric_send("NARNode.Cycle", 1);
    //1. Retrieve EVENT_SELECTIONS events from cyclings events priority queue (which includes both input and derivations)
    Cycle_PopEvents();
    //2. Process incoming belief events from FIFO, building implications utilizing input sequences and in 1. retrieved events.
    Cycle_ProcessInputBeliefEvents(currentTime);
    //3. Process incoming goal events from FIFO, propagating subgoals according to implications, triggering decisions when above decision threshold
    Cycle_ProcessInputGoalEvents(currentTime);
    //4. Perform inference between in 1. retrieved events and semantically/temporally related, high-priority concepts to derive and process new events
    Cycle_Inference(currentTime);
    //5. Apply relative forgetting for concepts according to CONCEPT_DURABILITY and events according to EVENT_DURABILITY
    Cycle_RelativeForgetting(currentTime);
    //6. Push in 1. selected events back to the queue as well, applying relative forgetting based on EVENT_DURABILITY_ON_USAGE
    Cycle_PushEvents(currentTime);
}
