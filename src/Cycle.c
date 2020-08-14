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

static long conceptProcessID = 0; //avoids duplicate concept processing

//Process an event, by creating a concept, or activating an existing
static Decision Cycle_ProcessGoal(Event *e, long currentTime)
{
    Decision best_decision = {0};
    //add a new concept for e if not yet existing
    Memory_Conceptualize(&e->term, currentTime);
    Event_SetTerm(e, e->term); // TODO make sure that hash needs to be calculated once instead already
    IN_DEBUG( puts("Event was selected:"); Event_Print(e); )
    //determine the concept it is related to
    bool e_hasVariable = Variable_hasVariable(&e->term, true, true, true);
    for(int i=0; i<UNIFICATION_DEPTH; i++)
    {
        ConceptChainElement* chain = InvertedAtomIndex_GetConceptChain(e->term.atoms[i]);
        while(chain != NULL)
        {
            Concept *c = chain->c;
            chain = chain->next;
            if(c != NULL && c->processID != conceptProcessID)
            {
                c->processID = conceptProcessID;
                Event ecp = *e;
                if(!e_hasVariable)  //concept matched to the event which doesn't have variables
                {
                    Substitution subs = Variable_Unify(&c->term, &e->term); //concept with variables, 
                    if(subs.success)
                    {
                        ecp.term = e->term;
                        Decision decision = Decision_SuggestOperation(c, &ecp, currentTime);
                        if(decision.execute && decision.desire >= best_decision.desire && (!best_decision.specialized || decision.specialized))
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
                            Decision decision = Decision_SuggestOperation(c, &ecp, currentTime);
                            if(decision.execute && decision.desire >= best_decision.desire && (!best_decision.specialized || decision.specialized))
                            {
                                best_decision = decision;
                            }
                        }
                    }
                }
            }
        }
    }
    return best_decision;
}

void Cycle_PopEvents(Event *selectionArray, double *selectionPriority, int *selectedCnt, PriorityQueue *queue, int cnt)
{
    *selectedCnt = 0;
    for(int i=0; i<cnt; i++)
    {
        Event *e;
        double priority = 0;
        if(!PriorityQueue_PopMax(queue, (void**) &e, &priority))
        {
            assert(queue->itemsAmount == 0, "No item was popped, only acceptable reason is when it's empty");
            IN_DEBUG( puts("Selecting event failed, maybe there is no event left."); )
            break;
        }
        selectionPriority[*selectedCnt] = priority;
        selectionArray[*selectedCnt] = *e; //needs to be copied because will be added in a batch
        (*selectedCnt)++; //that while processing, would make recycled pointers invalid to use
    }
}

//Propagate subgoals, leading to decisions
static void Cycle_GoalReasoning(long currentTime)
{
    Decision best_decision = {0};
    //process selected goals
    for(int i=0; i<goalsSelectedCnt; i++)
    {
        conceptProcessID++; //process the to e related concepts
        Event *goal = &selectedGoals[i];
        IN_DEBUG( fputs("selected goal ", stdout); Narsese_PrintTerm(&goal->term); puts(""); )
        Decision decision = Cycle_ProcessGoal(goal, currentTime);
        if(decision.execute && decision.desire >= best_decision.desire && (!best_decision.specialized || decision.specialized))
        {
            best_decision = decision;
        }
    }
    //pass goal spikes on to the next
    for(int i=0; i<goalsSelectedCnt; i++)
    {
        conceptProcessID++; //process subgoaling for the related concepts for each selected goal
        Event *goal = &selectedGoals[i];
        for(int k=0; k<UNIFICATION_DEPTH; k++)
        {
            ConceptChainElement* chain = InvertedAtomIndex_GetConceptChain(goal->term.atoms[k]);
            while(chain != NULL)
            {
                Concept *c = chain->c;
                chain = chain->next;
                if(c != NULL && c->processID != conceptProcessID && Variable_Unify(&c->term, &goal->term).success) //could be <a --> M>! matching to some <... =/> <$1 --> M>>.
                {
                    c->processID = conceptProcessID;
                    bool revised;
                    c->goal_spike = Inference_RevisionAndChoice(&c->goal_spike, goal, currentTime, &revised);
                    for(int opi=0; opi<=OPERATIONS_MAX; opi++)
                    {
                        for(int j=0; j<c->precondition_beliefs[opi].itemsAmount; j++)
                        {
                            Implication *imp = &c->precondition_beliefs[opi].array[j];
                            if(!Memory_ImplicationValid(imp))
                            {
                                Table_Remove(&c->precondition_beliefs[opi], j);
                                j--;
                                continue;
                            }
                            Event newGoal = Inference_GoalDeduction(&c->goal_spike, imp);
                            Event newGoalUpdated = Inference_EventUpdate(&newGoal, currentTime);
                            IN_DEBUG( fputs("derived goal ", stdout); Narsese_PrintTerm(&newGoalUpdated.term); puts(""); )
                            Memory_AddEvent(&newGoalUpdated, currentTime, selectedGoalsPriority[i] * Truth_Expectation(newGoalUpdated.truth), 0, false, true, false);
                        }
                    }
                }
            }
        }
    }
    if(best_decision.execute && best_decision.operationID > 0)
    {
        Decision_Execute(&best_decision);
        PriorityQueue_RESET(&cycling_goal_events, cycling_goal_events.items, cycling_goal_events.maxElements);
    }
}

void Cycle_BeliefReasoning(long currentTime)
{
    //Inferences
#if STAGE==2
    for(int i=0; i<beliefsSelectedCnt; i++)
    {
        conceptProcessID++; //process the related belief concepts
        long countConceptsMatched = 0;
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
            Event *e = &selectedBeliefs[i];
            Term subterms_of_e[2] = {0}; //subterms up to level 1
            for(int j=0; j<2; j++)
            {
                subterms_of_e[j] = Term_ExtractSubterm(&e->term, j+1);
            }
            double priority = selectedBeliefsPriority[i];
            Term dummy_term = {0};
            Truth dummy_truth = {0};
            RuleTable_Apply(e->term, dummy_term, e->truth, dummy_truth, e->occurrenceTime, e->stamp, currentTime, priority, 1, false, NULL, 0); 
            IN_DEBUG( puts("Event was selected:"); Event_Print(e); )
            for(int i=0; i<UNIFICATION_DEPTH; i++)
            {
                ConceptChainElement* chain = InvertedAtomIndex_GetConceptChain(e->term.atoms[i]);
                while(chain != NULL)
                {
                    Concept *c = chain->c;
                    chain = chain->next;
                    if(c != NULL && c->processID != conceptProcessID)
                    {
                        c->processID = conceptProcessID;
                        long validation_cid = c->id; //allows for lockfree rule table application (only adding to memory is locked)
                        if(c->priority < conceptPriorityThresholdCurrent)
                        {
                            continue;
                        }
                        #pragma omp critical(stats)
                        {
                            countConceptsMatchedNew++;
                            countConceptsMatched++;
                            Stats_countConceptsMatchedTotal++;
                        }
                        if(c->belief.type != EVENT_TYPE_DELETED && countConceptsMatched < BELIEF_CONCEPT_MATCH_TARGET)
                        {
                            //use eternal belief as belief
                            Event* belief = &c->belief;
                            Event future_belief = c->predicted_belief;
                            //but if there is a predicted one in the event's window, use this one
                            if(e->occurrenceTime != OCCURRENCE_ETERNAL && future_belief.type != EVENT_TYPE_DELETED &&
                               labs(e->occurrenceTime - future_belief.occurrenceTime) < EVENT_BELIEF_DISTANCE) //take event as belief if it's stronger
                            {
                                future_belief.truth = Truth_Projection(future_belief.truth, future_belief.occurrenceTime, e->occurrenceTime);
                                future_belief.occurrenceTime = e->occurrenceTime;
                                belief = &future_belief;
                            }
                            //unless there is an actual belief which falls into the event's window
                            Event project_belief = c->belief_spike;
                            if(e->occurrenceTime != OCCURRENCE_ETERNAL && project_belief.type != EVENT_TYPE_DELETED &&
                               labs(e->occurrenceTime - project_belief.occurrenceTime) < EVENT_BELIEF_DISTANCE) //take event as belief if it's stronger
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
    for(int i=0; i<cycling_belief_events.itemsAmount; i++)
    {
        cycling_belief_events.items[i].priority *= EVENT_DURABILITY;
    }
    for(int i=0; i<cycling_goal_events.itemsAmount; i++)
    {
        cycling_goal_events.items[i].priority *= EVENT_DURABILITY;
    }
    //Apply concept forgetting:
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        Concept *c = concepts.items[i].address;
        c->priority *= CONCEPT_DURABILITY;
        concepts.items[i].priority = Usage_usefulness(c->usage, currentTime); //how concept memory is sorted by, by concept usefulness
    }
    //Re-sort queues
    PriorityQueue_Rebuild(&concepts);
    PriorityQueue_Rebuild(&cycling_belief_events);
    PriorityQueue_Rebuild(&cycling_goal_events);
}

void Cycle_Perform(long currentTime)
{   
    Metric_send("NARNode.Cycle", 1);
    //1. Remove BELIEF/GOAL_EVENT_SELECTIONS events from cyclings events priority queue (which includes both input and derivations)
    Cycle_PopEvents(selectedGoals, selectedGoalsPriority, &goalsSelectedCnt, &cycling_goal_events, GOAL_EVENT_SELECTIONS);
    Cycle_PopEvents(selectedBeliefs, selectedBeliefsPriority, &beliefsSelectedCnt, &cycling_belief_events, BELIEF_EVENT_SELECTIONS);
    //2. Process incoming belief events from FIFO, building implications utilizing input sequences and in 1. retrieved events.
    Correlator_CorrelateEvents(currentTime);
    //3. Process incoming goal events from FIFO, propagating subgoals according to implications, triggering decisions when above decision threshold
    Cycle_GoalReasoning(currentTime);
    //4. Perform inference between in 1. retrieved events and semantically/temporally related, high-priority concepts to derive and process new events
    Cycle_BeliefReasoning(currentTime);
    //5. Apply relative forgetting for concepts according to CONCEPT_DURABILITY and events according to BELIEF_EVENT_DURABILITY
    Cycle_RelativeForgetting(currentTime);
}
