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

#include "Decision.h"

double CONDITION_THRESHOLD = CONDITION_THRESHOLD_INITIAL;
double DECISION_THRESHOLD = DECISION_THRESHOLD_INITIAL;
double ANTICIPATION_THRESHOLD = ANTICIPATION_THRESHOLD_INITIAL;
double ANTICIPATION_CONFIDENCE = ANTICIPATION_CONFIDENCE_INITIAL;
double MOTOR_BABBLING_CHANCE = MOTOR_BABBLING_CHANCE_INITIAL;
int BABBLING_OPS = OPERATIONS_MAX;

static void Decision_AddNegativeConfirmation(Event *precondition, Implication imp, int operationID, Concept *postc)
{
    Implication negative_confirmation = imp;
    Truth TNew = { .frequency = 0.0, .confidence = ANTICIPATION_CONFIDENCE };
    Truth TPast = Truth_Projection(precondition->truth, 0, round(imp.occurrenceTimeOffset));
    negative_confirmation.truth = Truth_Eternalize(Truth_Induction(TNew, TPast));
    negative_confirmation.stamp = (Stamp) {0}; //precondition->stamp;
    assert(negative_confirmation.truth.confidence >= 0.0 && negative_confirmation.truth.confidence <= 1.0, "(666) confidence out of bounds");
    Implication *added = Table_AddAndRevise(&postc->precondition_beliefs[operationID], &negative_confirmation);
    if(added != NULL)
    {
        added->sourceConcept = negative_confirmation.sourceConcept;
        added->sourceConceptId = negative_confirmation.sourceConceptId;
    }
}

//Inject action event after execution or babbling
void Decision_Execute(Decision *decision)
{
    int n_ops_to_execute = 0;
    for(int i=0; i<MAX_COMPOUND_OP_LEN; i++)
    {
        if(!decision->operationID[i])
        {
            break;
        }
        n_ops_to_execute += 1;
    }
    for(int i=n_ops_to_execute-1; i>=0; i--)
    {
        assert(decision->operationID[i], "Error");
        decision->op[i] = operations[decision->operationID[i]-1];
        Term feedbackTerm = decision->op[i].term; //atomic operation / operator
        //and add operator feedback event
        if(decision->arguments[i].atoms[0] > 0) //operation with args
        {
            Term operation = {0};
            operation.atoms[0] = Narsese_CopulaIndex(INHERITANCE); //<args --> ^op>
            if(!Term_OverrideSubterm(&operation, 1, &decision->arguments[i]) || !Term_OverrideSubterm(&operation, 2, &decision->op[i].term))
            {
                return;
            }
            feedbackTerm = operation;
        }
        Narsese_PrintTerm(&decision->op[i].term); fputs(" executed with args ", stdout); Narsese_PrintTerm(&decision->arguments[i]); puts(""); fflush(stdout);
        Feedback feedback = (*decision->op[i].action)(decision->arguments[i]);
        if(feedback.failed) //TODO improve (leaves option for operation to fail, but we don't want each op having to set it to true...)
        {
            return;
        }
        feedback.subs.success = true; //the value assignment didn't came from substitution but from the op execution so we simply set it to true
        if(decision->op[i].stdinOutput)
        {
            puts("//Operation result product expected: ");
            fflush(stdout);
            //Synch 0 input from Python interface
            char line0[10] = {0};
            if(fgets(line0, 10, stdin) == NULL)
            {
                return;
            }
            assert(line0[0] == '0' && line0[1] == '\n', "Synch 0 not sent");
            //Now the actual argument product:
            char line[1024] = {0};
            if(fgets(line, 1024, stdin) == NULL)
            {
                return;
            }
            Term specific_args = Narsese_Term(line);
            Term argpart = Term_ExtractSubterm(&decision->arguments[i], 2);
            feedback.subs = Variable_Unify(&argpart, &specific_args);
            if(!feedback.subs.success)
            {
                return;
            }
        }
        for(int j=0; j<i; j++) //also apply the substitution to all following operations in the compound op
        {
            bool success;
            decision->arguments[j] = Variable_ApplySubstitute(decision->arguments[j], feedback.subs, &success);
            if(!success)
            {
                return;
            }
        }
        bool success;
        feedbackTerm = Variable_ApplySubstitute(feedbackTerm, feedback.subs, &success);
        if(success)
        {
            NAR_AddInputBelief(feedbackTerm);
            //assumption of failure extension to specific cases not experienced before:
            if(ANTICIPATE_FOR_NOT_EXISTING_SPECIFIC_TEMPORAL_IMPLICATION && decision->missing_specific_implication.term.atoms[0])
            {
                Term postcondition = Term_ExtractSubterm(&decision->missing_specific_implication.term, 2);
                Concept *postc = Memory_Conceptualize(&postcondition, currentTime);
                if(postc != NULL)
                {
                    Decision_AddNegativeConfirmation(decision->reason, decision->missing_specific_implication, decision->operationID[i], postc);
                }
            }
        }
        else
        {
            return;
        }
    }
}

//"reflexes" to try different operations, especially important in the beginning
static Decision Decision_MotorBabbling()
{
    Decision decision = (Decision) {0};
    int n_ops = 0;
    for(int i=0; i<OPERATIONS_MAX && operations[i].term.atoms[0] != 0; i++)
    {
        n_ops = i+1;
    }
    if(n_ops > 0)
    {
        decision.operationID[0] = 1+(myrand() % (MIN(BABBLING_OPS, n_ops)));
        IN_DEBUG (
            printf(" NAR BABBLE %d\n", decision.operationID[0]);
        )
        decision.execute = true;
        decision.desire = 1.0;
        if(operations[decision.operationID[0]-1].arguments[0].atoms[0])
        {
            int n_args = 0;
            for(int i=0; i<OPERATIONS_BABBLE_ARGS_MAX && operations[decision.operationID[0]-1].arguments[i].atoms[0] != 0; i++)
            {
                n_args = i+1;
            }
            int argumentID = myrand() % n_args;
            //({SELF} * num)
            //*   "    arg  SELF
            //0   1    2    3
            decision.arguments[0].atoms[0] = Narsese_CopulaIndex(PRODUCT);  //product
            decision.arguments[0].atoms[1] = Narsese_CopulaIndex(EXT_SET); //ext set {SELF} on the left
            Term_OverrideSubterm(&decision.arguments[0], 2, &operations[decision.operationID[0]-1].arguments[argumentID]);
            decision.arguments[0].atoms[3] = SELF;
            decision.arguments[0].atoms[4] = Narsese_CopulaIndex(SET_TERMINATOR);
        }
    }
    return decision;
}

static Decision Decision_ConsiderNegativeOutcomes(Decision decision)
{
    Event OpGoalImmediateOutcomes = {0};
    //1. discount decision based on negative outcomes via revision
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        Concept *c = concepts.items[i].address;
        if(c->goal_spike.type != EVENT_TYPE_DELETED && (currentTime - c->goal_spike.occurrenceTime) < NEG_GOAL_AGE_MAX)
        {
            for(int j=0; j<c->precondition_beliefs[decision.operationID[0]].itemsAmount; j++)
            {
                Implication imp = c->precondition_beliefs[decision.operationID[0]].array[j];
                Concept *prec = imp.sourceConcept;
                if(prec->belief_spike.type != EVENT_TYPE_DELETED && currentTime - prec->belief_spike.occurrenceTime < EVENT_BELIEF_DISTANCE)
                {
                    Term imp_subject = Term_ExtractSubterm(&imp.term, 1);
                    Term opTerm2 = {0};
                    assert(Narsese_OperationSequenceAppendLeftNested(&opTerm2, &imp_subject), "Failed to extract operation in bad implication!");
                    if(Term_Equal(&decision.operationTerm, &opTerm2))
                    {
                        Event ContextualOperation = Inference_GoalDeduction(&c->goal_spike, &imp, currentTime); //(&/,a,op())! :\:
                        Event OpGoalLocal = Inference_GoalSequenceDeduction(&ContextualOperation, &prec->belief_spike, currentTime);
                        OpGoalImmediateOutcomes = Inference_RevisionAndChoice(&OpGoalImmediateOutcomes, &OpGoalLocal, currentTime, NULL);
                    }
                    IN_DEBUG ( fputs("//Considered: ", stdout); Narsese_PrintTerm(&imp.term); printf(". Truth: frequency=%f confidence=%f dt=%f\n", imp.truth.frequency, imp.truth.confidence, imp.occurrenceTimeOffset); )
                }
            }
        }
    }
    IN_DEBUG ( printf("//Evaluation local=%f global=%f\n",decision.desire, Truth_Expectation(OpGoalImmediateOutcomes.truth)); )
    if(Truth_Expectation(OpGoalImmediateOutcomes.truth) < 0.5)
    {
        decision = (Decision) {0};
    }
    return decision;
}

static Decision Decision_ConsiderImplication(long currentTime, Event *goal, Implication *imp)
{
    Decision decision = {0};
    IN_DEBUG
    (
        printf("CONSIDERED IMPLICATION: impTruth=(%f, %f)", imp->truth.frequency, imp->truth.confidence);
        Narsese_PrintTerm(&imp->term);
        puts("");
    )
    //now look at how much the precondition is fulfilled
    Concept *prec = imp->sourceConcept;
    Event *precondition = &prec->belief_spike; //a. :|:
    if(precondition != NULL)
    {
        Event ContextualOperation = Inference_GoalDeduction(goal, imp, currentTime); //(&/,a,op())! :\:
        Term potential_operation = {0};
        Term imp_subject = Term_ExtractSubterm(&imp->term, 1);
        assert(Narsese_OperationSequenceAppendLeftNested(&decision.operationTerm, &imp_subject), "Failed to extract operation in considered implication!");
        Truth desireValue = Inference_GoalSequenceDeduction(&ContextualOperation, precondition, currentTime).truth;
        double operationGoalTruthExpectation = Truth_Expectation(desireValue); //op()! :|:
        IN_DEBUG
        (
            printf("CONSIDERED PRECON: desire=%f ", operationGoalTruthExpectation);
            Narsese_PrintTerm(&prec->term);
            fputs("\nCONSIDERED PRECON truth ", stdout);
            Truth_Print(&precondition->truth);
            fputs("CONSIDERED goal truth ", stdout);
            Truth_Print(&goal->truth);
            fputs("CONSIDERED imp truth ", stdout);
            Truth_Print(&imp->truth);
            printf("CONSIDERED time %ld\n", precondition->occurrenceTime);
            Narsese_PrintTerm(&precondition->term); puts("");
        )
        decision.reason = precondition;
        decision.desire = operationGoalTruthExpectation;
        Term seq = Term_ExtractSubterm(&imp->term, 1);
        int i=1;
        while(Narsese_copulaEquals(seq.atoms[0], SEQUENCE)) //(a &/ ^op)
        {
            Term operation = Term_ExtractSubterm(&seq, 2);
            if(!Narsese_isOperation(&operation))
            {
                break;
            }
            if(i-1 >= MAX_COMPOUND_OP_LEN)
            {
                assert(false, "Operation sequence longer than the FIFO can build, increase MAX_COMPOUND_OP_LEN if this knowledge should be supported.");
            }
            if(!Narsese_isOperator(operation.atoms[0])) //it is an operation with args, not just an atomic operator, so remember the args
            {
                assert(Narsese_isOperator(operation.atoms[2]), "If it's not atomic, it needs to be an operation with args here");
                Term arguments = Term_ExtractSubterm(&operation, 1); //[* ' ARG SELF]
                if(arguments.atoms[3] != SELF) //either wasn't SELF or var didn't resolve to SELF
                {
                    return (Decision) {0};
                }
                decision.arguments[i-1] = arguments;
            }
            decision.operationID[i-1] = Memory_getOperationID(&operation);
            seq = Term_ExtractSubterm(&seq, 1);
            i++;
        }
    }
    return Decision_ConsiderNegativeOutcomes(decision);
}

Decision Decision_BestCandidate(Concept *goalconcept, Event *goal, long currentTime)
{
    Decision decision = {0};
    Implication bestImp = {0};
    long bestComplexity = COMPOUND_TERM_SIZE_MAX+1;
    bool genericGoalgenericConcept = Variable_hasVariable(&goalconcept->term, true, true, true) && Variable_hasVariable(&goal->term, true, true, true);
    Substitution subs = Variable_Unify(&goalconcept->term, &goal->term);
    if(subs.success)
    {
        for(int opi=1; opi<=OPERATIONS_MAX && operations[opi-1].term.atoms[0] != 0; opi++)
        {
            for(int j=0; j<goalconcept->precondition_beliefs[opi].itemsAmount; j++)
            {
                if(!Memory_ImplicationValid(&goalconcept->precondition_beliefs[opi].array[j]))
                {
                    Table_Remove(&goalconcept->precondition_beliefs[opi], j--);
                    continue;
                }
                Implication imp = goalconcept->precondition_beliefs[opi].array[j];
                bool impHasVariable = Variable_hasVariable(&imp.term, true, true, true);
                bool success;
                imp.term = Variable_ApplySubstitute(imp.term, subs, &success);
                if(success)
                {
                    assert(Narsese_copulaEquals(imp.term.atoms[0], TEMPORAL_IMPLICATION), "This should be a temporal implication!");
                    Term left_side_with_op = Term_ExtractSubterm(&imp.term, 1);
                    Term left_side = Narsese_GetPreconditionWithoutOp(&left_side_with_op); //might be something like <#1 --> a>
                    for(int cmatch_k=0; cmatch_k<concepts.itemsAmount; cmatch_k++)
                    {
                        Concept *cmatch = concepts.items[cmatch_k].address;
                        if(!Variable_hasVariable(&cmatch->term, true, true, true))
                        {
                            Substitution subs2 = Variable_Unify(&left_side, &cmatch->term);
                            if(subs2.success)
                            {
                                Implication specific_imp = imp; //can only be completely specific
                                bool success;
                                specific_imp.term = Variable_ApplySubstitute(specific_imp.term, subs2, &success);
                                if(success && (!genericGoalgenericConcept || !Variable_hasVariable(&specific_imp.term, true, false, false)))
                                {
                                    specific_imp.sourceConcept = cmatch;
                                    specific_imp.sourceConceptId = cmatch->id;
                                    Decision considered = Decision_ConsiderImplication(currentTime, goal, &specific_imp);
                                    int specific_imp_complexity = Term_Complexity(&specific_imp.term);
                                    if(impHasVariable)
                                    {
                                        bool inhibited = false;
                                        Term predicate = Term_ExtractSubterm(&specific_imp.term, 2);
                                        Concept *relatedc = Memory_FindConceptByTerm(&predicate);
                                        bool hypothesis_existed = false;
                                        if(relatedc != NULL)
                                        {
                                            for(int jj=0; jj<relatedc->precondition_beliefs[opi].itemsAmount; jj++)
                                            {
                                                Implication *relatedimp = &relatedc->precondition_beliefs[opi].array[jj];
                                                bool specific_exists = Term_Equal(&specific_imp.term, &relatedimp->term);
                                                if(specific_exists)
                                                {
                                                    hypothesis_existed = true;
                                                    if(relatedimp->truth.confidence > SUBSUMPTION_CONFIDENCE_THRESHOLD && relatedimp->truth.frequency < SUBSUMPTION_FREQUENCY_THRESHOLD)
                                                    {
                                                        inhibited = true;
                                                    }
                                                }
                                            }
                                        }
                                        if(!hypothesis_existed) //this specific implication was never observed before, so we have to keep track of it to apply anticipation
                                        {                       //in addition to Decision_Anticipate which applies it to existing implications
                                            considered.missing_specific_implication = specific_imp;
                                        }
                                        if(!inhibited && (considered.desire > decision.desire || (considered.desire == decision.desire && specific_imp_complexity < bestComplexity)))
                                        {
                                            decision = considered;
                                            bestComplexity = specific_imp_complexity;
                                            bestImp = imp;
                                            decision.usedContingency = goalconcept->precondition_beliefs[opi].array[j];
                                        }
                                    }
                                    else
                                    {
                                        if(considered.desire > decision.desire || (considered.desire == decision.desire && specific_imp_complexity < bestComplexity))
                                        {
                                            decision = considered;
                                            bestComplexity = specific_imp_complexity;
                                            bestImp = imp;
                                            decision.usedContingency = goalconcept->precondition_beliefs[opi].array[j];
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    if(decision.desire < DECISION_THRESHOLD)
    {
        return (Decision) {0}; 
    }
    //set execute and return execution
    printf("decision expectation=%f implication: ", decision.desire);
    Narsese_PrintTerm(&bestImp.term); fputs(". ", stdout); Stamp_print(&bestImp.stamp); printf(" Truth: frequency=%f confidence=%f dt=%f", bestImp.truth.frequency, bestImp.truth.confidence, bestImp.occurrenceTimeOffset);
    fputs(" precondition: ", stdout); Narsese_PrintTerm(&decision.reason->term); fputs(". :|: ", stdout); Stamp_print(&decision.reason->stamp); printf(" Truth: frequency=%f confidence=%f", decision.reason->truth.frequency, decision.reason->truth.confidence);
    printf(" occurrenceTime=%ld\n", decision.reason->occurrenceTime);
    decision.execute = true;
    return decision;
}

void Decision_Anticipate(int operationID, Term opTerm, long currentTime)
{
    assert(operationID >= 0 && operationID <= OPERATIONS_MAX, "Wrong operation id, did you inject an event manually?");
    for(int j=0; j<concepts.itemsAmount; j++)
    {
        Concept *postc = concepts.items[j].address;
        Implication valid_implications[TABLE_SIZE] = {0};
        int k;
        for(k=0; k<postc->precondition_beliefs[operationID].itemsAmount; k++)
        {
            if(!Memory_ImplicationValid(&postc->precondition_beliefs[operationID].array[k]))
            {
                Table_Remove(&postc->precondition_beliefs[operationID], k);
                k--;
                continue;
            }
            Implication imp = postc->precondition_beliefs[operationID].array[k]; //(&/,a,op) =/> b.
            valid_implications[k] = imp;
        }
        for(int h=0; h<k; h++)
        {
            Implication imp = valid_implications[h]; //(&/,a,op) =/> b.
            Concept *current_prec = imp.sourceConcept;
            Event *precondition = &current_prec->belief_spike;
            if(precondition != NULL && precondition->type != EVENT_TYPE_DELETED)
            {
                if(operationID > 0) //it's a real operation, check if the link's operation is the same
                {
                    Term imp_subject = Term_ExtractSubterm(&imp.term, 1);
                    Term a_term_nop = Narsese_GetPreconditionWithoutOp(&imp_subject);
                    Term operation = Narsese_getOperationTerm(&imp_subject);
                    Substitution subs = Variable_Unify(&a_term_nop, &precondition->term);
                    if(subs.success)
                    {
                        bool success2;
                        Term specificOp = Variable_ApplySubstitute(operation, subs, &success2);
                        if(!success2 || !Variable_Unify(&specificOp, &opTerm).success)
                        {
                            continue; //same op id but different op args
                        }
                    }
                }
                assert(precondition->occurrenceTime != OCCURRENCE_ETERNAL, "Precondition should not be eternal!");
                Event updated_precondition = Inference_EventUpdate(precondition, currentTime);
                Event op = { .type = EVENT_TYPE_BELIEF,
                             .truth = (Truth) { .frequency = 1.0, .confidence = 0.9 },
                             .occurrenceTime = currentTime };
                bool success;
                Event seqop = Inference_BeliefIntersection(&updated_precondition, &op, &success);
                if(success)
                {
                    Event result = Inference_BeliefDeduction(&seqop, &imp); //b. :/:
                    if(Truth_Expectation(result.truth) > ANTICIPATION_THRESHOLD || (result.truth.confidence < SUBSUMPTION_CONFIDENCE_THRESHOLD && result.truth.frequency == 0.0)) //also allow for failing derived implications to subsume
                    {
                        Decision_AddNegativeConfirmation(precondition, imp, operationID, postc);
                        Substitution subs = Variable_Unify(&current_prec->term, &precondition->term);
                        if(subs.success)
                        {
                            bool success2;
                            result.term = Variable_ApplySubstitute(result.term, subs, &success2);
                            if(success2)
                            {
                                Concept *c = Memory_Conceptualize(&result.term, currentTime);
                                if(c != NULL && !Stamp_checkOverlap(&precondition->stamp, &imp.stamp))
                                {
                                    c->usage = Usage_use(c->usage, currentTime, false);
                                    if(imp.occurrenceTimeOffset > 0.0)
                                    {
                                        Truth oldTruth = c->predicted_belief.truth;
                                        long oldOccurrenceTime = c->predicted_belief.occurrenceTime;
                                        c->predicted_belief = Inference_RevisionAndChoice(&c->predicted_belief, &result, currentTime, NULL);
                                        if(!Truth_Equal(&c->predicted_belief.truth, &oldTruth) || c->predicted_belief.occurrenceTime != oldOccurrenceTime)
                                        {
                                            Memory_printAddedEvent(&c->predicted_belief.stamp, &c->predicted_belief, 1.0, false, true, false, true, false);
                                        }
                                    }
                                    if(imp.occurrenceTimeOffset == 0.0 && result.occurrenceTime > c->belief_spike.occurrenceTime) //use as belief_spike if newer
                                    {
                                        Truth oldTruth = c->belief_spike.truth;
                                        long oldOccurrenceTime = c->belief_spike.occurrenceTime;
                                        c->belief_spike = Inference_RevisionAndChoice(&c->belief_spike, &result, currentTime, NULL);
                                        if(!Truth_Equal(&c->belief_spike.truth, &oldTruth) || c->belief_spike.occurrenceTime != oldOccurrenceTime)
                                        {
                                            Memory_printAddedEvent(&c->belief_spike.stamp, &c->belief_spike, 1.0, false, true, false, true, false);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

Decision Decision_Suggest(Concept *postc, Event *goal, long currentTime)
{
    Decision babble_decision = {0};
    //try motor babbling with a certain chance
    if(myrand() < (int)(MOTOR_BABBLING_CHANCE * MY_RAND_MAX))
    {
        babble_decision = Decision_MotorBabbling();
    }
    //try matching op if didn't motor babble
    Decision decision_suggested = Decision_BestCandidate(postc, goal, currentTime);
    if(!babble_decision.execute || decision_suggested.desire > MOTOR_BABBLING_SUPPRESSION_THRESHOLD)
    {
       return decision_suggested;
    }
    return babble_decision;
}

Decision Decision_BetterDecision(Decision best_decision, Decision decision)
{
    if(decision.execute && decision.desire >= best_decision.desire)
    {
        return decision;
    }
    return best_decision;
}
