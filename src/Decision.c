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
//Inject action event after execution or babbling
void Decision_Execute(Decision *decision)
{
    assert(decision->operationID > 0, "Operation 0 is reserved for no action");
    decision->op = operations[decision->operationID-1];
    Term feedback = decision->op.term; //atomic operation / operator
    //and add operator feedback
    if(decision->arguments.atoms[0] > 0) //operation with args
    {
        Term operation = {0};
        operation.atoms[0] = Narsese_AtomicTermIndex(":"); //<args --> ^op>
        if(!Term_OverrideSubterm(&operation, 1, &decision->arguments) || !Term_OverrideSubterm(&operation, 2, &decision->op.term))
        {
            return;
        }
        feedback = operation;
    }
    (*decision->op.action)(decision->arguments);
    NAR_AddInputBelief(feedback);
}

//"reflexes" to try different operations, especially important in the beginning
static Decision Decision_MotorBabbling()
{
    Decision decision = (Decision) {0};
    int n_ops = 0;
    for(int i=0; i<OPERATIONS_MAX && operations[i].action != 0; i++)
    {
        n_ops = i+1;
    }
    if(n_ops > 0)
    {
        decision.operationID = 1+(myrand() % (MIN(BABBLING_OPS, n_ops)));
        IN_DEBUG (
            printf(" NAR BABBLE %d\n", decision.operationID);
        )
        decision.execute = true;
        decision.desire = 1.0;
    }
    return decision;
}

static Decision Decision_ConsiderImplication(long currentTime, Event *goal, int considered_opi, Implication *imp, bool *preconditionAboveConditionThreshold)
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
        *preconditionAboveConditionThreshold |= Truth_Expectation(Truth_Projection(precondition->truth, precondition->occurrenceTime, currentTime)) > CONDITION_THRESHOLD;
        Event ContextualOperation = Inference_GoalDeduction(goal, imp); //(&/,a,op())! :\:
        double operationGoalTruthExpectation = Truth_Expectation(Inference_GoalSequenceDeduction(&ContextualOperation, precondition, currentTime).truth); //op()! :|:
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
        //<(precon &/ <args --> ^op>) =/> postcon>. -> [$ , postcon precon : _ _ _ _ args ^op
        Term operation = Term_ExtractSubterm(&imp->term, 4); //^op or [: args ^op]
        if(!Narsese_isOperator(operation.atoms[0])) //it is an operation with args, not just an atomic operator, so remember the args
        {
            assert(Narsese_isOperator(operation.atoms[2]), "If it's not atomic, it needs to be an operation with args here");
            Term arguments = Term_ExtractSubterm(&imp->term, 9); //[* ' ARG SELF]
            if(arguments.atoms[3] != SELF) //either wasn't SELF or var didn't resolve to SELF
            {
                return decision;
            }
            decision.arguments = arguments;
            
        }
        decision.operationID = considered_opi;
        decision.desire = operationGoalTruthExpectation;
    }
    return decision;
}

int stampID = -1;
Decision Decision_BestCandidate(Concept *goalconcept, Event *goal, long currentTime)
{
    bool preconditionAboveConditionThreshold = false;
    double implicationAboveConditionThresholdConfidence = 1.0;
    Implication minConfImpAboveConditionThreshold = {0};
    Decision minConfImplicationDecision = {0};
    Decision decision = {0};
    Implication bestImp = {0};
    long bestComplexity = COMPOUND_TERM_SIZE_MAX+1;
    Decision decisionGeneral = {0};
    Implication bestImpGeneral = {0};
    long bestComplexityGeneral = COMPOUND_TERM_SIZE_MAX+1;
    Substitution subs = Variable_Unify(&goalconcept->term, &goal->term);
    if(subs.success)
    {
        for(int opi=1; opi<=OPERATIONS_MAX && operations[opi-1].action != 0; opi++)
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
                    assert(Narsese_copulaEquals(imp.term.atoms[0], '$'), "This should be an implication!");
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
                                if(success && !Variable_hasVariable(&specific_imp.term, true, true, true))
                                {
                                    specific_imp.sourceConcept = cmatch;
                                    specific_imp.sourceConceptId = cmatch->id;
                                    bool preconditionAboveConditionThresholdCur = false;
                                    Decision considered = Decision_ConsiderImplication(currentTime, goal, opi, &specific_imp, &preconditionAboveConditionThresholdCur);
                                    preconditionAboveConditionThreshold |= preconditionAboveConditionThresholdCur;
                                    if(preconditionAboveConditionThresholdCur && specific_imp.truth.confidence < implicationAboveConditionThresholdConfidence)
                                    {
                                        minConfImpAboveConditionThreshold = imp;
                                        minConfImplicationDecision = considered;
                                        implicationAboveConditionThresholdConfidence = specific_imp.truth.confidence;
                                    }
                                    int specific_imp_complexity = Term_Complexity(&specific_imp.term);
                                    if(impHasVariable)
                                    {
                                        if(considered.desire > decisionGeneral.desire || (considered.desire == decisionGeneral.desire && specific_imp_complexity < bestComplexityGeneral))
                                        {
                                            decisionGeneral = considered;
                                            bestComplexityGeneral = specific_imp_complexity;
                                            bestImpGeneral = imp;
                                        }
                                    }
                                    else
                                    {
                                        if(considered.desire > decision.desire || (considered.desire == decision.desire && specific_imp_complexity < bestComplexity))
                                        {
                                            decision = considered;
                                            decision.specialized = true;
                                            bestComplexity = specific_imp_complexity;
                                            bestImp = imp;
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
    //use general solution only if the specific solution doesn't exceed the threshold
    if(decisionGeneral.desire > decision.desire && decision.desire < DECISION_THRESHOLD)
    {
        decision = decisionGeneral;
        bestImp = bestImpGeneral;
    }
    if(decision.desire < DECISION_THRESHOLD)
    {
		bool curiosityAllowed = preconditionAboveConditionThreshold && implicationAboveConditionThresholdConfidence < CURIOSITY_THRESHOLD && myrand() < (int)(CURIOSITY_CHANCE * MY_RAND_MAX);
        if(!curiosityAllowed)
        {
            return (Decision) {0};
        }
        decision = minConfImplicationDecision;   
        bestImp = minConfImpAboveConditionThreshold; 
    }
    //set execute and return execution
    printf("decision expectation %f impTruth=(%f, %f): future=%ld ", decision.desire, bestImp.truth.frequency, bestImp.truth.confidence, bestImp.occurrenceTimeOffset);
    Narsese_PrintTerm(&bestImp.term); puts("");
    decision.execute = true;
    return decision;
}

void Decision_Anticipate(int operationID, long currentTime)
{
    assert(operationID >= 0 && operationID <= OPERATIONS_MAX, "Wrong operation id, did you inject an event manually?");
    for(int j=0; j<concepts.itemsAmount; j++)
    {
        Concept *postc = concepts.items[j].address;
        for(int  h=0; h<postc->precondition_beliefs[operationID].itemsAmount; h++)
        {
            if(!Memory_ImplicationValid(&postc->precondition_beliefs[operationID].array[h]))
            {
                Table_Remove(&postc->precondition_beliefs[operationID], h);
                h--;
                continue;
            }
            Implication imp = postc->precondition_beliefs[operationID].array[h]; //(&/,a,op) =/> b.
            Concept *current_prec = imp.sourceConcept;
            Event *precondition = &current_prec->belief_spike;
            if(precondition != NULL && precondition->type != EVENT_TYPE_DELETED)
            {
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
                    if(Truth_Expectation(result.truth) > ANTICIPATION_THRESHOLD)
                    {
                        Implication negative_confirmation = imp;
                        Truth TNew = { .frequency = 0.0, .confidence = ANTICIPATION_CONFIDENCE };
                        Truth TPast = Truth_Projection(precondition->truth, 0, imp.occurrenceTimeOffset);
                        negative_confirmation.truth = Truth_Eternalize(Truth_Induction(TNew, TPast));
                        negative_confirmation.stamp = (Stamp) { .evidentalBase = { -stampID } };
                        stampID--;
                        assert(negative_confirmation.truth.confidence >= 0.0 && negative_confirmation.truth.confidence <= 1.0, "(666) confidence out of bounds");
                        Implication *added = Table_AddAndRevise(&postc->precondition_beliefs[operationID], &negative_confirmation);
                        if(added != NULL)
                        {
                            added->sourceConcept = negative_confirmation.sourceConcept;
                            added->sourceConceptId = negative_confirmation.sourceConceptId;
                        } 
                        Substitution subs = Variable_Unify(&current_prec->term, &precondition->term);
                        if(subs.success)
                        {
                            bool success2;
                            result.term = Variable_ApplySubstitute(result.term, subs, &success2);
                            if(success2)
                            {
                                Concept *c = Memory_Conceptualize(&result.term, currentTime);
                                if(c != NULL)
                                {
                                    c->usage = Usage_use(c->usage, currentTime, false);
                                    c->predicted_belief = result;
                                    Event eternal = result;
                                    eternal.truth = Truth_Eternalize(eternal.truth);
                                    eternal.occurrenceTime = OCCURRENCE_ETERNAL;
                                    c->belief = Inference_RevisionAndChoice(&c->belief, &eternal, currentTime, NULL);
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
