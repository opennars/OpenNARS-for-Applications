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
int anticipationStampID = -1;

void Decision_INIT()
{
    anticipationStampID = -1;
}

static void Decision_AddNegativeConfirmation(Event *precondition, Implication imp, int operationID, Concept *postc)
{
    Implication negative_confirmation = imp;
    Truth TNew = { .frequency = 0.0, .confidence = ANTICIPATION_CONFIDENCE };
    Truth TPast = Truth_Projection(precondition->truth, 0, round(imp.occurrenceTimeOffset));
    negative_confirmation.truth = Truth_Eternalize(Truth_Induction(TNew, TPast));
    negative_confirmation.stamp = (Stamp) { .evidentalBase = { -anticipationStampID } };
    anticipationStampID--;
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
    for(int i=0; i<MAX_SEQUENCE_LEN-1; i++)
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
        Term feedback = decision->op[i].term; //atomic operation / operator
        //and add operator feedback
        if(decision->arguments[i].atoms[0] > 0) //operation with args
        {
            Term operation = {0};
            operation.atoms[0] = Narsese_CopulaIndex(INHERITANCE); //<args --> ^op>
            if(!Term_OverrideSubterm(&operation, 1, &decision->arguments[i]) || !Term_OverrideSubterm(&operation, 2, &decision->op[i].term))
            {
                return;
            }
            feedback = operation;
        }
        Narsese_PrintTerm(&decision->op[i].term); fputs(" executed with args ", stdout); Narsese_PrintTerm(&decision->arguments[i]); puts(""); fflush(stdout);
        (*decision->op[i].action)(decision->arguments[i]);
        NAR_AddInputBelief(feedback);
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
            if(i >= MAX_SEQUENCE_LEN)
            {
                assert(false, "Operation sequence longer than the FIFO can build, increase MAX_SEQUENCE_LEN if this knowledge should be supported.");
            }
            if(!Narsese_isOperator(operation.atoms[0])) //it is an operation with args, not just an atomic operator, so remember the args
            {
                assert(Narsese_isOperator(operation.atoms[2]), "If it's not atomic, it needs to be an operation with args here");
                Term arguments = Term_ExtractSubterm(&imp->term, 9); //[* ' ARG SELF]
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
    return decision;
}

Decision Decision_BestCandidate(Concept *goalconcept, Event *goal, long currentTime)
{
    Decision decision = {0};
    Implication bestImp = {0};
    long bestComplexity = COMPOUND_TERM_SIZE_MAX+1;
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
                                if(success && !Variable_hasVariable(&specific_imp.term, true, true, true))
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
                                        }
                                    }
                                    else
                                    {
                                        if(considered.desire > decision.desire || (considered.desire == decision.desire && specific_imp_complexity < bestComplexity))
                                        {
                                            decision = considered;
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
    if(decision.desire < DECISION_THRESHOLD)
    {
        return (Decision) {0}; 
    }
    //set execute and return execution
    printf("decision expectation=%f implication: ", decision.desire);
    Narsese_PrintTerm(&bestImp.term); printf(". Truth: frequency=%f confidence=%f dt=%f", bestImp.truth.frequency, bestImp.truth.confidence, bestImp.occurrenceTimeOffset); 
    fputs(" precondition: ", stdout); Narsese_PrintTerm(&decision.reason->term); fputs(". :|: ", stdout);  printf("Truth: frequency=%f confidence=%f", decision.reason->truth.frequency, decision.reason->truth.confidence); 
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
                        if(!success2 || !Variable_Unify(&opTerm, &specificOp).success)
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
                                if(c != NULL)
                                {
                                    c->usage = Usage_use(c->usage, currentTime, false);
                                    c->predicted_belief = result;
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
