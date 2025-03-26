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
void Decision_Execute(long currentTime, Decision *decision)
{
    //begin code to handle forming of acquired relations
    Term contingency = decision->usedContingency.term;
    Term preconditon_with_op = Term_ExtractSubterm(&contingency, 1); //(0 copula, 1 subject, 2 predicate)
    Term precondition = Narsese_GetPreconditionWithoutOp(&preconditon_with_op);//decision->reason->term;
    //it can be a 3-element-sequence, assume the first is relational queue
    //(this is still general as the sequence can be derived in multiple directions instead of doing this here!)
    Term relationName = {0};
    if(Narsese_copulaEquals(precondition.atoms[0], SEQUENCE))
    {
        Term leftside = Term_ExtractSubterm(&precondition, 1);
        Term rightside = Term_ExtractSubterm(&precondition, 2);
        Term element3 = rightside;
        if(Narsese_copulaEquals(leftside.atoms[0], SEQUENCE))
        {
            Term element1 = Term_ExtractSubterm(&leftside, 1);
            Term element2 = Term_ExtractSubterm(&leftside, 2);
            if(Narsese_copulaEquals(element1.atoms[0], INHERITANCE) && 
               Narsese_copulaEquals(element1.atoms[1], PRODUCT))
            {
                Term prod = Term_ExtractSubterm(&element1, 1); //(loc1 * ocr1)
                relationName = Term_ExtractSubterm(&prod, 2); //ocr1
                Term newSequence = {0};
                newSequence.atoms[0] = Narsese_CopulaIndex(SEQUENCE);
                Term_OverrideSubterm(&newSequence, 1, &element2);
                Term_OverrideSubterm(&newSequence, 2, &element3);
                precondition = newSequence;
            }
        }
    }
    //fputs("PRECONDITION without op: ", stdout); Narsese_PrintTerm(&precondition); puts("");
    //TODO ENSURE COPULA STRUCTURE IS IN TERM
    // (<(sample * X1) --> (loc1 * ocr1)> &/ <(left * Y1) --> (loc2 * ocr2)>)
    // &/ --> -->  *  *  *  * sample X1 loc1 ocr1 left Y1 loc2 ocr2
    // 1  2   3    4  5  6  7 8      9  10   11   12   13 14   15  
    // 0  1   2    3  4  5  6 7      8  9    10   11   12 13   14  
    Term sample = Term_ExtractSubterm(&precondition, 7);
    Term X1 =     Term_ExtractSubterm(&precondition, 8);
    Term loc1 =   Term_ExtractSubterm(&precondition, 9);
    Term ocr1 =   Term_ExtractSubterm(&precondition, 10);
    Term left =   Term_ExtractSubterm(&precondition, 11);
    Term Y1 =     Term_ExtractSubterm(&precondition, 12);
    Term loc2 =   Term_ExtractSubterm(&precondition, 13);
    Term ocr2 =   Term_ExtractSubterm(&precondition, 14);
    
    bool proceed = Narsese_copulaEquals(precondition.atoms[0], SEQUENCE) &&
                                               Narsese_copulaEquals(precondition.atoms[1], INHERITANCE) &&
                                               Narsese_copulaEquals(precondition.atoms[2], INHERITANCE) &&
                                               Narsese_copulaEquals(precondition.atoms[3], PRODUCT) &&
                                               Narsese_copulaEquals(precondition.atoms[4], PRODUCT) &&
                                               Narsese_copulaEquals(precondition.atoms[5], PRODUCT) &&
                                               Narsese_copulaEquals(precondition.atoms[6], PRODUCT) &&
                                               Term_Equal(&loc1, &loc2);
    bool acquired_rel = false;
    if(proceed)
    {
        //                                                        relationName
        // (<(sample * left) --> (loc1 * loc2)> && <(X1 * Y1) --> (ocr1 * ocr2)>)
        // && --> -->  *  *  *  * sample left  loc1 loc2 X1 Y1 ocr1 ocr2
        // 1  2   3    4  5  6  7 8      9     10   11   12 13 14  15
        // 0  1   2    3  4  5  6 7      8     9    10   11 12 13  14
        Term conjunction = {0};
        conjunction.atoms[0] = Narsese_CopulaIndex(CONJUNCTION);
        conjunction.atoms[1] = Narsese_CopulaIndex(INHERITANCE);
        conjunction.atoms[2] = Narsese_CopulaIndex(INHERITANCE);
        conjunction.atoms[3] = Narsese_CopulaIndex(PRODUCT);
        conjunction.atoms[4] = Narsese_CopulaIndex(PRODUCT);
        conjunction.atoms[5] = Narsese_CopulaIndex(PRODUCT);
        bool success = true;
        success &= Term_OverrideSubterm(&conjunction, 7, &sample);
        success &= Term_OverrideSubterm(&conjunction, 8, &left);
        success &= Term_OverrideSubterm(&conjunction, 9, &loc1);
        success &= Term_OverrideSubterm(&conjunction, 10, &loc2);
        success &= Term_OverrideSubterm(&conjunction, 11, &X1);
        success &= Term_OverrideSubterm(&conjunction, 12, &Y1);
        if(relationName.atoms[0])
        {
            Term_OverrideSubterm(&conjunction, 6, &relationName);
        }
        else
        {
            conjunction.atoms[6] = Narsese_CopulaIndex(PRODUCT);
            success &= Term_OverrideSubterm(&conjunction, 13, &ocr1);
            success &= Term_OverrideSubterm(&conjunction, 14, &ocr2);
        }
        //fputs("ACQUIRED RELATION: ", stdout); Narsese_PrintTerm(&conjunction); puts("");
        Term implication = {0};
        implication.atoms[0] = Narsese_CopulaIndex(IMPLICATION);
        success &= Term_OverrideSubterm(&implication, 1, &conjunction);
        success &= Term_OverrideSubterm(&implication, 2, &contingency);
        if(success)
        {
            //fputs("IMPLICATION: ", stdout); Narsese_PrintTerm(&implication); puts("");
            Truth implication_truth = Truth_Induction(decision->reason->truth, decision->usedContingency.truth); //preconditoon truth
            bool success2;
            Term generalized_implication = Variable_IntroduceImplicationVariables(implication, &success2, true);
            //SAMPLE AND LEFT NEEDS TO MATCH THE decision->arguments[i] ({SELF} * (left * right))
            bool spatial_constraint_fulfilled = false;
            if(decision->arguments[0].atoms[0] > 0)
            {
                Term SELF_and_args = decision->arguments[0];
                if(Narsese_copulaEquals(SELF_and_args.atoms[0], PRODUCT))
                {
                    Term args = Term_ExtractSubterm(&SELF_and_args, 2);
                    if(Narsese_copulaEquals(args.atoms[0], PRODUCT))
                    {
                        Term arg1 = Term_ExtractSubterm(&args, 1);
                        Term arg2 = Term_ExtractSubterm(&args, 2);
                        if(Term_Equal(&arg1, &sample) && Term_Equal(&arg2, &left))
                        {
                            spatial_constraint_fulfilled =  true;
                        }
                    }
                }
            }
            //printf("FULFILLED CONSTRAINT? %d\n", (int) spatial_constraint_fulfilled);
            if(success2 && spatial_constraint_fulfilled)
            {
                fputs("GENERALIZED IMPLICATION: ", stdout); Narsese_PrintTerm(&generalized_implication); puts("");
                //Memory_AddMemoryHelper(currentTime, &implication, implication_truth);
                //Memory_AddMemoryHelper(currentTime, &generalized_implication, implication_truth, &decision->reason->stamp, &decision->usedContingency.stamp, false);
                Term dummy_term = {0};
                Event dummy = Event_InputEvent(dummy_term, EVENT_TYPE_BELIEF, (Truth) { .frequency = 0.5, .confidence = 0.0 }, 0, currentTime);
                //dummy.stamp = Stamp_make(&decision->reason->stamp, &decision->usedContingency.stamp); //decision->usedContingency.stamp;
                Memory_AddMemoryHelper(currentTime, &generalized_implication, implication_truth, &dummy.stamp, NULL, false);
                //extract the individual statements
                Term loc_loc = Term_ExtractSubterm(&conjunction, 1);
                Term ocr_ocr = Term_ExtractSubterm(&conjunction, 2);
                //Memory_AddMemoryHelper(currentTime, &conjunction, decision->reason->truth, &decision->reason->stamp, NULL, false);
                //Memory_AddMemoryHelper(currentTime, &loc_loc, decision->reason->truth);
                Concept *clastActedOnRelationBelief = Memory_FindConceptByTerm(&ocr_ocr);
                if(clastActedOnRelationBelief != NULL)
                {
                    decision->lastActedOnRelationBelief = clastActedOnRelationBelief->belief;
                    decision->lastActedOnRelationBelief.stamp = dummy.stamp;
                }
                else //create empty belief
                {
                    //Stamp stamp = Stamp_make(&decision->reason->stamp, &decision->usedContingency.stamp);
                    Event ev = (Event) { .term = ocr_ocr,
                                         .type = EVENT_TYPE_BELIEF, 
                                         .truth = (Truth) { .frequency = 0.5, .confidence = 0.0 }, 
                                         .stamp = dummy.stamp, //stamp, //(Stamp) {0},
                                         .occurrenceTime = OCCURRENCE_ETERNAL,
                                         .occurrenceTimeOffset = 0,
                                         .creationTime = currentTime,
                                         .input = false };
                    decision->lastActedOnRelationBelief = ev;
                }
                /*bool added = Memory_AddMemoryHelper(currentTime, &ocr_ocr, decision->specific_implication.truth, &decision->reason->stamp, NULL, true); //--
                if(added)
                {
                    //IN_DEBUGNEW( fputs("ACQUIRED REL1: ", stdout); Narsese_PrintTerm(&loc_loc); puts(""); )
                    //IN_DEBUGNEW( fputs("ACQUIRED REL2: ", stdout); Narsese_PrintTerm(&ocr_ocr); puts(""); )
                    fputs("ACQUIRED REL: ", stdout); Narsese_PrintTerm(&ocr_ocr); puts("");
                }*/
                acquired_rel = true; //gets in the way of functional equ as we would acquire a relation yet again from the functional equ
            }
            
        }
    }
    //end
    bool functional_equivalence_active = false;
    if(FUNCTIONAL_EQUIVALENCE && decision->specific_implication.term.atoms[0] && !Variable_hasVariable(&decision->specific_implication.term, true, true, false))
    {
        Term cons1 = Term_ExtractSubterm(&decision->specific_implication.term, 2);
        Concept *C_goal = Memory_FindConceptByTerm(&cons1);
        if(C_goal != NULL)
        {
            //now iterate through the implication table of the concept that has the same operation
            for(int j=0; j<C_goal->precondition_beliefs[decision->tableIndex].itemsAmount; j++)
            {
                Implication comparedImp = C_goal->precondition_beliefs[decision->tableIndex].array[j];
                if(Variable_hasVariable(&comparedImp.term, true, true, false))
                {
                    continue;
                }
                Term cons2 = Term_ExtractSubterm(&comparedImp.term, 2);
                Term prec_op1 = Term_ExtractSubterm(&decision->specific_implication.term, 1);
                Term prec_op2 = Term_ExtractSubterm(&comparedImp.term, 1);
                Term op1 = Term_ExtractSubterm(&prec_op1, 2);
                Term op2 = Term_ExtractSubterm(&prec_op2, 2);
                Term prec1 = Narsese_GetPreconditionWithoutOp(&prec_op1);
                Term prec2 = Narsese_GetPreconditionWithoutOp(&prec_op2);
                bool prec1_prec2_equal = Term_Equal(&prec1, &prec2);
                if(FUNCTIONAL_EQUIVALENCE_SIMPLIFY && !prec1_prec2_equal)
                {
                    if(Narsese_copulaEquals(prec1.atoms[0], SEQUENCE) && Narsese_copulaEquals(prec2.atoms[0], SEQUENCE))
                    {
                        DECOMPOSE_NEXT_SEQUENCE_ELEMENT:
                        {
                            Term Lpart_prec1 = Term_ExtractSubterm(&prec1, 1);
                            Term Rpart_prec1 = Term_ExtractSubterm(&prec1, 2);
                            Term Lpart_prec2 = Term_ExtractSubterm(&prec2, 1);
                            Term Rpart_prec2 = Term_ExtractSubterm(&prec2, 2);
                            if(Term_Equal(&Lpart_prec1, &Lpart_prec2))
                            {
                                prec1 = Rpart_prec1;
                                prec2 = Rpart_prec2;
                            }
                            else
                            if(Term_Equal(&Rpart_prec1, &Rpart_prec2))
                            {
                                prec1 = Lpart_prec1;
                                prec2 = Lpart_prec2;
                            }
                            else
                            {
                                goto NO_EQUAL_PART;
                            }
                            if(Narsese_copulaEquals(prec1.atoms[0], SEQUENCE) && Narsese_copulaEquals(prec2.atoms[0], SEQUENCE))
                            {
                               goto DECOMPOSE_NEXT_SEQUENCE_ELEMENT;
                            }
                        }
                        NO_EQUAL_PART:;
                    }
                }
                if(!prec1_prec2_equal && Term_Equal(&cons1, &cons2) && Term_Equal(&op1, &op2) && (!FUNCTIONAL_EQUIVALENCE_LENGTH_RESTRICTION || Narsese_SequenceLength(&prec1) == Narsese_SequenceLength(&prec2)))
                {
                    if(!Stamp_checkOverlap(&decision->specific_implication.stamp, &comparedImp.stamp))
                    {
                        Stamp equStamp = Stamp_make(&decision->specific_implication.stamp, &comparedImp.stamp);
                        Term equTerm1 = {0};
                        equTerm1.atoms[0] = Narsese_CopulaIndex(IMPLICATION);
                        bool success1 = Term_OverrideSubterm(&equTerm1, 1, &prec1);
                        bool success2 = Term_OverrideSubterm(&equTerm1, 2, &prec2);
                        Term Te_imp2 = {0};
                        Term Te_imp3 = {0};
                        if(functional_equivalence_active && success1 && success2 && Truth_Expectation(Memory_getTemporalLinkTruth(&prec1, &prec2)) <= FUNCTIONAL_EQUIVALENCE_NONTEMPORAL_EXP && Truth_Expectation(Memory_getTemporalLinkTruth(&prec2, &prec1)) <= FUNCTIONAL_EQUIVALENCE_NONTEMPORAL_EXP)
                        {
                            Event e_imp = { .term = equTerm1,
                                            .type = EVENT_TYPE_BELIEF,
                                            .truth = Truth_Induction(decision->specific_implication.truth, comparedImp.truth),
                                            .stamp = equStamp,
                                            .occurrenceTime = currentTime };
                            if(FUNCTIONAL_EQUIVALENCE_SPECIFIC && functional_equivalence_active)
                             {
                                if(functional_equivalence_active)
                                {
                                    Memory_AddEvent(&e_imp, currentTime, 1.0, false, true, false, 0, true);
                                }
                                Term contingency = e_imp.term; //decision->usedContingency.term;
                                //Term preconditon_with_op = Term_ExtractSubterm(&contingency, 1); //(0 copula, 1 subject, 2 predicate)
                                Term precondition = contingency; //Narsese_GetPreconditionWithoutOp(&preconditon_with_op);
                                //TODO ENSURE COPULA STRUCTURE IS IN TERM
                                // (<(sample * X1) --> (loc1 * ocr1)> =/> <(left * Y1) --> (loc2 * ocr2)>)
                                // =/> --> -->  *  *  *  * sample X1 loc1 ocr1 left Y1 loc2 ocr2
                                // 1   2   3    4  5  6  7 8      9  10   11   12   13 14   15
                                // 0   1   2    3  4  5  6 7      8  9    10   11   12 13   14
                                Term sample = Term_ExtractSubterm(&precondition, 7);
                                Term X1 =     Term_ExtractSubterm(&precondition, 8);
                                Term loc1 =   Term_ExtractSubterm(&precondition, 9);
                                Term ocr1 =   Term_ExtractSubterm(&precondition, 10);
                                Term left =   Term_ExtractSubterm(&precondition, 11);
                                
                                //SAMPLE AND LEFT NEEDS TO MATCH THE decision->arguments[i] ({SELF} * (left * right))
                                bool spatial_constraint_fulfilled = false;
                                if(decision->arguments[0].atoms[0] > 0)
                                {
                                    Term SELF_and_args = decision->arguments[0];
                                    if(Narsese_copulaEquals(SELF_and_args.atoms[0], PRODUCT))
                                    {
                                        Term args = Term_ExtractSubterm(&SELF_and_args, 2);
                                        if(Narsese_copulaEquals(args.atoms[0], PRODUCT))
                                        {
                                            Term arg1 = Term_ExtractSubterm(&args, 1);
                                            Term arg2 = Term_ExtractSubterm(&args, 2);
                                            if(Term_Equal(&arg1, &sample) && Term_Equal(&arg2, &left))
                                            {
                                                spatial_constraint_fulfilled =  true;
                                            }
                                        }
                                    }
                                }
                                printf("FULFILLED CONSTRAINT? %d\n", (int) spatial_constraint_fulfilled);
                                Term Y1 =     Term_ExtractSubterm(&precondition, 12);
                                Term loc2 =   Term_ExtractSubterm(&precondition, 13);
                                Term ocr2 =   Term_ExtractSubterm(&precondition, 14);
                                bool proceed = Narsese_copulaEquals(precondition.atoms[0], IMPLICATION) && spatial_constraint_fulfilled &&
                                                                           Narsese_copulaEquals(precondition.atoms[1], INHERITANCE) &&
                                                                           Narsese_copulaEquals(precondition.atoms[2], INHERITANCE) &&
                                                                           Narsese_copulaEquals(precondition.atoms[3], PRODUCT) &&
                                                                           Narsese_copulaEquals(precondition.atoms[4], PRODUCT) &&
                                                                           Narsese_copulaEquals(precondition.atoms[5], PRODUCT) &&
                                                                           Narsese_copulaEquals(precondition.atoms[6], PRODUCT) &&
                                                                           Term_Equal(&loc1, &loc2);
                                proceed = !acquired_rel && //whether we already acquired a rel
                                           proceed && Term_Equal(&left, &sample); //location need to match in addition
                                /*if(!proceed)
                                {
                                    fputs("TERM: ", stdout); Narsese_PrintTerm(&precondition); puts("");
                                }*/
                                if(proceed)
                                {
                                    IN_DEBUGNEW( fputs("EIMP: ", stdout); Narsese_PrintTerm(&e_imp.term); puts(""); )
                                    // puts("PROCEED");
                                    // ->
                                    // (<(sample * left) --> (loc1 * loc2)> && <(X1 * Y1) --> (ocr1 * ocr2)>)
                                    // && --> -->  *  *  *  * sample left  loc1 loc2 X1 Y1 ocr1 ocr2
                                    // 1  2   3    4  5  6  7 8      9     10   11   12 13 14  15
                                    // 0  1   2    3  4  5  6 7      8     9    10   11 12 13  14
                                    Term conjunction = {0};
                                    conjunction.atoms[0] = Narsese_CopulaIndex(CONJUNCTION);
                                    conjunction.atoms[1] = Narsese_CopulaIndex(INHERITANCE);
                                    conjunction.atoms[2] = Narsese_CopulaIndex(INHERITANCE);
                                    conjunction.atoms[3] = Narsese_CopulaIndex(PRODUCT);
                                    conjunction.atoms[4] = Narsese_CopulaIndex(PRODUCT);
                                    conjunction.atoms[5] = Narsese_CopulaIndex(PRODUCT);
                                    conjunction.atoms[6] = Narsese_CopulaIndex(PRODUCT);
                                    bool success = true;
                                    success &= Term_OverrideSubterm(&conjunction, 7, &sample);
                                    success &= Term_OverrideSubterm(&conjunction, 8, &left);
                                    success &= Term_OverrideSubterm(&conjunction, 9, &loc1);
                                    success &= Term_OverrideSubterm(&conjunction, 10, &loc2);
                                    success &= Term_OverrideSubterm(&conjunction, 11, &X1);
                                    success &= Term_OverrideSubterm(&conjunction, 12, &Y1);
                                    success &= Term_OverrideSubterm(&conjunction, 13, &ocr1);
                                    success &= Term_OverrideSubterm(&conjunction, 14, &ocr2);
                                    //fputs("ACQUIRED RELATION: ", stdout); Narsese_PrintTerm(&conjunction); puts("");
                                    Term implication = {0};
                                    implication.atoms[0] = Narsese_CopulaIndex(IMPLICATION);
                                    success &= Term_OverrideSubterm(&implication, 1, &conjunction);
                                    success &= Term_OverrideSubterm(&implication, 2, &contingency);
                                    if(success)
                                    {
                                        //fputs("IMPLICATION: ", stdout); Narsese_PrintTerm(&implication); puts("");
                                        Truth implication_truth = e_imp.truth; //Truth_Induction(decision->reason->truth, decision->usedContingency.truth); //preconditoon truth
                                        bool success2;
                                        Term generalized_implication = Variable_IntroduceImplicationVariables(implication, &success2, true);
                                        if(success2)
                                        {
                                            //fputs("GENERALIZED IMPLICATION: ", stdout); Narsese_PrintTerm(&generalized_implication); puts("");
                                            //Decision_AddMemoryHelper(currentTime, &implication, implication_truth);
                                            Memory_AddMemoryHelper(currentTime, &generalized_implication, implication_truth, &e_imp.stamp, NULL, false); //&decision->reason->stamp, &decision->usedContingency.stamp);
                                            //extract the individual statements
                                            Term loc_loc = Term_ExtractSubterm(&conjunction, 1);
                                            Term ocr_ocr = Term_ExtractSubterm(&conjunction, 2);
                                            IN_DEBUGNEW( fputs("ACQUIRED REL1: ", stdout); Narsese_PrintTerm(&loc_loc); puts(""); )
                                            IN_DEBUGNEW( fputs("ACQUIRED REL2: ", stdout); Narsese_PrintTerm(&ocr_ocr); puts(""); )
                                            //Memory_AddMemoryHelper(currentTime, &conjunction, e_imp.truth, &e_imp.stamp, NULL, false);
                                            //--//Decision_AddMemoryHelper(currentTime, &loc_loc, decision->reason->truth);
                                            Memory_AddMemoryHelper(currentTime, &ocr_ocr, e_imp.truth, &e_imp.stamp, NULL, true);
                                            //exit(0);
                                        }
                                    }
                                }
                             }
                            if(functional_equivalence_active)
                             {
                                Event e_imp2 = e_imp;
                                bool intro_success2;
                                e_imp2.term = Variable_IntroduceImplicationVariables2(e_imp.term, &intro_success2, true, 2);
                                if(intro_success2 && Variable_hasVariable(&e_imp2.term, true, true, false) && !Term_Equal(&Te_imp2, &e_imp2.term))
                                {
                                    Memory_AddEvent(&e_imp2, currentTime, 1.0, false, true, false, 0, true);
                                }
                                Event e_imp3 = e_imp;
                                bool intro_success3;
                                e_imp3.term = Variable_IntroduceImplicationVariables2(e_imp.term, &intro_success3, true, 1);
                                if(intro_success3 && Variable_hasVariable(&e_imp3.term, true, true, false) && !Term_Equal(&Te_imp3, &e_imp3.term))
                                {
                                    Memory_AddEvent(&e_imp3, currentTime, 1.0, false, true, false, 0, true);
                                }
                             }
                         }
                     }
                }
            }
        }
    }
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

static Decision Decision_ConsiderImplication(long currentTime, Event *goal, Implication *imp, Truth preconTruth)
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
        Event preconCopy = *precondition;
        preconCopy.truth = preconTruth;
        Truth desireValue = Inference_GoalSequenceDeduction(&ContextualOperation, &preconCopy, currentTime).truth;
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
                        if(!Variable_hasVariable(&cmatch->term, true, true, true) && cmatch->belief_spike.type != EVENT_TYPE_DELETED)
                        {
                            Substitution subs2 = Variable_UnifyWithAnalogy(cmatch->belief_spike.truth, &left_side, &cmatch->term);
                            if(subs2.success)
                            {
                                bool perfectMatch = subs2.truth.confidence == cmatch->belief_spike.truth.confidence;
                                bool hasCloserPreconditionLink = false;
                                for(int opk=1; !perfectMatch && opk<=OPERATIONS_MAX && operations[opk-1].term.atoms[0] != 0; opk++)
                                {
                                    for(int k=0; k<goalconcept->precondition_beliefs[opk].itemsAmount; k++)
                                    {
                                        Implication impk = goalconcept->precondition_beliefs[opk].array[k];
                                        Term left_side_with_opk = Term_ExtractSubterm(&impk.term, 1);
                                        Term left_sidek = Narsese_GetPreconditionWithoutOp(&left_side_with_opk);
                                        Substitution subs3 = Variable_UnifyWithAnalogy(cmatch->belief_spike.truth, &left_sidek, &cmatch->term);
                                        if(subs3.success && subs3.truth.confidence > subs2.truth.confidence)
                                        {
                                            hasCloserPreconditionLink = true;
                                            break;
                                        }
                                    }
                                    if(hasCloserPreconditionLink)
                                    {
                                        break;
                                    }
                                }
                                if(hasCloserPreconditionLink)
                                {
                                    continue;
                                }
                                Implication specific_imp = imp; //can only be completely specific
                                bool success;
                                specific_imp.term = Variable_ApplySubstitute(specific_imp.term, subs2, &success);
                                if(success && (!genericGoalgenericConcept || !Variable_hasVariable(&specific_imp.term, true, false, false)))
                                {
                                    specific_imp.sourceConcept = cmatch;
                                    specific_imp.sourceConceptId = cmatch->id;
                                    Decision considered = Decision_ConsiderImplication(currentTime, goal, &specific_imp, subs2.truth);
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
                                        if(!hypothesis_existed || !perfectMatch) //this specific implication was never observed before, so we have to keep track of it to apply anticipation
                                        {                       //in addition to Decision_Anticipate which applies it to existing implications
                                            considered.missing_specific_implication = specific_imp;
                                        }
                                        if(!inhibited && (considered.desire > decision.desire || (considered.desire == decision.desire && specific_imp_complexity < bestComplexity)))
                                        {
                                            decision = considered;
                                            bestComplexity = specific_imp_complexity;
                                            bestImp = imp;
                                            decision.usedContingency = goalconcept->precondition_beliefs[opi].array[j];
                                            decision.tableIndex = opi;
                                            decision.specific_implication = specific_imp;
                                        }
                                    }
                                    else
                                    {
                                        if(!perfectMatch)
                                        {
                                            considered.missing_specific_implication = specific_imp;
                                        }
                                        if(considered.desire > decision.desire || (considered.desire == decision.desire && specific_imp_complexity < bestComplexity))
                                        {
                                            decision = considered;
                                            bestComplexity = specific_imp_complexity;
                                            bestImp = imp;
                                            decision.usedContingency = goalconcept->precondition_beliefs[opi].array[j];
                                            decision.tableIndex = opi;
                                            decision.specific_implication = specific_imp;
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

void Decision_Anticipate(int operationID, Term opTerm, bool declarative, long currentTime)
{
    assert(operationID >= 0 && operationID <= OPERATIONS_MAX, "Wrong operation id, did you inject an event manually?");
    for(int j=0; j<concepts.itemsAmount; j++)
    {
        Concept *postc = concepts.items[j].address;
        Implication valid_implications[TABLE_SIZE*2] = {0};
        int k=0;
        if(!declarative)
        {
            for(; k<postc->precondition_beliefs[operationID].itemsAmount; k++)
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
        }
        else
        if(operationID == 0)
        {
            for(; k<postc->implication_links.itemsAmount; k++)
            {
                if(!Memory_ImplicationValid(&postc->implication_links.array[k]))
                {
                    Table_Remove(&postc->implication_links, k);
                    k--;
                    continue;
                }
                Implication imp = postc->implication_links.array[k]; //a ==> b.
                valid_implications[k] = imp;
            }
        }
        int handled_declarative_implications = 0;
        for(int h=0; h<k; h++)
        {
            Implication imp = valid_implications[h]; //(&/,a,op) =/> b.
            if(Narsese_copulaEquals(imp.term.atoms[0], IMPLICATION))
            {
                handled_declarative_implications++;
                if(handled_declarative_implications > TOP_K_DECLARATIVE_IMPLICATIONS)
                {
                    continue;
                }
            }
            Concept *current_prec = imp.sourceConcept;
            Term ImpPreconWithOp = Term_ExtractSubterm(&imp.term, 1);
            Term ImpPrecon = Narsese_GetPreconditionWithoutOp(&ImpPreconWithOp);
            //fputs("TRIED IMPL: ", stdout); Narsese_PrintTerm(&imp.term); puts("");
            for(int u=0; u<concepts.itemsAmount; u++)
            {
                Concept *cP = concepts.items[u].address;
                if(Variable_hasVariable(&cP->term, true, true, true))
                {
                    continue;
                }
                if(!declarative)
                {
                    cP = current_prec;
                }
                bool declarative_boost_skip = declarative && cP->belief.creationTime < currentTime-BELIEF_LAST_USED_TOLERANCE && cP->belief_spike.creationTime < currentTime-BELIEF_LAST_USED_TOLERANCE;
                Event *prec_eternal = &cP->belief;
                Event *prec_event = &cP->belief_spike;
                if(prec_eternal->type != EVENT_TYPE_DELETED || prec_event->type != EVENT_TYPE_DELETED)
                {
                    //fputs("TRIED PRECON: ", stdout); Narsese_PrintTerm(&cP->belief.term); puts("");
                    Substitution additionalSubstEternal = Variable_Unify(&ImpPrecon, &prec_eternal->term);
                    Substitution additionalSubstEvent = Variable_Unify(&ImpPrecon, &prec_event->term);
                    if(!declarative_boost_skip && (additionalSubstEternal.success || additionalSubstEvent.success))
                    {
                        //fputs("MATCHED PRECON: ", stdout); Narsese_PrintTerm(&cP->belief.term); puts("");
                        current_prec = cP;
                        if(operationID > 0) //it's a real operation, check if the link's operation is the same
                        {
                            Term imp_subject = Term_ExtractSubterm(&imp.term, 1);
                            Term a_term_nop = Narsese_GetPreconditionWithoutOp(&imp_subject);
                            Term operation = Narsese_getOperationTerm(&imp_subject);
                            Substitution subs = Variable_Unify(&a_term_nop, &prec_event->term);
                            if(subs.success)
                            {
                                bool success2;
                                Term specificOp = Variable_ApplySubstitute(operation, subs, &success2);
                                if(!success2 || !Variable_Unify(&specificOp, &opTerm).success)
                                {
                                    continue; //same op id but different op args
                                }
                            }
                            else
                            {
                                continue; //unification failed
                            }
                        }
                        Event op = { .type = EVENT_TYPE_BELIEF,
                                     .truth = (Truth) { .frequency = 1.0, .confidence = 0.9 },
                                     .occurrenceTime = currentTime };
                        bool success;
                        Event seqop = Inference_BeliefIntersection(prec_event, &op, &success);
                        if(success || operationID == 0)
                        {
                            Event resulteternal = Inference_BeliefDeduction(prec_eternal, &imp); //b. :/:
                            Event resultevent = Inference_BeliefDeduction(operationID == 0 ? prec_event : &seqop, &imp); //b. :/:
                            Substitution subsevent = Variable_Unify(&current_prec->term, &prec_event->term);
                            Substitution subseternal = Variable_Unify(&current_prec->term, &prec_eternal->term);
                            if(subsevent.success || subseternal.success)
                            {
                                if(subsevent.success && (Narsese_copulaEquals(imp.term.atoms[0], IMPLICATION) || Truth_Expectation(resultevent.truth) > ANTICIPATION_THRESHOLD || (resultevent.truth.confidence < SUBSUMPTION_CONFIDENCE_THRESHOLD && resultevent.truth.frequency == 0.0))) //also allow for failing derived implications to subsume
                                {
                                    if(Narsese_copulaEquals(imp.term.atoms[0], TEMPORAL_IMPLICATION))
                                    {
                                        Decision_AddNegativeConfirmation(prec_event, imp, operationID, postc);
                                    }
                                }
                                bool success2eternal, success2event;
                                if(subseternal.success)
                                {
                                    resulteternal.term = Variable_ApplySubstitute(resulteternal.term, subseternal, &success2eternal);
                                }
                                else
                                {
                                    success2eternal = false;
                                }
                                if(subsevent.success)
                                {
                                    resultevent.term = Variable_ApplySubstitute(resultevent.term, subsevent, &success2event);
                                }
                                else
                                {
                                    success2event = false;
                                }
                                if(success2eternal || success2event)
                                {
                                    bool success3eternal, success3event;
                                    if(additionalSubstEternal.success)
                                    {
                                        resulteternal.term = Variable_ApplySubstitute(resulteternal.term, additionalSubstEternal, &success3eternal);
                                    }
                                    else
                                    {
                                        success3eternal = false;
                                    }
                                    if(additionalSubstEvent.success)
                                    {
                                        resultevent.term = Variable_ApplySubstitute(resultevent.term, additionalSubstEvent, &success3event);
                                    }
                                    else
                                    {
                                        success3event = false;
                                    }
                                    success2eternal = success2eternal && success3eternal && subseternal.success && additionalSubstEternal.success;
                                    success2event = success2event && success3event && subsevent.success && additionalSubstEvent.success;
                                }
                                if(success2eternal || success2event)
                                {
                                    Concept *c_eternal = Memory_Conceptualize(&resulteternal.term, currentTime);
                                    Concept *c_event = Memory_Conceptualize(&resultevent.term, currentTime);
                                    //if(c != NULL )//&& !Stamp_checkOverlap(&precondition_event->stamp, &imp.stamp))
                                    {
                                        //c->usage = Usage_use(c->usage, currentTime, false); <- destroys plasticity when memory is full (due to overcommitment to current concepts)
                                        //ETERNAL BELIEF UPDATE: //-- TODO STAMP
                                        if(c_eternal != NULL && success2eternal && Narsese_copulaEquals(imp.term.atoms[0], IMPLICATION))// && !Stamp_checkOverlap(&c_eternal->belief.stamp, &imp.stamp))
                                        {
                                            Truth oldTruth = c_eternal->belief.truth;
                                            if(!Stamp_Equal(&c_eternal->belief.stamp, &resulteternal.stamp))
                                            {
                                                c_eternal->belief = Inference_RevisionAndChoice(&c_eternal->belief, &resulteternal, currentTime, NULL); //concept can be generic!
                                                if(!Truth_Equal(&c_eternal->belief.truth, &oldTruth))
                                                {
                                                    Relation rel = Memory_relationOfBelief(&c_eternal->belief);
                                                    if(rel.isRelation)
                                                    {
                                                        c_eternal->isRelation = true;
                                                        fputs("DERIVED", stdout); Narsese_PrintTerm(&imp.term); fputs(" |- ", stdout); Narsese_PrintTerm(&resulteternal.term); puts("");
                                                    }
                                                    Memory_printAddedEvent(&c_eternal->belief.stamp, &c_eternal->belief, 1.0, false, true, false, true, false);
                                                    Memory_AddMemoryHelper(currentTime, &resulteternal.term, resulteternal.truth, &resulteternal.stamp, NULL, false);
                                                }
                                            }
                                        }
                                        //BELIEF EVENTS UPDATE:
                                        if(c_event != NULL && success2event && !Stamp_checkOverlap(&c_event->belief_spike.stamp, &imp.stamp) && Narsese_copulaEquals(imp.term.atoms[0], TEMPORAL_IMPLICATION))
                                        {
                                            Truth oldTruth = c_event->predicted_belief.truth;
                                            long oldOccurrenceTime = c_event->predicted_belief.occurrenceTime;
                                            c_event->predicted_belief = Inference_RevisionAndChoice(&c_event->predicted_belief, &resultevent, currentTime, NULL);
                                            if(!Truth_Equal(&c_event->predicted_belief.truth, &oldTruth) || c_event->predicted_belief.occurrenceTime != oldOccurrenceTime)
                                            {
                                                if(PRINT_PREDICTIONS_AS_DERIVATIONS)
                                                {
                                                    Memory_printAddedEvent(&c_event->predicted_belief.stamp, &c_event->predicted_belief, 1.0, false, true, false, true, false);
                                                }
                                            }
                                        }
                                        else
                                        if(c_event != NULL && success2event && !Stamp_checkOverlap(&c_event->belief_spike.stamp, &imp.stamp))
                                        {
                                            Truth oldTruth = c_event->belief_spike.truth;
                                            long oldOccurrenceTime = c_event->belief_spike.occurrenceTime;
                                            //if(!Stamp_Equal(&c->belief_spike.stamp, &resultevent.stamp))
                                            {
                                                c_event->belief_spike = Inference_RevisionAndChoice(&c_event->belief_spike, &resultevent, currentTime, NULL);
                                                Event eternal_event = Event_Eternalized(&c_event->belief_spike);
                                                c_event->belief = Inference_RevisionAndChoice(&c_event->belief, &eternal_event, currentTime, NULL);
                                                c_event->belief.creationTime = currentTime; //for metrics
                                                if(!Truth_Equal(&c_event->belief_spike.truth, &oldTruth) || c_event->belief_spike.occurrenceTime != oldOccurrenceTime)
                                                {
                                                    Memory_printAddedEvent(&c_event->belief_spike.stamp, &c_event->belief_spike, 1.0, false, true, false, true, false);
                                                }
                                            }
                                        }                          
                                    }
                                }
                            }
                        }
                        //fputs("RESOLVED", stdout); Narsese_PrintTerm(&cP->belief.term); puts("");
                    }
                }
                if(!declarative)
                {
                    break;
                }
            }
            for(int u=0; u<concepts.itemsAmount && Narsese_copulaEquals(imp.term.atoms[0], IMPLICATION) && declarative; u++)
            {
                Concept *cP = concepts.items[u].address;
                if(Variable_hasVariable(&cP->term, true, true, true) ||
                   cP->belief.truth.frequency <= 0.5)
                {
                    continue;
                }
                //<(<($1 * #1) --> (ocr * ocr)> && <(#1 * $2) --> (ocr * ocr)>) ==> <($1 * $2) --> (ocr * ocr)>>
                //==> &&  cons
                //1   2   3
                //0   1   2
                if(Narsese_copulaEquals(imp.term.atoms[1], CONJUNCTION) && //only declarative iterates
                   Narsese_copulaEquals(imp.term.atoms[2], INHERITANCE) &&
                   Narsese_copulaEquals(cP->belief.term.atoms[0], INHERITANCE))
                {
                    //try to satisfy both with the same variable mapping
                    Term conj_general = Term_ExtractSubterm(&imp.term, 1);
                    for(int v=0; v<concepts.itemsAmount; v++) //todo use occurrence time index instead
                    {
                        Concept *cP2 = concepts.items[v].address;
                        bool boost_skip = cP->belief.creationTime < currentTime-BELIEF_LAST_USED_TOLERANCE && cP2->belief.creationTime < currentTime-BELIEF_LAST_USED_TOLERANCE;
                        if(Variable_hasVariable(&cP2->term, true, true, true) || !Narsese_copulaEquals(cP2->belief.term.atoms[0], INHERITANCE) || 
                           boost_skip || cP2->belief.truth.frequency <= 0.5)
                        {
                            continue;
                        }
                        //Term conj = {0};
                        //conj.atoms[0] = Narsese_CopulaIndex(CONJUNCTION);
                        //bool success1 = Term_OverrideSubterm(&conj, 1, &cP->term);
                        //bool success2 = Term_OverrideSubterm(&conj, 2, &cP2->term);
                        bool successOfConj;
                        Event prec_eternal = Inference_BeliefConjunction(&cP->belief, &cP2->belief, &successOfConj);
                        if(successOfConj)
                        {
                            //fputs("!!!>>> ", stdout); Narsese_PrintTerm(&conj); puts("");
                            Substitution subst = Variable_Unify(&conj_general, &prec_eternal.term);
                            Implication imp_w_subst = imp;
                            bool success;
                            if(subst.success)
                            {
                                imp_w_subst.term = Variable_ApplySubstitute(imp.term, subst, &success);
                                //if(!Stamp_checkOverlap(&cP->belief.stamp, &cP2->belief.stamp))
                                { //TODO STAMP //--
                                    Event resulteternal = Inference_BeliefDeduction(&prec_eternal, &imp_w_subst); //b. :/:
                                    //Stamp resstamp = Stamp_make(&imp.stamp, &stamp_conj);
                                    //Stamp_print(&stamp_conj); puts("");
                                    //Stamp stamp = imp.stamp;// Stamp_make(&stamp_conj, &imp.stamp); //imp.stamp;
                                    /*if(Memory_AddMemoryHelper(currentTime, &cons, truth_cons, &stamp_conj, &stamp, false))
                                    {
                                        fputs("DERIVED RELATION", stdout); Narsese_PrintTerm(&imp.term); fputs(" |- ", stdout); Narsese_PrintTerm(&cons); puts("");
                                    }*/
                                    Concept *c_eternal = Memory_Conceptualize(&resulteternal.term, currentTime);
                                    if(c_eternal != NULL) // && !Stamp_checkOverlap(&prec_eternal.stamp, &imp.stamp)) //-- TODO STAMP
                                    {
                                        Truth oldTruth = c_eternal->belief.truth;
                                        if(!Stamp_Equal(&c_eternal->belief.stamp, &resulteternal.stamp))
                                        {
                                            c_eternal->belief = Inference_RevisionAndChoice(&c_eternal->belief, &resulteternal, currentTime, NULL); //concept can be generic!
                                            if(!Truth_Equal(&c_eternal->belief.truth, &oldTruth))
                                            {
                                                Relation rel = Memory_relationOfBelief(&c_eternal->belief);
                                                if(rel.isRelation)
                                                {
                                                    c_eternal->isRelation = true;
                                                    fputs("DERIVED", stdout); Narsese_PrintTerm(&imp.term); fputs(" |- ", stdout); Narsese_PrintTerm(&resulteternal.term); puts("");
                                                }
                                                Memory_printAddedEvent(&c_eternal->belief.stamp, &c_eternal->belief, 1.0, false, true, false, true, false);
                                                Memory_AddMemoryHelper(currentTime, &resulteternal.term, resulteternal.truth, &resulteternal.stamp, NULL, false);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                //TRIED IMPL: <(<($1 * $2) --> (loc1 * loc2)> && <($3 * $4) --> (ocr1 * ocr2)>) ==> <((<($1 * $3) --> (loc * ocr)> &/ <($2 * $4) --> (loc * ocr)>) &/ <({SELF} * ($1 * $2)) --> ^match>) =/> G>>
                // ==> &&   =/>  -->  -->      * * *  *              $1 $2  loc1  loc2  $3 $4 ocr1  ocr2
                // 1   2    3    4    5    6 7 8 9 10 11 12 13 14 15 16 17  18    19    20 21 22    23
                // 0   1    2    3    4    5 6 7 8  9 10 11 12 13 14 15 16  17    18    19 20 21    22
                //derive implications:
                //<(<(sample * $2) --> (loc * loc)> && <(A1 * $4) --> (ocr * ocr)>) ==> <((<($1 * $3) --> (loc * ocr)> &/ <($2 * $4) --> (loc * ocr)>) &/ <({SELF} * ($1 * $2)) --> ^match>) =/> G>>.
                //<(<($1 * sample) --> (loc * loc)> && <($3 * A1) --> (ocr * ocr)>) ==> <((<($1 * $3) --> (loc * ocr)> &/ <($2 * $4) --> (loc * ocr)>) &/ <({SELF} * ($1 * $2)) --> ^match>) =/> G>>.
                //fputs(">>>>", stdout); Narsese_PrintTerm(&imp.term); puts("");
                if(Narsese_copulaEquals(imp.term.atoms[1], CONJUNCTION) &&
                   (Narsese_copulaEquals(imp.term.atoms[2], TEMPORAL_IMPLICATION) || Narsese_copulaEquals(imp.term.atoms[2], IMPLICATION)) && //DOES NOT HAVE TO BE A TEMPORAL IMPLICATION
                   Narsese_copulaEquals(imp.term.atoms[3], INHERITANCE) &&
                   Narsese_copulaEquals(imp.term.atoms[4], INHERITANCE) &&
                   Narsese_copulaEquals(imp.term.atoms[7], PRODUCT) &&
                   Narsese_copulaEquals(imp.term.atoms[8], PRODUCT) &&
                   Narsese_copulaEquals(imp.term.atoms[9], PRODUCT) //&&
                   /*Narsese_copulaEquals(imp.term.atoms[10], PRODUCT)*/)// &&  //not needed to be a product, can be any relation
                   //Variable_isIndependentVariable(imp.term.atoms[15]) &&
                   //Variable_isIndependentVariable(imp.term.atoms[16]) &&
                   //Variable_isIndependentVariable(imp.term.atoms[19]) &&
                   //Variable_isIndependentVariable(imp.term.atoms[20]))
                {
                    Term loc1 = Term_ExtractSubterm(&imp.term, 17);
                    Term loc2 = Term_ExtractSubterm(&imp.term, 18);
                    Term relprod = Term_ExtractSubterm(&imp.term, 10);
                    //Term ocr1 = Term_ExtractSubterm(&imp.term, 21); //not needed
                    //Term ocr2 = Term_ExtractSubterm(&imp.term, 22); //not needed
                    Atom var1 = imp.term.atoms[15];
                    Atom var2 = imp.term.atoms[16];
                    Atom var3 = imp.term.atoms[19];
                    Atom var4 = imp.term.atoms[20];
                    Term matchTerm = cP->belief.term;
                    //fputs(">>>", stdout); Narsese_PrintTerm(&matchTerm); puts("");
                    //TRIED PRECON: <(A1 * C1) --> (ocr * ocr)>
                    //<(A1 * C1) --> (ocr1 * ocr2)>
                    //--> * * A1 C1 ocr1  ocr2
                    //1   2 3 4  5  6    7
                    //0   1 2 3  4  5    6
                    if(cP->belief.truth.frequency > 0.5 &&
                       Narsese_copulaEquals(matchTerm.atoms[0], INHERITANCE) && 
                       Narsese_copulaEquals(matchTerm.atoms[1], PRODUCT) //&& 
                       /*Narsese_copulaEquals(matchTerm.atoms[2], PRODUCT)*/)
                    {
                        //printf("USED %f \n", cP->belief.truth.frequency);
                        Term A1 = Term_ExtractSubterm(&matchTerm, 3);
                        Term C1 = Term_ExtractSubterm(&matchTerm, 4);
                        Term relprod_ = Term_ExtractSubterm(&matchTerm, 2);
                        //Term ocr1_ = Term_ExtractSubterm(&matchTerm, 5);
                        //Term ocr2_ = Term_ExtractSubterm(&matchTerm, 6);
                        //fputs("PRE-SEARCH", stdout); Narsese_PrintTerm(&matchTerm); puts("");
                        if(/*Term_Equal(&ocr1_, &ocr2_) && Term_Equal(&ocr1_, &ocr1) && Term_Equal(&ocr2_, &ocr2) &&*/
                           Term_Equal(&relprod, &relprod_)  &&
                           Term_Equal(&loc1, &loc2))
                        {
                            //fputs("IMPL: ", stdout); Narsese_PrintTerm(&imp.term); puts(""); // exit(0);
                            //fputs("SEARCHING FOR", stdout); Narsese_PrintTerm(&matchTerm); puts("");// exit(0);
                            // search for: <(LOC1 * A1) --> (loc * ocr)>
                            // search for: <(LOC2 * C1) --> (loc * ocr)>
                            for(int v=0; v<concepts.itemsAmount; v++) //todo use occurrence time index instead
                            {
                                Concept *cCand1 = concepts.items[v].address;
                                if(Variable_hasVariable(&cCand1->term, true, true, true) || cCand1->belief_spike.occurrenceTime < currentTime - EVENT_BELIEF_DISTANCE)
                                {
                                    continue;
                                }
                                Term candidate = cCand1->belief_spike.term;
                                if(Narsese_copulaEquals(candidate.atoms[0], INHERITANCE) && 
                                   Narsese_copulaEquals(candidate.atoms[1], PRODUCT) && 
                                   Narsese_copulaEquals(candidate.atoms[2], PRODUCT))
                                {
                                    //fputs("POTENTIAL ", stdout); Narsese_PrintTerm(&candidate); puts("");// exit(0);
                                    bool C1_and_A1_is_same_variable = Variable_isIndependentVariable(C1.atoms[0]) && Variable_isIndependentVariable(A1.atoms[0]) && C1.atoms[0] == A1.atoms[0];
                                    //<(LOC1 * A1) --> (loc * ocr)>
                                    //--> * * LOC1 A1 loc  ocr
                                    //1   2 3 4    5  6    7
                                    //0   1 2 3    4  5    6
                                    Term LOC1 = Term_ExtractSubterm(&candidate, 3); //obtained if the rest matches
                                    Term potentially_A1 = Term_ExtractSubterm(&candidate, 4);
                                    Term potentially_loc = Term_ExtractSubterm(&candidate, 5);
                                    Term potentially_ocr = Term_ExtractSubterm(&candidate, 6);
                                    //fputs("A ", stdout); Narsese_PrintTerm(&potentially_loc); fputs(" === ", stdout); Narsese_PrintTerm(&loc1); puts("");
                                    //fputs("B ", stdout); Narsese_PrintTerm(&potentially_ocr); fputs(" === ", stdout); Narsese_PrintTerm(&ocr1); puts("");
                                    //fputs("C ", stdout); Narsese_PrintTerm(&A1); fputs(" === ", stdout); Narsese_PrintTerm(&potentially_A1); puts("");
                                    //fputs("POTENTIAL ", stdout); Narsese_PrintTerm(&potentially_loc); fputs(" === ", stdout); Narsese_PrintTerm(&loc1); puts("");
                                    Substitution substA1 = Variable_Unify(&A1, &potentially_A1);
                                    if(Term_Equal(&potentially_loc, &loc1) &&
                                       //Term_Equal(&potentially_ocr, &ocr1) &&  //--NO
                                       Term_Equal(&potentially_A1, &A1)) //--
                                       //substA1.success) //--
                                    {
                                        //we have LOC1 now
                                        Stamp stamp1 = cCand1->belief_spike.stamp;
                                        long occurrenceTime1 = cCand1->belief_spike.occurrenceTime;
                                        Term CTerm1 = cCand1->term;
                                        //fputs("FOUND LOC1: ", stdout); Narsese_PrintTerm(&CTerm1); puts("");
                                        for(int w=0; w<concepts.itemsAmount; w++) //todo use ocCand1urrence time index instead
                                        {
                                            Concept *cCand2 = concepts.items[w].address;
                                            if(w == u || Variable_hasVariable(&cCand2->term, true, true, true) || cCand2->belief_spike.occurrenceTime <= currentTime-EVENT_BELIEF_DISTANCE)
                                            {
                                                continue;
                                            }
                                            Term candidate2 = cCand2->belief_spike.term;
                                            //<(LOC2 * C1) --> (loc * ocr)>
                                            //--> * * LOC2 C1 loc  ocr
                                            //1   2 3 4    5  6    7
                                            //0   1 2 3    4  5    6
                                            Term LOC2 = Term_ExtractSubterm(&candidate2, 3); //obtained if the rest matches
                                            Term potentially_C1 = Term_ExtractSubterm(&candidate2, 4);
                                            Substitution substC1 = Variable_Unify(&C1, &potentially_C1);
                                            assert(!substC1.success || (substC1.success && Term_Equal(&C1, &potentially_C1)), "WHATC");
                                            assert(!substA1.success || (substA1.success && Term_Equal(&A1, &potentially_A1)), "WHATA");
                                            if(/*substC1.success &&*/ Term_Equal(&C1, &potentially_C1)) //--
                                            {
                                                //Narsese_PrintTerm(&cCand2->term); fputs(" === ", stdout); Narsese_PrintTerm(&CTerm1); puts("");
                                                if(!Term_Equal(&cCand2->term, &CTerm1)) //already have a LOC1 term found, don't use the same concept!
                                                {
                                                    //printf("TEST: %d %d\n", (int) C1_and_A1_is_same_variable, (int) Term_Equal(&LOC2, &LOC1));
                                                    //Narsese_PrintTerm(&LOC2);  fputs(" === ", stdout); Narsese_PrintTerm(&LOC1); puts("");
                                                    //fputs("Z2 ", stdout); Narsese_PrintTerm(&potentially_C1); fputs(" === ", stdout); Narsese_PrintTerm(&potentially_A1_used); puts("");
                                                    Stamp stamp2 = cCand2->belief_spike.stamp;
                                                    long occurrenceTime2 = cCand2->belief_spike.occurrenceTime;
                                                    Term CTerm2 = cCand2->term;
                                                    //fputs("FOUND LOC2: ", stdout); Narsese_PrintTerm(&CTerm2); puts("");
                                                    if(stamp1.evidentialBase[0] < stamp2.evidentialBase[0]) //both terms got found
                                                    {
                                                        //IN_DEBUGNEW( printf("OCCURRENCE TIMES %ld %ld\n", occurrenceTime1, occurrenceTime2); )
                                                        //puts("BOTH TERMS FOUND"); exit(0);
                                                        Substitution subs1 = { .success = true };
                                                        subs1.map[var1] = LOC1;
                                                        subs1.map[var2] = LOC2;
                                                        subs1.map[var3] = A1;
                                                        subs1.map[var4] = C1;
                                                        bool success1;
                                                        Term version1 = Variable_ApplySubstitute(imp.term, subs1, &success1);
                                                        //fputs("$1: ", stdout); Narsese_PrintTerm(&LOC1); puts("");
                                                        //fputs("$2: ", stdout); Narsese_PrintTerm(&LOC2); puts("");
                                                        //fputs("$3: ", stdout); Narsese_PrintTerm(&A1); puts("");
                                                        //fputs("$4: ", stdout); Narsese_PrintTerm(&C1); puts("");
                                                        //fputs("SUBS INTO: ", stdout); Narsese_PrintTerm(&imp.term); puts("");
                                                        Stamp combinedStamp = Stamp_make(&stamp1, &stamp2);
                                                        if(success1)
                                                        {
                                                            Term newcontingency = Term_ExtractSubterm(&version1, 2);
                                                            bool success2;
                                                            newcontingency = Variable_ApplySubstitute(newcontingency, substA1, &success2);
                                                            if(success2)
                                                            {
                                                                bool added = Memory_AddMemoryHelper(currentTime, &newcontingency, Truth_Deduction(imp.truth, cP->belief.truth), &imp.stamp, &combinedStamp, false);
                                                                if(added)
                                                                {
                                                                    //print all only when added
                                                                    IN_DEBUGNEW
                                                                    (
                                                                        fputs("IMPL: ", stdout); Narsese_PrintTerm(&imp.term); puts("");
                                                                        fputs("SEARCHING FOR", stdout); Narsese_PrintTerm(&matchTerm); puts("");// exit(0);
                                                                        fputs("TRUTH", stdout); Truth_Print(&cP->belief.truth); puts("");// exit(0);
                                                                        fputs("FOUND LOC1: ", stdout); Narsese_PrintTerm(&CTerm1); puts("");
                                                                        fputs("FOUND LOC2: ", stdout); Narsese_PrintTerm(&CTerm2); puts("");
                                                                        fputs("TEST RESULT 1: ", stdout); Narsese_PrintTerm(&newcontingency); puts("");
                                                                    )
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
