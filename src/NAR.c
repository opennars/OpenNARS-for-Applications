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

#include "NAR.h"

long currentTime = 1;
static bool initialized = false;
static int op_k = 0;
double QUESTION_PRIMING = QUESTION_PRIMING_INITIAL;

void NAR_INIT()
{
    assert(pow(TRUTH_PROJECTION_DECAY_INITIAL,EVENT_BELIEF_DISTANCE) >= MIN_CONFIDENCE, "Bad params, increase projection decay or decrease event belief distance!");
    Decision_INIT();
    Memory_INIT(); //clear data structures
    Event_INIT(); //reset base id counter
    Narsese_INIT();
    Cycle_INIT();
    currentTime = 1; //reset time
    initialized = true;
    op_k = 0;
}

void NAR_Cycles(int cycles)
{
    assert(initialized, "NAR not initialized yet, call NAR_INIT first!");
    for(int i=0; i<cycles; i++)
    {
        IN_DEBUG( puts("\nNew system cycle:\n----------"); )
        Cycle_Perform(currentTime);
        currentTime++;
    }
}

Event NAR_AddInput(Term term, char type, Truth truth, bool eternal, double occurrenceTimeOffset)
{
    assert(initialized, "NAR not initialized yet, call NAR_INIT first!");
    Event ev = Event_InputEvent(term, type, truth, occurrenceTimeOffset, currentTime);
    if(eternal)
    {
        ev.occurrenceTime = OCCURRENCE_ETERNAL;
    }
    Memory_AddInputEvent(&ev, currentTime);
    NAR_Cycles(1);
    return ev;
}

Event NAR_AddInputBelief(Term term)
{
    Event ret = NAR_AddInput(term, EVENT_TYPE_BELIEF, NAR_DEFAULT_TRUTH, false, 0);
    return ret;
}

Event NAR_AddInputGoal(Term term)
{
    return NAR_AddInput(term, EVENT_TYPE_GOAL, NAR_DEFAULT_TRUTH, false, 0);
}

void NAR_AddOperation(char *term_name, Action procedure)
{
    assert(procedure != 0, "Cannot add an operation with null-procedure");
    assert(initialized, "NAR not initialized yet, call NAR_INIT first!");
    Term term = Narsese_AtomicTerm(term_name);
    assert(term_name[0] == '^', "This atom does not belong to an operator!");
    //check if term already exists
    int existing_k = Memory_getOperationID(&term);
    //use the running k if not existing yet
    int use_k = existing_k == 0 ? op_k+1 : existing_k;
    //if it wasn't existing, also increase the running k and check if it's still in bounds
    assert(use_k <= OPERATIONS_MAX, "Too many operators, increase OPERATIONS_MAX!");
    if(existing_k == 0)
    {
        op_k++;
    }
    operations[use_k-1] = (Operation) { .term = term, .action = procedure };
}

void NAR_AddInputNarsese(char *narsese_sentence)
{
    Term term;
    Truth tv;
    char punctuation;
    int tense;
    double occurrenceTimeOffset;
    Narsese_Sentence(narsese_sentence, &term, &punctuation, &tense, &tv, &occurrenceTimeOffset);
#if STAGE==2
    //apply reduction rules to term:
    term = RuleTable_Reduce(term);
#endif    
    if(punctuation == '?')
    {
        //simplistic priming for Q&A:
        if(!Variable_hasVariable(&term, false, false, true))
        {
            Concept *c = Memory_FindConceptByTerm(&term); //triggers concept RESTORE
            if(c != NULL)
            {
                c->priority = MAX(c->priority, QUESTION_PRIMING);
            }
        }
        //answer questions:
        Truth best_truth = { .frequency = 0.0, .confidence = 1.0 };
        Truth best_truth_projected = { .frequency = 0.0, .confidence = 1.0 };
        Term best_term = {0};
        long answerOccurrenceTime = OCCURRENCE_ETERNAL;
        long answerCreationTime = 0;
        bool isImplication = Narsese_copulaEquals(term.atoms[0], TEMPORAL_IMPLICATION);
        fputs("Input: ", stdout);
        Narsese_PrintTerm(&term);
        fputs("?", stdout);
        puts(tense == 1 ? " :|:" : (tense == 2 ? " :\\:" : (tense == 3 ? " :/:" : ""))); 
        fflush(stdout);
        for(int i=0; i<concepts.itemsAmount; i++)
        {
            Concept *c = concepts.items[i].address;
            //compare the predicate of implication, or if it's not an implication, the term
            Term toCompare = isImplication ? Term_ExtractSubterm(&term, 2) : term; 
            if(!Variable_Unify2(&toCompare, &c->term, true).success)
            {
                goto Continue;
            }
            if(isImplication)
            {
                for(int op_k = 0; op_k<OPERATIONS_MAX; op_k++)
                {
                    for(int j=0; j<c->precondition_beliefs[op_k].itemsAmount; j++)
                    {
                        Implication *imp = &c->precondition_beliefs[op_k].array[j];
                        if(!Variable_Unify2(&term, &imp->term, true).success)
                        {
                            continue;
                        }
                        if(Truth_Expectation(imp->truth) >= Truth_Expectation(best_truth))
                        {
                            best_truth = imp->truth;
                            best_term = imp->term;
                            answerCreationTime = imp->creationTime;
                        }
                    }
                }
            }
            else
            if(tense)
            {
                if(c->belief_spike.type != EVENT_TYPE_DELETED && (tense == 1 || tense == 2))
                {
                    Truth potential_best_truth = Truth_Projection(c->belief_spike.truth, c->belief_spike.occurrenceTime, currentTime);
                    if( Truth_Expectation(potential_best_truth) >  Truth_Expectation(best_truth_projected) || //look at occcurrence time in case it's too far away to make a numerical distinction after truth projection:
                       (Truth_Expectation(potential_best_truth) == Truth_Expectation(best_truth_projected) && c->belief_spike.occurrenceTime > answerOccurrenceTime))
                    {
                        best_truth_projected = potential_best_truth;
                        best_truth = c->belief_spike.truth;
                        best_term = c->belief_spike.term;
                        answerOccurrenceTime = c->belief_spike.occurrenceTime;
                        answerCreationTime = c->belief_spike.creationTime;
                    }
                }
                if(c->predicted_belief.type != EVENT_TYPE_DELETED && (tense == 1 || tense == 3))
                {
                    Truth potential_best_truth = Truth_Projection(c->predicted_belief.truth, c->predicted_belief.occurrenceTime, currentTime);
                    if( Truth_Expectation(potential_best_truth) >  Truth_Expectation(best_truth_projected) || //look at occcurrence time in case it's too far away to make a numerical distinction after truth projection:
                       (Truth_Expectation(potential_best_truth) == Truth_Expectation(best_truth_projected) && c->predicted_belief.occurrenceTime > answerOccurrenceTime))
                    {
                        best_truth_projected = potential_best_truth;
                        best_truth = c->predicted_belief.truth;
                        best_term = c->predicted_belief.term;
                        answerOccurrenceTime = c->predicted_belief.occurrenceTime;
                        answerCreationTime = c->predicted_belief.creationTime;
                    }
                }
            }
            else
            {
                if(c->belief.type != EVENT_TYPE_DELETED && Truth_Expectation(c->belief.truth) >= Truth_Expectation(best_truth))
                {
                    best_truth = c->belief.truth;
                    best_term = c->belief.term;
                    answerCreationTime = c->belief.creationTime;
                }
            }
            Continue:;
        }
        fputs("Answer: ", stdout);
        if(best_truth.confidence == 1.0)
        {
            puts("None.");
        }
        else
        {
            Narsese_PrintTerm(&best_term);
            if(answerOccurrenceTime == OCCURRENCE_ETERNAL)
            {
                printf(". creationTime=%ld ", answerCreationTime);
            }
            else
            {
                printf(". :|: occurrenceTime=%ld creationTime=%ld ", answerOccurrenceTime, answerCreationTime);
            }
            Truth_Print(&best_truth);
        }
        fflush(stdout);
    }
    //input beliefs and goals
    else
    {
        // dont add the input if it is an eternal goal
        if(punctuation == '!' && !tense)
        {
            assert(false, "Eternal goals are not supported!\n");
        }
        else
        {
            assert(punctuation != '.' || tense < 2, "Future and past belief events are not supported!\n");
            NAR_AddInput(term, punctuation == '!' ? EVENT_TYPE_GOAL : EVENT_TYPE_BELIEF, tv, !tense, occurrenceTimeOffset);
        }
    }
}

