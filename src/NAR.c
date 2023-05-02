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

static void NAR_PrintAnswer(Stamp stamp, Term best_term, Truth best_truth, long answerOccurrenceTime, long answerCreationTime)
{
    fputs("Answer: ", stdout);
    if(best_truth.confidence == 1.1)
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
        Stamp_print(&stamp);
        fputs(" ", stdout);
        Truth_Print(&best_truth);
    }
    fflush(stdout);
}

void NAR_AddInputNarsese2(char *narsese_sentence, bool queryCommand, double answerTruthExpThreshold)
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
        //answer questions:
        Truth best_truth = { .frequency = 0.0, .confidence = 1.1 };
        Truth best_truth_projected = { .frequency = 0.0, .confidence = 1.0 };
        Concept* best_belief_concept = NULL;
        Term best_term = {0};
        Stamp best_stamp = {0};
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
                        if(queryCommand && Truth_Expectation(imp->truth) > answerTruthExpThreshold)
                        {
                            NAR_PrintAnswer(imp->stamp, imp->term, imp->truth, answerOccurrenceTime, imp->creationTime);
                        }
                        if(Truth_Expectation(imp->truth) >= Truth_Expectation(best_truth))
                        {
                            best_stamp = imp->stamp;
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
                    if(queryCommand && Truth_Expectation(potential_best_truth) > answerTruthExpThreshold)
                    {
                        NAR_PrintAnswer(c->belief_spike.stamp, c->belief_spike.term, c->belief_spike.truth, c->belief_spike.occurrenceTime, c->belief_spike.creationTime);
                    }
                    if( Truth_Expectation(potential_best_truth) >  Truth_Expectation(best_truth_projected) || //look at occcurrence time in case it's too far away to make a numerical distinction after truth projection:
                       (Truth_Expectation(potential_best_truth) == Truth_Expectation(best_truth_projected) && c->belief_spike.occurrenceTime > answerOccurrenceTime))
                    {
                        best_stamp = c->belief_spike.stamp;
                        best_truth_projected = potential_best_truth;
                        best_truth = c->belief_spike.truth;
                        best_term = c->belief_spike.term;
                        best_belief_concept = c;
                        answerOccurrenceTime = c->belief_spike.occurrenceTime;
                        answerCreationTime = c->belief_spike.creationTime;
                    }
                }
                if(c->predicted_belief.type != EVENT_TYPE_DELETED && (tense == 1 || tense == 3))
                {
                    Truth potential_best_truth = Truth_Projection(c->predicted_belief.truth, c->predicted_belief.occurrenceTime, currentTime);
                    if(queryCommand && Truth_Expectation(potential_best_truth) > answerTruthExpThreshold)
                    {
                        NAR_PrintAnswer(c->predicted_belief.stamp, c->predicted_belief.term, c->predicted_belief.truth, c->predicted_belief.occurrenceTime, c->predicted_belief.creationTime);
                    }
                    if( Truth_Expectation(potential_best_truth) >  Truth_Expectation(best_truth_projected) || //look at occcurrence time in case it's too far away to make a numerical distinction after truth projection:
                       (Truth_Expectation(potential_best_truth) == Truth_Expectation(best_truth_projected) && c->predicted_belief.occurrenceTime > answerOccurrenceTime))
                    {
                        best_stamp = c->predicted_belief.stamp;
                        best_truth_projected = potential_best_truth;
                        best_truth = c->predicted_belief.truth;
                        best_term = c->predicted_belief.term;
                        best_belief_concept = c;
                        answerOccurrenceTime = c->predicted_belief.occurrenceTime;
                        answerCreationTime = c->predicted_belief.creationTime;
                    }
                }
            }
            else
            {
                if(c->belief.type != EVENT_TYPE_DELETED && queryCommand && Truth_Expectation(c->belief.truth) > answerTruthExpThreshold)
                {
                    NAR_PrintAnswer(c->belief.stamp, c->belief.term, c->belief.truth, c->belief.occurrenceTime, c->belief.creationTime);
                }
                if(c->belief.type != EVENT_TYPE_DELETED && Truth_Expectation(c->belief.truth) >= Truth_Expectation(best_truth))
                {
                    best_stamp = c->belief.stamp;
                    best_truth = c->belief.truth;
                    best_term = c->belief.term;
                    best_belief_concept = c;
                    answerCreationTime = c->belief.creationTime;
                }
            }
            Continue:;
        }
        //simplistic priming for Q&A:
        if(best_belief_concept != NULL && QUESTION_PRIMING > 0.0)
        {
            best_belief_concept->priority = MAX(best_belief_concept->priority, QUESTION_PRIMING);
            best_belief_concept->usage = Usage_use(best_belief_concept->usage, currentTime, tense == 0 ? true : false);
        }
        if(!queryCommand)
        {
            NAR_PrintAnswer(best_stamp, best_term, best_truth, answerOccurrenceTime, answerCreationTime);
        }
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

void NAR_AddInputNarsese(char *narsese_sentence)
{
    NAR_AddInputNarsese2(narsese_sentence, false, 0.0);
}
