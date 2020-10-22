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

void NAR_INIT()
{
    assert(pow(TRUTH_PROJECTION_DECAY_INITIAL,EVENT_BELIEF_DISTANCE) >= MIN_CONFIDENCE, "Bad params, increase projection decay or decrease event belief distance!");
    Memory_INIT(); //clear data structures
    Event_INIT(); //reset base id counter
    Narsese_INIT();
    currentTime = 1; //reset time
    initialized = true;
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

Event NAR_AddInput(Term term, char type, Truth truth, bool eternal, bool isUserKnowledge)
{
    assert(initialized, "NAR not initialized yet, call NAR_INIT first!");
    Event ev = Event_InputEvent(term, type, truth, currentTime);
    if(eternal)
    {
        ev.occurrenceTime = OCCURRENCE_ETERNAL;
        ev.isUserKnowledge = isUserKnowledge;
    }
    Memory_AddInputEvent(&ev, currentTime);
    NAR_Cycles(1);
    return ev;
}

Event NAR_AddInputBelief(Term term)
{
    Event ret = NAR_AddInput(term, EVENT_TYPE_BELIEF, NAR_DEFAULT_TRUTH, false, false);
    return ret;
}

Event NAR_AddInputGoal(Term term)
{
    return NAR_AddInput(term, EVENT_TYPE_GOAL, NAR_DEFAULT_TRUTH, false, false);
}

void NAR_AddOperation(Term term, Action procedure)
{
    assert(initialized, "NAR not initialized yet, call NAR_INIT first!");
    char* term_name = Narsese_atomNames[(int) term.atoms[0]-1];
    assert(term_name[0] == '^', "This atom does not belong to an operator!");
    operations[Narsese_OperatorIndex(term_name) - 1] = (Operation) { .term = term, .action = procedure };
}

void NAR_AddInputNarsese(char *narsese_sentence)
{
    Term term;
    Truth tv;
    char punctuation;
    bool isEvent;
    bool isUserKnowledge;
    Narsese_Sentence(narsese_sentence, &term, &punctuation, &isEvent, &isUserKnowledge, &tv);
#if STAGE==2
    //apply reduction rules to term:
    term = RuleTable_Reduce(term, false);
#endif    
    if(punctuation == '?')
    {
        //answer questions:
        Truth best_truth = { .frequency = 0.0, .confidence = 1.0 };
        Truth best_truth_projected = {0};
        Term best_term = {0};
        long answerOccurrenceTime = OCCURRENCE_ETERNAL;
        long answerCreationTime = 0;
        bool isImplication = Narsese_copulaEquals(term.atoms[0], '$');
        fputs("Input: ", stdout);
        Narsese_PrintTerm(&term);
        fputs("?", stdout);
        puts(isEvent ? " :|:" : ""); 
        fflush(stdout);
        for(int i=0; i<concepts.itemsAmount; i++)
        {
            Concept *c = concepts.items[i].address;
            //compare the predicate of implication, or if it's not an implication, the term
            Term toCompare = isImplication ? Term_ExtractSubterm(&term, 2) : term; 
            if(!Variable_Unify(&toCompare, &c->term).success)
            {
                goto Continue;
            }
            if(isImplication)
            {
                Term subject = Term_ExtractSubterm(&term, 1);
                int op_k = Narsese_getOperationID(&subject);
                for(int j=0; j<c->precondition_beliefs[op_k].itemsAmount; j++)
                {
                    Implication *imp = &c->precondition_beliefs[op_k].array[j];
                    if(!Variable_Unify(&term, &imp->term).success)
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
            else
            if(isEvent)
            {
                if(c->belief_spike.type != EVENT_TYPE_DELETED)
                {
                    Truth potential_best_truth = Truth_Projection(c->belief_spike.truth, c->belief_spike.occurrenceTime, currentTime);
                    if(Truth_Expectation(potential_best_truth) >= Truth_Expectation(best_truth_projected))
                    {
                        best_truth_projected = potential_best_truth;
                        best_truth = c->belief_spike.truth;
                        best_term = c->belief_spike.term;
                        answerOccurrenceTime = c->belief_spike.occurrenceTime;
                        answerCreationTime = c->belief_spike.creationTime;
                    }
                }
                if(c->predicted_belief.type != EVENT_TYPE_DELETED)
                {
                    Truth potential_best_truth = Truth_Projection(c->predicted_belief.truth, c->predicted_belief.occurrenceTime, currentTime);
                    if(Truth_Expectation(potential_best_truth) >= Truth_Expectation(best_truth_projected))
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
        if(punctuation == '!' && !isEvent)
        {
            puts("Warning: Eternal goals are not supported, input is ignored!\n");
        }
        else
        {
            NAR_AddInput(term, punctuation == '!' ? EVENT_TYPE_GOAL : EVENT_TYPE_BELIEF, tv, !isEvent, isUserKnowledge);
        }
    }
}

