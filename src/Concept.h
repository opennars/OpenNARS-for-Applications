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

#ifndef H_CONCEPT
#define H_CONCEPT

///////////////////
//   Concept     //
///////////////////
//A concept named by a term

//References//
//-----------//
#include "FIFO.h"
#include "Table.h"

//Data structure//
//--------------//
typedef struct {
    long id;
    Term term;
    Event belief; //the highest confident eternal belief
    Event belief_spike;
    Event predicted_belief;
    Event goal_spike;
    Event goal; //overall desire for this concept
    Table precondition_beliefs[OPERATIONS_MAX+1];
    double priority;
    bool hasUserKnowledge;
    long processID; //avoids duplicate processing
} Concept;

//Methods//
//-------//
//Select a belief from the concept (belief, predicted_belief or belief_spike if close to queryTime distance)
Event Concept_SelectBelief(Concept *c, long queryTime);
//The priority of a concept in respect to the current time
double Concept_Priority(Concept *c, long currentTime);

#endif
