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
#include "Usage.h"

//Data structure//
//--------------//
typedef struct {
    long id;
    Usage usage;
    Term term;
    Event belief; //the highest confident eternal belief
    Event belief_spike;
    Event predicted_belief;
    Event goal_spike;
    Table precondition_beliefs[OPERATIONS_MAX+1];
    Table implied_contingencies; //righthand of ==> includes =/> with op
    double priority;
    long processID; //avoids duplicate processing
    long processID2; //avoids duplicate processing
    long lastSensorimotorActivation;
    bool isResultSequence; //whether the concept is a sequence of form ((A ^Op) Result)
} Concept;

//Methods//
//-------//
//todo

#endif
