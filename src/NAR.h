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

#ifndef H_NAR
#define H_NAR

//////////////////////////////////////
//  NAR - The main reasoner module  //
//////////////////////////////////////
//The API to send events to the reaasoner
//and to register operations it can invoke
//plus initialization and execution of inference steps

//References//
//-----------//
#include "Cycle.h"
#include "Narsese.h"
#include "Config.h"

//Parameters//
//----------//
#define NAR_DEFAULT_TRUTH ((Truth) { .frequency = NAR_DEFAULT_FREQUENCY, .confidence = NAR_DEFAULT_CONFIDENCE })
extern long currentTime;
extern double QUESTION_PRIMING;

//Callback function types//
//-----------------------//
//typedef Feedback (*Action)(Term);     //already defined in Memory

//Methods//
//-------//
//Init/Reset system
void NAR_INIT();
//Run the system for a certain amount of cycles
void NAR_Cycles(int cycles);
//Add input
Event NAR_AddInput(Term term, char type, Truth truth, bool eternal, double occurrenceTimeOffset);
Event NAR_AddInputBelief(Term term);
Event NAR_AddInputGoal(Term term);
//Add an operation
void NAR_AddOperation(char *atomname, Action procedure);
//Add an Narsese sentence:
void NAR_AddInputNarsese(char *narsese_sentence);
//Add an Narsese sentence with query functionality for questions:
void NAR_AddInputNarsese2(char *narsese_sentence, bool queryCommand, double answerTruthExpThreshold);
#endif
