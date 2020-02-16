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

#ifndef NAR_H
#define NAR_H

//////////////////////////////
//  NAR - Yet Another NARS  //
//////////////////////////////

//References//
//-----------//
#include "Cycle.h"
#include "Config.h"

//Parameters//
//----------//
#define NAR_DEFAULT_TRUTH ((Truth) { .frequency = NAR_DEFAULT_FREQUENCY, .confidence = NAR_DEFAULT_CONFIDENCE })
extern long currentTime;

//Callback function types//
//-----------------------//
//typedef void (*Action)(void);     //already defined in Memory

//Methods//
//-------//
//Init/Reset system
void NAR_INIT();
//Run the system for a certain amount of cycles
void NAR_Cycles(int cycles);
//Add input
Event NAR_AddInput(Term term, char type, Truth truth, bool eternal);
Event NAR_AddInputBelief(Term term);
Event NAR_AddInputGoal(Term term);
//Add an operation
void NAR_AddOperation(Term term, Action procedure);

#endif
