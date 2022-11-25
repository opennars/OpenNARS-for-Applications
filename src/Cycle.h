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
 
#ifndef H_CYCLE
#define H_CYCLE

///////////////////////////////////
//  NAR Control Cycle            //
///////////////////////////////////
//The overall inference control cycle of ONA
//Special properties:
//- Needs a constant upper bound of time to complete
//- Handles temporal, procedural, and declarative reasoning

//References//
//----------//
#include "Globals.h"
#include "Decision.h"
#include "Inference.h"
#include "RuleTable.h"
#include "Variable.h"
#include "Stats.h"
#include "./NetworkNAR/Metric.h"

//Methods//
//-------//
//Apply one operating cyle
void Cycle_Perform(long currentTime);
//Init cycle module
void Cycle_INIT();

#endif
