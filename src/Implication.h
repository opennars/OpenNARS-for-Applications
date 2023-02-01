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

#ifndef H_IMPLICATION
#define H_IMPLICATION

//////////////////////
//  Implication     //
//////////////////////
//Temporal implications, essentially allowing the system to predict
//events from other events.
//Usually, in learning scenarios:
//- Positive evidence of a temporal implication comes from a successful prediction
//- Negative evidence of a temporal implication comes from a prediction failure

//References//
//----------//
#include "Term.h"
#include "Stamp.h"

//Data structure//
//--------------//
typedef struct {
    Term term;
    Truth truth;
    Stamp stamp;
    //for deciding occurrence time of conclusion:
    double occurrenceTimeOffset;
    //for efficient spike propagation:
    void *sourceConcept;
    long sourceConceptId; //to check whether it's still the same
    long creationTime;
} Implication;

#endif


