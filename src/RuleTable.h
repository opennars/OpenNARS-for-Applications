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

#ifndef RULETABLE_H
#define RULETABLE_H

/////////////////
//  RuleTable  //
/////////////////
//The rule table which .c is generated at compile time
//by NAL_GenerateRuleTable

//References//
//----------//
#include "NAL.h"

//Methods//
//-------//
void RuleTable_Apply(Term term1, Term term2, Truth truth1, Truth truth2, long conclusionOccurrence, long occurrenceTimeOffset, Stamp conclusionStamp, 
                     long currentTime, double parentPriority, double conceptPriority, bool doublePremise, Concept *validation_concept, long validation_cid);
Term RuleTable_Reduce(Term term1, bool doublePremise);

#endif
