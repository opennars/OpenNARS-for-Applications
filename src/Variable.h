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

#ifndef H_VARIABLE
#define H_VARIABLE

////////////////////////
// Variable utilities //
////////////////////////
//Support for NAL-6 variables

//References//
//-----------//
#include "Narsese.h"

//Data structure//
//--------------//
//Substitution, mapping variable atoms to terms
typedef struct {
    Term map[TERMS_MAX];
    bool success;
} Substitution;

//Methods//
//-------//
bool Variable_isIndependentVariable(Atom atom);
bool Variable_isDependentVariable(Atom atom);
bool Variable_isQueryVariable(Atom atom);
bool Variable_isVariable(Atom atom);
bool Variable_hasVariable(Term *term, bool independent, bool dependent, bool query);
Substitution Variable_Unify(Term *general, Term *specific);
Term Variable_ApplySubstitute(Term term, Substitution substitution);
Term IntroduceImplicationVariables(Term implication);

#endif
