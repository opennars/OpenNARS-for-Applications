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

#ifndef H_NAL
#define H_NAL

///////////////////
// NAL ecosystem //
///////////////////
//Declarative NAL 1-6 reasoning abilities not covered by Inference.h

//References//
//----------//
#include "Truth.h"
#include "Stamp.h"
#include "Narsese.h"
#include "Memory.h"

//Methods//
//-------//
//Generates inference rule code
void NAL_GenerateRuleTable();
//Method for the derivation of new events as called by the generated rule table
void NAL_DerivedEvent(Term conclusionTerm, long conclusionOccurrence, Truth conclusionTruth, Stamp stamp, long currentTime, double parentPriority, double conceptPriority, double occurrenceTimeOffset, Concept *validation_concept, long validation_cid, bool varIntro, bool allowOnlyExtVarIntroAndTwoIndependentVars);
//macro for syntactic representation, increases readability, double premise inference
#define R2(premise1, premise2, _, conclusion, truthFunction)         NAL_GenerateRule(#premise1, #premise2, #conclusion, #truthFunction, true, false, false); NAL_GenerateRule(#premise2, #premise1, #conclusion, #truthFunction, true, true, false);
#define R2VarIntro(premise1, premise2, _, conclusion, truthFunction) NAL_GenerateRule(#premise1, #premise2, #conclusion, #truthFunction, true, false, true);  NAL_GenerateRule(#premise2, #premise1, #conclusion, #truthFunction, true, true, true);
//macro for syntactic representation, increases readability, single premise inference
#define R1(premise1, _, conclusion, truthFunction) NAL_GenerateRule(#premise1, NULL, #conclusion, #truthFunction, false, false, false);
//macro for bidirectional transformation rules
#define R1Bidirectional(rep1, _, rep2, truthFunction) NAL_GenerateRule(#rep1, NULL, #rep2, #truthFunction, false, false, false); NAL_GenerateRule(#rep2, NULL, #rep1, #truthFunction, false, false, false);
//macro for term reductions
#define ReduceTerm(pattern, replacement) NAL_GenerateReduction("(" #pattern " --> M) ", "(" #replacement " --> M)"); NAL_GenerateReduction("(M --> " #pattern ")", "(M --> " #replacement ")");
//macro for statement reductions
#define ReduceStatement(pattern, replacement) NAL_GenerateReduction(#pattern, #replacement); 

#endif

//Inference rules//
//---------------//
#ifdef H_NAL_RULES

#if SEMANTIC_INFERENCE_NAL_LEVEL >= 1
#endif

#if SEMANTIC_INFERENCE_NAL_LEVEL >= 2
#endif

#if SEMANTIC_INFERENCE_NAL_LEVEL >= 3
#endif

#if SEMANTIC_INFERENCE_NAL_LEVEL >= 4
#endif

#if SEMANTIC_INFERENCE_NAL_LEVEL >= 5
#endif

#if SEMANTIC_INFERENCE_NAL_LEVEL == 5
#endif

#if SEMANTIC_INFERENCE_NAL_LEVEL >= 6
#endif

//Mandatory NAL7/8 is not optional and handled by sensorimotor inference, see Inference.h!

#endif

#ifdef H_NAL_REDUCTIONS

//NAL term reductions
//!Extensional intersection, union, conjunction reductions:
ReduceTerm( (A & A), A )
ReduceTerm( (A | A), A )
ReduceStatement( (A && A), A )
ReduceStatement( (A || A), A )
//!Extensional set reductions:
ReduceTerm( ({A} | {B}), {A B} )
ReduceTerm( ({A B} | {C}), {(A . B) C} )
ReduceTerm( ({C} | {A B}), {C (A . B)} )
//!Intensional set reductions:
ReduceTerm( ([A] & [B]), [A B] )
ReduceTerm( ([A B] & [C]), [(A . B) C] )
ReduceTerm( ([A] & [B C]), [A (B . C)] )
//!Reduction for set element copula:
ReduceTerm( {(A . B)}, {A B} )
ReduceTerm( [(A . B)], [A B] )

#endif
