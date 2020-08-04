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
//Declarative reasoning abilities not covered by the sensorimotor component

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
void NAL_DerivedEvent(Term conclusionTerm, long conclusionOccurrence, Truth conclusionTruth, Stamp stamp, long currentTime, double parentPriority, double conceptPriority, long occurrenceTimeOffset, Concept *validation_concept, long validation_cid);
//macro for syntactic representation, increases readability, double premise inference
#define R2(premise1, premise2, _, conclusion, truthFunction) NAL_GenerateRule(#premise1, #premise2, #conclusion, #truthFunction, true,false); NAL_GenerateRule(#premise2, #premise1, #conclusion, #truthFunction, true, true);
//macro for syntactic representation, increases readability, single premise inference
#define R1(premise1, _, conclusion, truthFunction) NAL_GenerateRule(#premise1, NULL, #conclusion, #truthFunction, false, false);
//macro for bidirectional transformation rules
#define RTrans(rep1, _, rep2, truthFunction) NAL_GenerateRule(#rep1, NULL, #rep2, #truthFunction, false, false); NAL_GenerateRule(#rep2, NULL, #rep1, #truthFunction, false, false);
//macro for term reductions
#define ReduceTerm(pattern, replacement) NAL_GenerateReduction("(" #pattern " --> M) ", "(" #replacement " --> M)"); NAL_GenerateReduction("(M --> " #pattern ")", "(M --> " #replacement ")");
//macro for statement reductions
#define ReduceStatement(pattern, replacement) NAL_GenerateReduction(#pattern, #replacement); 

#endif

//Inference rules//
//---------------//
#ifdef H_NAL_RULES

//NAL1 rules
R2( (S --> M), (M --> P), |-, (S --> P), Truth_Deduction )
R2( (A --> B), (A --> C), |-, (C --> B), Truth_Induction )
R2( (A --> C), (B --> C), |-, (B --> A), Truth_Abduction )
R2( (A --> B), (B --> C), |-, (C --> A), Truth_Exemplification )
//NAL2 rules
R1( (S <-> P), |-, (P <-> S), Truth_StructuralDeduction )
R1( (S --> {P}), |-, (S <-> {P}), Truth_StructuralDeduction )
R1( ([S] --> P), |-, ([S] <-> P), Truth_StructuralDeduction )
R2( (P --> M), (S --> M), |-, (S <-> P), Truth_Comparison )
R2( (M --> P), (M --> S), |-, (S <-> P), Truth_Comparison )
R2( (M --> P), (S <-> M), |-, (S --> P), Truth_Analogy )
R2( (P --> M), (S <-> M), |-, (P --> S), Truth_Analogy )
R2( (M <-> P), (S <-> M), |-, (S <-> P), Truth_Resemblance )
R1( ({A} <-> {B}), |-, (A <-> B), Truth_StructuralDeduction )
R1( ([A] <-> [B]), |-, (A <-> B), Truth_StructuralDeduction )
//NAL3 rules
R1( ((S | P) --> M), |-, (S --> M), Truth_StructuralDeduction )
R1( (M --> (S & P)), |-, (M --> S), Truth_StructuralDeduction )
R1( ((S | P) --> M), |-, (P --> M), Truth_StructuralDeduction )
R1( (M --> (S & P)), |-, (M --> P), Truth_StructuralDeduction )
R2( (P --> M), (S --> M), |-, ((S | P) --> M), Truth_Intersection )
R2( (P --> M), (S --> M), |-, ((S & P) --> M), Truth_Union )
R2( (P --> M), (S --> M), |-, ((S ~ P) --> M), Truth_Difference )
R2( (M --> P), (M --> S), |-, (M --> (P & S)), Truth_Intersection )
R2( (M --> P), (M --> S), |-, (M --> (P | S)), Truth_Union )
R2( (M --> P), (M --> S), |-, (M --> (P - S)), Truth_Difference )
//NAL4 rules
RTrans( ((A * B) --> R), -|-, (A --> (R /1 B)), Truth_StructuralDeduction )
RTrans( ((A * B) --> R), -|-, (B --> (R /2 A)), Truth_StructuralDeduction )
RTrans( (R --> (A * B)), -|-, ((R \\1 B) --> A), Truth_StructuralDeduction )
RTrans( (R --> (A * B)), -|-, ((R \\2 A) --> B), Truth_StructuralDeduction )
//NAL5 rules
R1( (! A), |-, A, Truth_Negation )
R1( (&& A B), |-, A, Truth_StructuralDeduction )
R1( (&& A B), |-, B, Truth_StructuralDeduction )
//NAL6 variable introduction
R2( (M --> A), (M --> B), |-, (($1 --> A) ==> ($1 --> B)), Truth_Induction )
R2( (M --> A), (M --> B), |-, (($1 --> B) ==> ($1 --> A)), Truth_Induction )
R2( (A --> M), (B --> M), |-, ((A --> $1) ==> (B --> $1)), Truth_Induction )
R2( (A --> M), (B --> M), |-, ((B --> $1) ==> (A --> $1)), Truth_Induction )
R2( ((A * B) --> R), ((B * A) --> R), |-, ((($1 * $2) --> R) ==> (($2 * $1) --> R)), Truth_Induction ) //symmetry
R2( ((A * B) --> R), (! ((B * A) --> R) ), |-, ((($1 * $2) --> R) ==> (! (($2 * $1) --> R))), Truth_Induction ) //antisymmetry
R2( (((A * B) | (B * C)) --> R), ((A * C) --> R), |-, (((($1 * #1) | (#1 * $2)) --> R) ==> (($1 * $2) --> R)), Truth_Induction ) //transitivity
//NAL5/7/8 temporal induction and conditional inference is handled by MSC links, see Inference.h!

#endif

#ifdef H_NAL_REDUCTIONS

//Extensional intersection, union, conjunction
ReduceTerm( (A & A), A )
ReduceTerm( (A | A), A )
ReduceStatement( (A && A), A )
//Extensional set
ReduceTerm( ({A} | {B}), {A B} )
//Intensional set
ReduceTerm( ([A] & [B]), [A B] )

#endif
