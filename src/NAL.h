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
//!Syllogistic rules for Inheritance:
R2( (S --> M), (M --> P), |-, (S --> P), Truth_Deduction )
R2( (A --> B), (A --> C), |-, (C --> B), Truth_Induction )
R2( (A --> C), (B --> C), |-, (B --> A), Truth_Abduction )
R2( (A --> B), (B --> C), |-, (C --> A), Truth_Exemplification )
R2( S, (S --> P), |-, P, Truth_Deduction )
R2( P, (S --> P), |-, S, Truth_Abduction )
#endif

#if SEMANTIC_INFERENCE_NAL_LEVEL >= 2
//!Rules for Similarity:
R1( (S <-> P), |-, (P <-> S), Truth_StructuralIntersection )
R2( (M <-> P), (S <-> M), |-, (S <-> P), Truth_Resemblance )
R2( (P --> M), (S --> M), |-, (S <-> P), Truth_Comparison )
R2( (M --> P), (M --> S), |-, (S <-> P), Truth_Comparison )
R2( (M --> P), (S <-> M), |-, (S --> P), Truth_Analogy )
R2( (P --> M), (S <-> M), |-, (P --> S), Truth_Analogy )
R2( S, (S <-> P), |-, P, Truth_Analogy )
R2( S, (P <-> S), |-, P, Truth_Analogy )
//!Dealing with properties and instances:
R1( (S --> {P}), |-, (S <-> {P}), Truth_StructuralIntersection )
R1( ([S] --> P), |-, ([S] <-> P), Truth_StructuralIntersection )
R2( ({M} --> P), (S <-> M), |-, ({S} --> P), Truth_Analogy )
R2( (P --> [M]), (S <-> M), |-, (P --> [S]), Truth_Analogy )
R1( ({A} <-> {B}), |-, (A <-> B), Truth_StructuralIntersection )
R1( ([A] <-> [B]), |-, (A <-> B), Truth_StructuralIntersection )
#endif

#if SEMANTIC_INFERENCE_NAL_LEVEL >= 3
//!Set decomposition:
R1( ({A B} --> M), |-, <{A} --> M>, Truth_StructuralDeduction )
R1( ({A B} --> M), |-, <{B} --> M>, Truth_StructuralDeduction )
R1( (M --> [A B]), |-, <M --> [A]>, Truth_StructuralDeduction )
R1( (M --> [A B]), |-, <M --> [B]>, Truth_StructuralDeduction )
//!Extensional and intensional intersection decomposition:
R1( ((S | P) --> M), |-, (S --> M), Truth_StructuralDeduction )
R1( (M --> (S & P)), |-, (M --> S), Truth_StructuralDeduction )
R1( ((S | P) --> M), |-, (P --> M), Truth_StructuralDeduction )
R1( (M --> (S & P)), |-, (M --> P), Truth_StructuralDeduction )
R1( ((A ~ S) --> M), |-, (A --> M), Truth_StructuralDeduction )
R1( (M --> (B - S)), |-, (M --> B), Truth_StructuralDeduction )
R1( ((A ~ S) --> M), |-, (S --> M), Truth_StructuralDeductionNegated )
R1( (M --> (B - S)), |-, (M --> S), Truth_StructuralDeductionNegated )
//!Extensional and intensional intersection composition: (sets via reductions)
R2( (P --> M), (S --> M), |-, ((P | S) --> M), Truth_Intersection )
R2( (P --> M), (S --> M), |-, ((P & S) --> M), Truth_Union )
R2( (P --> M), (S --> M), |-, ((P ~ S) --> M), Truth_Difference )
R2( (M --> P), (M --> S), |-, (M --> (P & S)), Truth_Intersection )
R2( (M --> P), (M --> S), |-, (M --> (P | S)), Truth_Union )
R2( (M --> P), (M --> S), |-, (M --> (P - S)), Truth_Difference )
//!Extensional and intensional intersection decomposition:
R2( (S --> M), ((S | P) --> M), |-, (P --> M), Truth_DecomposePNN )
R2( (P --> M), ((S | P) --> M), |-, (S --> M), Truth_DecomposePNN )
R2( (S --> M), ((S & P) --> M), |-, (P --> M), Truth_DecomposeNPP )
R2( (P --> M), ((S & P) --> M), |-, (S --> M), Truth_DecomposeNPP )
R2( (S --> M), ((S ~ P) --> M), |-, (P --> M), Truth_DecomposePNP )
R2( (S --> M), ((P ~ S) --> M), |-, (P --> M), Truth_DecomposeNNN )
R2( (M --> S), (M --> (S & P)), |-, (M --> P), Truth_DecomposePNN )
R2( (M --> P), (M --> (S & P)), |-, (M --> S), Truth_DecomposePNN )
R2( (M --> S), (M --> (S | P)), |-, (M --> P), Truth_DecomposeNPP )
R2( (M --> P), (M --> (S | P)), |-, (M --> S), Truth_DecomposeNPP )
R2( (M --> S), (M --> (S - P)), |-, (M --> P), Truth_DecomposePNP )
R2( (M --> S), (M --> (P - S)), |-, (M --> P), Truth_DecomposeNNN )
#endif

#if SEMANTIC_INFERENCE_NAL_LEVEL >= 4
//!Transformation rules between product and image:
R1Bidirectional( ((A * B) --> R), -|-, (A --> (R /1 B)),  Truth_StructuralIntersection )
R1Bidirectional( ((A * B) --> R), -|-, (B --> (R /2 A)),  Truth_StructuralIntersection )
R1Bidirectional( (R --> (A * B)), -|-, ((R \\1 B) --> A), Truth_StructuralIntersection )
R1Bidirectional( (R --> (A * B)), -|-, ((R \\2 A) --> B), Truth_StructuralIntersection )
//!Comparative relations
R2( ({R} |-> [P]), ({S} |-> [P]), |-, (({R} * {S}) --> (+ P)), Truth_FrequencyGreater )
R2( ((A * B) --> (+ P)), ((B * C) --> (+ P)), |-, ((A * C) --> (+ P)), Truth_Deduction )
R2( ({R} |-> [P]), ({S} |-> [P]), |-, (({R} * {S}) --> (= P)), Truth_FrequencyEqual )
R2( ((A * B) --> (= P)), ((B * C) --> (= P)), |-, ((A * C) --> (= P)), Truth_Deduction )
R1( ((A * B) --> (= P)), |-, ((B * A) --> (= P)), Truth_StructuralIntersection )
//!Optional rules for more efficient reasoning about relation components:
R2( ((A * B) --> R), ((C * B) --> R), |-, (C --> A), Truth_Abduction )
R2( ((A * B) --> R), ((A * C) --> R), |-, (C --> B), Truth_Abduction )
R2( (R --> (A * B)), (R --> (C * B)), |-, (C --> A), Truth_Induction )
R2( (R --> (A * B)), (R --> (A * C)), |-, (C --> B), Truth_Induction )
R2( ((A * B) --> R), (C --> A), |-, ((C * B) --> R), Truth_Deduction )
R2( ((A * B) --> R), (A --> C), |-, ((C * B) --> R), Truth_Induction )
R2( ((A * B) --> R), (C <-> A), |-, ((C * B) --> R), Truth_Analogy )
R2( ((A * B) --> R), (C --> B), |-, ((A * C) --> R), Truth_Deduction )
R2( ((A * B) --> R), (B --> C), |-, ((A * C) --> R), Truth_Induction )
R2( ((A * B) --> R), (C <-> B), |-, ((A * C) --> R), Truth_Analogy )
R2( (R --> (A * B)), (A --> C), |-, (R --> (C * B)), Truth_Deduction )
R2( (R --> (A * B)), (C --> A), |-, (R --> (C * B)), Truth_Abduction )
R2( (R --> (A * B)), (C <-> A), |-, (R --> (C * B)), Truth_Analogy )
R2( (R --> (A * B)), (B --> C), |-, (R --> (A * C)), Truth_Deduction )
R2( (R --> (A * B)), (C --> B), |-, (R --> (A * C)), Truth_Abduction )
R2( (R --> (A * B)), (C <-> B), |-, (R --> (A * C)), Truth_Analogy )
R2( ((A * B) --> R), ((C * B) --> R), |-, (A <-> C), Truth_Comparison )
R2( ((A * B) --> R), ((A * C) --> R), |-, (B <-> C), Truth_Comparison )
R2( (R --> (A * B)), (R --> (C * B)), |-, (A <-> C), Truth_Comparison )
R2( (R --> (A * B)), (R --> (A * C)), |-, (B <-> C), Truth_Comparison )
#endif

#if SEMANTIC_INFERENCE_NAL_LEVEL >= 5
//!Negation conjunction and disjunction decomposition:
R1( (! A), |-, A, Truth_Negation )
R1( (A && B), |-, A, Truth_StructuralDeduction )
R1( (A && B), |-, B, Truth_StructuralDeduction )
R1( (A && B), |-, (B && A), Truth_StructuralIntersection )
R2( S, (&& S A), |-, A, Truth_DecomposePNN ) //Truth_AnonymousAnalogy in Cycle_SpecialInferences for var elimination
R2( S, (|| S A), |-, A, Truth_DecomposeNPP )
R2( S, (&& (! S) A), |-, A, Truth_DecomposeNNN )
R2( S, (|| (! S) A), |-, A, Truth_DecomposePPP )
//!Syllogistic rules for Implication:
R2( (S ==> M), (M ==> P), |-, (S ==> P), Truth_Deduction )
R2( (A ==> B), (A ==> C), |-, (C ==> B), Truth_Induction )
R2( (A ==> C), (B ==> C), |-, (B ==> A), Truth_Abduction )
R2( (A ==> B), (B ==> C), |-, (C ==> A), Truth_Exemplification )
//!Conditional composition for conjunction and disjunction:
R2( (A ==> C), (B ==> C), |-, ((A && B) ==> C), Truth_Union )
R2( (A ==> C), (B ==> C), |-, ((A || B) ==> C), Truth_Intersection )
R2( (C ==> A), (C ==> B), |-, (C ==> (A && B)), Truth_Intersection )
R2( (C ==> A), (C ==> B), |-, (C ==> (A || B)), Truth_Union )
//!Multi-conditional inference:
R2( ((S && P) ==> M), (S ==> M), |-, P, Truth_Abduction )
R2( ((C && M) ==> P), (S ==> M), |-, ((C && S) ==> P), Truth_Deduction )
R2( ((C && P) ==> M), ((C && S) ==> M), |-, (S ==> P), Truth_Abduction )
R2( ((C && M) ==> P), (M ==> S), |-, ((C && S) ==> P), Truth_Induction )
//!Rules for equivalence:
R1( (S <=> P), |-, (P <=> S), Truth_StructuralIntersection )
R2( (S ==> P), (P ==> S), |-, (S <=> P), Truth_Intersection )
R2( (P ==> M), (S ==> M), |-, (S <=> P), Truth_Comparison )
R2( (M ==> P), (M ==> S), |-, (S <=> P), Truth_Comparison )
R2( (M ==> P), (S <=> M), |-, (S ==> P), Truth_Analogy )
R2( (P ==> M), (S <=> M), |-, (P ==> S), Truth_Analogy )
R2( (M <=> P), (S <=> M), |-, (S <=> P), Truth_Resemblance )
#endif

#if SEMANTIC_INFERENCE_NAL_LEVEL == 5
//!Higher-order decomposition in Cycle_SpecialInferences (with var elimination in Cycle_SpecialInferences)
R2( A, (A ==> B), |-, B, Truth_Deduction )
R2( A, ((A && B) ==> C), |-, (B ==> C), Truth_Deduction )
R2( B, (A ==> B), |-, A, Truth_Abduction )
R2( A, (A <=> B), |-, B, Truth_Analogy )
#endif

#if SEMANTIC_INFERENCE_NAL_LEVEL >= 6
//!First var intro step:
R2VarIntro( (C --> A), (C --> B), |-, ((C --> B) ==> (C --> A)), Truth_Induction )
R2VarIntro( (A --> C), (B --> C), |-, ((B --> C) ==> (A --> C)), Truth_Induction )
R2VarIntro( (C --> A), (C --> B), |-, ((C --> B) <=> (C --> A)), Truth_Comparison )
R2VarIntro( (A --> C), (B --> C), |-, ((B --> C) <=> (A --> C)), Truth_Comparison )
R2VarIntro( (! (C --> A)), (C --> B), |-, ((C --> B) ==> (! (C --> A))), Truth_Induction )
R2VarIntro( (! (A --> C)), (B --> C), |-, ((B --> C) ==> (! (A --> C))), Truth_Induction )
R2VarIntro( (! (C --> A)), (C --> B), |-, ((C --> B) <=> (! (C --> A))), Truth_Comparison )
R2VarIntro( (! (A --> C)), (B --> C), |-, ((B --> C) <=> (! (A --> C))), Truth_Comparison )
R2VarIntro( (C --> A), (! (C --> B)), |-, ((! (C --> B)) ==> (C --> A)), Truth_Induction )
R2VarIntro( (A --> C), (! (B --> C)), |-, ((! (B --> C)) ==> (A --> C)), Truth_Induction )
R2VarIntro( (C --> A), (! (C --> B)), |-, ((! (C --> B)) <=> (C --> A)), Truth_Comparison )
R2VarIntro( (A --> C), (! (B --> C)), |-, ((! (B --> C)) <=> (A --> C)), Truth_Comparison )
R2VarIntro( (C --> A), (C --> B), |-, ((C --> B) && (C --> A)), Truth_Intersection )
R2VarIntro( (A --> C), (B --> C), |-, ((B --> C) && (A --> C)), Truth_Intersection )
R2VarIntro( (! (C --> A)), (C --> B), |-, ((C --> B) && (! (C --> A))), Truth_Intersection )
R2VarIntro( (! (A --> C)), (B --> C), |-, ((B --> C) && (! (A --> C))), Truth_Intersection )
R2VarIntro( (C --> A), (! (C --> B)), |-, ((! (C --> B)) && (C --> A)), Truth_Intersection )
R2VarIntro( (A --> C), (! (B --> C)), |-, ((! (B --> C)) && (A --> C)), Truth_Intersection )
//!Second var intro step:
R2VarIntro( (<$1 --> B> ==> <$1 --> C>), A, |-, (A && (<$1 --> B> ==> <$1 --> C>)), Truth_Intersection )
R2VarIntro( (<#1 --> B> && <#1 --> C>), A, |-, (A ==> (<#1 --> B> && <#1 --> C>)), Truth_Induction )
R2VarIntro( (B ==> C), A, |-, ((A && B) ==> C), Truth_Induction )
//!Relation symmetry, asymmetry, and transitivity:
R2VarIntro( ((A * B) --> R), ((B * A) --> S), |-, (((B * A) --> S) ==> ((A * B) --> R)), Truth_Induction )
R2VarIntro( (! ((B * A) --> R)), ((A * B) --> S), |-, (((A * B) --> S) ==> (! ((B * A) --> R))), Truth_Induction )
R2VarIntro( ((B * A) --> R), (! ((A * B) --> S)), |-, ((! ((A * B) --> S)) ==> ((B * A) --> R)), Truth_Induction )
R2( ((A * B) --> R), ((B * C) --> S), |-, (((A * B) --> R) && ((B * C) --> S)), Truth_Intersection )
R2VarIntro( ((A * C) --> M), (((A * B) --> R) && ((B * C) --> S)), |-, ((((A * B) --> R) && ((B * C) --> S)) ==> ((A * C) --> M)), Truth_Induction )
//!Variable elimination in Cycle_SpecialInferences
#endif

//Mandatory NAL7/8 is not optional and handled by sensorimotor inference, see Inference.h!

#endif

#ifdef H_NAL_REDUCTIONS

//NAL term reductions
//!Extensional intersection, union, conjunction reductions:
ReduceTerm( (A & A), A )
ReduceTerm( (A | A), A )
ReduceStatement( (A && A), A )
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
