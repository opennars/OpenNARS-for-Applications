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
void NAL_DerivedEvent(Term conclusionTerm, long conclusionOccurrence, Truth conclusionTruth, Stamp stamp, long currentTime, double parentPriority, double conceptPriority, double occurrenceTimeOffset, Concept *validation_concept, long validation_cid, bool varIntro);
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
R2( ({M} --> P), (S <-> M), |-, ({S} --> P), Truth_Analogy )
R2( (P --> [M]), (S <-> M), |-, (P --> [S]), Truth_Analogy )
R2( (M <-> P), (S <-> M), |-, (S <-> P), Truth_Resemblance )
R1( ({A} <-> {B}), |-, (A <-> B), Truth_StructuralDeduction )
R1( ([A] <-> [B]), |-, (A <-> B), Truth_StructuralDeduction )
//NAL3 rules
R1( ({A B} --> M), |-, <{A} --> M>, Truth_StructuralDeduction )
R1( ({A B} --> M), |-, <{B} --> M>, Truth_StructuralDeduction )
R1( (M --> [A B]), |-, <M --> [A]>, Truth_StructuralDeduction )
R1( (M --> [A B]), |-, <M --> [B]>, Truth_StructuralDeduction )
R1( ((S | P) --> M), |-, (S --> M), Truth_StructuralDeduction )
R1( (M --> (S & P)), |-, (M --> S), Truth_StructuralDeduction )
R1( ((S | P) --> M), |-, (P --> M), Truth_StructuralDeduction )
R1( (M --> (S & P)), |-, (M --> P), Truth_StructuralDeduction )
R1( ((A ~ S) --> M), |-, (A --> M), Truth_StructuralDeduction )
R1( (M --> (B - S)), |-, (M --> B), Truth_StructuralDeduction )
R1( ((A ~ S) --> M), |-, (S --> M), Truth_StructuralDeductionNegated )
R1( (M --> (B - S)), |-, (M --> S), Truth_StructuralDeductionNegated )
R2( (P --> M), (S --> M), |-, ((P | S) --> M), Truth_Intersection )
R2( (P --> M), (S --> M), |-, ((P & S) --> M), Truth_Union )
R2( (P --> M), (S --> M), |-, ((P ~ S) --> M), Truth_Difference )
R2( (M --> P), (M --> S), |-, (M --> (P & S)), Truth_Intersection )
R2( (M --> P), (M --> S), |-, (M --> (P | S)), Truth_Union )
R2( (M --> P), (M --> S), |-, (M --> (P - S)), Truth_Difference )
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
//NAL4 rules
RTrans( ((A * B) --> R), -|-, (A --> (R /1 B)), Truth_StructuralDeduction )
RTrans( ((A * B) --> R), -|-, (B --> (R /2 A)), Truth_StructuralDeduction )
RTrans( (R --> (A * B)), -|-, ((R \\1 B) --> A), Truth_StructuralDeduction )
RTrans( (R --> (A * B)), -|-, ((R \\2 A) --> B), Truth_StructuralDeduction )
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
//NAL5 rules
R1( (! A), |-, A, Truth_Negation )
R1( (A && B), |-, A, Truth_StructuralDeduction )
R1( (A && B), |-, B, Truth_StructuralDeduction )
R1( (A && B), |-, (B && A), Truth_StructuralDeduction )
R2( (S ==> M), (M ==> P), |-, (S ==> P), Truth_Deduction )
R2( (A ==> B), (A ==> C), |-, (C ==> B), Truth_Induction )
R2( (A ==> C), (B ==> C), |-, (B ==> A), Truth_Abduction )
//NAL5/6 rules:
R2( (C --> A), (C --> B), |-, ((C --> B) ==> (C --> A)), Truth_Induction )
R2( (A --> C), (B --> C), |-, ((A --> C) ==> (B --> C)), Truth_Induction )
R2( (C --> A), (C --> B), |-, ((C --> B) && (C --> A)), Truth_Intersection )
R2( (A --> C), (B --> C), |-, ((A --> C) && (B --> C)), Truth_Intersection )
R2( (A ==> C), (B ==> C), |-, ((A && B) ==> C), Truth_Induction )
R2( A, (B ==> C), |-, ((A && B) ==> C), Truth_Induction )
R2( A, (B ==> C), |-, (A && (B ==> C)), Truth_Intersection )
//R2( A, (A ==> B), |-, B, Truth_Deduction ) //part of Cycle_SpecialInferences due to the need to eliminate variables
//R2( A, ((A && B) ==> C), |-, (B ==> C), Truth_Deduction ) //part of Cycle_SpecialInferences due to the need to eliminate variables
//R2( B, (A ==> B), |-, A, Truth_Abduction ) //part of Cycle_SpecialInferences due to the need to eliminate variables
//NAL6 rules
R2( ((A * B) --> R), ((B * A) --> S), |-, (((A * B) --> S) ==> ((B * A) --> R)), Truth_Induction )
R2( (! ((B * A) --> R)), ((A * B) --> S), |-, (((A * B) --> S) ==> (! ((B * A) --> R))), Truth_Induction )
R2( ((A * B) --> R), ((B * C) --> S), |-, (((A * B) --> R) && ((B * C) --> S)), Truth_Intersection )
R2( ((A * C) --> M), (((A * B) --> R) && ((B * C) --> S)), |-, ((((A * B) --> R) && ((B * C) --> S)) ==> ((A * C) --> M)), Truth_Induction )
//new NAL7 substitution rules to allow semantic inference to create additional contingencies:
//consequent
R2( (A =/> B), (S ==> B), |-, (A =/> S), Truth_Abduction )
R2( (A =/> B), (B ==> S), |-, (A =/> S), Truth_Deduction )
R2( (A =/> (P --> B)), (P <-> S), |-, (A =/> (S --> B)), Truth_Analogy )
R2( (A =/> (B --> P)), (P <-> S), |-, (A =/> (B --> S)), Truth_Analogy )
//first sequence element (deeper isn't currently linked)
R2( ((A &/ B) =/> C), (A ==> S), |-, ((S &/ B) =/> C), Truth_Abduction )
R2( ((A &/ B) =/> C), (S ==> A), |-, ((S &/ B) =/> C), Truth_Deduction )
R2( (((P --> A) &/ B) =/> C), (A <-> S), |-, (((P --> S) &/ B) =/> C), Truth_Analogy )
R2( (((A --> P) &/ B) =/> C), (A <-> S), |-, (((S --> P) &/ B) =/> C), Truth_Analogy )
//second sequence element (deeper isn't currently linked)
R2( ((A &/ B) =/> C), (B ==> S), |-, ((A &/ S) =/> C), Truth_Abduction )
R2( ((A &/ B) =/> C), (S ==> B), |-, ((A &/ S) =/> C), Truth_Deduction )
R2( ((A &/ (P --> B)) =/> C), (B <-> S), |-, ((A &/ (P --> S)) =/> C), Truth_Analogy )
R2( ((A &/ (B --> P)) =/> C), (B <-> S), |-, ((A &/ (S --> P)) =/> C), Truth_Analogy )
//Other NAL7/8 temporal induction, conditional inference and introduction of variables is handled by sensorimotor inference, see Inference.h!

#endif

#ifdef H_NAL_REDUCTIONS

//Extensional intersection, union, conjunction
ReduceTerm( (A & A), A )
ReduceTerm( (A | A), A )
ReduceStatement( (A && A), A )
//Extensional set
ReduceTerm( ({A} | {B}), {A B} )
ReduceTerm( ({A B} | {C}), {(A . B) C} )
ReduceTerm( ({C} | {A B}), {C (A . B)} )
//Intensional set
ReduceTerm( ([A] & [B]), [A B] )
ReduceTerm( ([A B] & [C]), [(A . B) C] )
ReduceTerm( ([A] & [B C]), [A (B . C)] )
//Reduce for set element copula
ReduceTerm( {(A . B)}, {A B} )
ReduceTerm( [(A . B)], [A B] )

#endif
