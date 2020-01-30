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
void NAL_DerivedEvent(Term conclusionTerm, long conclusionOccurrence, Truth conclusionTruth, Stamp stamp, long currentTime, double parentPriority);
//macro for syntactic representation, increases readability, double premise inference
#define R2(premise1, premise2, _, conclusion, truthFunction) NAL_GenerateRule(#premise1, #premise2, #conclusion, #truthFunction, true,false); NAL_GenerateRule(#premise2, #premise1, #conclusion, #truthFunction, true, true);
//macro for syntactic representation, increases readability, single premise inference
#define R1(premise1, _, conclusion, truthFunction) NAL_GenerateRule(#premise1, NULL, #conclusion, #truthFunction, false, false);
//macro for bidirectional transformation rules
#define RTrans(rep1, _, rep2, truthFunction) R1( rep1, _, rep2, truthFunction ); R1( rep2, _, rep1, truthFunction );
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
R2( (A --> B), (A --> C), |-, (C --> B), Truth_Abduction )
R2( (A --> C), (B --> C), |-, (B --> A), Truth_Induction )
R2( (A --> B), (B --> C), |-, (C --> A), Truth_Exemplification )
//NAL2 rules
R2( (P --> M), (S --> M), |-, (S <-> P), Truth_Comparison )
R2( (M --> P), (M --> S), |-, (S <-> P), Truth_Comparison )
R2( (M --> P), (S <-> M), |-, (S --> P), Truth_Analogy )
R2( (P --> M), (S <-> M), |-, (P --> S), Truth_Analogy )
R2( (M <-> P), (S <-> M), |-, (S <-> P), Truth_Resemblance )
R1( ({A} <-> {B}), |-, (A <-> B), Truth_StructuralDeduction )
R1( ([A] <-> [B]), |-, (A <-> B), Truth_StructuralDeduction )
//NAL3 rules
R1( ((S | P) --> M), |-, (S --> M), Truth_StructuralDeduction )
R1( (M --> (S & P)), |-, (S --> M), Truth_StructuralDeduction )
R1( ((S | P) --> M), |-, (P --> M), Truth_StructuralDeduction )
R1( (M --> (S & P)), |-, (P --> M), Truth_StructuralDeduction )
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
//NAL5/7/8 temporal induction and detachment is handled by MSC links, see Inference.h!

#endif

#ifdef H_NAL_REDUCTIONS

//Extensional intersection and union
ReduceTerm( (A & A), A )
ReduceTerm( (A | A), A )
//Extensional set
ReduceTerm( ({A} & {B}), {A B} )
//Intensional set
ReduceTerm( ([A] & [B]), [A B] )
//Statement reductions (due to identities)
ReduceStatement((S --> {P}), (S <-> {P}) )
ReduceStatement(([S] --> P), ([S] <-> P) )

#endif
