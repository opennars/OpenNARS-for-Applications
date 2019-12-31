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
#include "Encode.h"
#include "Memory.h"

//Methods//
//-------//
//Generates inference rule code
void NAL_GenerateRuleTable();
void NAL_DerivedEvent(Term conclusionTerm, long conclusionOccurrence, Truth conclusionTruth, Stamp stamp, long currentTime);

#endif

//Inference rules//
//---------------//
#ifdef H_NAL_RULES

//NAL1 rules
R( (S --> M), (M --> P), |-, (S --> P), Truth_Deduction )
R( (A --> B), (A --> C), |-, (C --> B), Truth_Abduction )
R( (A --> C), (B --> C), |-, (B --> A), Truth_Induction )
R( (A --> B), (B --> C), |-, (C --> A), Truth_Exemplification )
//NAL2 rules
R( (S --> P), (P --> S), |-, (S <-> P), Truth_Intersection )
R( (P --> M), (S --> M), |-, (S <-> P), Truth_Comparison )
R( (M --> P), (M --> S), |-, (S <-> P), Truth_Comparison )
R( (M --> P), (S <-> M), |-, (S --> P), Truth_Analogy )
R( (P --> M), (S <-> M), |-, (P --> S), Truth_Analogy )
R( (M <-> P), (S <-> M), |-, (S <-> P), Truth_Resemblance )
//NAL3 rules
R( (P --> M), (S --> M), |-, ((S | P) --> M), Truth_Intersection )
R( (P --> M), (S --> M), |-, ((S & P) --> M), Truth_Union )
R( (P --> M), (S --> M), |-, ((S ~ P) --> M), Truth_Difference )
R( (M --> P), (M --> S), |-, (M --> (P & S)), Truth_Intersection )
R( (M --> P), (M --> S), |-, (M --> (P | S)), Truth_Union )
R( (M --> P), (M --> S), |-, (M --> (P - S)), Truth_Difference )
R( (C --> {A}), (C --> {B}), |-, (C --> {A,B}), Truth_Union )
R( (C --> [A]), (C --> [B]), |-, (C --> [A,B]), Truth_Intersection )
R( ({A} --> C), ({B} --> C), |-, (C --> {A,B}), Truth_Intersection )
R( ([A] --> C), ([B] --> C), |-, (C --> [A,B]), Truth_Union )
R( (C --> {A,B}), (C --> {B}), |-, (C --> {B}), Truth_Intersection )
R( (C --> [A,B]), (C --> [B]), |-, (C --> [B]), Truth_Union )
R( ({A,B} --> C), ({B} --> C), |-, (C --> {B}), Truth_Union )
R( ([A,B] --> C), ([B] --> C), |-, (C --> [B]), Truth_Intersection )
R( (A --> [B,C]), X, |-, (A --> [C,B]), Truth_Identity )
R( ({A,B} --> C), X, |-, ({B,A} --> C), Truth_Identity )
//NAL4 rules
R( ((A * B) --> R), X, |-, (A --> (R / B)), Truth_Identity )
R( ((A * B) --> R), X, |-, (B --> (R % A)), Truth_Identity )
R( (R --> (A * B)), X, |-, ((R \ B) --> A), Truth_Identity )
R( (R --> (A * B)), X, |-, ((R % A) --> B), Truth_Identity )
R( (A --> (R / B)), X, |-, ((A * B) --> R), Truth_Identity )
R( (B --> (R % A)), X, |-, ((A * B) --> R), Truth_Identity )
R( ((R \ B) --> A), X, |-, ((A * B) --> R), Truth_Identity )
R( ((R % A) --> B), X, |-, (R --> (A * B)), Truth_Identity )

#endif
