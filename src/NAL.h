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

#endif
