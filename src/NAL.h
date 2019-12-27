#ifndef H_NAL
#define H_NAL

///////////////////
// NAL ecosystem //
///////////////////
//Declarative reasoning abilities not covered by the sensorimotor component

//References//
//----------//
#include "Encode.h"

//Methods//
//-------//
//Generates inference rule code
void NAL_GenerateRuleTable();

#endif

//Inference rules//
//---------------//
#ifdef H_NAL_RULES

//NAL1 rules
R( (S --> M), (M --> P), |-, (S --> P), Truth_Deduction )
R( (A --> B), (A --> C), |-, (C --> B), Truth_Abduction )
R( (A --> C), (B --> C), |-, (B --> A), Truth_Induction )

#endif
