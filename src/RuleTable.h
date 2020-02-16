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
void RuleTable_Apply(Term term1, Term term2, Truth truth1, Truth truth2, long conclusionOccurrence, Stamp conclusionStamp, 
                     long currentTime, double parentPriority, double conceptPriority, bool doublePremise, Concept *validation_concept, long validation_cid);
Term RuleTable_Reduce(Term term1, bool doublePremise);

#endif
