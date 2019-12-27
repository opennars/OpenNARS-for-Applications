#ifndef RULETABLE_H
#define RULETABLE_H

/////////////////
//  RuleTable  //
/////////////////
//The rule table which .c is generated at compile time
//by NAL_GenerateRuleTable

//References//
//----------//
#include "Term.h"
#include "Truth.h"

//Methods//
//-------//
void RuleTable(Term term1, Term term2, Truth truth1, Truth truth2);

#endif
