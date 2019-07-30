 #ifndef RULETABLE_H
#define RULETABLE_H

//References//
//----------//
#include <stdbool.h>
#include <stdio.h>
#include "Globals.h"
#include "Memory.h"

//Methods//
//-------//
//Event b getting processed in foreign concept A (B is only used as ref for adding pre-conditions)
void RuleTable_Composition(Event *a, Event *b, int operationID);
//Concept e getting processed in its native concept c
//void RuleTable_Decomposition(Concept *c, Event *e, long currentTime);

#endif
