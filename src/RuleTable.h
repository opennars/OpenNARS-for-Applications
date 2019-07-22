 #ifndef RULETABLE_H
#define RULETABLE_H

//References//
//----------//
#include <stdbool.h>
#include <stdio.h>
#include "Globals.h"
#include "Memory.h"

//Parameters//
//----------//
//concept wait time to be used when it was used

//min confidence
#define MIN_CONFIDENCE 0.01
#define MIN_PRIORITY 0.01
#define MAX_FORWARD 5
#define MAX_BACKWARD 5
#define MAX_INDUCTIONS 5
#define MAX_DERIVATIONS EVENT_SELECTIONS*CONCEPT_SELECTIONS*(MAX_FORWARD+MAX_BACKWARD+MAX_INDUCTIONS)
#define ALLOW_ABDUCTION false

//Cycle dependency//
//----------------//
extern Event derivations[];
extern int eventsDerived;

//Methods//
//-------//
//Event b getting processed in foreign concept A (B is only used as ref for adding pre-conditions)
void RuleTable_Composition(long currentTime, Event *a, Event *b, int operationID);
//Concept e getting processed in its native concept c
//void RuleTable_Decomposition(Concept *c, Event *e, long currentTime);

#endif
