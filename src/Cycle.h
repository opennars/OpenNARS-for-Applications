#ifndef H_CYCLE
#define H_CYCLE

/////////////////////////////////////
//  ALANN Control Cycle            //
/////////////////////////////////////
//A FIFO-like structure, that only supports put in and overwrites
//the oldest task when full

//References//
//-----------//
#include "Globals.h"
#include "RuleTable.h"
#include "Decision.h"

//Parameters//
//----------//
#define CONCEPT_FORMATION_NOVELTY 0.2
#define PROPAGATE_GOAL_SPIKES true

//Methods//
//-------//
//Apply one operating cyle
void Cycle_Perform(long currentTime);

#endif
