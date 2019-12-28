#ifndef H_CYCLE
#define H_CYCLE

///////////////////////////////////
//  YAN Control Cycle            //
///////////////////////////////////
//A FIFO-like structure, that only supports put in and overwrites
//the oldest task when full

//References//
//-----------//
#include "Globals.h"
#include "Decision.h"
#include "Inference.h"
#include "RuleTable.h"

//Parameters//
//----------//
//Inferences per cycle (amount of events from cycling events)
#define EVENT_SELECTIONS 1
#define EVENT_DURABILITY 0.9
#define MIN_PRIORITY 0.01

//Methods//
//-------//
//Apply one operating cyle
void Cycle_Perform(long currentTime);

#endif
