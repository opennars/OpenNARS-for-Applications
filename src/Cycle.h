#ifndef H_CYCLE
#define H_CYCLE

/////////////////////////////////////
//  ALANN Control Cycle            //
/////////////////////////////////////
//A FIFO-like structure, that only supports put in and overwrites
//the oldest task when full

//References//
//-----------//
#include "Memory.h"
#include "Globals.h"

//Parameters//
//----------//
#define EVENT_SELECTIONS 5
//for temporal induction:
#define CONCEPT_SELECTIONS 5
//for decision making:
#define DECISION_THRESHOLD 0.5
//motor babbling chance
#define MOTOR_BABBLING_CHANCE 0.001
//concept wait time to be used when it was used
#define CONCEPT_WAIT_TIME 100
//min confidence
#define MIN_CONFIDENCE 0.01
#define MIN_PRIORITY 0.01
#define MAX_PREDICTIONS 5

//Methods//
//-------//
//Apply one inference cyle
void cycle(long currentTime);

#endif
