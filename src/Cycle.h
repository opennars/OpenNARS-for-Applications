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

//Parameters//
//----------//
#define EVENT_SELECTIONS 10

//Methods//
//-------//
//Apply one inference cyle
void cycle();

#endif
