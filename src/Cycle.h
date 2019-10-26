#ifndef H_CYCLE
#define H_CYCLE

///////////////////////////////////
//  MSC Control Cycle            //
///////////////////////////////////
//A FIFO-like structure, that only supports put in and overwrites
//the oldest task when full

//References//
//-----------//
#include "Globals.h"
#include "Decision.h"
#include "Inference.h"

//Methods//
//-------//
//Apply one operating cyle
void Cycle_Perform(long currentTime);

#endif
