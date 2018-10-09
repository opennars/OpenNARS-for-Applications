#ifndef H_CONTROL
#define H_CONTROL

/////////////////////////////////////
//  ALANN Control Loop             //
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
