#ifndef YAN_H
#define YAN_H

//////////////////////////////
//  YAN - Yet Another NARS  //
//////////////////////////////

//References//
//-----------//
#include "Cycle.h"
#include "Config.h"

//Parameters//
//----------//
#define YAN_DEFAULT_TRUTH ((Truth) { .frequency = YAN_DEFAULT_FREQUENCY, .confidence = YAN_DEFAULT_CONFIDENCE })
extern long currentTime;

//Callback function types//
//-----------------------//
//typedef void (*Action)(void);     //already defined in Memory

//Methods//
//-------//
//Init/Reset system
void YAN_INIT();
//Run the system for a certain amount of cycles
void YAN_Cycles(int cycles);
//Add input
Event YAN_AddInput(Term term, char type, Truth truth, bool eternal);
Event YAN_AddInputBelief(Term term);
Event YAN_AddInputGoal(Term term);
//Add an operation
void YAN_AddOperation(Term term, Action procedure);

#endif
