#ifndef MSC_H
#define MSC_H

///////////////////////////////////////////
//  MSC - Minimal Sensorimotor Component //
///////////////////////////////////////////

//References//
//-----------//
#include "Cycle.h"

//Parameters//
//----------//
#define MSC_DEFAULT_FREQUENCY  1.0
#define MSC_DEFAULT_CONFIDENCE 0.9
#define MSC_DEFAULT_TRUTH ((Truth) { .frequency = MSC_DEFAULT_FREQUENCY, .confidence = MSC_DEFAULT_CONFIDENCE })
extern long currentTime;

//Callback function types//
//-----------------------//
//typedef void (*Action)(void);     //already defined in Memory

//Methods//
//-------//
//Init/Reset system
void MSC_INIT();
//Run the system for a certain amount of cycles
void MSC_Cycles(int cycles);
//Add input
Event MSC_AddInput(Term term, char type, Truth truth, int operationID);
Event MSC_AddInputBelief(Term term, int operationID);
Event MSC_AddInputGoal(Term term);
//Add an operation
void MSC_AddOperation(Term term, Action procedure);

#endif
