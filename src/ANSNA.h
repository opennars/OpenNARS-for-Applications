#ifndef ANSNA_H
#define ANSNA_H

////////////////////////////////////////////////////
//  ANSNA - Adaptive Neuro-Symbolic Network Agent //
////////////////////////////////////////////////////

//References//
//-----------//
#include "Cycle.h"

//Parameters//
//----------//
#define ANSNA_DEFAULT_FREQUENCY  1.0
#define ANSNA_DEFAULT_CONFIDENCE 0.9
#define ANSNA_DEFAULT_TRUTH ((Truth) { .frequency = ANSNA_DEFAULT_FREQUENCY, .confidence = ANSNA_DEFAULT_CONFIDENCE })
extern long currentTime;

//Callback function types//
//-----------------------//
//typedef void (*Action)(void);     //already defined in Memory

//Methods//
//-------//
//Init/Reset system
void ANSNA_INIT();
//Run the system for a certain amount of cycles
void ANSNA_Cycles(int cycles);
//Add input
Event ANSNA_AddInput(Term term, char type, Truth truth, int operationID);
Event ANSNA_AddInputBelief(Term term, int operationID);
Event ANSNA_AddInputGoal(Term term);
//Add an operation
void ANSNA_AddOperation(Term term, Action procedure);

#endif
