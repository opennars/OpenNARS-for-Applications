#ifndef ANSNA_H
#define ANSNA_H

////////////////////////////////////////////////////
//  ANSNA - Adaptive Neuro-Symbolic Network Agent //
////////////////////////////////////////////////////

//References//
//-----------//
#include "Cycle.h"

typedef void (*Action)(void);

//Methods//
//-------//
//Init/Reset system
void ANSNA_INIT();
//Run the system for a certain amount of cycles
void ANSNA_Cycles(int cycles);
//Add input
void ANSNA_AddInput(SDR sdr, char type, Truth truth);
//Add an operation
void ANSNA_AddOperation(SDR sdr, Action procedure);

#endif
