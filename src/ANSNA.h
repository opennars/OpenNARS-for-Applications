#ifndef ANSNA_H
#define ANSNA_H

////////////////////////////////////////////////////
//  ANSNA - Adaptive Neuro-Symbolic Network Agent //
////////////////////////////////////////////////////

//References//
//-----------//
#include "Cycle.h"

//Methods//
//-------//
//Init ANSNA
void ANSNA_INIT();
//Reset system
void ANSNA_RESET();
//Run the system for a certain amount of cycles
void ANSNA_Cycles(int cycles);
//Add input
void ANSNA_AddInput(SDR sdr, char type, Truth truth);

#endif
