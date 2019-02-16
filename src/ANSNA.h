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

//Callback function types//
//-----------------------//
//typedef void (*Action)(void);             //already defined in Memory, just listed here to make
//typedef void (*EventInspector)(Event *);  //it clear what it is

//Methods//
//-------//
//Init/Reset system
void ANSNA_INIT();
//Run the system for a certain amount of cycles
void ANSNA_Cycles(int cycles);
//Add input
Event ANSNA_AddInput(SDR sdr, char type, Truth truth);
Event ANSNA_AddInputBelief(SDR sdr);
Event ANSNA_AddInputGoal(SDR sdr);
//Add an operation
void ANSNA_AddOperation(SDR sdr, Action procedure);
//Add an operation
void ANSNA_AddEventInspector(SDR sdr, EventInspector procedure);

#endif
