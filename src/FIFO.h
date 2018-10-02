#ifndef H_FIFO
#define H_FIFO

/////////////////////////////////////
//  First in first out (forgotten) //
/////////////////////////////////////
//A FIFO-like structure, that only supports put in and overwrites
//the oldest task when full

//References//
//-----------//
#include "Event.h"
#include "Inference.h"

//Parameters//
//----------//
#define FIFO_SIZE 1000

//Data structure//
//--------------//
typedef struct
{
    int currentIndex;
    Event array[FIFO_SIZE];
} FIFO;

//Methods//
//-------//
//Reset FIFO
void FIFO_RESET(FIFO *fifo);
//Add an event to the FIFO
void FIFO_Add(Event *event, FIFO *fifo);
//Add an event to the FIFO with potential revision, 
//return revised element if revision worked, else event
//also see https://github.com/patham9/ANSNA/wiki/Event-Revision
Event FIFO_AddAndRevise(Event *event, FIFO *fifo);

#endif
