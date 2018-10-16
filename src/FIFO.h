#ifndef H_FIFO
#define H_FIFO

/////////////////////////////////////
//  First in first out (forgotten) //
/////////////////////////////////////
//A FIFO-like structure, that only supports put in and overwrites
//the oldest task when full

//References//
//-----------//
#include "Inference.h"

//Parameters//
//----------//
#define FIFO_SIZE 1000

//Data structure//
//--------------//
typedef struct
{
    int itemsAmount;
    int currentIndex;
    Event array[FIFO_SIZE];
} FIFO;
typedef struct
{
    Event *originalEvent;
    Event projectedEvent;
} FIFO_Query_Result;

//Methods//
//-------//
//Add an event to the FIFO
void FIFO_Add(Event *event, FIFO *fifo);
//Add an event to the FIFO with potential revision, 
//return revised element if revision worked, else {0}
//also see https://github.com/patham9/ANSNA/wiki/Event-Revision
Event FIFO_AddAndRevise(Event *event, FIFO *fifo);
//Get the best item for the occurrenceTime, projected to occurrenceTime, 
//plus the information where it was
FIFO_Query_Result FIFO_GetHighestConfidentProjectedTo(FIFO *fifo, long occurrenceTime, SDR *referenceSdr);
//Get the newest element
Event FIFO_GetNewestElement(FIFO *fifo);

#endif
