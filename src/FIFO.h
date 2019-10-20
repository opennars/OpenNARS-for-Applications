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
#include "Globals.h"
#include "Encode.h"

//Parameters//
//----------//
#define FIFO_SIZE 20
#define MAX_SEQUENCE_LEN 1

//Data structure//
//--------------//
typedef struct
{
    int itemsAmount;
    int currentIndex;
    Event array[MAX_SEQUENCE_LEN][FIFO_SIZE];
} FIFO;
typedef struct
{
    Event *originalEvent;
    Event projectedEvent;
} FIFO_Query_Result;

//Methods//
//-------//
//Resets the FIFO
void FIFO_RESET(FIFO *fifo);
//Add an event to the FIFO
void FIFO_Add(Event *event, FIFO *fifo);
//Get the newest element
Event* FIFO_GetNewestSequence(FIFO *fifo, int len);
//Get the k-th newest FIFO element
Event* FIFO_GetKthNewestSequence(FIFO *fifo, int k, int len);

#endif
