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
#define FIFO_SIZE 10
#define MAX_SEQUENCE_LEN 5
#define SDR_MAX_SATURATION TERM_ONES*2
#define FIFO_PERFORM_SEQUENCING false

//Data structure//
//--------------//
typedef struct
{
    int itemsAmount;
    int currentIndex;
    Event array[FIFO_SIZE];
    Event sequence_array[FIFO_SIZE];
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
//Copy a FIFO
void FIFO_COPY(FIFO *src, FIFO *target);
//Add an event to the FIFO
void FIFO_Add(Event *event, FIFO *fifo);
//Get the newest element
Event* FIFO_GetNewestElement(FIFO *fifo);
//Get the k-th newest FIFO element
Event* FIFO_GetKthNewestElement(FIFO *fifo, int k);
//Get the k-th newest FIFO sequence
Event* FIFO_GetKthNewestSequence(FIFO *fifo, int k);
//Get the newest sequence
Event* FIFO_GetNewestSequence(FIFO *fifo);

#endif
