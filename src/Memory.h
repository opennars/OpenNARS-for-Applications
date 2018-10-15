#ifndef MEMORY_H
#define MEMORY_H

///////////////////
//  ANSNA Memory //
///////////////////
//The conce-based memory of ANSNA
//See voting mechanism to make it fast:
//https://github.com/patham9/ANSNA/wiki/Voting-Schema

//References//
//////////////
#include "Concept.h"
#include "PriorityQueue.h"

//Parameters//
//----------//
#define CONCEPTS_MAX 1000 //TODO statically alloc once on INIT, as can lead in value too large for the compiler to handle
#define EVENTS_MAX 64
#define VOTING 1
#define EXHAUSTIVE 2
#define USE_HASHING true
#define MATCH_STRATEGY VOTING
#define MEMORY_MATCH_NO_CONCEPT -1
#define OPERATIONS_MAX 1000

//Data structure//
//--------------//
//Data structures
PriorityQueue concepts;
PriorityQueue events;
typedef void (*Action)(void);
typedef struct
{
    SDR sdr;
    Action action;
}Operation;
Operation operations[OPERATIONS_MAX];

//Methods//
//-------//
//Init memory
void Memory_INIT();
//Add concept to memory
Concept* Memory_addConcept(SDR *sdr, Attention attention);
//Return closest concept
int Memory_getClosestConcept(Event *event);
//Add event to memory
void Memory_addEvent(Event *event);
//Add operation to memory
void Memory_addOperation(Operation op);
#endif
