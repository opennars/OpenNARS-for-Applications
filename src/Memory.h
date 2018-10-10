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
#include "SDR.h"
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

//Data structure//
//--------------//
PriorityQueue concepts;
PriorityQueue events; 

//Macros


//Methods//
//-------//
//TODO do check which methods shouldn't be just PriorityQueue methods
//Init memory
void Memory_RESET();
//Add concept to memory
void Memory_addConcept(SDR *sdr, Attention attention);
//Return closest concept
int Memory_getClosestConcept(Event *event);
//Add event to memory
void Memory_addEvent(Event *event);
#endif
