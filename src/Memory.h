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
#define CONCEPTS_MAX 10000
#define EVENTS_MAX 64
#define VOTING 1
#define EXHAUSTIVE 2
#define USE_HASHING true
#define MATCH_STRATEGY VOTING

//Data structure//
//--------------//
PriorityQueue concepts;
PriorityQueue events; 

//Methods//
//-------//
//TODO do check which methods shouldn't be just PriorityQueue methods
//Init memory
void Memory_RESET();
//Add concept to memory
void Memory_addConcept(Concept *concept);
//Return closest concept
Concept* Memory_getClosestConceptByName(SDR *taskSDR);

#endif
