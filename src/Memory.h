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
#define CONCEPTS_MAX 1000
#define USE_HASHING true
#define CONCEPT_FORMATION_NOVELTY 0.2
#define PROPAGATE_GOAL_SPIKES true
//only propagate promising spikes:
#define PROPAGATION_TRUTH_EXPECTATION_THRESHOLD 0.5

//Data structure//
//--------------//
//Data structures
PriorityQueue concepts;
FIFO belief_events;
FIFO goal_events;
typedef void (*Action)(void);
typedef void (*EventInspector)(Event *);
typedef struct
{
    SDR sdr;
    Action action;
}Operation;
Operation operations[OPERATIONS_MAX];
EventInspector event_inspector;

//Methods//
//-------//
//Init memory
void Memory_INIT();
//Find a concept
bool Memory_FindConceptBySDR(SDR *sdr, SDR_HASH_TYPE sdr_hash, int *returnIndex);
//Create a new concept
Concept* Memory_Conceptualize(SDR *sdr);
//Return closest concept
bool Memory_getClosestConcept(SDR *sdr, SDR_HASH_TYPE sdr_hash, int *returnIndex);
//Add an already existing concept to memory that was taken out from the concept priority queue
bool Memory_addConcept(Concept *concept, long currentTime);
//Add event to memory
bool Memory_addEvent(Event *event);
//Add operation to memory
void Memory_addOperation(Operation op);
//Whether an event is novel in respect to a concept
bool Memory_EventIsNovel(Event *event, Concept *c_matched_to);
//Propagate spikes
void Memory_SpikePropagation(long currentTime);
//Match event to concept
Event Memory_MatchEventToConcept(Concept *c, Event *e);

#endif
