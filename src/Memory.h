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
#define CONCEPTS_MAX 1024 //always adjust both
#define USE_HASHING true
#define PROPAGATE_GOAL_SPIKES true
#define PROPAGATION_THRESHOLD_INITIAL 0.6
extern double PROPAGATION_THRESHOLD;
#define PROPAGATION_ITERATIONS 5
#define CONCEPT_FORMATION_NOVELTY_INITIAL 0.2
extern double CONCEPT_FORMATION_NOVELTY;

//Data structure//
//--------------//
//Data structures
PriorityQueue concepts;
FIFO belief_events;
FIFO goal_events;
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
//Find a concept
bool Memory_FindConceptBySDR(SDR *sdr, SDR_HASH_TYPE sdr_hash, int *returnIndex);
//Create a new concept
void Memory_Conceptualize(SDR *sdr);
//Return closest concept
bool Memory_getClosestConcept(SDR *sdr, SDR_HASH_TYPE sdr_hash, int *returnIndex);
//Add an already existing concept to memory that was taken out from the concept priority queue
void Memory_addConcept(Concept *concept, long currentTime);
//Add event to memory
bool Memory_addEvent(Event *event);
//Add operation to memory
void Memory_addOperation(Operation op);
//Match event to concept
Event Memory_MatchEventToConcept(Concept *c, Event *e);
//Whether an event is novel in respect to a concept
bool Memory_EventIsNovel(Event *event, Concept *c_matched_to);
//relink implication, so that link stays intact after forgetting
void Memory_RelinkImplication(Implication *imp);

#endif
