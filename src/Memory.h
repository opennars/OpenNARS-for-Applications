#ifndef MEMORY_H
#define MEMORY_H

/////////////////
//  YAN Memory //
/////////////////
//The concept-based memory of YAN

//References//
//////////////
#include "Concept.h"
#include "PriorityQueue.h"

//Parameters//
//----------//
#define CONCEPTS_MAX 1024 //always adjust both
#define USE_HASHING true
#define PROPAGATE_GOAL_SPIKES true
#define PROPAGATION_THRESHOLD_INITIAL 0.501
extern double PROPAGATION_THRESHOLD;
#define PROPAGATION_ITERATIONS 5

//Data structure//
//--------------//
//Data structures
PriorityQueue concepts;
FIFO belief_events;
FIFO goal_events;
typedef void (*Action)(void);
typedef struct
{
    Term term;
    Action action;
}Operation;
Operation operations[OPERATIONS_MAX];

//Methods//
//-------//
//Init memory
void Memory_INIT();
//Find a concept
bool Memory_FindConceptByTerm(Term *term, /*Term_HASH_TYPE term_hash,*/ int *returnIndex);
//Create a new concept
void Memory_Conceptualize(Term *term);
//Add an already existing concept to memory that was taken out from the concept priority queue
void Memory_addConcept(Concept *concept, long currentTime);
//Add event to memory
bool Memory_addEvent(Event *event);
//Add operation to memory
void Memory_addOperation(Operation op);
//check if implication is still valid (source concept might be forgotten)
bool Memory_ImplicationValid(Implication *imp);

#endif
