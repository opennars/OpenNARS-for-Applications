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
//Inferences per cycle (amount of events from cycling events)
#define EVENT_SELECTIONS 1
#define CONCEPTS_MAX 16384
#define CYCLING_EVENTS_MAX 8192
#define CYCLING_EVENTS_SELECT 10
#define USE_HASHING true
#define PROPAGATE_GOAL_SPIKES true
#define PROPAGATION_THRESHOLD_INITIAL 0.501
extern double PROPAGATION_THRESHOLD;
#define PROPAGATION_ITERATIONS 5
extern bool PRINT_DERIVATIONS;
#define PRINT_INPUT_INITIAL true
extern bool PRINT_INPUT;
#define PRINT_DERIVATIONS_INITIAL false
#define PRINT_CONTROL_INFO false
#define PRINT_DERIVATIONS_PRIORITY_THRESHOLD 0.0
#define MIN_CONFIDENCE 0.01
#define MIN_PRIORITY 0.0001
#define EVENT_DURABILITY 0.9
#define CONCEPT_DURABILITY 0.99

//Data structure//
//--------------//
extern Event selectedEvents[EVENT_SELECTIONS]; //better to be global
extern double selectedEventsPriority[EVENT_SELECTIONS]; //better to be global
extern int eventsSelected;
//Concepts in main memory:
PriorityQueue concepts;
//cycling events cycling in main memory:
PriorityQueue cycling_events;
//Input event buffers:
FIFO belief_events;
FIFO goal_events;
typedef void (*Action)(Term);
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
Concept* Memory_Conceptualize(Term *term);
//Add an already existing concept to memory that was taken out from the concept priority queue
void Memory_addConcept(Concept *concept, long currentTime);
//Add event to memory
void Memory_addEvent(Event *event, long currentTime, double priority, bool input, bool derived, bool readded, bool revised);
void Memory_addInputEvent(Event *event, long currentTime);
//Add operation to memory
void Memory_addOperation(int id, Operation op);
//check if implication is still valid (source concept might be forgotten)
bool Memory_ImplicationValid(Implication *imp);
//print added implication
void Memory_printAddedImplication(Term *implication, Truth *truth, bool input, bool revised);
//print added event
void Memory_printAddedEvent(Event *event, double priority, bool input, bool derived, bool revised);

#endif
