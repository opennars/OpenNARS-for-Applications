#ifndef MEMORY_H
#define MEMORY_H

///////////////////
//  ANSNA Memory //
///////////////////
//The conce-based memory of ANSNA

//References//
//////////////
#include "Concept.h"
#include "PriorityQueue.h"

//Parameters//
//----------//
#define CONCEPTS_MAX 10000
#define BUFFER_TASKS_MAX 64

//Data structure//
//--------------//
DefinePriorityQueue(Memory, CONCEPTS_MAX, Concept, concepts)
Memory memory;
DefinePriorityQueue(AttentionBuffer, BUFFER_TASKS_MAX, Task, tasks)
AttentionBuffer buffer; 
//TODO do we really need an additional one or can cyclingTasks, inputTasks and derivedTasks compete in the same queue?

//Methods//
//-------//
//TODO do check which methods shouldn't be just PriorityQueue methods
//Init memory
void Memory_RESET();
//Add concept to memory
void Memory_addConcept(Concept *concept);
//Return closest concept
Concept* Memory_getClosestConceptByName(SDR *name);

#endif
