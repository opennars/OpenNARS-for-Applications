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
#define CONCEPT_COUNT 64
#define CYCLING_TASKS_COUNT 64

//Data structure//
//--------------//
DefinePriorityQueue(Memory, CONCEPT_COUNT, Concept, concepts)
Memory memory;
DefinePriorityQueue(CyclingTasks, CYCLING_TASKS_COUNT, Task, tasks)
CyclingTasks tasks; 
//TODO do we really need an additional one or can cyclingTasks, inputTasks and derivedTasks compete in the same queue?

//Methods//
//-------//
//TODO do check which methods shouldn't be just PriorityQueue methods
//Init memory
void memory_RESET(Memory *memory);
//Add concept to memory
void memory_appendConcept(Memory *memory, Concept *concept);
//Return closest concept
Concept *memory_getClosestConceptByName(Memory *memory, SDR *name);

#endif
