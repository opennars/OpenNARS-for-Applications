#ifndef MEMORY_H
#define MEMORY_H

///////////////////
//  ANSNA Memory //
///////////////////
//The conce-based memory of ANSNA

//References//
//////////////
#include "Concept.h"

//Parameters//
//----------//
#define CONCEPT_COUNT 64

typedef struct {
    //null pointer indicates free space
    Concept *concepts[CONCEPT_COUNT];
} Memory;

//Methods//
//-------//
//Init memory
void memory_init(Memory *memory);
//Add concept to memory
void memory_appendConcept(Memory *memory, Concept *concept);
//Return closest concept
Concept *memory_getClosestConceptByName(Memory *memory, SDR *name);

#endif
