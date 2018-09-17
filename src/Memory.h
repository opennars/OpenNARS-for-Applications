#ifndef MEMORY_H
#define MEMORY_H

///////////////////
//  ANSNA Memory //
///////////////////

//Description//
//-----------//
//The memory of ANSNA

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

void memory_init(Memory *memory);

void memory_appendConcept(Memory *memory, Concept *concept);


#endif
