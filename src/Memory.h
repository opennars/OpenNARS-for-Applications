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
#define CONCEPT_TERMS 512
#define CONCEPT_COUNT 64

typedef struct {
    //null pointer indicates free space
    Concept *concepts[CONCEPT_COUNT];
} Memory;

void concept_init(Concept *concept, SDR *name)

#endif
