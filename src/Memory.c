#include "Memory.h"
#include "Concept.h"

void memory_init(Memory *memory)
{
    for (int i=0;i<CONCEPT_COUNT;i++) {
        memory->concepts[i] = (void*)0;
    }
}

void memory_appendConcept(Memory *memory, Concept *concept)
{
    // naive simple algorithm to search next free space
    // TODO< keep track of last insert index and reuse >
    for (int i=0;i<CONCEPT_COUNT;i++) {
        if( !memory->concepts[i]) {
            memory->concepts[i] = concept;
            return;
        }
    }
}
