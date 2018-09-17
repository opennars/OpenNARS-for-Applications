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

Concept *memory_getClosestConceptByName(Memory *memory, SDR *name) {
    Concept *closestConceptByName = (Concept*)0;
    double closestConceptDistance = 1000000000000000.0; // TODO< positive inf >

    for (int i=0;i<CONCEPT_COUNT;i++) {
        if( !memory->concepts[i]) {
            continue;
        }

        double distance = SDREqualTerm(*(memory->concepts[i]->name), *name);
        if (distance < closestConceptDistance) {
            closestConceptDistance = distance;
            closestConceptByName = memory->concepts[i];
        }
    }

    return closestConceptByName;
}
