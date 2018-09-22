#include "Memory.h"
#include "Concept.h"

void memory_RESET(Memory *memory)
{
    memory->concepts_amount = 0;
}

void memory_appendConcept(Memory *memory, Concept *concept)
{
    // naive simple algorithm to search next free space
    // TODO< keep track of last insert index and reuse >
    /*for (int i=0;i<CONCEPT_COUNT;i++) {
        if( !memory->concepts[i]) {
            memory->concepts[i] = concept;
            return;
        }
    }*/
}

Concept *memory_getClosestConceptByName(Memory *memory, SDR *name) {
    /*Concept *closestConceptByName = NULL;
    double nearestConceptCloseness = -1;

    for (int i=0; i<memory->n_concepts; i++) {
        if( !memory->concepts[i]) {
            continue;
        }

        double closeness = SDR_Similarity(memory->concepts[i]->name, *name);
        if (closeness > nearestConceptCloseness) {
            nearestConceptCloseness = closeness;
            closestConceptByName = memory->concepts[i];
        }
    }*/

    return NULL; //closestConceptByName;
}
