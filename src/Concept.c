#include "Concept.h"

void concept_init(Concept *concept, SDR *name)
{
    concept->name = name;
    for (int i=0;i<CONCEPT_TASKS;i++) {
        concept->tasks[i] = (void*)0;
    }
}
