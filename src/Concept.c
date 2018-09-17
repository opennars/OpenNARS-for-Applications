#include "Concept.h"

void concept_init(Concept *concept, SDR *name)
{
    concept->name = name;
    for (int i=0;i<CONCEPT_TERMS;i++) {
        concept->terms[i] = (void*)0;
    }
}
