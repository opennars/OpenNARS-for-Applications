#include "Concept.h"

void Concept_SetName(Concept *concept, SDR name)
{
    concept->name = name;
    //Generate hash too:
    concept->name_hash = SDR_Hash(&name);
}
