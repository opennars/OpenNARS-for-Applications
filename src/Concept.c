#include "Concept.h"

void Concept_SetName(Concept *concept, SDR name)
{
    concept->name = name;
    //Generate CRC checksum too:
    concept->name_hash = SDR_Hash(&name);
}
