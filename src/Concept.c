#include "Concept.h"

void Concept_SetSDR(Concept *concept, SDR sdr)
{
    concept->sdr = sdr;
    //Generate hash too:
    concept->sdr_hash = SDR_Hash(&sdr);
}

void Concept_Print(Concept *concept)
{
    puts("Concept:");
    SDR_Print(&concept->sdr);
    Usage_Print(&concept->usage);
    puts("");
}

