#include "Concept.h"

void Concept_SetSDR(Concept *concept, SDR sdr)
{
    concept->sdr = sdr;
    //Generate hash too:
    concept->sdr_hash = SDR_Hash(&sdr);
}

void Concept_Print(Concept *concept)
{
    printf("Concept:\n");
    Attention_Print(&concept->attention);
    SDR_PrintWhereTrue(&concept->sdr);
    Usage_Print(&concept->usage);
    printf("\n");
}
