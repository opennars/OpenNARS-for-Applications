#include "Concept.h"

void Concept_SetTerm(Concept *concept, Term sdr)
{
    concept->sdr = sdr;
    //Generate hash too:
    //concept->term_hash = Term_Hash(&term);
}

void Concept_Print(Concept *concept)
{
    puts("Concept:");
    Term_Print(&concept->sdr);
    Usage_Print(&concept->usage);
    puts("");
}

