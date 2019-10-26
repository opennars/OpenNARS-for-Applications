#include "Concept.h"

void Concept_SetTerm(Concept *concept, Term term)
{
    concept->term = term;
    //Generate hash too:
    //concept->term_hash = Term_Hash(&term);
}

void Concept_Print(Concept *concept)
{
    puts("Concept:");
    Term_Print(&concept->term);
    Usage_Print(&concept->usage);
    puts("");
}

