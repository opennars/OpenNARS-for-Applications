#include "Implication.h"

void Implication_SetTerm(Implication *implication, Term term)
{
    implication->term = term;
    //Generate hash too:
    //implication->term_hash = Term_Hash(&term);
}

void Implication_Print(Implication *implication)
{
    puts("Implication:");
    Term_Print(&implication->term);
    Truth_Print(&implication->truth);
    Stamp_print(&implication->stamp);
    printf("occurrenceTimeOffset=%ld\n\n", implication->occurrenceTimeOffset);
}
