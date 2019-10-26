#include "Implication.h"

void Implication_SetTerm(Implication *implication, Term sdr)
{
    implication->sdr = sdr;
    //Generate hash too:
    //implication->sdr_hash = Term_Hash(&sdr);
}

void Implication_Print(Implication *implication)
{
    puts("Implication:");
    Term_Print(&implication->sdr);
    Truth_Print(&implication->truth);
    Stamp_print(&implication->stamp);
    printf("occurrenceTimeOffset=%ld\n\n", implication->occurrenceTimeOffset);
}
