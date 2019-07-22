#include "Implication.h"

void Implication_SetSDR(Implication *implication, SDR sdr)
{
    implication->sdr = sdr;
    //Generate hash too:
    implication->sdr_hash = SDR_Hash(&sdr);
}

void Implication_Print(Implication *implication)
{
    printf("Implication:\n");
    SDR_PrintWhereTrue(&implication->sdr);
    Truth_Print(&implication->truth);
    Stamp_print(&implication->stamp);
    printf("occurrenceTimeOffset=%ld\n\n", implication->occurrenceTimeOffset);
}

double Implication_Currentness(Implication *implication)
{
    double distance = (implication->occurrenceTimeOffset / (1.0f + implication->occurrenceTimeOffset));
    return 1.0 - distance;
}
