#include "Implication.h"

void Implication_SetSDR(Implication *implication, SDR sdr)
{
    implication->sdr = sdr;
    //Generate hash too:
    implication->sdr_hash = SDR_Hash(&sdr);
}
