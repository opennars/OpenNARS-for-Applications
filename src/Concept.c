#include "Concept.h"

void Concept_SetSDR(Concept *concept, SDR sdr)
{
    concept->sdr = sdr;
    //Generate hash too:
    concept->sdr_hash = SDR_Hash(&sdr);
    //Initialize counter to what the sdr has:
    int k = 0;
    ITERATE_SDR_BITS(i,j,
        concept->sdr_bit_counter[k] = SDR_ReadBitInBlock(&sdr, i, j) ? 1.0 : -1.0;
        concept->sdr_bit_counter[k] *= CONCEPT_INTERPOLATION_INIT_STRENGTH;
        k++;
    )
}

void Concept_Print(Concept *concept)
{
    printf("Concept:\n");
    Attention_Print(&concept->attention);
    SDR_PrintWhereTrue(&concept->sdr);
    Usage_Print(&concept->usage);
    printf("\n");
}

void Concept_SDRInterpolation(Concept *concept, SDR *eventSDR, Truth matchTruth)
{
    double u = Truth_Expectation(matchTruth);
    int k = 0;
    ITERATE_SDR_BITS(i,j,
        double oldValue = concept->sdr_bit_counter[k];
        double count = SDR_ReadBitInBlock(eventSDR,i,j) ? 1.0 : -1.0;
        concept->sdr_bit_counter[k] += CONCEPT_INTERPOLATION_STRENGTH * u * count;
        double newValue = concept->sdr_bit_counter[k];
        if(oldValue<=0 && newValue >= 0)
        {
            SDR_WriteBit(&concept->sdr,k,1);
        }
        else
        if(oldValue>0 && newValue <= 0)
        {
            SDR_WriteBit(&concept->sdr,k,0);
        }
        k++;
    )
}
