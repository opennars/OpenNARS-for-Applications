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
    puts("Concept:");
    SDR_Print(&concept->sdr);
    Usage_Print(&concept->usage);
    puts("");
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
        if(oldValue<0.5 && newValue >= 0.5)
        {
            SDR_WriteBit(&concept->sdr,k,1);
        }
        else
        if(oldValue>=0.5 && newValue < 0.5)
        {
            SDR_WriteBit(&concept->sdr,k,0);
        }
        k++;
    )
}

void Concept_CheckAnticipationDisappointment(int layer, Concept *c, long currentTime)
{
    for(int j=0; j<ANTICIPATIONS_MAX; j++)
    {
        if(c->anticipation_deadline[j] > 0)
        {
            IN_DEBUG ( printf("currentTime = %ld, deadline = %ld layer=%d\n", currentTime, c->anticipation_deadline[j], layer); )
            if(currentTime > c->anticipation_deadline[j])
            {
                //disappointed
                Implication *added = Table_AddAndRevise(&c->precondition_beliefs[c->anticipation_operation_id[j]], &c->anticipation_negative_confirmation[j], c->anticipation_negative_confirmation[j].debug);
                if(added != NULL)
                {
                    added->sourceConcept = c->anticipation_negative_confirmation[j].sourceConcept;
                    added->sourceConceptSDR = c->anticipation_negative_confirmation[j].sourceConceptSDR;
                }
                c->anticipation_deadline[j] = 0;
                IN_DEBUG
                (
                    printf("DISAPPOINTED %s layer=%d (f,c)=(%f,%f)\n", c->anticipation_negative_confirmation[j].debug, layer, c->anticipation_negative_confirmation[j].truth.frequency, c->anticipation_negative_confirmation[j].truth.confidence);
                    getchar();
                    puts("START");
                    for(int i=0; i<c->precondition_beliefs[c->anticipation_operation_id[j]].itemsAmount; i++)
                    {
                        puts(c->precondition_beliefs[c->anticipation_operation_id[j]].array[i].debug);
                        puts("");
                        Implication_Print(&c->precondition_beliefs[c->anticipation_operation_id[j]].array[i]);
                    }
                    puts("ADDITION END");
                 )
            }   
        }
    }
}

void Concept_ConfirmAnticipation(int layer, Concept *c, Event *e)
{
    bool confirmed = false;
    for(int i=0; i<ANTICIPATIONS_MAX; i++)
    {
        if(c->anticipation_deadline[i] > 0 && e->type == EVENT_TYPE_BELIEF)
        {
            //confirmed
            c->anticipation_deadline[i] = 0;
            confirmed = true;
        }
    }
    if(confirmed)
    {
        IN_DEBUG
        (
            printf("CONFIRMED %s layer=%d\n", c->debug, layer);
            getchar();
        )
    }
}


