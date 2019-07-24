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
    SDR_PrintWhereTrue(&concept->sdr);
    Usage_Print(&concept->usage);
    puts("");
}

void Concept_CheckAnticipationDisappointment(Concept *c, long currentTime)
{
    for(int j=0; j<ANTICIPATIONS_MAX; j++)
    {
        if(c->anticipation_deadline[j] > 0)
        {
            IN_DEBUG ( printf("currentTime = %ld, deadline = %ld\n", currentTime, c->anticipation_deadline[j]); )
            if(currentTime > c->anticipation_deadline[j])
            {
                //disappointed
                Table_AddAndRevise(&c->precondition_beliefs[c->anticipation_operation_id[j]], &c->anticipation_negative_confirmation[j], c->anticipation_negative_confirmation[j].debug);
                c->anticipation_deadline[j] = 0;
                IN_DEBUG
                (
                    printf("DISAPPOINTED %s\n", c->anticipation_negative_confirmation[j].debug);
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

void Concept_ConfirmAnticipation(Concept *c, Event *e)
{
    for(int i=0; i<ANTICIPATIONS_MAX; i++)
    {
        if(c->anticipation_deadline[i]> 0 && e->type == EVENT_TYPE_BELIEF)
        {
            //confirmed
            c->anticipation_deadline[i] = 0;
            IN_DEBUG
            (
                printf("CONFIRMED %s\n", c->debug);
                getchar();
            )
        }
    }
}
