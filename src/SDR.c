#include "SDR.h"

void SDR_Print(SDR *sdr)
{
    for(int i=0; i<MAX_SEQUENCE_LEN; i++)
    {
        if(sdr->terms[i] != 0)
        {
            printf("%d", (int) sdr->terms[i]);
        }
    }
    puts("===");
}

SDR SDR_Tuple(SDR *a, SDR *b)
{
    SDR ret = {0};
    int i=0;
    for(; i<MAX_SEQUENCE_LEN; i++)
    {
        if(a->terms[i] == 0)
        {
            break;
        }
        ret.terms[i] = a->terms[i];
    }
    for(int j=0; i<MAX_SEQUENCE_LEN; i++, j++)
    {
        if(b->terms[i] == 0)
        {
            break;
        }
        ret.terms[i] = a->terms[j];
    }
    return ret;
}

bool SDR_Equal(SDR *a, SDR *b)
{
    for(int i=0; i<MAX_SEQUENCE_LEN; i++)
    {
        if(a->terms[i] != b->terms[i])
        {
            return false;
        }
    }
    return true;
}

