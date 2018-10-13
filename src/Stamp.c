#include "Stamp.h"

Stamp Stamp_make(Stamp *stamp1, Stamp *stamp2)
{
    Stamp ret = {0};
    bool processStamp1 = true;
    bool processStamp2 = true;
    for (int j=0, i=0; i<STAMP_SIZE; i++)
    {
        if(processStamp1)
        {
            if(stamp1->evidentalBase[i] != STAMP_FREE)
            {
                ret.evidentalBase[j] = stamp1->evidentalBase[i];
                j++;
                if(j >= STAMP_SIZE)
                {
                    break;
                }
            }
            else
            {
                processStamp1 = false;
            }
        }
        if(processStamp2)
        {
            if(stamp2->evidentalBase[i] != STAMP_FREE)
            {
                ret.evidentalBase[j] = stamp2->evidentalBase[i];
                j++;
                if(j >= STAMP_SIZE)
                {
                    break;
                }
            }
            else
            {
                processStamp2 = false;
            }
        }
        if(!processStamp1 && !processStamp2)
        {
            break;
        }
    }
    return ret;
}

bool Stamp_checkOverlap(Stamp *a, Stamp *b)
{
    for (int i=0;i<STAMP_SIZE;i++)
    {
        if (a->evidentalBase[i] == STAMP_FREE) 
        {
            break;
        }
        for (int j=0;j<STAMP_SIZE;j++)
        {
            if (b->evidentalBase[j] == STAMP_FREE)
            {
                break;
            }
            if (a->evidentalBase[i] == b->evidentalBase[j])
            {
                return true;
            }
        }
    }
    return false;
}

void Stamp_print(Stamp *stamp)
{
    printf("stamp=");
    for(int i=0; i<STAMP_SIZE; i++)
    {
        if(stamp->evidentalBase[i] == STAMP_FREE)
        {
            break;
        }
        printf("%ld,", stamp->evidentalBase[i]);
    }
    printf("\n");
}
