#include "Term.h"

void Term_Print(Term *sdr)
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

Term Term_Tuple(Term *a, Term *b)
{
    Term ret = {0};
    assert(a->terms[0] > 0, "issue with term encoding1");
    assert(b->terms[0] > 0, "issue with term encoding2");
    assert(b->terms[1] == 0, "issue with term encoding4");
    /*int i=0;
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
    }*/
    assert(MAX_SEQUENCE_LEN == 2, "Not generalized, outcommented code not yet working");
    ret.terms[0] = a->terms[0];
    ret.terms[1] = b->terms[0];
    return ret;
}

bool Term_Equal(Term *a, Term *b)
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

