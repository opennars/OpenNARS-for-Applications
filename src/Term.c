#include "Term.h"

void Term_Print(Term *term)
{
    for(int i=0; i<MAX_SEQUENCE_LEN; i++)
    {
        if(term->atoms[i] != 0)
        {
            printf("%d", (int) term->atoms[i]);
        }
    }
    puts("===");
}

//TODO USE TREE ARRAY ENCODING!!!!
Term Term_Sequence(Term *a, Term *b)
{
    Term ret = {0};
    int i=0;
    for(; i<MAX_SEQUENCE_LEN; i++)
    {
        if(a->atoms[i] == 0)
        {
            break;
        }
        ret.atoms[i] = a->atoms[i];
    }
    for(int j=0; i<MAX_SEQUENCE_LEN; i++, j++)
    {
        if(b->atoms[j] == 0)
        {
            break;
        }
        ret.atoms[i] = b->atoms[j];
    }
    return ret;
}

bool Term_Equal(Term *a, Term *b)
{
    for(int i=0; i<MAX_SEQUENCE_LEN; i++)
    {
        if(a->atoms[i] != b->atoms[i])
        {
            return false;
        }
    }
    return true;
}

