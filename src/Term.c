#include "Term.h"

void Term_Print(Term *term)
{
    for(int i=0; i<MAX_SEQUENCE_LEN; i++)
    {
        if(term->terms[i] != 0)
        {
            printf("%d", (int) term->terms[i]);
        }
    }
    puts("===");
}

Term Term_Sequence(Term *a, Term *b)
{
    Term ret = {0};
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
        if(b->terms[j] == 0)
        {
            break;
        }
        ret.terms[i] = b->terms[j];
    }
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

