#include "Term.h"

void Term_Print(Term *term)
{
    for(int i=0; i<MAX_SEQUENCE_LEN; i++)
    {
        if(term->atoms[i] != 0)
        {
            printf("%d", (int) term->atoms[i]);
        }
        else
        {
            fputs("@", stdout);
        }
    }
}

bool Term_Equal(Term *a, Term *b)
{
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        if(a->atoms[i] != b->atoms[i])
        {
            return false;
        }
    }
    return true;
}

static void Term_RelativeOverride(Term *term, int i, Term *subterm, int j)
{
    if(i < COMPOUND_TERM_SIZE_MAX && j < COMPOUND_TERM_SIZE_MAX)
    {
        term->atoms[i] = subterm->atoms[j];
        Term_RelativeOverride(term, (i+1)*2-1,   subterm, (j+1)*2-1);   //override left child
        Term_RelativeOverride(term, (i+1)*2+1-1, subterm, (j+1)*2+1-1); //override right child
    }
}

void Term_OverrideSubterm(Term *term, int i, Term *subterm)
{
    Term_RelativeOverride(term, i, subterm, 0); //subterm starts at its root, but its a subterm in term at position i
}

Term Term_ExtractSubterm(Term *term, int j)
{
    Term ret = {0}; //ret is where to "write into" 
    Term_RelativeOverride(&ret, 0, term, j); //where we begin to write at root, 0
    return ret; //reading from term beginning at i
}

int Term_Complexity(Term *term)
{
    int s = 0;
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        if(term->atoms[i])
        {
            s += 1;
        }
    }
    return s;
}
