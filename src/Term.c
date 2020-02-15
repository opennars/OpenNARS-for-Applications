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
    return memcmp(a, b, sizeof(Term)) == 0;
}

static bool Term_RelativeOverride(Term *term, int i, Term *subterm, int j)
{
    if(i >= COMPOUND_TERM_SIZE_MAX)
    {
        return false;
    }
    if(j < COMPOUND_TERM_SIZE_MAX)
    {
        term->atoms[i] = subterm->atoms[j];
        int left_in_subterm = (j+1)*2-1;
        if(left_in_subterm < COMPOUND_TERM_SIZE_MAX && subterm->atoms[left_in_subterm] != 0)
        {
            if(!Term_RelativeOverride(term, (i+1)*2-1, subterm, left_in_subterm))   //override left child
            {
                return false;
            }
        }
        int right_in_subterm = (j+1)*2+1-1;
        if(right_in_subterm < COMPOUND_TERM_SIZE_MAX && subterm->atoms[right_in_subterm] != 0)
        {
            if(!Term_RelativeOverride(term, (i+1)*2+1-1, subterm, right_in_subterm)) //override right child
            {
                return false;
            }
        }
    }
    return true;
}

bool Term_OverrideSubterm(Term *term, int i, Term *subterm)
{
    return Term_RelativeOverride(term, i, subterm, 0); //subterm starts at its root, but its a subterm in term at position i
}

Term Term_ExtractSubterm(Term *term, int j)
{
    Term ret = {0}; //ret is where to "write into" 
    Term_RelativeOverride(&ret, 0, term, j); //where we begin to write at root, 0 (always succeeds as we extract just a subset)
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

TERM_HASH_TYPE Term_Hash(Term *term)
{
    int pieces = TERM_ATOMS_SIZE / TERM_HASH_TYPE_SIZE;
    assert(TERM_HASH_TYPE_SIZE*pieces == TERM_ATOMS_SIZE, "Not a multiple, issue in hash calculation");
    TERM_HASH_TYPE *pt = (TERM_HASH_TYPE*) &term->atoms;
    TERM_HASH_TYPE hash = 0;
    for(int i=0; i<pieces; i++, pt++)
    {
        hash ^= *pt;
    }
    return hash;
}
