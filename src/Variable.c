#include "Variable.h"

bool Variable_isVariable(Atom atom)
{
    return Narsese_atomNames[(int) atom-1][0] == '$' && Narsese_atomNames[(int) atom-1][1] != 0;
}

bool Variable_hasVariable(Term *term)
{
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        Atom atom = term->atoms[i];
        if(Variable_isVariable(atom))
        {
            return true;
        }
    }
    return false;
}

Substitution Variable_Unify(Term *general, Term *specific)
{
    Substitution substitution = {0};
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        Atom general_atom = general->atoms[i];
        if(Variable_isVariable(general_atom))
        {
            Term subtree = Term_ExtractSubterm(specific, i);
            if(substitution.map[(int) general_atom].atoms[0] != 0 && !Term_Equal(&substitution.map[(int) general_atom], &subtree))
            {
                return substitution;
            }
            substitution.map[(int) general_atom] = subtree;
        }
    }
    substitution.success = true;
    return substitution;
}

Term Variable_ApplySubstitute(Term general, Substitution substitution)
{
    assert(substitution.success, "A substitution from unsuccessful unification cannot be used to substitute variables!");
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        Atom general_atom = general.atoms[i];
        if(Variable_isVariable(general_atom) && substitution.map[(int) general_atom].atoms[0] != 0)
        {
            Term_OverrideSubterm(&general, i, &substitution.map[(int) general_atom]);
        }
    }
    return general;
}
