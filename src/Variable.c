#include "Variable.h"

bool Variable_isIndependentVariable(Atom atom)
{
    return atom > 0 && Narsese_atomNames[(int) atom-1][1] != 0 && Narsese_atomNames[(int) atom-1][0] == '$';
}

bool Variable_isDependentVariable(Atom atom)
{
    return atom > 0 && Narsese_atomNames[(int) atom-1][1] != 0 && Narsese_atomNames[(int) atom-1][0] == '#';
}

bool Variable_isQueryVariable(Atom atom)
{
    return atom > 0 && Narsese_atomNames[(int) atom-1][1] != 0 && Narsese_atomNames[(int) atom-1][0] == '?';
}

bool Variable_isVariable(Atom atom)
{
    return Variable_isIndependentVariable(atom) || Variable_isDependentVariable(atom) || Variable_isQueryVariable(atom);
}

bool Variable_hasVariable(Term *term, bool independent, bool dependent, bool query)
{
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        Atom atom = term->atoms[i];
        if((independent && Variable_isIndependentVariable(atom)) || (dependent && Variable_isDependentVariable(atom)) || (query && Variable_isQueryVariable(atom)))
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
        if(general_atom)
        {
            if(Variable_isVariable(general_atom))
            {
                Term subtree = Term_ExtractSubterm(specific, i);
                if(Variable_isQueryVariable(general_atom) && Variable_isVariable(subtree.atoms[0])) //not valid to substitute a variable for a question var
                {
                    return substitution;
                }
                if(substitution.map[(int) general_atom].atoms[0] != 0 && !Term_Equal(&substitution.map[(int) general_atom], &subtree)) //unificiation var consistency criteria
                {
                    return substitution;
                }
                substitution.map[(int) general_atom] = subtree;
            }
            else
            {
                if(general_atom != specific->atoms[i]) //inequality since specific atom differs
                {
                    return substitution;
                }
            }
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
        if(substitution.map[(int) general_atom].atoms[0] != 0)
        {
            Term_OverrideSubterm(&general, i, &substitution.map[(int) general_atom]);
        }
    }
    return general;
}

//Search for variables which appear twice extensionally, if also appearing in the right side of the implication
//then introduce as independent variable, else as dependent variable
static void countExtensionTerms(Term *cur_inheritance, int *appearing)
{
    if(Narsese_copulaEquals(cur_inheritance->atoms[0], ':')) //inheritance
    {
        Term subject = Term_ExtractSubterm(cur_inheritance, 1);
        for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
        {
            if(Narsese_IsNonCopulaAtom(subject.atoms[i]))
            {
                appearing[(int) subject.atoms[i]] += 1;
            }
        }
    }
}

Term IntroduceImplicationVariables(Term implication)
{
    assert(Narsese_copulaEquals(implication.atoms[0], '$'), "An implication is expected here!");
    Term left_side = Term_ExtractSubterm(&implication, 1);
    Term right_side = Term_ExtractSubterm(&implication, 2);
    bool right_contains[TERMS_MAX] = {0};
    int appearing[TERMS_MAX] = {0};
    if(Narsese_copulaEquals(right_side.atoms[0], ':')) //inheritance
    {
        Term subject = Term_ExtractSubterm(&right_side, 1);
        for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
        {
            Atom atom = subject.atoms[i];
            if(Narsese_IsNonCopulaAtom(atom))
            {
                right_contains[(int) atom] = true;
                appearing[(int) atom] += 1;
            }
        }
    }
    while(Narsese_copulaEquals(left_side.atoms[0], '+')) //sequence
    {
        Term potential_inheritance = Term_ExtractSubterm(&left_side, 2);
        countExtensionTerms(&potential_inheritance, appearing);
        left_side = Term_ExtractSubterm(&left_side, 1);
    }
    countExtensionTerms(&left_side, appearing);
    Substitution subs = { .success = true };
    int depvar_i = 1;
    int indepvar_i = 1;
    bool already_handled[TERMS_MAX] = {0};
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        Atom atom = implication.atoms[i];
        if(!already_handled[(int) atom] && appearing[(int) atom] > 1)
        {
            if(right_contains[(int) atom])
            {
                assert(indepvar_i <= 9, "More than 9 variables being introduced? That's not supported.");
                char varname[3] = "$1";
                varname[1] = (char) ('0' + indepvar_i);
                subs.map[(int) atom] = Narsese_AtomicTerm(varname);
                indepvar_i++;
            }
            else
            {
                assert(depvar_i <= 9, "More than 9 variables being introduced? That's not supported.");
                char varname[3] = "#1";
                varname[1] = (char) ('0' + depvar_i);
                subs.map[(int) atom] = Narsese_AtomicTerm(varname);
                depvar_i++;
            }
        }
        already_handled[(int) atom] = true;
    }
    return Variable_ApplySubstitute(implication, subs);
}
