/* 
 * The MIT License
 *
 * Copyright 2020 The OpenNARS authors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

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

Substitution Variable_Unify2(Term *general, Term *specific, bool unifyQueryVarOnly)
{
    Substitution substitution = {0};
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        Atom general_atom = general->atoms[i];
        if(general_atom)
        {
            bool is_allowed_var = unifyQueryVarOnly ? Variable_isQueryVariable(general_atom) : Variable_isVariable(general_atom);
            if(is_allowed_var)
            {
                assert(general_atom <= 27, "Variable_Unify: Problematic variable encountered, only $1-$9, #1-#9 and ?1-?9 are allowed!");
                Term subtree = Term_ExtractSubterm(specific, i);
                if(Variable_isQueryVariable(general_atom) && Variable_isVariable(subtree.atoms[0])) //not valid to substitute a variable for a question var
                {
                    return substitution;
                }
                if(substitution.map[(int) general_atom].atoms[0] != 0 && !Term_Equal(&substitution.map[(int) general_atom], &subtree)) //unificiation var consistency criteria
                {
                    return substitution;
                }
                if(Narsese_copulaEquals(subtree.atoms[0], SET_TERMINATOR)) //not allowed to unify with set terminator
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

Substitution Variable_Unify(Term *general, Term *specific)
{
    return Variable_Unify2(general, specific, false);
}

Term Variable_ApplySubstitute(Term general, Substitution substitution, bool *success)
{
    assert(substitution.success, "A substitution from unsuccessful unification cannot be used to substitute variables!");
    *success = true;
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        Atom general_atom = general.atoms[i];
        bool is_variable = Variable_isVariable(general_atom);
        assert(!is_variable || general_atom <= 27, "Variable_ApplySubstitute: Problematic variable encountered, only $1-$9, #1-#9 and ?1-?9 are allowed!");
        if(is_variable && substitution.map[(int) general_atom].atoms[0] != 0)
        {
            if(!Term_OverrideSubterm(&general, i, &substitution.map[(int) general_atom]))
            {
                *success = false;
            }
        }
    }
    return general;
}

//Search for variables which appear twice extensionally, if also appearing in the right side of the implication
//then introduce as independent variable, else as dependent variable
static void countStatementAtoms(Term *cur_inheritance, int *appearing, bool extensionally, bool ignore_structure)
{
    bool similarity = Narsese_copulaEquals(cur_inheritance->atoms[0], SIMILARITY);
    if(Narsese_copulaEquals(cur_inheritance->atoms[0], INHERITANCE) || similarity) //inheritance and similarity
    {
        Term subject = Term_ExtractSubterm(cur_inheritance, 1);
        Term predicate = Term_ExtractSubterm(cur_inheritance, 2);
        if(extensionally || similarity)
        {
            if(Narsese_copulaEquals(subject.atoms[0], INT_IMAGE1) || Narsese_copulaEquals(subject.atoms[0], INT_IMAGE2))
            {
                Term relation = Term_ExtractSubterm(&subject, 1);
                countStatementAtoms(&relation, appearing, extensionally, true);
            }
            else
            {
                countStatementAtoms(&subject, appearing, extensionally, true);
            }
            if(Narsese_copulaEquals(predicate.atoms[0], EXT_IMAGE1) || Narsese_copulaEquals(predicate.atoms[0], EXT_IMAGE2))
            {
                Term potential_image = Term_ExtractSubterm(&predicate, 2);
                countStatementAtoms(&potential_image, appearing, extensionally, true);
            }
        }
        if(!extensionally || similarity)
        {
            if(Narsese_copulaEquals(predicate.atoms[0], EXT_IMAGE1) || Narsese_copulaEquals(predicate.atoms[0], EXT_IMAGE2))
            {
                Term relation = Term_ExtractSubterm(&predicate, 1);
                countStatementAtoms(&relation, appearing, extensionally, true);
            }
            else
            {
                countStatementAtoms(&predicate, appearing, extensionally, true);
            }
            if(Narsese_copulaEquals(subject.atoms[0], INT_IMAGE1) || Narsese_copulaEquals(subject.atoms[0], INT_IMAGE2))
            {
                Term argument = Term_ExtractSubterm(&subject, 2);
                countStatementAtoms(&argument, appearing, extensionally, true);
            }
        }
    }
    if(ignore_structure) //check avoids introducing vars for entire statements
    {
        if(VARS_IN_MULTI_ELEMENT_SETS_FILTER && (Narsese_copulaEquals(cur_inheritance->atoms[0], EXT_SET) || Narsese_copulaEquals(cur_inheritance->atoms[0], INT_SET)) && !Narsese_copulaEquals(cur_inheritance->atoms[2], SET_TERMINATOR))
        {
            return;
        }
        for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
        {
            Atom atom = cur_inheritance->atoms[i];
            if(Narsese_IsSimpleAtom(atom) || Variable_isVariable(atom))
            {
                appearing[(int) cur_inheritance->atoms[i]] += 1;
            }
        }
    }
}

static void countHigherOrderStatementAtoms(Term *term, int *appearing, bool extensionally)
{
    if(Narsese_copulaEquals(term->atoms[0], NEGATION))
    {
        Term substatement = Term_ExtractSubterm(term, 1);
        countHigherOrderStatementAtoms(&substatement, appearing, extensionally);
        return;
    }
    else
    if(Narsese_copulaEquals(term->atoms[0], SEQUENCE) || Narsese_copulaEquals(term->atoms[0], CONJUNCTION) || Narsese_copulaEquals(term->atoms[0], TEMPORAL_IMPLICATION)
        || Narsese_copulaEquals(term->atoms[0], IMPLICATION) || Narsese_copulaEquals(term->atoms[0], EQUIVALENCE))
    {
        Term subject = Term_ExtractSubterm(term, 1);
        Term predicate = Term_ExtractSubterm(term, 2);
        countHigherOrderStatementAtoms(&subject, appearing, extensionally);
        countHigherOrderStatementAtoms(&predicate, appearing, extensionally);
        return;
    }
    countStatementAtoms(term, appearing, extensionally, false);
}

int appearing_left[ATOMS_MAX] = {0};
int appearing_right[ATOMS_MAX] = {0};
char variable_id[ATOMS_MAX] = {0};
Term Variable_IntroduceImplicationVariables(Term implication, bool *success, bool extensionally)
{
    #pragma omp critical(VarIntro)
    {
        assert(Narsese_copulaEquals(implication.atoms[0], TEMPORAL_IMPLICATION) || Narsese_copulaEquals(implication.atoms[0], IMPLICATION) || Narsese_copulaEquals(implication.atoms[0], EQUIVALENCE), "An implication is expected here!");
        Term left_side = Term_ExtractSubterm(&implication, 1);
        Term right_side = Term_ExtractSubterm(&implication, 2);
        memset(appearing_left, 0, ATOMS_MAX*sizeof(int));
        memset(appearing_right, 0, ATOMS_MAX*sizeof(int));
        countHigherOrderStatementAtoms(&left_side, appearing_left, extensionally);
        countHigherOrderStatementAtoms(&right_side, appearing_right, extensionally);
        char depvar_i = 1;
        char indepvar_i = 1;
        memset(variable_id, 0, ATOMS_MAX*sizeof(char));
        Term implication_copy = implication;
        *success = true;
        for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
        {
            Atom atom = implication_copy.atoms[i];
            if(appearing_left[(int) atom] >= 2 || appearing_right[(int) atom] >= 2 || (appearing_left[(int) atom] && appearing_right[(int) atom]))
            {
                if(appearing_right[(int) atom] && appearing_left[(int) atom])
                {
                    int var_id = variable_id[(int) atom] = variable_id[(int) atom] ? variable_id[(int) atom] : indepvar_i++;
                    if(var_id <= 9) //can only introduce up to 9 variables
                    {
                        char varname[3] = { '$', ('0' + var_id), 0 }; //$i
                        Term varterm = Narsese_AtomicTerm(varname);
                        if(!Term_OverrideSubterm(&implication, i, &varterm))
                        {
                            *success = false;
                            break;
                        }
                    }
                }
                else
                {
                    int var_id = variable_id[(int) atom] = variable_id[(int) atom] ? variable_id[(int) atom] : depvar_i++;
                    if(var_id <= 9) //can only introduce up to 9 variables
                    {
                        char varname[3] = { '#', ('0' + var_id), 0 }; //#i
                        Term varterm = Narsese_AtomicTerm(varname);
                        if(!Term_OverrideSubterm(&implication, i, &varterm))
                        {
                            *success = false;
                            break;
                        }
                    }
                }
            }
        }
    }
    return implication;
}

Term Variable_IntroduceConjunctionVariables(Term conjunction, bool *success, bool extensionally)
{
    #pragma omp critical(VarIntro)
    {
        assert(Narsese_copulaEquals(conjunction.atoms[0], CONJUNCTION), "A conjunction is expected here!");
        memset(appearing_left, 0, ATOMS_MAX*sizeof(int));
        Term left_side = conjunction;
        countHigherOrderStatementAtoms(&left_side, appearing_left, extensionally);
        char depvar_i = 1;
        memset(variable_id, 0, ATOMS_MAX*sizeof(char));
        Term conjunction_copy = conjunction;
        *success = true;
        for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
        {
            Atom atom = conjunction_copy.atoms[i];
            if(appearing_left[(int) atom] >= 2)
            {
                int var_id = variable_id[(int) atom] = variable_id[(int) atom] ? variable_id[(int) atom] : depvar_i++;
                if(var_id <= 9) //can only introduce up to 9 variables
                {
                    char varname[3] = { '#', ('0' + var_id), 0 }; //#i
                    Term varterm = Narsese_AtomicTerm(varname);
                    if(!Term_OverrideSubterm(&conjunction, i, &varterm))
                    {
                        *success = false;
                        break;
                    }
                }
            }
        }
    }
    return conjunction;
}

void Variable_Normalize(Term *term)
{
    int independent_i = 1, dependent_i = 1, query_i = 1;
    bool normalized[COMPOUND_TERM_SIZE_MAX] = {0};
    //replace variables with numeric representation, then return the term
    for(int j=0; j<COMPOUND_TERM_SIZE_MAX; j++)
    {
        Atom atom = term->atoms[j];
        char varType = Variable_isIndependentVariable(atom)  ? '$' :            (Variable_isDependentVariable(atom) ? '#' :          '?');
        int *varIndex = Variable_isIndependentVariable(atom) ? &independent_i : (Variable_isDependentVariable(atom) ? &dependent_i : &query_i);
        if(!normalized[j] && Variable_isVariable(atom))
        {
            assert(*varIndex<=9, "Variable overflow in variable normalization!");
            char varname[3] = { varType, ('0' + *varIndex), 0 }; //$i, #j, ?k
            (*varIndex)++;
            for(int k=j; k<COMPOUND_TERM_SIZE_MAX; k++)
            {
                Atom atom2 = term->atoms[k];
                if(atom == atom2)
                {
                    term->atoms[k] = Narsese_AtomicTermIndex(varname);
                    normalized[k] = true;
                }
            }
        }
    }
}
