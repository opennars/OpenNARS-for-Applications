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
    Term generalcpy = *general;
    Substitution substitution = {0};
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        Atom general_atom = generalcpy.atoms[i];
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
                if(Narsese_copulaEquals(subtree.atoms[0], '@')) //not allowed to unify with set terminator
                {
                    return substitution;
                }
                substitution.map[(int) general_atom] = subtree;
            }
            else
            {
                int left_child_i = (i+1)*2-1;
                bool is_function = left_child_i < COMPOUND_TERM_SIZE_MAX && Narsese_copulaEquals(general_atom, '*') && (general->atoms[left_child_i] == f_plus || general->atoms[left_child_i] == f_minus);
                bool specific_has_value = !Variable_isVariable(specific->atoms[i]) && Narsese_IsNumericAtom(specific->atoms[i]);
                if(is_function && specific_has_value)
                {
                    int right_child_i = (i+1)*2+1-1;
                    double specific_value = Narsese_NumericAtomValue(specific->atoms[i]);
                    Atom general_args = general->atoms[right_child_i];
                    if(Narsese_copulaEquals(general_args, '*'))
                    {
                        Atom left_arg = general->atoms[(right_child_i+1)*2-1];
                        Atom right_arg = general->atoms[(right_child_i+1)*2+1-1];
                        char valueStr[350];
                        if(Narsese_IsNumericAtom(left_arg) && Variable_isVariable(right_arg))
                        {
                            double left_arg_value = Narsese_NumericAtomValue(left_arg);
                            double value = general->atoms[left_child_i] == f_plus ?  specific_value - left_arg_value : left_arg_value - specific_value;
                            sprintf(valueStr, "%f", value);
                            substitution.map[(int) right_arg] = Narsese_AtomicTerm(valueStr);
                        }
                        else
                        if(Narsese_IsNumericAtom(right_arg) && Variable_isVariable(left_arg))
                        {
                            double right_arg_value = Narsese_NumericAtomValue(right_arg);
                            double value = general->atoms[left_child_i] == f_minus ? specific_value + right_arg_value : specific_value - right_arg_value;
                            sprintf(valueStr, "%f", value);
                            substitution.map[(int) left_arg] = Narsese_AtomicTerm(valueStr);
                        }
                        Term_RemoveCompoundSubtermAt(&generalcpy, i); //avoid failing unification due to different subterm structure
                    }
                }
                else
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

//Search for variables which appear twice extensionally/intensionally, if also appearing in the right side of the implication
//then introduce as independent variable, else as dependent variable
static Atom countAtoms(Term *cur_inheritance, int *appearing, bool extensionally)
{
    Atom referenceValueAtom = 0; //the value extracted to relate all other values to in induction
    if(Narsese_copulaEquals(cur_inheritance->atoms[0], ':') || Narsese_copulaEquals(cur_inheritance->atoms[0], '=')) //inheritance and similarity
    {
        Term side = Term_ExtractSubterm(cur_inheritance, extensionally ? 1 : 2);
        for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
        {
            Atom atom = side.atoms[i];
            if(Narsese_IsNonCopulaAtom(atom))
            {
                if(Narsese_IsNumericAtom(atom))
                {
                    referenceValueAtom = atom;
                }
                if(appearing != NULL && atom != f_plus && atom != f_minus) //don't intro vars for functions
                {
                    appearing[(int) side.atoms[i]] += 1;
                }
            }
        }
    }
    return referenceValueAtom;
}

void relateNumbers(Term *implication, Atom referenceValueAtom)
{
    if(referenceValueAtom == 0)
        return;
    Term imp = *implication;
    double referenceValue = Narsese_NumericAtomValue(referenceValueAtom);
    Term operator = {0};
    if(Narsese_copulaEquals(implication->atoms[0], '+'))
    {
        //(a &/ ^op) =/> b
        //=/>  &/     a  ^op
        //0    1   2  3  4
        Term op = Term_ExtractSubterm(implication, 4);
        if(Narsese_isOperation(&op))
        {
            operator = op;
        }
    }
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        Atom atom = imp.atoms[i];
        if(Narsese_IsNumericAtom(atom))
        {
            double value = Narsese_NumericAtomValue(atom);
            double offset = fabs(value - referenceValue);
            char offsetStr[350];
            sprintf(offsetStr, "%f", offset);
            Atom offsetAtom = Narsese_AtomicTermIndex(offsetStr);
            //(f * (referenceValue * offset))
            //* f *            referenceValue    offset
            //0 1 2   3    4   5                 6
            Term relata = {0};
            relata.atoms[0] = Narsese_AtomicTermIndex("*");
            relata.atoms[2] = Narsese_AtomicTermIndex("*");
            relata.atoms[5] = referenceValueAtom;
            relata.atoms[6] = offsetAtom;
            if(value == referenceValue)
            {
                continue;
            }
            if(value > referenceValue)
            {
                relata.atoms[1] = f_plus;
            }
            else
            if(value < referenceValue)
            {
                relata.atoms[1] = f_minus;
            }
            //Now replace the original numeric atom with the arithmetic expression
            Term implicationCopy = *implication;
            double success = Term_OverrideSubterm(&implicationCopy, i, &relata);
            if(success)
            {
                *implication = implicationCopy;
            }
        }
    }
    if(operator.atoms[0])
    {
        Term_RemoveCompoundSubtermAt(implication, 4);
        Term_OverrideSubterm(implication, 4, &operator);
    }
}

Term IntroduceImplicationVariables(Term implication, bool *success, bool extensionally)
{
    assert(Narsese_copulaEquals(implication.atoms[0], '$'), "An implication is expected here!");
    Term left_side = Term_ExtractSubterm(&implication, 1);
    Term right_side = Term_ExtractSubterm(&implication, 2);
    //build numerical relationships if numeric terms are included:
    Atom referenceValueAtom = countAtoms(&right_side, NULL, extensionally);
    if(referenceValueAtom != 0)
    {
        relateNumbers(&implication, referenceValueAtom);
        left_side = Term_ExtractSubterm(&implication, 1); //re-extract sides as subst was in implication,
        right_side = Term_ExtractSubterm(&implication, 2); //could be further optimized
    }
    //continue with var intro:
    int appearing_left[ATOMS_MAX] = {0};
    int appearing_right[ATOMS_MAX] = {0};
    while(Narsese_copulaEquals(left_side.atoms[0], '+')) //sequence
    {
        Term potential_inheritance = Term_ExtractSubterm(&left_side, 2);
        countAtoms(&potential_inheritance, appearing_left, extensionally);
        left_side = Term_ExtractSubterm(&left_side, 1);
    }
    countAtoms(&left_side, appearing_left, extensionally);
    countAtoms(&right_side, appearing_right, extensionally);
    char depvar_i = 1;
    char indepvar_i = 1;
    char variable_id[ATOMS_MAX] = {0};
    Term implication_copy = implication;
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        Atom atom = implication_copy.atoms[i];
        if(appearing_left[(int) atom] >= 2 || (appearing_left[(int) atom] && appearing_right[(int) atom]))
        {
            if(appearing_right[(int) atom])
            {
                int var_id = variable_id[(int) atom] = variable_id[(int) atom] ? variable_id[(int) atom] : indepvar_i++;
                if(var_id <= 9) //can only introduce up to 9 variables
                {
                    char varname[3] = { '$', ('0' + var_id), 0 }; //$i
                    Term varterm = Narsese_AtomicTerm(varname);
                    if(!Term_OverrideSubterm(&implication, i, &varterm))
                    {
                        *success = false;
                        return implication;
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
                        return implication;
                    }
                }
            }
        }
    }
    *success = true;
    return implication;
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
