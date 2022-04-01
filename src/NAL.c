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

#include "NAL.h"

static int atomsCounter = 1; //allows to avoid memset
static int atomsAppeared[ATOMS_MAX] = {0};
static bool NAL_AtomAppearsTwice(Term *conclusionTerm)
{
    if(!ATOM_APPEARS_TWICE_FILTER)
        return false;
    if(Narsese_copulaEquals(conclusionTerm->atoms[0], INHERITANCE) || Narsese_copulaEquals(conclusionTerm->atoms[0], SIMILARITY)) //similarity or inheritance
    {
        atomsCounter++;
        for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
        {
            Atom atom = conclusionTerm->atoms[i];
            if(atomsAppeared[conclusionTerm->atoms[i]] == atomsCounter) //atom already appeared
            {
                return true;
            }
            if(Narsese_IsSimpleAtom(atom))
            {
                atomsAppeared[atom] = atomsCounter;
            }
        }
    }
    if(Narsese_copulaEquals(conclusionTerm->atoms[0], EQUIVALENCE) || Narsese_copulaEquals(conclusionTerm->atoms[0], IMPLICATION))
    {
        Term t1 = Term_ExtractSubterm(conclusionTerm, 1);
        Term t2 = Term_ExtractSubterm(conclusionTerm, 2);
        if(Term_Equal(&t1, &t2))
        {
            return true;
        }
    }
    return false;
}

static bool NAL_NestedHOLStatement(Term *conclusionTerm)
{
    if(!NESTED_HOL_STATEMENT_FILTER)
        return false;
    //We don't allow two ==> or <=> in one statement:
    int imp_equ = 0;
    int temp_equ = 0;
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        if(Narsese_copulaEquals(conclusionTerm->atoms[i], IMPLICATION) || Narsese_copulaEquals(conclusionTerm->atoms[i], EQUIVALENCE))
        {
            imp_equ++;
        }
        if(Narsese_copulaEquals(conclusionTerm->atoms[i], TEMPORAL_IMPLICATION))
        {
            temp_equ++;
        }
        if(imp_equ >= 2 || temp_equ >= 2)
        {
            return true;
        }
    }
    return false;
}

static bool NAL_InhOrSimHasDepVar(Term *conclusionTerm)
{
    if(!INH_OR_SIM_HAS_DEP_VAR_FILTER)
        return false;
    if(Narsese_copulaEquals(conclusionTerm->atoms[0], INHERITANCE) ||
       Narsese_copulaEquals(conclusionTerm->atoms[0], SIMILARITY))
    {
        if(Variable_hasVariable(conclusionTerm, false, true, false))
        {
            return true;
        }
    }
    return false;
}

static bool NAL_HOLStatementComponentHasInvalidInhOrSim(Term *conclusionTerm, bool firstIteration)
{
    if(!HOL_STATEMENT_COMPONENT_HAS_INVALID_INH_OR_SIM_FILTER)
        return false;
    if(Narsese_copulaEquals(conclusionTerm->atoms[0], EQUIVALENCE) || Narsese_copulaEquals(conclusionTerm->atoms[0], IMPLICATION) || Narsese_copulaEquals(conclusionTerm->atoms[0], CONJUNCTION) || Narsese_copulaEquals(conclusionTerm->atoms[0], DISJUNCTION))
    {
        Term subject = Term_ExtractSubterm(conclusionTerm, 1);
        Term predicate = Term_ExtractSubterm(conclusionTerm, 2);
        return NAL_HOLStatementComponentHasInvalidInhOrSim(&subject, false) || NAL_HOLStatementComponentHasInvalidInhOrSim(&predicate, false);
    }
    if(!firstIteration && (Narsese_copulaEquals(conclusionTerm->atoms[0], INHERITANCE) || Narsese_copulaEquals(conclusionTerm->atoms[0], SIMILARITY)))
    {
        Term subject = Term_ExtractSubterm(conclusionTerm, 1);
        Term predicate = Term_ExtractSubterm(conclusionTerm, 2);
        if(Term_Equal(&subject, &predicate) || ((Variable_isIndependentVariable(subject.atoms[0])   || Variable_isDependentVariable(subject.atoms[0])) && 
                                                (Variable_isIndependentVariable(predicate.atoms[0]) || Variable_isDependentVariable(predicate.atoms[0]))))
        {
            return true;
        }
        if(!Variable_hasVariable(conclusionTerm, true, true, false) && HOL_COMPONENT_NO_VAR_IS_INVALID_FILTER)
        {
            return true;
        }
        if(!Narsese_HasSimpleAtom(conclusionTerm) && HOL_COMPONENT_NO_ATOMIC_IS_INVALID_FILTER)
        {
            return true;
        }
        bool SubjectIsImage =   Narsese_copulaEquals(  subject.atoms[0], INT_IMAGE1) || Narsese_copulaEquals(subject.atoms[0], INT_IMAGE2);
        bool PredicateIsImage = Narsese_copulaEquals(predicate.atoms[0], EXT_IMAGE1) || Narsese_copulaEquals(subject.atoms[0], EXT_IMAGE2);
        if(TERMS_WITH_VARS_AND_ATOMS_FILTER && !SubjectIsImage && !PredicateIsImage &&
           ((Variable_hasVariable(&subject, true, true, false)   && Narsese_HasSimpleAtom(&subject)) ||
            (Variable_hasVariable(&predicate, true, true, false) && Narsese_HasSimpleAtom(&predicate))))
        {
            return true;
        }
        if(TERMS_WITH_VARS_AND_ATOMS_FILTER && SubjectIsImage)
        {
            Term relation = Term_ExtractSubterm(&subject, 1);
            Term relata =   Term_ExtractSubterm(&subject, 2);
            if((Variable_hasVariable(&relation, true, true, false) && Narsese_HasSimpleAtom(&relation)) ||
               (Variable_hasVariable(&relata,   true, true, false) && Narsese_HasSimpleAtom(&relata)))
            {
                return true;
            }
        }
        if(TERMS_WITH_VARS_AND_ATOMS_FILTER && PredicateIsImage)
        {
            Term relation = Term_ExtractSubterm(&predicate, 1);
            Term relata =   Term_ExtractSubterm(&predicate, 2);
            if((Variable_hasVariable(&relation, true, true, false) && Narsese_HasSimpleAtom(&relation)) ||
               (Variable_hasVariable(&relata,   true, true, false) && Narsese_HasSimpleAtom(&relata)))
            {
                return true;
            }
        }
    }
    return false;
}

static bool NAL_JunctionNotRightNested(Term *conclusionTerm)
{
    if(!JUNCTION_NOT_RIGHT_NESTED_FILTER)
    {
        return false;
    }
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        if(Narsese_copulaEquals(conclusionTerm->atoms[i], CONJUNCTION) || Narsese_copulaEquals(conclusionTerm->atoms[i], DISJUNCTION))
        {
            int i_right_child = ((i+1)*2+1)-1;
            if(i < COMPOUND_TERM_SIZE_MAX &&  (Narsese_copulaEquals(conclusionTerm->atoms[i_right_child], CONJUNCTION) || Narsese_copulaEquals(conclusionTerm->atoms[i_right_child], DISJUNCTION)))
            {
                return true;
            }
        }
    }
    return false;
}

static bool EmptySetOp(Term *conclusionTerm) //to be refined, with atom appears twice restriction for now it's fine
{
    if(Narsese_copulaEquals(conclusionTerm->atoms[0], INHERITANCE))
    {
        if(Narsese_copulaEquals(conclusionTerm->atoms[1], EXT_INTERSECTION) || Narsese_copulaEquals(conclusionTerm->atoms[1], INT_DIFFERENCE))
        {
            if(Narsese_copulaEquals(conclusionTerm->atoms[3], EXT_SET) && Narsese_copulaEquals(conclusionTerm->atoms[4], EXT_SET))
            {
                return true;
            }
        }
        if(Narsese_copulaEquals(conclusionTerm->atoms[2], INT_INTERSECTION) || Narsese_copulaEquals(conclusionTerm->atoms[2], EXT_DIFFERENCE))
        {
            if(Narsese_copulaEquals(conclusionTerm->atoms[5], INT_SET) && Narsese_copulaEquals(conclusionTerm->atoms[6], INT_SET))
            {
                return true;
            }
        }
    }
    return false;
}

void NAL_DerivedEvent(Term conclusionTerm, long conclusionOccurrence, Truth conclusionTruth, Stamp stamp, long currentTime, double parentPriority, double conceptPriority, double occurrenceTimeOffset, Concept *validation_concept, long validation_cid, bool varIntro)
{
    if(varIntro && (Narsese_copulaEquals(conclusionTerm.atoms[0], TEMPORAL_IMPLICATION) || Narsese_copulaEquals(conclusionTerm.atoms[0], IMPLICATION) || Narsese_copulaEquals(conclusionTerm.atoms[0], EQUIVALENCE)))
    {
        bool success;
        Term conclusionTermWithVarExt = Variable_IntroduceImplicationVariables(conclusionTerm, &success, true);
        if(success && !Term_Equal(&conclusionTermWithVarExt, &conclusionTerm) && !NAL_HOLStatementComponentHasInvalidInhOrSim(&conclusionTermWithVarExt, true))
        {
            NAL_DerivedEvent(conclusionTermWithVarExt, conclusionOccurrence, conclusionTruth, stamp, currentTime, parentPriority, conceptPriority, occurrenceTimeOffset, validation_concept, validation_cid, false);
        }
        bool success2;
        Term conclusionTermWithVarInt = Variable_IntroduceImplicationVariables(conclusionTerm, &success2, false);
        if(success2 && !Term_Equal(&conclusionTermWithVarInt, &conclusionTerm) && !NAL_HOLStatementComponentHasInvalidInhOrSim(&conclusionTermWithVarInt, true))
        {
            NAL_DerivedEvent(conclusionTermWithVarInt, conclusionOccurrence, conclusionTruth, stamp, currentTime, parentPriority, conceptPriority, occurrenceTimeOffset, validation_concept, validation_cid, false);
        }
        if(Narsese_copulaEquals(conclusionTerm.atoms[0], IMPLICATION) || Narsese_copulaEquals(conclusionTerm.atoms[0], EQUIVALENCE))
        {
            return;
        }
    }
    if(varIntro && Narsese_copulaEquals(conclusionTerm.atoms[0], CONJUNCTION))
    {
        bool success;
        Term conclusionTermWithVarExt = Variable_IntroduceConjunctionVariables(conclusionTerm, &success, true);
        if(success && !Term_Equal(&conclusionTermWithVarExt, &conclusionTerm) && !NAL_HOLStatementComponentHasInvalidInhOrSim(&conclusionTermWithVarExt, true))
        {
            NAL_DerivedEvent(conclusionTermWithVarExt, conclusionOccurrence, conclusionTruth, stamp, currentTime, parentPriority, conceptPriority, occurrenceTimeOffset, validation_concept, validation_cid, false);
        }
        bool success2;
        Term conclusionTermWithVarInt = Variable_IntroduceConjunctionVariables(conclusionTerm, &success2, false);
        if(success2 && !Term_Equal(&conclusionTermWithVarInt, &conclusionTerm) && !NAL_HOLStatementComponentHasInvalidInhOrSim(&conclusionTermWithVarInt, true))
        {
            NAL_DerivedEvent(conclusionTermWithVarInt, conclusionOccurrence, conclusionTruth, stamp, currentTime, parentPriority, conceptPriority, occurrenceTimeOffset, validation_concept, validation_cid, false);
        }
        return;
    }
    Event e = { .term = conclusionTerm,
                .type = EVENT_TYPE_BELIEF, 
                .truth = conclusionTruth, 
                .stamp = stamp,
                .occurrenceTime = conclusionOccurrence,
                .occurrenceTimeOffset = occurrenceTimeOffset,
                .creationTime = currentTime };
    #pragma omp critical(Memory)
    {
        if(validation_concept == NULL || validation_concept->id == validation_cid) //concept recycling would invalidate the derivation (allows to lock only adding results to memory)
        {
            if(!NAL_AtomAppearsTwice(&conclusionTerm) && !NAL_NestedHOLStatement(&conclusionTerm) && !NAL_InhOrSimHasDepVar(&conclusionTerm) && !NAL_JunctionNotRightNested(&conclusionTerm) && !EmptySetOp(&conclusionTerm))
            {
                Memory_AddEvent(&e, currentTime, conceptPriority*parentPriority*Truth_Expectation(conclusionTruth), false, true, false, false);
            }
        }
    }
}
