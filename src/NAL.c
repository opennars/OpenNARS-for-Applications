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

int ruleID = 0;
static void NAL_GeneratePremisesUnifier(int i, Atom atom, int premiseIndex)
{
    if(atom)
    {
        //upper case atoms are treated as variables in the meta rule language
        if(Narsese_atomNames[atom-1][0] >= 'A' && Narsese_atomNames[atom-1][0] <= 'Z')
        {
            //unification failure by inequal value assignment (value at position i versus previously assigned one), and variable binding
            printf("subtree = Term_ExtractSubterm(&term%d, %d);\n", premiseIndex, i);
            printf("if((substitutions[%d].atoms[0]!=0 && !Term_Equal(&substitutions[%d], &subtree)) || Narsese_copulaEquals(subtree.atoms[0], SET_TERMINATOR)){ goto RULE_%d; }\n", atom, atom, ruleID);
            printf("substitutions[%d] = subtree;\n", atom);
        }
        else
        {
            //structural constraint given by copulas at position i
            printf("if(term%d.atoms[%d] != %d){ goto RULE_%d; }\n", premiseIndex, i, atom, ruleID);
        }
    }
}

static void NAL_GenerateConclusionSubstitution(int i, Atom atom)
{
    if(atom)
    {
        if(Narsese_atomNames[atom-1][0] >= 'A' && Narsese_atomNames[atom-1][0] <= 'Z')
        {
            //conclusion term gets variables substituted
            printf("if(!Term_OverrideSubterm(&conclusion,%d,&substitutions[%d])){ goto RULE_%d; }\n", i, atom, ruleID);
        }
        else
        {
            //conclusion term inherits structure from meta rule, namely the copula
            printf("conclusion.atoms[%d] = %d;\n", i, atom);
        }
    }
}

static void NAL_GenerateConclusionTerm(char *premise1, char *premise2, char* conclusion, bool doublePremise)
{
    Term term1 = Narsese_Term(premise1);
    Term term2 = doublePremise ? Narsese_Term(premise2) : (Term) {0};
    Term conclusion_term = Narsese_Term(conclusion);
    printf("RULE_%d:\n{\n", ruleID++);
    //skip double/single premise rule if single/double premise
    if(doublePremise) { printf("if(!doublePremise) { goto RULE_%d; }\n", ruleID); }
    if(!doublePremise) { printf("if(doublePremise) { goto RULE_%d; }\n", ruleID); }
    puts("Term substitutions[27+NUM_ELEMENTS(Narsese_RuleTableVars)] = {0}; Term subtree = {0};"); //27 because of 9 indep, 9 dep, 9 query vars
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        NAL_GeneratePremisesUnifier(i, term1.atoms[i], 1);
    }
    if(doublePremise)
    {
        for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
        {
            NAL_GeneratePremisesUnifier(i, term2.atoms[i], 2);
        }
    }
    puts("Term conclusion = {0};");
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        NAL_GenerateConclusionSubstitution(i, conclusion_term.atoms[i]);
    }
}

static void NAL_GenerateRule(char *premise1, char *premise2, char* conclusion, char* truthFunction, bool doublePremise, bool switchTruthArgs)
{
    NAL_GenerateConclusionTerm(premise1, premise2, conclusion, doublePremise);
    if(switchTruthArgs)
    {
        printf("Truth conclusionTruth = %s(truth2,truth1);\n", truthFunction);
    }
    else
    {
        printf("Truth conclusionTruth = %s(truth1,truth2);\n", truthFunction);
    }
    printf("NAL_DerivedEvent(RuleTable_Reduce(conclusion), conclusionOccurrence, conclusionTruth, conclusionStamp, currentTime, parentPriority, conceptPriority, occurrenceTimeOffset, validation_concept, validation_cid, %s == Truth_Induction || %s == Truth_Intersection);}\n", truthFunction, truthFunction);
}

static void NAL_GenerateReduction(char *premise1, char* conclusion)
{
    NAL_GenerateConclusionTerm(premise1, NULL, conclusion, false);
    puts("IN_DEBUG( fputs(\"Reduced: \", stdout); Narsese_PrintTerm(&term1); fputs(\" -> \", stdout); Narsese_PrintTerm(&conclusion); puts(\"\"); ) \nreturn conclusion;\n}");
}

void NAL_GenerateRuleTable()
{
    puts("#include \"RuleTable.h\"");
    puts("void RuleTable_Apply(Term term1, Term term2, Truth truth1, Truth truth2, long conclusionOccurrence, double occurrenceTimeOffset, Stamp conclusionStamp, long currentTime, double parentPriority, double conceptPriority, bool doublePremise, Concept *validation_concept, long validation_cid)\n{\ngoto RULE_0;");
#define H_NAL_RULES
#include "NAL.h"
#undef H_NAL_RULES
    printf("RULE_%d:;\n}\n", ruleID);
    printf("Term RuleTable_Reduce(Term term1)\n{\nbool doublePremise = false;\ngoto RULE_%d;\n", ruleID);
#define H_NAL_REDUCTIONS
#include "NAL.h"
#undef H_NAL_REDUCTIONS
    printf("RULE_%d:;\nreturn term1;\n}\n\n", ruleID);
}

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
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        if(Narsese_copulaEquals(conclusionTerm->atoms[i], IMPLICATION) || Narsese_copulaEquals(conclusionTerm->atoms[i], EQUIVALENCE) || Narsese_copulaEquals(conclusionTerm->atoms[i], TEMPORAL_IMPLICATION))
        {
            imp_equ++;
        }
        if(imp_equ >= 2)
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
        if(!Variable_hasVariable(conclusionTerm, true, true, false) && HOL_COMPONENT_NO_VAR_IS_INVALID)
        {
            return true; //no specific components if specific hyp not allowed
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

void NAL_DerivedEvent(Term conclusionTerm, long conclusionOccurrence, Truth conclusionTruth, Stamp stamp, long currentTime, double parentPriority, double conceptPriority, double occurrenceTimeOffset, Concept *validation_concept, long validation_cid, bool varIntro)
{
    if(ALLOW_VAR_INTRO_IN_DERIVATIONS && varIntro && (Narsese_copulaEquals(conclusionTerm.atoms[0], TEMPORAL_IMPLICATION) || Narsese_copulaEquals(conclusionTerm.atoms[0], IMPLICATION)))
    {
        bool success;
        Term conclusionTermWithVarExt = Variable_IntroduceImplicationVariables(conclusionTerm, &success, true);
        if(success && !Term_Equal(&conclusionTermWithVarExt, &conclusionTerm))
        {
            NAL_DerivedEvent(conclusionTermWithVarExt, conclusionOccurrence, conclusionTruth, stamp, currentTime, parentPriority, conceptPriority, occurrenceTimeOffset, validation_concept, validation_cid, false);
        }
        bool success2;
        Term conclusionTermWithVarInt = Variable_IntroduceImplicationVariables(conclusionTerm, &success2, false);
        if(success2 && !Term_Equal(&conclusionTermWithVarInt, &conclusionTerm))
        {
            NAL_DerivedEvent(conclusionTermWithVarInt, conclusionOccurrence, conclusionTruth, stamp, currentTime, parentPriority, conceptPriority, occurrenceTimeOffset, validation_concept, validation_cid, false);
        }
        if(Narsese_copulaEquals(conclusionTerm.atoms[0], IMPLICATION) && !ALLOW_SPECIFIC_VERSIONS_OF_VAR_INTRO_DERIVATIONS)
        {
            return;
        }
    }
    if(ALLOW_VAR_INTRO_IN_DERIVATIONS && varIntro && Narsese_copulaEquals(conclusionTerm.atoms[0], CONJUNCTION))
    {
        bool success;
        Term conclusionTermWithVarExt = Variable_IntroduceConjunctionVariables(conclusionTerm, &success, true);
        if(success && !Term_Equal(&conclusionTermWithVarExt, &conclusionTerm))
        {
            NAL_DerivedEvent(conclusionTermWithVarExt, conclusionOccurrence, conclusionTruth, stamp, currentTime, parentPriority, conceptPriority, occurrenceTimeOffset, validation_concept, validation_cid, false);
        }
        bool success2;
        Term conclusionTermWithVarInt = Variable_IntroduceConjunctionVariables(conclusionTerm, &success2, false);
        if(success2 && !Term_Equal(&conclusionTermWithVarInt, &conclusionTerm))
        {
            NAL_DerivedEvent(conclusionTermWithVarInt, conclusionOccurrence, conclusionTruth, stamp, currentTime, parentPriority, conceptPriority, occurrenceTimeOffset, validation_concept, validation_cid, false);
        }
        if(!ALLOW_SPECIFIC_VERSIONS_OF_VAR_INTRO_DERIVATIONS)
        {
            return;
        }
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
            if(!NAL_AtomAppearsTwice(&conclusionTerm) && !NAL_NestedHOLStatement(&conclusionTerm) && !NAL_InhOrSimHasDepVar(&conclusionTerm) && 
               !NAL_HOLStatementComponentHasInvalidInhOrSim(&conclusionTerm, true) && !NAL_JunctionNotRightNested(&conclusionTerm))
            {
                Memory_AddEvent(&e, currentTime, conceptPriority*parentPriority*Truth_Expectation(conclusionTruth), false, true, false);
            }
        }
    }
}
