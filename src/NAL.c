#include "NAL.h"

int ruleID = 0;
static void NAL_GeneratePremisesUnifier(int i, Atom atom, int premiseIndex)
{
    if(atom)
    {
        //upper case atoms are treated as variables in the meta rule language
        if(Encode_atomNames[atom-1][0] >= 'A' && Encode_atomNames[atom-1][0] <= 'Z')
        {
            //unification failure by inequal value assignment (value at position i versus previously assigned one), and variable binding
            printf("subtree = Term_ExtractSubterm(&term%d, %d);\n", premiseIndex, i);
            printf("if(substitutions[%d].atoms[0]!=0 && !Term_Equal(&substitutions[%d], &subtree)){ goto RULE_%d; }\n", atom, atom, ruleID);
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
        if(Encode_atomNames[atom-1][0] >= 'A' && Encode_atomNames[atom-1][0] <= 'Z')
        {
            //conclusion term gets variables substituteda
            printf("Term_OverrideSubterm(&conclusion,%d,&substitutions[%d]);\n", i, atom);
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
    Term term1 = Encode_Term(premise1);
    Term term2 = doublePremise ? Encode_Term(premise2) : (Term) {0};
    Term conclusion_term = Encode_Term(conclusion);
    printf("RULE_%d:\n{\n", ruleID++);
    //skip double/single premise rule if single/double premise
    if(doublePremise) { printf("if(!doublePremise) { goto RULE_%d; }\n", ruleID); }
    if(!doublePremise) { printf("if(doublePremise) { goto RULE_%d; }\n", ruleID); }
    puts("Term substitutions[255] = {0}; Term subtree = {0};");
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
    puts("NAL_DerivedEvent(RuleTable_Reduce(conclusion, false), conclusionOccurrence, conclusionTruth, conclusionStamp, currentTime, parentPriority);}\n");
}

static void NAL_GenerateReduction(char *premise1, char* conclusion)
{
    NAL_GenerateConclusionTerm(premise1, NULL, conclusion, false);
    puts("fputs(\"Reduced:\", stdout); Encode_PrintTerm(&term1); puts(\" -> \"); Encode_PrintTerm(&conclusion); puts(\"\"); return conclusion;\n}");
}

void NAL_GenerateRuleTable()
{
    puts("#include \"RuleTable.h\"");
    puts("void RuleTable_Apply(Term term1, Term term2, Truth truth1, Truth truth2, long conclusionOccurrence, Stamp conclusionStamp, long currentTime, double parentPriority, bool doublePremise)\n{\ngoto RULE_0;");
#define H_NAL_RULES
#include "NAL.h"
#undef H_NAL_RULES
    printf("RULE_%d:;\n}\n", ruleID);
    printf("Term RuleTable_Reduce(Term term1, bool doublePremise)\n{\ngoto RULE_%d;\n", ruleID);
#define H_NAL_REDUCTIONS
#include "NAL.h"
#undef H_NAL_REDUCTIONS
    printf("RULE_%d:;\nreturn term1;\n}\n\n", ruleID);
}

void NAL_DerivedEvent(Term conclusionTerm, long conclusionOccurrence, Truth conclusionTruth, Stamp stamp, long currentTime, double parentPriority)
{
    Event e = { .term = conclusionTerm,
                .type = EVENT_TYPE_BELIEF, 
                .truth = conclusionTruth, 
                .stamp = stamp,
                .occurrenceTime = conclusionOccurrence };
    Memory_addEvent(&e, currentTime, parentPriority*Truth_Expectation(conclusionTruth), false, true, false, false);
}
