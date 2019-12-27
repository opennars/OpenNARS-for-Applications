#include "NAL.h"
//macro for syntactic representation, increases readability
#define R(premise1, premise2, _, conclusion, truthFunction) NAL_GenerateRule(#premise1, #premise2, #conclusion, #truthFunction);

int ruleID = 0;
static void NAL_GeneratePremisesUnifier(int i, Atom atom, int premiseIndex)
{
    if(atom)
    {
        //upper case atoms are treated as variables in the meta rule language
        if(atom_names[atom-1][0] >= 'A' && atom_names[atom-1][0] <= 'Z')
        {
            //unification failure by inequal value assignment (value at position i versus previously assigned one), and variable binding
            printf("if(substitutions[%d] && substitutions[%d] != term%d.atoms[%d]){ goto RULE_%d; }\n", atom, atom, premiseIndex, i, ruleID);
            printf("substitutions[%d] = term%d.atoms[%d];\n",atom, premiseIndex, i);
        }
        else
        {
            //structural constraint given by copulas at position i
            printf("if(term%d.atoms[%d] != %d){ goto RULE_%d; }\n", premiseIndex, i, atom, ruleID);
        }
    }
}

static void NAL_GenerateConclusionSubstitution(int i, Atom atom, Term *conclusion_term)
{
    if(atom)
    {
        if(atom_names[atom-1][0] >= 'A' && atom_names[atom-1][0] <= 'Z')
        {
            //conclusion term gets variables substituted
            printf("assert(substitutions[%d]>0,\"Meta variable was not substituted, check inference rule!\");\n", atom);
            printf("conclusion.atoms[%d] = substitutions[%d];\n", i, conclusion_term->atoms[i]);
        }
        else
        {
            //conclusion term inherits structure from meta rule
            printf("conclusion.atoms[%d] = %d;\n", i, conclusion_term->atoms[i]);
        }
    }
}

static void NAL_GenerateRule(char *premise1, char *premise2, char* conclusion, char* truthFunction)
{
    Term term1 = Encode_Term(premise1);
    Term term2 = Encode_Term(premise2);
    Term conclusion_term = Encode_Term(conclusion);
    printf("RULE_%d:\n{\n",ruleID++);
    puts("Atom substitutions[255] = {0};");
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        NAL_GeneratePremisesUnifier(i, term1.atoms[i], 1);
    }
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        NAL_GeneratePremisesUnifier(i, term2.atoms[i], 2);
    }
    puts("Term conclusion = {0};");
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        NAL_GenerateConclusionSubstitution(i, conclusion_term.atoms[i], &conclusion_term);
    }
    printf("Truth conclusionTruth = %s(truth1,truth2);\n", truthFunction);
    puts("derivedEvent(conclusion, conclusionTruth);");
    puts("}");
}

void NAL_GenerateRuleTable()
{
    puts("#include \"RuleTable.h\"");
    puts("void RuleTable(Term term1, Term term2, Truth truth1, Truth truth2)\n{");
#define H_NAL_RULES
#include "NAL.h"
#undef H_NAL_RULES
    printf("RULE_%d:;\n}\n", ruleID);
}
