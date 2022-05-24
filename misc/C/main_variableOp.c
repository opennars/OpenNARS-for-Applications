//build with: gcc -lONA -lm main_variableOp.c
#include <stdio.h>
#include "./../../src/NAR.h"
//#include <ona/NAR.h>

bool executed = false; //arg = ({SELF} * (input * output)) 
Feedback NAR_Op(Term arg) // 1 2  3   4     5  6      7
{                         // * {} *   SELF     input  output
    puts("Hello world");
    executed=true;
    Feedback feedback = {0};
    Atom variable = arg.atoms[7-1];
    Term result = Narsese_AtomicTerm("42");
    if(Variable_isVariable(variable))
    {
        feedback.subs.map[(int) variable] = result;
    }
    else
    if(variable != result.atoms[0]) //if the var already is a specific value, then the value has to match
    {
        feedback.failed = true;
    }
    return feedback;
}

int main()
{
    NAR_INIT();
    NAR_AddOperation("^op", NAR_Op);
    NAR_AddInputNarsese("<((a &/ <({SELF} * (2 * #1)) --> ^op>) &/ <({SELF} * (#1 * $2)) --> ^op>) =/> <$2 --> result>>.");
    NAR_AddInputNarsese("a. :|:");
    NAR_AddInputNarsese("<42 --> result>! :|:");
    Globals_assert(executed, "Eecution should have happened!");
    return 0;
}
