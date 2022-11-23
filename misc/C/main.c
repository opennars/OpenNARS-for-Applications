//build with: gcc -lONA -lm main.c
#include <stdio.h>
#include "./../../src/NAR.h"
//#include <ona/NAR.h>

bool executed = false;
Feedback NAR_Op()
{
    puts("Hello world");
    executed=true;
    return (Feedback) {0};
}
int main()
{
    NAR_INIT();
    NAR_AddOperation("^op", NAR_Op);
    NAR_AddInputNarsese("<(a &/ ^op) =/> g>.");
    NAR_AddInputNarsese("a. :|:");
    NAR_AddInputNarsese("g! :|:");
    Globals_assert(executed, "Eecution should have happened!");
    return 0;
}
