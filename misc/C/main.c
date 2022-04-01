#include <stdio.h>
#include "./../../src/NAR.h"
//#include <ona/NAR.h>

bool executed = false;
void NAR_Op()
{
    puts("Hello world");
    executed=true;
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
