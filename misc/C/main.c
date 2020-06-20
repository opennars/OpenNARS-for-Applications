#include <stdio.h>
#include "ona/NAR.h"

bool executed = false;
void NAR_Op()
{
    puts("Hello world");
    executed=true;
}
int main()
{
    NAR_INIT();
    NAR_AddOperation(Narsese_AtomicTerm("^op"), NAR_Op);
    NAR_AddInputNarsese("<(a &/ ^op) =/> g>.");
    NAR_AddInputNarsese("a. :|:");
    NAR_AddInputNarsese("g! :|:");
    assert(executed, "Eecution should have happened!");
    return 0;
}
