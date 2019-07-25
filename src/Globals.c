#include "Globals.h"
#include <stdio.h>
#include <stdlib.h>

int OUTPUT = 1;
void assert(bool b, char* message)
{
    if(!b)
    {
        puts(message);
        puts("Test failed.");
        exit(1);
    }
}
