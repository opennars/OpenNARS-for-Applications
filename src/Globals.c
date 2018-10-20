#include "Globals.h"
#include <stdio.h>
#include <stdlib.h>

void assert(bool b, char* message)
{
    if(!b)
    {
        printf("%s", message);
        printf("\nTest failed.\n");
        exit(1);
    }
}
