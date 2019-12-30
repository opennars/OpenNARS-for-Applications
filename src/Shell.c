#include "Shell.h"

void Shell_Start()
{
    YAN_INIT();
    OUTPUT = 0;
    INPUT = false;
    for(;;)
    {
        char line[1024];
        scanf("%1023[^\n]", line);
        int size = strlen(line);
        if(size==0)
        {
            YAN_Cycles(1);
        }
        else
        {
            if(!strcmp(line,"*volume=0"))
            {
                PRINT_DERIVATIONS = false;
            }
            else
            if(!strcmp(line,"*volume=100"))
            {
                PRINT_DERIVATIONS = true;
            }
            else
            if(!strcmp(line,"100000"))
            {
                puts("performing 100k inference steps:");
                YAN_Cycles(100000);
                puts("done with 100k additional inference steps.");
            }
            else
            {
                YAN_AddInput(Encode_Term(line), EVENT_TYPE_BELIEF, YAN_DEFAULT_TRUTH, 0, true);
            }
        }
        memset(line, 0, 1024);
        getchar();
    }
}
