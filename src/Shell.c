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

#include "Shell.h"

static void Shell_op_left(Term args)
{
    fputs("^left executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
static void Shell_op_right(Term args)
{
    fputs("^right executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
static void Shell_op_up(Term args)
{
    fputs("^up executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
static void Shell_op_down(Term args)
{
    fputs("^down executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
static void Shell_op_say(Term args)
{
    fputs("^say executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
static void Shell_op_pick(Term args)
{
    fputs("^pick executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
static void Shell_op_drop(Term args)
{
    fputs("^drop executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
static void Shell_op_go(Term args)
{
    fputs("^go executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
static void Shell_op_activate(Term args)
{
    fputs("^activate executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
static void Shell_op_deactivate(Term args)
{
    fputs("^deactivate executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
void Shell_NARInit()
{
    fflush(stdout);
    NAR_INIT();
    PRINT_DERIVATIONS = true;
    NAR_AddOperation(Narsese_AtomicTerm("^left"), Shell_op_left); 
    NAR_AddOperation(Narsese_AtomicTerm("^right"), Shell_op_right); 
    NAR_AddOperation(Narsese_AtomicTerm("^up"), Shell_op_up); 
    NAR_AddOperation(Narsese_AtomicTerm("^down"), Shell_op_down);
    NAR_AddOperation(Narsese_AtomicTerm("^say"), Shell_op_say);
    NAR_AddOperation(Narsese_AtomicTerm("^pick"), Shell_op_pick);
    NAR_AddOperation(Narsese_AtomicTerm("^drop"), Shell_op_drop);
    NAR_AddOperation(Narsese_AtomicTerm("^go"), Shell_op_go);
    NAR_AddOperation(Narsese_AtomicTerm("^activate"), Shell_op_activate);
    NAR_AddOperation(Narsese_AtomicTerm("^deactivate"), Shell_op_deactivate);
}

bool Shell_ProcessInput(char *line)
{
    //trim string, for IRC etc. convenience
    for(int i=strlen(line)-1; i>=0; i--)
    {
        if(!isspace(line[i]))
        {
            break;
        }
        line[i] = 0;
    }
    int size = strlen(line);
    if(size==0)
    {
        NAR_Cycles(1);
    }
    else
    {
        //accept comments, commands, timestep, and narsese
        if(line[0] == '/' && line[1] == '/')
        {
            fputs("Comment: ", stdout);
            puts(&line[2]); fflush(stdout);
            return false;
        }
        else
        if(!strcmp(line,"*reset"))
        {
            return true;
        }
        else
        if(!strcmp(line,"*volume=0"))
        {
            PRINT_DERIVATIONS = false;
        }
        else
        if(!strcmp(line,"*stats"))
        {
            Stats_Print(currentTime);
        }
        else
        if(!strcmp(line,"quit"))
        {
            exit(0);
        }
        else
        if(!strcmp(line,"*volume=100"))
        {
            PRINT_DERIVATIONS = true;
        }
        else
        if(!strcmp(line,"*motorbabbling=false"))
        {
            MOTOR_BABBLING_CHANCE = 0.0;
        }
        else
        if(!strcmp(line,"*motorbabbling=true"))
        {
            MOTOR_BABBLING_CHANCE = MOTOR_BABBLING_CHANCE_INITIAL;
        }
        else
        if(strspn(line, "0123456789"))
        {
            unsigned int steps;
            sscanf(line, "%u", &steps);
            printf("performing %u inference steps:\n", steps); fflush(stdout);
            NAR_Cycles(steps);
            printf("done with %u additional inference steps.\n", steps); fflush(stdout);
        }
        else
        if(!strcmp(line,"*reset"))
        {
            Memory_INIT();
        }
        else
        {
            NAR_AddInputNarsese(line);
        }
    }
    fflush(stdout);
    return false;
}

void Shell_Start()
{
    Shell_NARInit();
    for(;;)
    {
        char line[1024] = {0};
        if(fgets(line, 1024, stdin) == NULL)
        {
            Stats_Print(currentTime);
            exit(0);
        }
        if(Shell_ProcessInput(line)) //reset?
        {
            Shell_NARInit();
        }
    }
}
