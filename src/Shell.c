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
    fputs(Narsese_operatorNames[0], stdout); fputs(" executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
static void Shell_op_right(Term args)
{
    fputs(Narsese_operatorNames[1], stdout); fputs(" executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
static void Shell_op_up(Term args)
{
    fputs(Narsese_operatorNames[2], stdout); fputs(" executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
static void Shell_op_down(Term args)
{
    fputs(Narsese_operatorNames[3], stdout); fputs(" executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
static void Shell_op_say(Term args)
{
    fputs(Narsese_operatorNames[4], stdout); fputs(" executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
static void Shell_op_pick(Term args)
{
    fputs(Narsese_operatorNames[5], stdout); fputs(" executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
static void Shell_op_drop(Term args)
{
    fputs(Narsese_operatorNames[6], stdout); fputs(" executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
static void Shell_op_go(Term args)
{
    fputs(Narsese_operatorNames[7], stdout); fputs(" executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
static void Shell_op_activate(Term args)
{
    fputs(Narsese_operatorNames[8], stdout); fputs(" executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
static void Shell_op_deactivate(Term args)
{
    fputs(Narsese_operatorNames[9], stdout); fputs(" executed with args ", stdout); Narsese_PrintTerm(&args); puts(""); fflush(stdout);
}
void Shell_NARInit()
{
    fflush(stdout);
    NAR_INIT();
    PRINT_DERIVATIONS = true;
    int k=0; if(k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm("^left"), Shell_op_left); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm("^right"), Shell_op_right); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm("^up"), Shell_op_up); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm("^down"), Shell_op_down); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm("^say"), Shell_op_say); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm("^pick"), Shell_op_pick); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm("^drop"), Shell_op_drop); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm("^go"), Shell_op_go); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm("^activate"), Shell_op_activate); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm("^deactivate"), Shell_op_deactivate); if(++k >= OPERATIONS_MAX) { return; };
    assert(false, "Shell_NARInit: Ran out of operators, add more there, or decrease OPERATIONS_MAX!");
}

int Shell_ProcessInput(char *line)
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
            return SHELL_CONTINUE;
        }
        else
        if(!strcmp(line,"*reset"))
        {
            return SHELL_RESET;
        }
        else
        if(!strcmp(line,"*volume=0"))
        {
            PRINT_DERIVATIONS = false;
        }
        else
        if(!strcmp(line,"*stats"))
        {
            puts("//*stats");
            Stats_Print(currentTime);
            puts("//*done");
        }
        else
        if(!strcmp(line,"*inverted_atom_index"))
        {
            InvertedAtomIndex_Print();
        }
        else
        if(!strcmp(line,"*concepts"))
        {
            puts("//*concepts");
            for(int opi=0; opi<OPERATIONS_MAX; opi++)
            {
                if(operations[opi].term.atoms[0])
                {
                    printf("*setopname %d ", opi+1);
                    Narsese_PrintTerm(&operations[opi].term);
                    puts("");
                }
            }
            for(int i=0; i<concepts.itemsAmount; i++)
            {
                Concept *c = concepts.items[i].address;
                assert(c != NULL, "Concept is null");
                fputs("//", stdout);
                Narsese_PrintTerm(&c->term);
                printf(": { \"priority\": %f, \"usefulness\": %f, \"useCount\": %ld, \"lastUsed\": %ld, \"frequency\": %f, \"confidence\": %f, \"termlinks\": [", c->priority, concepts.items[i].priority, c->usage.useCount, c->usage.lastUsed, c->belief.truth.frequency, c->belief.truth.confidence);
                Term left = Term_ExtractSubterm(&c->term, 1);
                Term left_left = Term_ExtractSubterm(&left, 1);
                Term left_right = Term_ExtractSubterm(&left, 2);
                Term right = Term_ExtractSubterm(&c->term, 2);
                Term right_left = Term_ExtractSubterm(&right, 1);
                Term right_right = Term_ExtractSubterm(&right, 2);
                fputs("\"", stdout);
                Narsese_PrintTerm(&left);
                fputs("\", ", stdout);
                fputs("\"", stdout);
                Narsese_PrintTerm(&right);
                fputs("\", ", stdout);
                fputs("\"", stdout);
                Narsese_PrintTerm(&left_left);
                fputs("\", ", stdout);
                fputs("\"", stdout);
                Narsese_PrintTerm(&left_right);
                fputs("\", ", stdout);
                fputs("\"", stdout);
                Narsese_PrintTerm(&right_left);
                fputs("\", ", stdout);
                fputs("\"", stdout);
                Narsese_PrintTerm(&right_right);
                fputs("\"", stdout);
                puts("]}");
                if(c->belief.type != EVENT_TYPE_DELETED)
                {
                    Memory_printAddedEvent(&c->belief, 1, true, false, false, false);
                }
                for(int opi=0; opi<OPERATIONS_MAX; opi++)
                {
                    for(int h=0; h<c->precondition_beliefs[opi].itemsAmount; h++)
                    {
                        Implication *imp = &c->precondition_beliefs[opi].array[h];
                        Memory_printAddedImplication(&imp->term, &imp->truth, imp->occurrenceTimeOffset, 1, true, false, false);
                    }
                }
            }
            puts("//*done");
        }
        else
        if(!strcmp(line,"*cycling_belief_events"))
        {
            puts("//*cycling_belief_events");
            for(int i=0; i<cycling_belief_events.itemsAmount; i++)
            {
                Event *e = cycling_belief_events.items[i].address;
                assert(e != NULL, "Event is null");
                Narsese_PrintTerm(&e->term);
                printf(": { \"priority\": %f, \"time\": %ld } ", cycling_belief_events.items[i].priority, e->occurrenceTime);
                Truth_Print(&e->truth);
            }
            puts("//*done");
        }
        else
        if(!strcmp(line,"*cycling_goal_events"))
        {
            puts("//*cycling_goal_events");
            for(int i=0; i<cycling_goal_events.itemsAmount; i++)
            {
                Event *e = cycling_goal_events.items[i].address;
                assert(e != NULL, "Event is null");
                Narsese_PrintTerm(&e->term);
                printf(": {\"priority\": %f, \"time\": %ld } ", cycling_goal_events.items[i].priority, e->occurrenceTime);
                Truth_Print(&e->truth);
            }
            puts("//*done");
        }
        else
        if(!strcmp(line,"quit"))
        {
            return SHELL_EXIT;
        }
        else
        if(!strcmp(line,"*volume=100"))
        {
            PRINT_DERIVATIONS = true;
        }
        else
        if(!strncmp("*babblingops=", line, strlen("*babblingops=")))
        {
            sscanf(&line[strlen("*babblingops=")], "%d", &BABBLING_OPS);
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
        if(!strncmp("*motorbabbling=", line, strlen("*motorbabbling=")))
        {
            sscanf(&line[strlen("*motorbabbling=")], "%lf", &MOTOR_BABBLING_CHANCE);
        }
        else
        if(!strncmp("*setoprange ", line, strlen("*setoprange "))) //min max
        {
            int opID;
            double min_range;
            double max_range;
            char intstr[50];
            sscanf(&line[strlen("*setoprange ")], "%d %lf %lf %s", &opID, &min_range, &max_range, (char*) &intstr);
            printf("RANGE: %f %f \n", min_range, max_range);
            assert(min_range <= max_range, "Min range of op arg needs to be less or equal than max");
            operations[opID-1].min_range = min_range;
            operations[opID-1].max_range = max_range;
            if(!strcmp(intstr, "int"))
            {
                operations[opID-1].intbabbling = true;
            }
            else
            {
                operations[opID-1].intbabbling = false;
            }
        }
        else
        if(!strncmp("*setopname ", line, strlen("*setopname ")))
        {
            int opID;
            char opname[ATOMIC_TERM_LEN_MAX] = {0};
            sscanf(&line[strlen("*setopname ")], "%d %s", &opID, (char*) &opname);
            Term previousOpAtom = Narsese_AtomicTerm(Narsese_operatorNames[opID-1]);
            strncpy(Narsese_operatorNames[opID-1], opname, ATOMIC_TERM_LEN_MAX);
            strncpy(Narsese_atomNames[previousOpAtom.atoms[0]-1], opname, ATOMIC_TERM_LEN_MAX);
            operations[opID-1].term = Narsese_AtomicTerm(Narsese_operatorNames[opID-1]);
        }
        else
        if(strspn(line, "0123456789") && strlen(line) == strspn(line, "0123456789"))
        {
            unsigned int steps;
            sscanf(line, "%u", &steps);
            printf("performing %u inference steps:\n", steps); fflush(stdout);
            NAR_Cycles(steps);
            printf("done with %u additional inference steps.\n", steps); fflush(stdout);
        }
        else
        {
            NAR_AddInputNarsese(line);
        }
    }
    fflush(stdout);
    return SHELL_CONTINUE;
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
            break;
        }
        int cmd = Shell_ProcessInput(line);
        if(cmd == SHELL_RESET) //reset?
        {
            Shell_NARInit();
        }
        else
        if(cmd == SHELL_EXIT)
        {
            break;
        }
    }
}
