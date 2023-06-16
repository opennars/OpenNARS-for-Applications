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

static Feedback Shell_op_nop(Term args)
{
    return (Feedback) {0};
}
void Shell_NARInit()
{
    fflush(stdout);
    NAR_INIT();
    PRINT_DERIVATIONS = true;
    int k=0; if(k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation("^left", Shell_op_nop); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation("^right", Shell_op_nop); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation("^up", Shell_op_nop); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation("^down", Shell_op_nop); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation("^say", Shell_op_nop); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation("^pick", Shell_op_nop); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation("^drop", Shell_op_nop); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation("^go", Shell_op_nop); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation("^activate", Shell_op_nop); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation("^deactivate", Shell_op_nop); if(++k >= OPERATIONS_MAX) { return; };
    assert(false, "Shell_NARInit: Ran out of operators, add more there, or decrease OPERATIONS_MAX!");
}

int Shell_ProcessInput(char *line)
{
    //trim string, for IRC etc. convenience
    for(int i=strlen(line)-1; i>=0; i--)
    {
        if(!isspace((int) line[i]))
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
            PRINT_EVENTS_PRIORITY_THRESHOLD = 1.0;
        }
        else
        if(!strcmp(line,"*volume=100"))
        {
            PRINT_EVENTS_PRIORITY_THRESHOLD = 0.0;
        }
        else
        if(!strncmp("*volume=", line, strlen("*volume=")))
        {
            int volume = 0;
            sscanf(&line[strlen("*volume=")], "%d", &volume);
            PRINT_EVENTS_PRIORITY_THRESHOLD = 1.0 - ((double) volume) / 100.0;
        }
        else
        if(!strncmp("*decisionthreshold=", line, strlen("*decisionthreshold=")))
        {
            sscanf(&line[strlen("*decisionthreshold=")], "%lf", &DECISION_THRESHOLD);
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
        if(!strcmp(line,"*opconfig"))
        {
            puts("//*opconfig");
            printf("*motorbabbling=%f\n", MOTOR_BABBLING_CHANCE);
            printf("*babblingops=%d\n", BABBLING_OPS);
            for(int opi=0; opi<OPERATIONS_MAX; opi++)
            {
                if(operations[opi].term.atoms[0])
                {
                    printf("*setopname %d ", opi+1);
                    Narsese_PrintTerm(&operations[opi].term);
                    puts("");
                }
                for(int oparg=0; oparg<OPERATIONS_BABBLE_ARGS_MAX; oparg++)
                {
                    if(operations[opi].arguments[oparg].atoms[0])
                    {
                        printf("*setoparg %d %d ", opi+1, oparg+1);
                        Narsese_PrintTerm(&operations[opi].arguments[oparg]);
                        puts("");
                    }
                }
            }
        }
        else
        if(!strcmp(line,"*concepts"))
        {
            puts("//*concepts");
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
                    Memory_printAddedEvent(&c->belief.stamp, &c->belief, 1, true, false, false, false, false);
                }
                for(int opi=0; opi<OPERATIONS_MAX; opi++)
                {
                    for(int h=0; h<c->precondition_beliefs[opi].itemsAmount; h++)
                    {
                        Implication *imp = &c->precondition_beliefs[opi].array[h];
                        Memory_printAddedImplication(&imp->stamp, &imp->term, &imp->truth, imp->occurrenceTimeOffset, 1, true, false, false);
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
            for(int layer=0; layer<CYCLING_GOAL_EVENTS_LAYERS; layer++)
            {
                for(int i=0; i<cycling_goal_events[layer].itemsAmount; i++)
                {
                    Event *e = cycling_goal_events[layer].items[i].address;
                    assert(e != NULL, "Event is null");
                    Narsese_PrintTerm(&e->term);
                    printf(": {\"priority\": %f, \"time\": %ld } ", cycling_goal_events[layer].items[i].priority, e->occurrenceTime);
                    Truth_Print(&e->truth);
                }
            }
            puts("//*done");
        }
        else
        if(!strcmp(line,"quit"))
        {
            return SHELL_EXIT;
        }
        else
        if(!strncmp("*babblingops=", line, strlen("*babblingops=")))
        {
            sscanf(&line[strlen("*babblingops=")], "%d", &BABBLING_OPS);
        }
        else
        if(!strncmp("*currenttime=", line, strlen("*currenttime=")))
        {
            sscanf(&line[strlen("*currenttime=")], "%ld", &currentTime);
        }
        else
        if(!strncmp("*stampid=", line, strlen("*stampid=")))
        {
            sscanf(&line[strlen("*stampid=")], "%ld", &base);
        }
        else
        if(!strncmp("*stampimport=[", line, strlen("*stampimport=[")))
        {
            // Find the position of the first '[' character
            char *start = strchr(line, '[');
            // Find the position of the last ']' character
            char *end = strrchr(line, ']');
            // Extract the substring between the '[' and ']' characters
            char substr[1000];
            strncpy(substr, start + 1, end - start - 1);
            substr[end - start - 1] = '\0';
            // Tokenize the substring using ',' as the delimiter
            char *token = strtok(substr, ",");
            // Reset import stamp:
            importstamp = (Stamp) {0};
            // Parse each token and store it in the stamp if it fits
            int i = 0;
            while(token != NULL && i < STAMP_SIZE)
            {
                importstamp.evidentalBase[i++] = strtol(token, NULL, 10);
                token = strtok(NULL, ",");
            }
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
        if(!strncmp("*questionpriming=", line, strlen("*questionpriming=")))
        {
            sscanf(&line[strlen("*questionpriming=")], "%lf", &QUESTION_PRIMING);
        }
        else
        if(!strncmp("*setopname ", line, strlen("*setopname ")))
        {
            assert(concepts.itemsAmount == 0, "Operators can only be registered right after initialization / reset!");
            int opID;
            char opname[ATOMIC_TERM_LEN_MAX+1] = {0};
            opname[ATOMIC_TERM_LEN_MAX-1] = 0;
            sscanf(&line[strlen("*setopname ")], "%d %" STR(ATOMIC_TERM_LEN_MAX) "s", &opID, (char*) &opname);
            assert(opID >= 1 && opID <= OPERATIONS_MAX, "Operator index out of bounds, it can only be between 1 and OPERATIONS_MAX!");
            Term newTerm = Narsese_AtomicTerm(opname);
            for(int i=0; i<OPERATIONS_MAX; i++)
            {
                if(Term_Equal(&operations[i].term, &newTerm)) //already exists, so clear the duplicate name
                {
                    for(int k=i; k<OPERATIONS_MAX; k++) //and the names of all the operations after it
                    {
                        operations[k].term = (Term) {0};
                    }
                }
            }
            operations[opID - 1].term = newTerm;
            if(!operations[opID - 1].action) //allows to use more ops than are registered by the C code to be utilized through NAR.py
            {
                operations[opID - 1].action = Shell_op_nop;
            }
        }
        else
        if(!strncmp("*setoparg ", line, strlen("*setoparg ")))
        {
            int opID;
            int opArgID;
            char argname[NARSESE_LEN_MAX+1] = {0};
            argname[NARSESE_LEN_MAX-1] = 0;
            sscanf(&line[strlen("*setoparg ")], "%d %d %" STR(NARSESE_LEN_MAX) "[^\n]", &opID, &opArgID, (char*) &argname);
            assert(opID >= 1 && opID <= OPERATIONS_MAX, "Operator index out of bounds, it can only be between 1 and OPERATIONS_MAX!");
            assert(opArgID >= 1 && opArgID <= OPERATIONS_BABBLE_ARGS_MAX, "Operator arg index out of bounds, it can only be between 1 and OPERATIONS_BABBLE_ARGS_MAX!");
            operations[opID - 1].arguments[opArgID-1] = Narsese_Term(argname);
        }
        else
        if(!strncmp("*query ", line, strlen("*query ")))
        {
            double threshold;
            char narsese[NARSESE_LEN_MAX+1] = {0};
            narsese[NARSESE_LEN_MAX-1] = 0;
            sscanf(&line[strlen("*query ")], "%lf %" STR(NARSESE_LEN_MAX) "[^\n]", &threshold, (char*) &narsese);
            assert(threshold >= 0.0 && threshold <= 1.0, "Query truth exp out of bounds!");
            NAR_AddInputNarsese2(narsese, true, threshold);
        }
        else
        if(!strncmp("*setopstdin ", line, strlen("*setopstdin ")))
        {
            int opID;
            sscanf(&line[strlen("*setopstdin ")], "%d", &opID);
            operations[opID - 1].stdinOutput = true;
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
        if(!strncmp("*concurrent", line, strlen("*concurrent")))
        {
            currentTime-=1;
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
            if(EXIT_STATS)
            {
                Stats_Print(currentTime);
            }
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
