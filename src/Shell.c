#include "Shell.h"

static void Shell_op_1()
{
    puts("^1 executed");
}
static void Shell_op_2()
{
    puts("^2 executed");
}
static void Shell_op_3()
{
    puts("^3 executed");
}
static void Shell_op_4()
{
    puts("^4 executed");
}
static void Shell_op_5()
{
    puts("^5 executed");
}
void Shell_Start()
{
    YAN_INIT();
    YAN_AddOperation(Encode_AtomicTerm("^1"), Shell_op_1); 
    YAN_AddOperation(Encode_AtomicTerm("^2"), Shell_op_2); 
    YAN_AddOperation(Encode_AtomicTerm("^3"), Shell_op_3); 
    YAN_AddOperation(Encode_AtomicTerm("^4"), Shell_op_4);
    YAN_AddOperation(Encode_AtomicTerm("^5"), Shell_op_5);
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
            //accept commands
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
            if(!strcmp(line,"1000"))
            {
                puts("performing 1000 inference steps:");
                YAN_Cycles(1000);
                puts("done with 1000 additional inference steps.");
            }
            else
            {
                //parse event marker, punctuation, and finally the term:
                int str_len = strlen(line);
                bool isEvent = str_len >= 3 && line[str_len-1] == ':' && line[str_len-2] == '|' && line[str_len-3] == ':'; 
                int punctuation_offset = isEvent ? 5 : 1;
                char punctuation = line[str_len-punctuation_offset];
                assert(punctuation == '!' || punctuation == '?' || punctuation == '.', "Punctuation has to be belief . goal ! or question ?");
                line[str_len-punctuation_offset] = 0; //we will only parse the term before it
                Term term = Encode_Term(line);
#if STAGE==2
                //apply reduction rules to term:
                term = RuleTable_Reduce(term, false);
#endif
                //answer questions:
                Truth best_truth = {0};
                Term best_term = {0};
                if(punctuation == '?')
                {
                    for(int i=0; i<concepts.itemsAmount; i++)
                    {
                        Concept *c = concepts.items[i].address;
                        //now match the concept term, for now just supporting one question var
                        //TODO use unification approach as the generated RuleTable already uses.
                        for(int j=0; j<COMPOUND_TERM_SIZE_MAX; j++)
                        {
                            if(term.atoms[j] != 0 && c->term.atoms[j] != term.atoms[j] && Encode_atomNames[term.atoms[j]-1][0] != '?')
                            {
                                goto Continue;
                            }
                        }
                        if(isEvent)
                        {
                            if(c->belief_spike.type != EVENT_TYPE_DELETED)
                            {
                                Truth potential_best_truth = Truth_Projection(c->belief_spike.truth, c->belief_spike.occurrenceTime, currentTime);
                                if(Truth_Expectation(potential_best_truth) > Truth_Expectation(best_truth))
                                {
                                    best_truth = potential_best_truth;
                                    best_term = c->belief_spike.term;
                                }
                            }
                        }
                        else
                        {
                            if(c->belief.type != EVENT_TYPE_DELETED)
                            {
                                if(Truth_Expectation(c->belief.truth) > Truth_Expectation(best_truth))
                                {
                                    best_truth = c->belief.truth;
                                    best_term = c->belief.term;
                                }
                            }
                        }
                        Continue:;
                    }
                    fputs("Answer: ", stdout);
                    Encode_PrintTerm(&best_term);
                    fputs(" ", stdout);
                    Truth_Print(&best_truth);
                }
                //input beliefs and goals
                else
                {
                    if(punctuation == '!')
                    {
                        YAN_AddInput(term, EVENT_TYPE_GOAL, YAN_DEFAULT_TRUTH, 0, !isEvent);
                    }
                    else
                    {
                        YAN_AddInput(term, EVENT_TYPE_BELIEF, YAN_DEFAULT_TRUTH, 0, !isEvent);
                    }
                }
            }
        }
        memset(line, 0, 1024);
        getchar();
    }
}
