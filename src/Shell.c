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
            if(!strcmp(line,"1000"))
            {
                puts("performing 1000 inference steps:");
                YAN_Cycles(1000);
                puts("done with 1000 additional inference steps.");
            }
            else
            {
                Term term = Encode_Term(line);
#if STAGE==2
                term = RuleTable_Reduce(term, false);
#endif
                Truth best_truth = {0};
                Term best_term = {0};
                int str_len = strlen(line);
                if(line[str_len-1] == '?' || (str_len>=5 && line[str_len-5] == '?'))
                {
                    for(int i=0; i<concepts.itemsAmount; i++)
                    {
                        Concept *c = concepts.items[i].address;
                        //now match the concept term, for now just supporting one question var
                        //TODO use unification approach as the generated RuleTable already uses.
                        for(int j=0; j<COMPOUND_TERM_SIZE_MAX; j++)
                        {
                            if(term.atoms[j] != 0 && c->term.atoms[j] != term.atoms[j] && atom_names[term.atoms[j]-1][0] != '?')
                            {
                                goto Continue;
                            }
                        }
                        if(c->belief.type != EVENT_TYPE_DELETED)
                        {
                            if(Truth_Expectation(c->belief.truth) > Truth_Expectation(best_truth))
                            {
                                best_truth = c->belief.truth;
                                best_term = c->belief.term;
                            }
                        }
                        Continue:;
                    }
                    fputs("Answer: ", stdout);
                    Encode_PrintTerm(&best_term);
                    fputs(" ", stdout);
                    Truth_Print(&best_truth);
                }
                else
                {
                    bool isEvent = str_len >= 3 && line[str_len-1] == ':' && line[str_len-2] == '|' && line[str_len-3] == ':'; 
		    if(line[str_len-1] == '!' || (str_len>=5 && line[str_len-5] == '!'))
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
