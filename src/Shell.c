#include "Shell.h"

static void Shell_op_left(Term args)
{
    fputs("^left executed with args ", stdout); Narsese_PrintTerm(&args); puts("");
    
}
static void Shell_op_right(Term args)
{
    fputs("^right executed with args ", stdout); Narsese_PrintTerm(&args); puts("");
}
static void Shell_op_up(Term args)
{
    fputs("^up executed with args ", stdout); Narsese_PrintTerm(&args); puts("");
}
static void Shell_op_down(Term args)
{
    fputs("^down executed with args ", stdout); Narsese_PrintTerm(&args); puts("");
}
static void Shell_op_say(Term args)
{
    fputs("^say executed with args ", stdout); Narsese_PrintTerm(&args); puts("");
}
static void Shell_op_pick(Term args)
{
    fputs("^pick executed with args ", stdout); Narsese_PrintTerm(&args); puts("");
}
static void Shell_op_drop(Term args)
{
    fputs("^drop executed with args ", stdout); Narsese_PrintTerm(&args); puts("");
}
static void Shell_op_go(Term args)
{
    fputs("^go executed with args ", stdout); Narsese_PrintTerm(&args); puts("");
}
static void Shell_op_activate(Term args)
{
    fputs("^activate executed with args ", stdout); Narsese_PrintTerm(&args); puts("");
}
static void Shell_op_deactivate(Term args)
{
    fputs("^deactivate executed with args ", stdout); Narsese_PrintTerm(&args); puts("");
}
void Shell_Start()
{
INIT:
    fflush(stdout);
    YAN_INIT();
    PRINT_DERIVATIONS = true;
    YAN_AddOperation(Narsese_AtomicTerm("^left"), Shell_op_left); 
    YAN_AddOperation(Narsese_AtomicTerm("^right"), Shell_op_right); 
    YAN_AddOperation(Narsese_AtomicTerm("^up"), Shell_op_up); 
    YAN_AddOperation(Narsese_AtomicTerm("^down"), Shell_op_down);
    YAN_AddOperation(Narsese_AtomicTerm("^say"), Shell_op_say);
    YAN_AddOperation(Narsese_AtomicTerm("^pick"), Shell_op_pick);
    YAN_AddOperation(Narsese_AtomicTerm("^drop"), Shell_op_drop);
    YAN_AddOperation(Narsese_AtomicTerm("^go"), Shell_op_go);
    YAN_AddOperation(Narsese_AtomicTerm("^activate"), Shell_op_activate);
    YAN_AddOperation(Narsese_AtomicTerm("^deactivate"), Shell_op_deactivate);
    for(;;)
    {
        char line[1024] = {0};
        if(fgets(line, 1024, stdin) == NULL)
        {
            Stats_Print(currentTime);
            exit(0);
        }
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
            YAN_Cycles(1);
        }
        else
        {
            //accept commands
            if(line[0] == '/' && line[1] == '/')
            {
                fputs("Comment: ", stdout);
                puts(&line[2]); fflush(stdout);
                continue;
            }
            else
            if(!strcmp(line,"**"))
            {
                goto INIT;
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
            if(strspn(line, "0123456789"))
            {
                unsigned int steps;
                sscanf(line, "%u", &steps);
                printf("performing %u inference steps:\n", steps); fflush(stdout);
                YAN_Cycles(steps);
                printf("done with %u additional inference steps.\n", steps); fflush(stdout);
            }
            else
            {
                Term term;
                Truth tv;
                char punctuation;
                bool isEvent;
                Narsese_Sentence(line, &term, &punctuation, &isEvent, &tv);
#if STAGE==2
                //apply reduction rules to term:
                term = RuleTable_Reduce(term, false);
#endif
                //answer questions:
                Truth best_truth = { .frequency = 0.0, .confidence = 1.0 };
                Truth best_truth_projected = {0};
                Term best_term = {0};
                long answerOccurrenceTime = OCCURRENCE_ETERNAL;
                long answerCreationTime = 0;
                if(punctuation == '?')
                {
                    bool isImplication = Narsese_copulaEquals(term.atoms[0], '$');
                    fputs("Input: ", stdout);
                    Narsese_PrintTerm(&term);
                    fputs("?", stdout);
                    puts(isEvent ? " :|:" : ""); 
                    fflush(stdout);
                    for(int i=0; i<concepts.itemsAmount; i++)
                    {
                        Concept *c = concepts.items[i].address;
                        //compare the predicate of implication, or if it's not an implication, the term
                        Term toCompare = isImplication ? Term_ExtractSubterm(&term, 2) : term; 
                        if(!Variable_Unify(&toCompare, &c->term).success)
                        {
                            goto Continue;
                        }
                        if(isImplication)
                        {
                            Term subject = Term_ExtractSubterm(&term, 1);
                            int op_k = Narsese_getOperationID(&subject);
                            for(int j=0; j<c->precondition_beliefs[op_k].itemsAmount; j++)
                            {
                                Implication *imp = &c->precondition_beliefs[op_k].array[j];
                                if(!Variable_Unify(&term, &imp->term).success)
                                {
                                    continue;
                                }
                                if(Truth_Expectation(imp->truth) >= Truth_Expectation(best_truth))
                                {
                                    best_truth = imp->truth;
                                    best_term = imp->term;
                                    answerCreationTime = imp->creationTime;
                                }
                            }
                        }
                        else
                        if(isEvent)
                        {
                            if(c->belief_spike.type != EVENT_TYPE_DELETED)
                            {
                                Truth potential_best_truth = Truth_Projection(c->belief_spike.truth, c->belief_spike.occurrenceTime, currentTime);
                                if(Truth_Expectation(potential_best_truth) >= Truth_Expectation(best_truth_projected))
                                {
                                    best_truth_projected = potential_best_truth;
                                    best_truth = c->belief_spike.truth;
                                    best_term = c->belief_spike.term;
                                    answerOccurrenceTime = c->belief_spike.occurrenceTime;
                                    answerCreationTime = c->belief_spike.creationTime;
                                }
                            }
                            if(c->predicted_belief.type != EVENT_TYPE_DELETED)
                            {
                                Truth potential_best_truth = Truth_Projection(c->predicted_belief.truth, c->predicted_belief.occurrenceTime, currentTime);
                                if(Truth_Expectation(potential_best_truth) >= Truth_Expectation(best_truth_projected))
                                {
                                    best_truth_projected = potential_best_truth;
                                    best_truth = c->predicted_belief.truth;
                                    best_term = c->predicted_belief.term;
                                    answerOccurrenceTime = c->predicted_belief.occurrenceTime;
                                    answerCreationTime = c->predicted_belief.creationTime;
                                }
                            }
                        }
                        else
                        {
                            if(c->belief.type != EVENT_TYPE_DELETED && Truth_Expectation(c->belief.truth) >= Truth_Expectation(best_truth))
                            {
                                best_truth = c->belief.truth;
                                best_term = c->belief.term;
                                answerCreationTime = c->belief.creationTime;
                            }
                        }
                        Continue:;
                    }
                    fputs("Answer: ", stdout);
                    if(best_truth.confidence == 0)
                    {
                        puts("None.");
                    }
                    else
                    {
                        Narsese_PrintTerm(&best_term);
                        if(answerOccurrenceTime == OCCURRENCE_ETERNAL)
                        {
                            printf(". creationTime=%ld ", answerCreationTime);
                        }
                        else
                        {
                            printf(". :|: occurrenceTime=%ld creationTime=%ld ", answerOccurrenceTime, answerCreationTime);
                        }
                        Truth_Print(&best_truth);
                    }
                    fflush(stdout);
                }
                //input beliefs and goals
                else
                {
                    if(punctuation == '!')
                    {
                        YAN_AddInput(term, EVENT_TYPE_GOAL, YAN_DEFAULT_TRUTH, !isEvent);
                    }
                    else
                    {
                        YAN_AddInput(term, EVENT_TYPE_BELIEF, tv, !isEvent);
                    }
                }
            }
        }
        fflush(stdout);
    }
}
