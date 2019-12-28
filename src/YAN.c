#include "YAN.h"

long currentTime = 1;

void YAN_INIT()
{
    Memory_INIT(); //clear data structures
    Event_INIT(); //reset base id counter
    Encode_INIT();
    currentTime = 1; //reset time
}

void YAN_Cycles(int cycles)
{
    for(int i=0; i<cycles; i++)
    {
        IN_DEBUG( puts("\nNew system cycle:\n----------"); )
        Cycle_Perform(currentTime);
        currentTime++;
    }
}

Event YAN_AddInput(Term term, char type, Truth truth, int operationID)
{
    Event ev = Event_InputEvent(term, type, truth, currentTime);
    int closest_concept_i=0;
    if(Memory_FindConceptByTerm(&term, /*Term_Hash(&term),*/ &closest_concept_i))
    {
        Concept *c = concepts.items[closest_concept_i].address;
        if(strlen(c->debug) == 0)
        {
            char debug[20];
            //assign index as name to event and concept since concept has no name yet
            sprintf(debug, "%d", c->id);
            strcpy(ev.debug, debug);
            strcpy(c->debug, debug);
        }
        else
        {
            //name event according to concept
            strcpy(ev.debug, c->debug);
        }
        char* st = type == EVENT_TYPE_BELIEF ? "." : "!";
        printf("Input: %s%s :|: %%%f;%f%%\n", c->debug, st, truth.frequency, truth.confidence);
    }
    ev.operationID = operationID;
    Memory_addEvent(&ev, 0, true);
    IN_OUTPUT( fputs("INPUT ", stdout); Event_Print(&ev); )
    YAN_Cycles(1);
    return ev;
}

Event YAN_AddInputBelief(Term term, int operationID)
{
    Event ret = YAN_AddInput(term, EVENT_TYPE_BELIEF, YAN_DEFAULT_TRUTH, operationID);
    return ret;
}

Event YAN_AddInputGoal(Term term)
{
    return YAN_AddInput(term, EVENT_TYPE_GOAL, YAN_DEFAULT_TRUTH, 0);
}

void YAN_AddOperation(Term term, Action procedure)
{
    Memory_addOperation((Operation) {.term = term, .action = procedure});
}

