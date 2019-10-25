#include "ANSNA.h"

long currentTime = 1;

void ANSNA_INIT()
{
    SDR_INIT();
    Memory_INIT(); //clear data structures
    Event_INIT(); //reset base id counter
    currentTime = 1; //reset time
}

void ANSNA_Cycles(int cycles)
{
    for(int i=0; i<cycles; i++)
    {
        IN_DEBUG( puts("\nNew system cycle:\n----------"); )
        Cycle_Perform(currentTime);
        currentTime++;
    }
}

Event ANSNA_AddInput(SDR sdr, char type, Truth truth)
{
    Event ev = Event_InputEvent(sdr, type, truth, currentTime);
    int closest_concept_i=0;
    if(Memory_getClosestConcept(&sdr, SDR_Hash(&sdr), &closest_concept_i))
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
    for(int i=0; i<OPERATIONS_MAX; i++)
    {
        if(operations[i].action == 0)
        {
            break;
        }
        if(SDR_Equal(&operations[i].sdr, &sdr)) //TODO relax to allow parametrized actions
        {
            ev.operationID = i+1;
            break;
        }
    }
    Memory_addEvent(&ev);
    IN_OUTPUT( fputs("INPUT ", stdout); Event_Print(&ev); )
    ANSNA_Cycles(1);
    return ev;
}

Event ANSNA_AddInputBelief(SDR sdr)
{
    Event ret = ANSNA_AddInput(sdr, EVENT_TYPE_BELIEF, ANSNA_DEFAULT_TRUTH);
    return ret;
}

Event ANSNA_AddInputGoal(SDR sdr)
{
    return ANSNA_AddInput(sdr, EVENT_TYPE_GOAL, ANSNA_DEFAULT_TRUTH);
}

void ANSNA_AddOperation(SDR sdr, Action procedure)
{
    Memory_addOperation((Operation) {.sdr = sdr, .action = procedure});
}

