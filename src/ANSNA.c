#include "ANSNA.h"

long currentTime = 1;
void ANSNA_INIT()
{
    SDR_INIT();
    Memory_INIT(); //clear data structures
    Event_INIT(); //reset base id counter
    currentTime = 1; //reset time
    event_inspector = NULL;
}

void ANSNA_Cycles(int cycles)
{
    for(int i=0; i<cycles; i++)
    {
        IN_DEBUG( puts("\nNew inference cycle:\n----------"); )
        Cycle_Perform(currentTime);
        currentTime++;
    }
}

Event ANSNA_AddInput(SDR sdr, char type, Truth truth, char *debug)
{
    Event ev = Event_InputEvent(sdr, type, truth, currentTime);
    ev.debug = debug;
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

Event ANSNA_AddInputBelief(SDR sdr, char *debug)
{
    Event ret = ANSNA_AddInput(sdr, EVENT_TYPE_BELIEF, ANSNA_DEFAULT_TRUTH, debug);
    return ret;
}

Event ANSNA_AddInputGoal(SDR sdr, char *debug)
{
    return ANSNA_AddInput(sdr, EVENT_TYPE_GOAL, ANSNA_DEFAULT_TRUTH, debug);
}

void ANSNA_AddOperation(SDR sdr, Action procedure)
{
    Memory_addOperation((Operation) {.sdr = sdr, .action = procedure});
}

void ANSNA_Util_PrintExistingEventNarsese(Event e)
{
    int closest_concept_i=0;
    if(Memory_getClosestConcept(&e.sdr, e.sdr_hash, &closest_concept_i))
    {
        Concept *c = concepts.items[closest_concept_i].address;
        char* st = e.type == EVENT_TYPE_BELIEF ? "." : "!";
        printf("Input: %ld%s :|: %%%f;%f%%\n", c->id, st, e.truth.frequency, e.truth.confidence);
    }
}
