#include "YAN.h"

long currentTime = 1;

void YAN_INIT()
{
    Memory_INIT(); //clear data structures
    Event_INIT(); //reset base id counter
    Narsese_INIT();
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

Event YAN_AddInput(Term term, char type, Truth truth, bool eternal)
{
    Event ev = Event_InputEvent(term, type, truth, currentTime);
    if(eternal)
    {
        ev.occurrenceTime = OCCURRENCE_ETERNAL;
    }
    Memory_addInputEvent(&ev, 0);
    YAN_Cycles(1);
    return ev;
}

Event YAN_AddInputBelief(Term term)
{
    Event ret = YAN_AddInput(term, EVENT_TYPE_BELIEF, YAN_DEFAULT_TRUTH, false);
    return ret;
}

Event YAN_AddInputGoal(Term term)
{
    return YAN_AddInput(term, EVENT_TYPE_GOAL, YAN_DEFAULT_TRUTH, false);
}

void YAN_AddOperation(Term term, Action procedure)
{
    char* term_name = Narsese_atomNames[(int) term.atoms[0]-1];
    assert(term_name[0] == '^', "This atom does not belong to an operator!");
    Memory_addOperation(Narsese_OperatorIndex(term_name), (Operation) {.term = term, .action = procedure});
}

