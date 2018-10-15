#include "ANSNA.h"


long currentTime = 0;
void ANSNA_INIT()
{
    SDR_INIT();
    Memory_INIT(); //clear data structures
    Event_INIT(); //reset base id counter
    currentTime = 0; //reset time
}

void ANSNA_Cycles(int cycles)
{
    for(int i=0; i<cycles; i++)
    {
        cycle(currentTime);
        currentTime++;
    }
}

void ANSNA_AddInput(SDR sdr, char type, Truth truth)
{
    Event ev = Event_InputEvent(sdr, type, truth, currentTime);
    Memory_addEvent(&ev);
}

void ANSNA_AddOperation(SDR *sdr, Action procedure)
{
    Memory_addOperation((Operation) {.sdr = *sdr, .action = procedure});
}
