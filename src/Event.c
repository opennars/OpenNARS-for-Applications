#include "Event.h"

void Event_SetSDR(Event *event, SDR sdr)
{
    event->sdr = sdr;
    //Generate hash too:
    event->sdr_hash = SDR_Hash(&sdr);
}

long base = 1;
Event Event_InputEvent(SDR sdr, char type, Truth truth, long currentTime)
{
    return (Event) { .attention = Attention_inputEvent(&truth, currentTime),
                     .sdr = sdr, 
                     .type = type, 
                     .truth = truth, 
                     .stamp = (Stamp) { .evidentalBase = { base++ } }, 
                     .occurrenceTime = currentTime };
}

void Event_RESET()
{
    base = 1;
}
