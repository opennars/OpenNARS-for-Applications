#include "Event.h"

void Event_SetSDR(Event *event, SDR sdr)
{
    event->sdr = sdr;
    //Generate hash too:
    event->sdr_hash = SDR_Hash(&sdr);
}

long base = 1;
Event Event_InputEvent(SDR sdr, char type, Truth truth, long occurrenceTime)
{
    return (Event) { .attention = Attention_inputEvent(&truth),
                     .sdr = sdr, 
                     .type = type, 
                     .truth = truth, 
                     .stamp = (Stamp) { .evidentalBase = { base++ } }, 
                     .occurrenceTime = occurrenceTime };
}

void Event_RESET()
{
    base = 1;
}
