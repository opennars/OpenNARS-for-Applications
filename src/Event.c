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

void Event_INIT()
{
    base = 1;
}

void Event_Print(Event *event)
{
    printf("Event:\n");
    Attention_Print(&event->attention);
    SDR_PrintWhereTrue(&event->sdr);
    //printf("SDR hash=%d", event->sdr_hash);
    printf(event->type == EVENT_TYPE_GOAL ? "type=goal\n" : (EVENT_TYPE_BELIEF ? "type=belief\n" : "type=deleted\n" ));
    Truth_Print(&event->truth);
    Stamp_print(&event->stamp);
    printf("occurrenceTime=%d\n\n", event->occurrenceTime);
}
