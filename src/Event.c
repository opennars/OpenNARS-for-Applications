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
    return (Event) { .sdr = sdr,
                     .sdr_hash = SDR_Hash(&sdr),
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
    printf("Event: %s\n", event->debug);
    SDR_Print(&event->sdr);
    //printf("SDR hash=%d", event->sdr_hash);
    printf(event->type == EVENT_TYPE_GOAL ? "type=goal\n" : (EVENT_TYPE_BELIEF ? "type=belief\n" : "type=deleted\n" ));
    printf("operationID=%d\n", event->operationID);
    Truth_Print(&event->truth);
    Stamp_print(&event->stamp);
    printf("occurrenceTime=%ld\n\n", event->occurrenceTime);
}

