#include "Event.h"

void Event_SetTerm(Event *event, Term sdr)
{
    event->sdr = sdr;
    //Generate hash too:
    //event->sdr_hash = Term_Hash(&sdr);
}

long base = 1;
Event Event_InputEvent(Term sdr, char type, Truth truth, long currentTime)
{
    return (Event) { .sdr = sdr,
                     /*.sdr_hash = Term_Hash(&sdr),*/
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
    Term_Print(&event->sdr);
    //printf("Term hash=%d", event->sdr_hash);
    printf(event->type == EVENT_TYPE_GOAL ? "type=goal\n" : (EVENT_TYPE_BELIEF ? "type=belief\n" : "type=deleted\n" ));
    printf("operationID=%d\n", event->operationID);
    Truth_Print(&event->truth);
    Stamp_print(&event->stamp);
    printf("occurrenceTime=%ld\n\n", event->occurrenceTime);
}

