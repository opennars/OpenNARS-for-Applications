#include "Event.h"

void Event_SetSDR(Event *event, SDR sdr)
{
    event->sdr = sdr;
    //Generate hash too:
    event->sdr_hash = SDR_Hash(&sdr);
}
