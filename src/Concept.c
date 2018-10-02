#include "Concept.h"

void Concept_RESET(Concept *concept, SDR name)
{
    concept->name = name;
    FIFO_RESET(&concept->event_beliefs);
    FIFO_RESET(&concept->event_goals);
    concept->precondition_beliefs_amount = 0;
    concept->postcondition_beliefs_amount = 0;
    //Generate CRC checksum too:
    concept->name_hash = SDR_Hash(&name);
}
