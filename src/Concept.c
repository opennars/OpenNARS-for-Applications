#include "Concept.h"

void Concept_RESET(Concept *concept, SDR name)
{
    concept->name = name;
    concept->event_beliefs_amount = 0;
    concept->precondition_beliefs_amount = 0;
    concept->postcondition_beliefs_amount = 0;
    //Generate CRC checksum too:
    uint64_t crc = 0; 
    ITERATE_SDR_BLOCKS(i,
        crc = crc64(crc, &(name.blocks[i]), sizeof(SDR_BLOCK_TYPE));
    )
    concept->name_hash = crc;
}
