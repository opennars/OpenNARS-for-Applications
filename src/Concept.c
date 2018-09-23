#include "Concept.h"

void Concept_RESET(Concept *concept, SDR name)
{
    concept->name = name;
    concept->event_beliefs_amount = 0;
    concept->precondition_beliefs_amount = 0;
    concept->postcondition_beliefs_amount = 0;
    //Generate CRC checksum too:
    CONCEPT_HASH_TYPE hash = 0; 
    ITERATE_SDR_BLOCKS(i,
		int pieces = SDR_BLOCK_SIZE / (sizeof(CONCEPT_HASH_TYPE));
		for(int j=0; j<pieces; j++)
		{
			int shift_right = j*8*sizeof(CONCEPT_HASH_TYPE); //each j shifts 8*NUM_BYTES_OF_CONCEPT_HASH_TYPE
			hash |= name.blocks[i] >> shift_right;
		}
    )
    concept->name_hash = hash;
}
