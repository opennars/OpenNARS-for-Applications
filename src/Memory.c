#include "Memory.h"
#include "Concept.h"

void memory_RESET()
{
    memory.concepts_amount = 0;
}

CONCEPT_HASH_TYPE bitToConcept[SDR_SIZE][CONCEPTS_MAX];
void Memory_addConcept(Concept *concept)
{
    //try to add it, and if successful add to voting structure
    Concept evicted;
    PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&memory, &concept, sizeof(Concept), CONCEPTS_MAX, &evicted);
    if(feedback.added)
    {
        for(int j=0; j<SDR_SIZE; j++)
        {
            if(SDR_ReadBit(&(concept->name), j))
            {
                for(int i=0; i<CONCEPTS_MAX; i++)
                {
                    if(bitToConcept[j][i] == 0)
                    {
                        bitToConcept[j][i] = concept->name_hash;
                        break; //already inserted  
                    }
                }
             }
         }    
    }
    if(feedback.evicted)
    {
		//if a concept was evicted, delete from voting structure
		for(int j=0; j<SDR_SIZE; j++)
        {
            if(SDR_ReadBit(&(concept->name), j))
            {
                for(int i=0; i<CONCEPTS_MAX; i++)
                {
                    if(bitToConcept[j][i] == evicted.name_hash)
                    {
                        bitToConcept[j][i] = 0;
                        break; //already deleted  
                    }
                }
             }
         }
    }
}

Concept* Memory_getClosestConceptByName(SDR *name)
{
    
    /*Concept *closestConceptByName = NULL;
    double nearestConceptCloseness = -1;

    for (int i=0; i<memory->n_concepts; i++) {
        if( !memory->concepts[i]) {
            continue;
        }

        double closeness = SDR_Similarity(memory->concepts[i]->name, *name);
        if (closeness > nearestConceptCloseness) {
            nearestConceptCloseness = closeness;
            closestConceptByName = memory->concepts[i];
        }
    }*/

    return NULL; //closestConceptByName;
}
