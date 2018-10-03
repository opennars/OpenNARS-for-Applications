#include "Memory.h"
#include "Concept.h"

Concept concept_storage[CONCEPTS_MAX];
Event event_storage[EVENTS_MAX];
Item concept_items_storage[CONCEPTS_MAX];
Item event_items_storage[EVENTS_MAX];

void memory_RESET()
{
    concepts.items = concept_items_storage;
    events.items = event_items_storage;
    for(int i=0; i<CONCEPTS_MAX; i++)
    {
        concept_storage[i] = (Concept) {0};
        concepts.items[i] = (Item) { .address = &(concept_storage[i]) };
    }
    for(int i=0; i<EVENTS_MAX; i++)
    {
        event_storage[i] = (Event) {0};
        events.items[i] = (Item) { .address = &(event_storage[i]) };
    }
}

#if MATCH_STRATEGY == VOTING
SDR_HASH_TYPE bitToConcept[SDR_SIZE][CONCEPTS_MAX];
int bitToConceptAmount[SDR_SIZE];
#endif

void Memory_addConcept(Concept *concept)
{
    //try to add it, and if successful add to voting structure
    PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&concepts, concept->attention.priority, CONCEPTS_MAX);
#if MATCH_STRATEGY == VOTING
    if(feedback.added)
    {
        for(int j=0; j<SDR_SIZE; j++)
        {
            if(SDR_ReadBit(&(concept->name), j))
            {
                int i = bitToConceptAmount[j]; //insert on top
                bitToConcept[j][i] = concept->name_hash;
                bitToConceptAmount[j]++;
             }
         }
         Concept *toRecycle = (Concept*) feedback.addedItem.address;
         *toRecycle = (Concept) {0};
         Concept_SetName(toRecycle, concept->name);
    }
    if(feedback.evicted)
    {
        //if a concept was evicted, delete from voting structure
        for(int j=0; j<SDR_SIZE; j++)
        {
            if(SDR_ReadBit(&(concept->name), j))
            {
                for(int i=0; i<bitToConceptAmount[j]; i++)
                {
                    if(bitToConcept[j][i] == ((Concept*)feedback.evictedItem.address)->name_hash)
                    {
                        bitToConcept[j][i] = 0;
                        //Now move the above ones down to remove the gap
                        for(int k=i; k<bitToConceptAmount[j]-1; k++)
                        {
                            bitToConcept[j][k] = bitToConcept[j][k+1];
                        }
                        //and decrement the counter
                        bitToConceptAmount[j]--;
                        break; //already deleted  
                    }
                }
             }
         }
    }
#endif
}

typedef struct
{
    SDR_HASH_TYPE concept;
    int count;
} Vote;
Concept* Memory_getClosestConceptByName(SDR *taskSDR)
{
#if MATCH_STRATEGY == VOTING
    SDR_HASH_TYPE taskhash = SDR_Hash(taskSDR);
    Vote voting[CONCEPTS_MAX] = {0};
    int votes = 0;
    Vote best = {0};
    for(int j=0; j<SDR_SIZE; j++)
    {
        if(SDR_ReadBit(taskSDR, j))
        {
            for(int i=0; i<bitToConceptAmount[j]; i++)
            {
                int use_index = votes;
                bool existed = false;
                //check first if the SDR already got a vote
                //and if yes, increment that one instead creating a new one
                SDR_HASH_TYPE voted_concept = bitToConcept[j][i];
                for(int h=0; h<votes; h++)
                {
                    if(voting[h].concept == voted_concept)
                    {
                        use_index = h;
                        existed = true;
                        break;
                    }
                }
                voting[use_index].concept = voted_concept;
                voting[use_index].count = existed ? voting[use_index].count+1 : 1;
                if(voting[use_index].count > best.count)
                {
                    best = voting[use_index];
                }
                if(!existed)
                {
                    votes++;
                }
            }
        }
    }
    if(votes == 0)
    {
        return NULL;
    }
    //And now retrieve a concept with the same hash:
    //TODO, NO, REFERENCES CAN BE USED NOW!!
    for(int i=0; i<concepts.items_amount; i++)
    {
        if(((Concept*)concepts.items[i].address)->name_hash == best.concept)
        {
            return &(concepts.items[i]);
        }
    }
    return NULL;
#endif
#if MATCH_STRATEGY == EXHAUSTIVE
    Concept *best = NULL;
    double bestValSoFar = -1;
    for(int i=0; i<concepts.items_amount; i++)
    {
        double curVal = SDR_Inheritance(taskSDR, &(concepts.items[i].name));
        if(curVal > bestValSoFar)
        {
            bestValSoFar = curVal;
            best = (Concept*) (memory.items[i]->address);
        }
    }
    return best;
#endif
}
