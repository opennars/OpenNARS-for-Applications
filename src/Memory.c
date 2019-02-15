#include "Memory.h"

Concept concept_storage[CONCEPTS_MAX];
Event event_storage[EVENTS_MAX];
Item concept_items_storage[CONCEPTS_MAX];
Item event_items_storage[EVENTS_MAX];

int operations_index = 0;

void Memory_ResetEvents()
{
    PriorityQueue_RESET(&events, event_items_storage, EVENTS_MAX);
    for(int i=0; i<EVENTS_MAX; i++)
    {
        event_storage[i] = (Event) {0};
        events.items[i] = (Item) { .address = &(event_storage[i]) };
    }
}

void Memory_ResetConcepts()
{
    PriorityQueue_RESET(&concepts, concept_items_storage, CONCEPTS_MAX);
    for(int i=0; i<CONCEPTS_MAX; i++)
    {
        concept_storage[i] = (Concept) {0};
        concepts.items[i] = (Item) { .address = &(concept_storage[i]) };
    }   
}

long concept_id = 1;
void Memory_INIT()
{
    Memory_ResetConcepts();
    Memory_ResetEvents();
    for(int i=0; i<OPERATIONS_MAX; i++)
    {
        operations[i] = (Operation) {0};
    }
    operations_index = 0;
    concept_id = 1;
}

#if MATCH_STRATEGY == VOTING
Concept* bitToConcept[SDR_SIZE][CONCEPTS_MAX];
int bitToConceptAmount[SDR_SIZE];
#endif

bool Memory_FindConceptBySDR(SDR *sdr, SDR_HASH_TYPE sdr_hash, int *returnIndex)
{
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        Concept *existing = concepts.items[i].address;
        if(!USE_HASHING || existing->sdr_hash == sdr_hash)
        {
            if(SDR_Equal(&existing->sdr, sdr))
            {
                if(returnIndex != NULL)
                {
                    *returnIndex = i;
                }
                return true;
            }
        }
    }
    return false;
}

Concept* Memory_Conceptualize(SDR *sdr, Attention attention)
{
    Concept *addedConcept = NULL;
    //try to add it, and if successful add to voting structure
    PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&concepts, attention.priority);
    if(feedback.added)
    {
        addedConcept = feedback.addedItem.address;
        *addedConcept = (Concept) {0};
        Concept_SetSDR(addedConcept, *sdr);
        addedConcept->attention = attention;
        addedConcept->id = concept_id++;
    }
#if MATCH_STRATEGY == VOTING
    if(feedback.added)
    {
        for(int j=0; j<SDR_SIZE; j++)
        {
            if(SDR_ReadBit(&(addedConcept->sdr), j))
            {
                int i = bitToConceptAmount[j]; //insert on top
                bitToConcept[j][i] = addedConcept;
                bitToConceptAmount[j]++;
             }
         }
    }
    if(feedback.evicted)
    {
        Concept *concept = feedback.evictedItem.address;
        //if a concept was evicted, delete from voting structure
        for(int j=0; j<SDR_SIZE; j++)
        {
            if(SDR_ReadBit(&(concept->sdr), j))
            {
                for(int i=0; i<bitToConceptAmount[j]; i++)
                {
                    if(bitToConcept[j][i] == (Concept*)feedback.evictedItem.address)
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
    return addedConcept;
}

typedef struct
{
    Concept *concept;
    int count;
} Vote;
bool Memory_getClosestConcept(Event *event, int *returnIndex)
{
    SDR *eventSDR = &(event->sdr);
    if(concepts.itemsAmount == 0)
    {
        return false;   
    }
    int foundSameConcept_i;
    if(Memory_FindConceptBySDR(&event->sdr, event->sdr_hash, &foundSameConcept_i))
    {
        *returnIndex = foundSameConcept_i;
    }
#if MATCH_STRATEGY == VOTING
    Vote voting[CONCEPTS_MAX] = {0};
    int votes = 0;
    Vote best = {0};
    for(int j=0; j<SDR_SIZE; j++)
    {
        if(SDR_ReadBit(eventSDR, j))
        {
            for(int i=0; i<bitToConceptAmount[j]; i++)
            {
                int use_index = votes;
                bool existed = false;
                //check first if the SDR already got a vote
                //and if yes, increment that one instead creating a new one
                Concept *voted_concept = bitToConcept[j][i];
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
        return false;
    }
    //TODO IMPROVE:
    int best_i = 0;
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        if(concepts.items[i].address == best.concept)
        {
            *returnIndex = best_i;
            return true;
        }
    }
    return false;
#endif
#if MATCH_STRATEGY == EXHAUSTIVE
    int best_i = -1;
    double bestValSoFar = -1;
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        double curVal = Truth_Expectation(SDR_Inheritance(eventSDR, &(((Concept*)concepts.items[i].address)->sdr)));
        if(curVal > bestValSoFar)
        {
            bestValSoFar = curVal;
            best_i = i;
        }
    }
    *returnIndex = best_i;
    return true;
#endif
}

bool Memory_addEvent(Event *event)
{
    if(event_inspector != NULL)
    {
        (*event_inspector)(event);
    }
    PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&events, event->attention.priority);
    if(feedback.added)
    {
        Event *toRecyle = feedback.addedItem.address;
        *toRecyle = *event;
        return true;
    }
    return false;
}

bool Memory_addConcept(Concept *concept)
{
    PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&concepts, concept->attention.priority);
    if(feedback.added)
    {
        Concept *toRecyle = feedback.addedItem.address;
        *toRecyle = *concept;
        return true;
    }
    return false;
}

void Memory_addOperation(Operation op)
{
    operations[operations_index%OPERATIONS_MAX] = op;
    operations_index++;
}
