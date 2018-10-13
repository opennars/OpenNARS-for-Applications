#include "Memory.h"

Concept concept_storage[CONCEPTS_MAX];
Event event_storage[EVENTS_MAX];
Item concept_items_storage[CONCEPTS_MAX];
Item event_items_storage[EVENTS_MAX];

void memory_RESET()
{
    PriorityQueue_RESET(&concepts, concept_items_storage, CONCEPTS_MAX);
    PriorityQueue_RESET(&events, event_items_storage, EVENTS_MAX);
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
Concept* bitToConcept[SDR_SIZE][CONCEPTS_MAX];
int bitToConceptAmount[SDR_SIZE];
#endif

Concept* Memory_addConcept(SDR *sdr, Attention attention)
{
    Concept *addedConcept = NULL;
    //try to add it, and if successful add to voting structure
    PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&concepts, attention.priority);
    if(feedback.added)
    {
        addedConcept = (Concept*) feedback.addedItem.address;
        *addedConcept = (Concept) {0};
        Concept_SetSDR(addedConcept, *sdr);
        addedConcept->attention = attention;
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
int Memory_getClosestConcept(Event *event)
{
    SDR *eventSDR = &(event->sdr);
#if USE_HASHING == true
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        Concept *c = ((Concept*)concepts.items[i].address);
        if(c->sdr_hash == event->sdr_hash)
        {
            if(SDR_Equal(&c->sdr, &event->sdr))
            {
                return i;
            }
        }
    }
#endif
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
        return MEMORY_MATCH_NO_CONCEPT;
    }
    //TODO IMPROVE:
    int best_i = 0;
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        if(concepts.items[i].address == best.concept)
        {
            return best_i;
        }
    }
    return MEMORY_MATCH_NO_CONCEPT;
#endif
#if MATCH_STRATEGY == EXHAUSTIVE
    int best_i = MEMORY_MATCH_NO_CONCEPT;
    double bestValSoFar = -1;
    for(int i=0; i<concepts.items_amount; i++)
    {
        double curVal = SDR_Inheritance(eventSDR, &(concepts.items[i].sdr));
        if(curVal > bestValSoFar)
        {
            bestValSoFar = curVal;
            best_i = i;
        }
    }
    return best_i;
#endif
}

void Memory_addEvent(Event *event)
{
    PriorityQueue_Push_Feedback pushed = PriorityQueue_Push(&events, event->attention.priority);
    if(pushed.added)
    {
        Event *toRecyle = pushed.addedItem.address;
        *toRecyle = *event;
    }
}
