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
