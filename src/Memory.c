#include "Memory.h"

Concept concept_storage[CONCEPTS_MAX];
Item concept_items_storage[CONCEPTS_MAX];
int operations_index = 0;
long concept_id = 1;

static void Memory_ResetEvents()
{
    FIFO_RESET(&belief_events);
    FIFO_RESET(&goal_events);
}

static void Memory_ResetConcepts()
{
    PriorityQueue_RESET(&concepts, concept_items_storage, CONCEPTS_MAX);
    for(int i=0; i<CONCEPTS_MAX; i++)
    {
        concept_storage[i] = (Concept) {0};
        concepts.items[i] = (Item) { .address = &(concept_storage[i]) };
    }   
}

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

Concept* Memory_Conceptualize(SDR *sdr)
{
    Concept *addedConcept = NULL;
    //try to add it, and if successful add to voting structure
    PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&concepts, 0.0);
    if(feedback.added)
    {
        addedConcept = feedback.addedItem.address;
        *addedConcept = (Concept) {0};
        Concept_SetSDR(addedConcept, *sdr);
        addedConcept->id = concept_id++;
    }
    return addedConcept;
}

bool Memory_getClosestConcept(SDR *sdr, SDR_HASH_TYPE sdr_hash, int *returnIndex)
{
    if(concepts.itemsAmount == 0)
    {
        return false;   
    }
    int foundSameConcept_i;
    if(Memory_FindConceptBySDR(sdr, sdr_hash, &foundSameConcept_i))
    {
        *returnIndex = foundSameConcept_i;
        return true;
    }
    int best_i = -1;
    double bestValSoFar = -1;
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        double curVal = Truth_Expectation(SDR_Inheritance(sdr, &(((Concept*)concepts.items[i].address)->sdr)));
        if(curVal > bestValSoFar)
        {
            bestValSoFar = curVal;
            best_i = i;
        }
    }
    if(best_i == -1) //TODO how?
    {
        return false;
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
    if(event->type == EVENT_TYPE_BELIEF)
    {
        FIFO_Add(event, &belief_events); //not revised yet
        return true;
    }
    if(event->type == EVENT_TYPE_GOAL)
    {
        FIFO_Add(event, &goal_events);
        return true;
    }
    assert(false, "errornous event type");
    return true;
}

bool Memory_addConcept(Concept *concept, long currentTime)
{
    PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&concepts, Usage_usefulness(&concept->usage, currentTime));
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
