#include "Memory.h"

Concept concept_storage[CONCEPT_LAYERS][CONCEPTS_MAX]; //TODO optimize
Item concept_items_storage[CONCEPT_LAYERS][CONCEPTS_MAX]; //in space consumption
int operations_index = 0;

static void Memory_ResetEvents()
{
    FIFO_RESET(&belief_events);
    FIFO_RESET(&goal_events);
}

static void Memory_ResetConcepts()
{
    int curCAmount = CONCEPTS_MAX;
    for(int l=0; l<CONCEPT_LAYERS; l++)
    {
        PriorityQueue_RESET(&concepts[l], concept_items_storage[l], curCAmount);
        for(int i=0; i<CONCEPTS_MAX; i++)
        {
            concept_storage[l][i] = (Concept) {0};
            concepts[l].items[i] = (Item) { .address = &(concept_storage[l][i]) };
        }
        curCAmount /= 2;
    }   
}

int concept_id = 0;
void Memory_INIT()
{
    Memory_ResetConcepts();
    Memory_ResetEvents();
    for(int i=0; i<OPERATIONS_MAX; i++)
    {
        operations[i] = (Operation) {0};
    }
    operations_index = 0;
    concept_id = 0;
}

bool Memory_FindConceptBySDR(int layer, SDR *sdr, SDR_HASH_TYPE sdr_hash, int *returnIndex)
{
    for(int i=0; i<concepts[layer].itemsAmount; i++)
    {
        Concept *existing = concepts[layer].items[i].address;
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

void Memory_Conceptualize(SDR *sdr)
{
    SDR_HASH_TYPE hash = SDR_Hash(sdr);
    for(int l=0; l<CONCEPT_LAYERS; l++)
    {
        if(!Memory_FindConceptBySDR(l, sdr, hash, NULL))
        {
            Concept *addedConcept = NULL;
            //try to add it, and if successful add to voting structure
            PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&concepts[l], 0.0);
            if(feedback.added)
            {
                addedConcept = feedback.addedItem.address;
                *addedConcept = (Concept) {0};
                Concept_SetSDR(addedConcept, *sdr);
                addedConcept->id = concept_id;
                concept_id++;
            }
        }
    }
}

bool Memory_getClosestConcept(int layer, SDR *sdr, SDR_HASH_TYPE sdr_hash, int *returnIndex)
{
    if(concepts[layer].itemsAmount == 0)
    {
        return false;   
    }
    int foundSameConcept_i;
    if(Memory_FindConceptBySDR(layer, sdr, sdr_hash, &foundSameConcept_i))
    {
        *returnIndex = foundSameConcept_i;
        return true;
    }
    int best_i = -1;
    double bestValSoFar = -1;
    for(int i=0; i<concepts[layer].itemsAmount; i++)
    {
        double curVal = Truth_Expectation(SDR_Inheritance(sdr, &(((Concept*)concepts[layer].items[i].address)->sdr)));
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

void Memory_addConcept(int layer, Concept *concept, long currentTime)
{
    PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&concepts[layer], Usage_usefulness(&concept->usage, currentTime));
    if(feedback.added)
    {
        Concept *toRecyle = feedback.addedItem.address;
        *toRecyle = *concept;
    }
}

void Memory_addOperation(Operation op)
{
    operations[operations_index%OPERATIONS_MAX] = op;
    operations_index++;
}

Event Memory_MatchEventToConcept(Concept *c, Event *e)
{
    Event eMatch = *e;
    eMatch.sdr = c->sdr;
    eMatch.truth = Truth_Deduction(SDR_Inheritance(&e->sdr, &c->sdr), e->truth);
    return eMatch;
}
