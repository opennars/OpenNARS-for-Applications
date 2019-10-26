#include "Memory.h"

double PROPAGATION_THRESHOLD = PROPAGATION_THRESHOLD_INITIAL;

Concept concept_storage[CONCEPTS_MAX];
Item concept_items_storage[CONCEPTS_MAX];
int operations_index = 0;

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

bool Memory_FindConceptByTerm(Term *term, int *returnIndex)
{
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        Concept *existing = concepts.items[i].address;
        //if(!USE_HASHING || existing->term_hash == term_hash)
        {
            if(Term_Equal(&existing->term, term))
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

void Memory_Conceptualize(Term *term)
{
    //Term_HASH_TYPE hash = Term_Hash(term);
    if(!Memory_FindConceptByTerm(term, /*hash,*/ NULL))
    {
        Concept *addedConcept = NULL;
        //try to add it, and if successful add to voting structure
        PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&concepts, 0.0);
        if(feedback.added)
        {
            addedConcept = feedback.addedItem.address;
            *addedConcept = (Concept) {0};
            Concept_SetTerm(addedConcept, *term);
            addedConcept->id = concept_id;
            concept_id++;
        }
    }
}

bool Memory_addEvent(Event *event)
{
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

void Memory_addConcept(Concept *concept, long currentTime)
{
    PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&concepts, Usage_usefulness(concept->usage, currentTime));
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

bool Memory_ImplicationValid(Implication *imp)
{
    return Term_Equal(&imp->sourceConceptTerm, &((Concept*) imp->sourceConcept)->term);
}
