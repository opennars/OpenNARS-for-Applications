#include "Memory.h"

double PROPAGATION_THRESHOLD = PROPAGATION_THRESHOLD_INITIAL;
bool PRINT_DERIVATIONS = PRINT_DERIVATIONS_INITIAL;
Concept concept_storage[CONCEPTS_MAX];
Item concept_items_storage[CONCEPTS_MAX];
Event cycling_event_storage[CYCLING_EVENTS_MAX];
Item cycling_event_items_storage[CYCLING_EVENTS_MAX];
int operations_index = 0;

static void Memory_ResetEvents()
{
    FIFO_RESET(&belief_events);
    FIFO_RESET(&goal_events);
    PriorityQueue_RESET(&cycling_events, cycling_event_items_storage, CYCLING_EVENTS_MAX);
    for(int i=0; i<CYCLING_EVENTS_MAX; i++)
    {
        cycling_event_storage[i] = (Event) {0};
        cycling_events.items[i] = (Item) { .address = &(cycling_event_storage[i]) };
    }
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

//Add event for cycling through the system (inference and context)
//called by addEvent for eternal knowledge
static void Memory_addCyclingEvent(Event *event, double priority)
{
    PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&cycling_events, priority);
    if(feedback.added)
    {
        Event *toRecyle = feedback.addedItem.address;
        *toRecyle = *event;
    }
}

static void Memory_printAddedEvent(Event *event, double priority, bool input, bool derived, bool revised)
{
    if(PRINT_DERIVATIONS && priority > PRINT_DERIVATIONS_PRIORITY_THRESHOLD && (input || derived))
    {
        fputs(revised ? "Revised: " : (input ? "Input: " : "Derived: "), stdout);
        Encode_PrintTerm(&event->term);
        fputs((event->type == EVENT_TYPE_BELIEF ? ". " : "! "), stdout);
        fputs(event->occurrenceTime == OCCURRENCE_ETERNAL ? "" : ":|: ", stdout);
        printf("Priority=%f ", priority);
        Truth_Print(&event->truth);
    }
}

void Memory_addEvent2(Event *event, long currentTime, double priority, bool input, bool derived, bool readded, bool revised)
{
    Memory_printAddedEvent(event, priority, input, derived, revised);
    if(event->occurrenceTime != OCCURRENCE_ETERNAL)
    {
        if(input)
        {
            //process event
            if(event->type == EVENT_TYPE_BELIEF)
            {
                FIFO_Add(event, &belief_events); //not revised yet
            }
            else
            if(event->type == EVENT_TYPE_GOAL)
            {
                FIFO_Add(event, &goal_events);
            }
        }
    }
    else
    {
        if(event->type == EVENT_TYPE_BELIEF)
        {
            bool revision_happened = false;
            if(!readded)
            {
                //process eternal knowledge
                //eternal ones get conceptualized and event added directly
                Memory_Conceptualize(&event->term);
                int concept_i;
                if(Memory_FindConceptByTerm(&event->term, &concept_i))
                {
                    Concept *c = concepts.items[concept_i].address;
                    c->belief = Inference_IncreasedActionPotential(&c->belief, event, currentTime, &revision_happened);
                    if(revision_happened)
                    {
                        revision_happened = true;
                        Memory_addEvent2(&c->belief, currentTime, priority, input, derived, false, true);
                    }
                }
                else
                {
                    assert(false, "Concept creation failed, it should always be able to create one, even when full, by removing the worst!");
                }
            }
            if(!revision_happened)
            {
                Memory_addCyclingEvent(event, priority); //task gets replaced with revised one, more radical than OpenNARS!!
            }
        }
        else
        {
            assert(false, "Eternal goals are not supported");
        }
    }
    assert(event->type == EVENT_TYPE_BELIEF || event->type == EVENT_TYPE_GOAL, "Errornous event type");
}

void Memory_addEvent(Event *event, long currentTime, bool input, bool derived)
{
    Memory_addEvent2(event, currentTime, Truth_Expectation(event->truth), input, derived, false, false);
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
