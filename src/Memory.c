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

Event selectedEvents[EVENT_SELECTIONS]; //better to be global
double selectedEventsPriority[EVENT_SELECTIONS]; //better to be global
int eventsSelected = 0;

static bool Memory_containsEvent(Event *event)
{
    for(int i=0; i<cycling_events.itemsAmount; i++)
    {
        if(Event_Equal(event, cycling_events.items[i].address))
        {
            return true;
        }
    }
    for(int i=0; i<eventsSelected; i++)
    {
        if(Event_Equal(event, &selectedEvents[i]))
        {
            return true;
        }
    }
    return false;
}

//Add event for cycling through the system (inference and context)
//called by addEvent for eternal knowledge
static bool Memory_addCyclingEvent(Event *e, double priority, long currentTime)
{
    if(Memory_containsEvent(e))
    {
        return false;
    }
    int concept_i = 0;
    if(Memory_FindConceptByTerm(&e->term, &concept_i))
    {
        Concept *c = concepts.items[concept_i].address;
        if(e->type == EVENT_TYPE_BELIEF && e->occurrenceTime == OCCURRENCE_ETERNAL && c->belief.type != EVENT_TYPE_DELETED && ((e->occurrenceTime == OCCURRENCE_ETERNAL && c->belief.truth.confidence > e->truth.confidence) || (e->occurrenceTime != OCCURRENCE_ETERNAL && Truth_Projection(c->belief_spike.truth, c->belief_spike.occurrenceTime, currentTime).confidence > Truth_Projection(e->truth, e->occurrenceTime, currentTime).confidence)))
        {
            return false; //the belief has a higher confidence and was already revised up (or a cyclic transformation happened!), get rid of the event!
        }   //more radical than OpenNARS!
    }
    PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&cycling_events, priority);
    if(feedback.added)
    {
        Event *toRecyle = feedback.addedItem.address;
        *toRecyle = *e;
        return true;
    }
    return false;
}

static void Memory_printAddedEvent(Event *event, double priority, bool input, bool derived, bool revised)
{
    if(PRINT_DERIVATIONS && priority > PRINT_DERIVATIONS_PRIORITY_THRESHOLD && (input || derived || revised))
    {
        fputs(revised ? "Revised: " : (input ? "Input: " : "Derived: "), stdout);
        Encode_PrintTerm(&event->term);
        fputs((event->type == EVENT_TYPE_BELIEF ? ". " : "! "), stdout);
        fputs(event->occurrenceTime == OCCURRENCE_ETERNAL ? "" : ":|: ", stdout);
        printf("Priority=%f ", priority);
        Truth_Print(&event->truth);
    }
}

void Memory_addEvent(Event *event, long currentTime, double priority, bool input, bool derived, bool readded, bool revised)
{
    if(event->truth.confidence < MIN_CONFIDENCE || priority < MIN_PRIORITY)
    {
        return;
    }
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
    if(event->type == EVENT_TYPE_BELIEF)
    {
        if(!readded)
        {
            Memory_Conceptualize(&event->term);
            int concept_i;
            if(Memory_FindConceptByTerm(&event->term, &concept_i))
            {
                Concept *c = concepts.items[concept_i].address;
		Event eternal_event = *event;
		if(event->occurrenceTime != OCCURRENCE_ETERNAL)
                {
                    eternal_event.occurrenceTime = OCCURRENCE_ETERNAL;
                    eternal_event.truth = Truth_Eternalize(event->truth);
		    c->belief_spike = Inference_IncreasedActionPotential(&c->belief_spike, event, currentTime, NULL);
		}
		bool revision_happened = false;
		//check if higher order now, implication "$"
		if(atom_names[(int) event->term.atoms[0]][0] == '$' &&
		   atom_names[(int) event->term.atoms[0]][1] == 0) //TODO proper method
		{
                    //get predicate and add
		    //the subject to precondition
		    //table as an implication
		    Term subject = Term_ExtractSubterm(&event->term, 1);
		    Term predicate = Term_ExtractSubterm(&event->term, 2);
		    int concept_i;
		    if(Memory_FindConceptByTerm(&predicate, &concept_i))
		    {
		        Concept *c = concepts.items[concept_i].address;
			Implication imp = {
                            .term = subject,
			    .truth = eternal_event.truth,
			    .stamp = eternal_event.stamp,
			    .sourceConcept = c,
			    .sourceConceptTerm = subject
			};
			//now extract operation id
			int opi = 0;
			if(atom_names[(int) subject.atoms[0]][0] == '#' &&
			   atom_names[(int) subject.atoms[0]][1] == 0) //TODO
			{ //TODO make sure sequences are right-encoded so that operation is the predicate and the subject can be a sequence itself
                            Term potential_op = Term_ExtractSubterm(&subject, 2);
			    Term precon_nop = Term_ExtractSubterm(&subject, 1);
                            if(atom_names[(int) potential_op.atoms[0]][0] == '^')
		            {
                               opi = atom_names[(int) potential_op.atoms[0]][1] - (int) '0';
			       //get rid of op as MSC links cannot use it
			       Term_OverrideSubterm(&imp.term, 1, &precon_nop);
			    }
			}
			Table_AddAndRevise(&c->precondition_beliefs[opi], &imp, "");
		    }
		}
		else
		{
                    c->belief = Inference_IncreasedActionPotential(&c->belief, &eternal_event, currentTime, &revision_happened);
		}
                if(revision_happened)
                {
                    Memory_addEvent(&c->belief, currentTime, priority, false, false, false, true);
                }
            }
            else
            {
                assert(false, "Concept creation failed, it should always be able to create one, even when full, by removing the worst!");
            }
        }
        if(Memory_addCyclingEvent(event, priority, currentTime) && !readded) //task gets replaced with revised one, more radical than OpenNARS!!
        {
            Memory_printAddedEvent(event, priority, input, derived, revised);
        }
    }
    if(event->occurrenceTime == OCCURRENCE_ETERNAL && event->type == EVENT_TYPE_GOAL)
    {
        assert(false, "Eternal goals are not supported");
    }
    assert(event->type == EVENT_TYPE_BELIEF || event->type == EVENT_TYPE_GOAL, "Errornous event type");
}

void Memory_addInputEvent(Event *event, long currentTime)
{
    Memory_addEvent(event, currentTime, Truth_Expectation(event->truth), true, false, false, false);
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
