/* 
 * The MIT License
 *
 * Copyright 2020 The OpenNARS authors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Memory.h"

//Concepts in main memory:
PriorityQueue concepts;
//cycling events cycling in main memory:
PriorityQueue cycling_belief_events;
PriorityQueue cycling_goal_events;
//Hashtable of concepts used for fast retrieval of concepts via term:
HashTable HTconcepts;
//Input event buffers:
FIFO belief_events;
FIFO goal_events;
//Operations
Operation operations[OPERATIONS_MAX];
//Parameters
bool PRINT_DERIVATIONS = PRINT_DERIVATIONS_INITIAL;
bool PRINT_INPUT = PRINT_INPUT_INITIAL;
//Storage arrays for the datastructures
Concept concept_storage[CONCEPTS_MAX];
Item concept_items_storage[CONCEPTS_MAX];
Event cycling_belief_event_storage[CYCLING_BELIEF_EVENTS_MAX];
Item cycling_belief_event_items_storage[CYCLING_BELIEF_EVENTS_MAX];
Event cycling_goal_event_storage[CYCLING_GOAL_EVENTS_MAX];
Item cycling_goal_event_items_storage[CYCLING_GOAL_EVENTS_MAX];
//Dynamic concept firing threshold
double conceptPriorityThreshold = 0.0;

static void Memory_ResetEvents()
{
    FIFO_RESET(&belief_events);
    FIFO_RESET(&goal_events);
    PriorityQueue_RESET(&cycling_belief_events, cycling_belief_event_items_storage, CYCLING_BELIEF_EVENTS_MAX);
    PriorityQueue_RESET(&cycling_goal_events, cycling_goal_event_items_storage, CYCLING_GOAL_EVENTS_MAX);
    for(int i=0; i<CYCLING_BELIEF_EVENTS_MAX; i++)
    {
        cycling_belief_event_storage[i] = (Event) {0};
        cycling_belief_events.items[i] = (Item) { .address = &(cycling_belief_event_storage[i]) };
    }
    for(int i=0; i<CYCLING_GOAL_EVENTS_MAX; i++)
    {
        cycling_goal_event_storage[i] = (Event) {0};
        cycling_goal_events.items[i] = (Item) { .address = &(cycling_goal_event_storage[i]) };
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
VMItem* HTconcepts_storageptrs[CONCEPTS_MAX];
VMItem HTconcepts_storage[CONCEPTS_MAX];
VMItem* HTconcepts_HT[CONCEPTS_HASHTABLE_BUCKETS]; //the hash of the concept term is the index

void Memory_INIT()
{
    HashTable_INIT(&HTconcepts, HTconcepts_storage, HTconcepts_storageptrs, HTconcepts_HT, CONCEPTS_HASHTABLE_BUCKETS, CONCEPTS_MAX, (Equal) Term_Equal, (Hash) Term_Hash);
    conceptPriorityThreshold = 0.0;
    Memory_ResetConcepts();
    Memory_ResetEvents();
    InvertedAtomIndex_INIT();
    for(int i=0; i<OPERATIONS_MAX; i++)
    {
        operations[i] = (Operation) {0};
    }
    concept_id = 0;
}

Concept *Memory_FindConceptByTerm(Term *term)
{
    return HashTable_Get(&HTconcepts, term);
}

Concept* Memory_Conceptualize(Term *term, long currentTime)
{
    if(Narsese_isOperation(term)) //don't conceptualize operations
    {
        return NULL;
    }
    Concept *ret = Memory_FindConceptByTerm(term);
    if(ret == NULL)
    {
        Concept *recycleConcept = NULL;
        //try to add it, and if successful add to voting structure
        PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&concepts, 1);
        if(feedback.added)
        {
            recycleConcept = feedback.addedItem.address;
            //if something was evicted in the adding process delete from hashmap first
            if(feedback.evicted)
            {
                IN_DEBUG( assert(HashTable_Get(&HTconcepts, &recycleConcept->term) != NULL, "VMItem to delete does not exist!"); )
                HashTable_Delete(&HTconcepts, &recycleConcept->term);
                IN_DEBUG( assert(HashTable_Get(&HTconcepts, &recycleConcept->term) == NULL, "VMItem to delete was not deleted!"); )
                //and also delete from inverted atom index:
                InvertedAtomIndex_RemoveConcept(recycleConcept->term, recycleConcept);
            }
            //Add term to inverted atom index as well:
            InvertedAtomIndex_AddConcept(*term, recycleConcept);
            //proceed with recycling of the concept in the priority queue
            *recycleConcept = (Concept) {0};
            recycleConcept->term = *term;
            recycleConcept->id = concept_id;
            recycleConcept->usage = (Usage) { .useCount = 1, .lastUsed = currentTime };
            concept_id++;
            //also add added concept to HashMap:
            IN_DEBUG( assert(HashTable_Get(&HTconcepts, &recycleConcept->term) == NULL, "VMItem to add already exists!"); )
            HashTable_Set(&HTconcepts, &recycleConcept->term, recycleConcept);
            IN_DEBUG( assert(HashTable_Get(&HTconcepts, &recycleConcept->term) != NULL, "VMItem to add was not added!"); )
            return recycleConcept;
        }
    }
    else
    {
        return ret;
    }
    return NULL;
}

Event selectedBeliefs[BELIEF_EVENT_SELECTIONS]; //better to be global
double selectedBeliefsPriority[BELIEF_EVENT_SELECTIONS]; //better to be global
int beliefsSelectedCnt = 0;
Event selectedGoals[GOAL_EVENT_SELECTIONS]; //better to be global
double selectedGoalsPriority[GOAL_EVENT_SELECTIONS]; //better to be global
int goalsSelectedCnt = 0;

static bool Memory_containsEvent(PriorityQueue *queue, Event *event)
{
    for(int i=0; i<queue->itemsAmount; i++)
    {
        if(Event_Equal(event, queue->items[i].address))
        {
            return true;
        }
    }
    return false;
}

//Add event for cycling through the system (inference and context)
//called by addEvent for eternal knowledge
bool Memory_addCyclingEvent(Event *e, double priority, long currentTime)
{
    assert(e->type == EVENT_TYPE_BELIEF || e->type == EVENT_TYPE_GOAL, "Only belief and goals events can be added to cycling events queue!");
    if((e->type == EVENT_TYPE_BELIEF && Memory_containsEvent(&cycling_belief_events, e)) ||
       (e->type == EVENT_TYPE_GOAL && Memory_containsEvent(&cycling_goal_events, e))) //avoid duplicate derivations
    {
        return false;
    }
    Concept *c = Memory_FindConceptByTerm(&e->term);
    if(c != NULL)
    {
        if(e->type == EVENT_TYPE_BELIEF && c->belief.type != EVENT_TYPE_DELETED && ((e->occurrenceTime == OCCURRENCE_ETERNAL && c->belief.truth.confidence > e->truth.confidence) || (e->occurrenceTime != OCCURRENCE_ETERNAL && Truth_Projection(c->belief_spike.truth, c->belief_spike.occurrenceTime, currentTime).confidence > Truth_Projection(e->truth, e->occurrenceTime, currentTime).confidence)))
        {
            return false; //the belief has a higher confidence and was already revised up (or a cyclic transformation happened!), get rid of the event!
        }   //more radical than OpenNARS!
        if(e->type == EVENT_TYPE_GOAL && c->goal_spike.type != EVENT_TYPE_DELETED && ((e->occurrenceTime == OCCURRENCE_ETERNAL && c->goal_spike.truth.confidence > e->truth.confidence) || (e->occurrenceTime != OCCURRENCE_ETERNAL && Truth_Projection(c->goal_spike.truth, c->goal_spike.occurrenceTime, currentTime).confidence > Truth_Projection(e->truth, e->occurrenceTime, currentTime).confidence)))
        {
            return false; //the belief has a higher confidence and was already revised up (or a cyclic transformation happened!), get rid of the event!
        }   //more radical than OpenNARS!
    }
    PriorityQueue *priority_queue = e->type == EVENT_TYPE_BELIEF ? &cycling_belief_events : &cycling_goal_events;
    PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(priority_queue, priority);
    if(feedback.added)
    {
        Event *toRecyle = feedback.addedItem.address;
        *toRecyle = *e;
        return true;
    }
    return false;
}

static void Memory_printAddedKnowledge(Term *term, char type, Truth *truth, long occurrenceTime, double priority, bool input, bool derived, bool revised)
{
    if(((input && PRINT_INPUT) || PRINT_DERIVATIONS) && priority > PRINT_DERIVATIONS_PRIORITY_THRESHOLD && (input || derived || revised))
    {
        fputs(revised ? "Revised: " : (input ? "Input: " : "Derived: "), stdout);
        Narsese_PrintTerm(term);
        fputs((type == EVENT_TYPE_BELIEF ? ". " : "! "), stdout);
        if(occurrenceTime != OCCURRENCE_ETERNAL)
        {
            printf(":|: occurrenceTime=%ld ", occurrenceTime);
        }
        printf("Priority=%f ", priority);
        Truth_Print(truth);
        fflush(stdout);
    }
}

void Memory_printAddedEvent(Event *event, double priority, bool input, bool derived, bool revised)
{
    Memory_printAddedKnowledge(&event->term, event->type, &event->truth, event->occurrenceTime, priority, input, derived, revised);
}

void Memory_printAddedImplication(Term *implication, Truth *truth, bool input, bool revised)
{
    Memory_printAddedKnowledge(implication, EVENT_TYPE_BELIEF, truth, OCCURRENCE_ETERNAL, 1, input, true, revised);
}

void Memory_ProcessBeliefEvent(Event *event, long currentTime, double priority, long occurrenceTimeOffset, bool input, bool derived, bool revised, bool isImplication)
{
    Event eternal_event = *event;
    if(event->occurrenceTime != OCCURRENCE_ETERNAL)
    {
        eternal_event.occurrenceTime = OCCURRENCE_ETERNAL;
        eternal_event.truth = Truth_Eternalize(event->truth);
    }
    if(isImplication)
    {
        //get predicate and add the subject to precondition table as an implication
        Term subject = Term_ExtractSubterm(&event->term, 1);
        Term predicate = Term_ExtractSubterm(&event->term, 2);
        Concept *target_concept = Memory_Conceptualize(&predicate, currentTime);
        if(target_concept != NULL)
        {
            target_concept->usage = Usage_use(target_concept->usage, currentTime);
            Implication imp = { .truth = eternal_event.truth,
                                .stamp = eternal_event.stamp,
                                .occurrenceTimeOffset = occurrenceTimeOffset,
                                .creationTime = currentTime };
            Term sourceConceptTerm = subject;
            //now extract operation id
            int opi = 0;
            if(Narsese_copulaEquals(subject.atoms[0], '+')) //sequence
            {
                Term potential_op = Term_ExtractSubterm(&subject, 2);
                if(Narsese_isOperation(&potential_op)) //atom starts with ^, making it an operator
                {
                    opi = Narsese_getOperationID(&potential_op); //"<(a * b) --> ^op>" to ^op index
                    sourceConceptTerm = Term_ExtractSubterm(&subject, 1); //gets rid of op as MSC links cannot use it
                }
                else
                {
                    sourceConceptTerm = subject;
                }
            }
            else
            {
                sourceConceptTerm = subject;
            }
            Concept *source_concept = Memory_Conceptualize(&sourceConceptTerm, currentTime);
            if(source_concept != NULL)
            {
                source_concept->usage = Usage_use(source_concept->usage, currentTime);
                imp.sourceConceptId = source_concept->id;
                imp.sourceConcept = source_concept;
                imp.term = event->term;
                Table_AddAndRevise(&target_concept->precondition_beliefs[opi], &imp);
                Memory_printAddedEvent(event, priority, input, derived, revised);
            }
        }
    }
    else
    {
        Concept *c = Memory_Conceptualize(&event->term, currentTime);
        if(c != NULL)
        {
            c->usage = Usage_use(c->usage, currentTime);
            c->priority = MAX(c->priority, priority);
            if(event->occurrenceTime != OCCURRENCE_ETERNAL && event->occurrenceTime <= currentTime)
            {
                c->belief_spike = Inference_RevisionAndChoice(&c->belief_spike, event, currentTime, NULL);
                c->belief_spike.creationTime = currentTime; //for metrics
            }
            if(event->occurrenceTime != OCCURRENCE_ETERNAL && event->occurrenceTime > currentTime)
            {
                c->predicted_belief = Inference_RevisionAndChoice(&c->predicted_belief, event, currentTime, NULL);
                c->predicted_belief.creationTime = currentTime;
            }
            bool revision_happened = false;
            c->belief = Inference_RevisionAndChoice(&c->belief, &eternal_event, currentTime, &revision_happened);
            c->belief.creationTime = currentTime; //for metrics
            if(revision_happened)
            {
                Memory_AddEvent(&c->belief, currentTime, priority, 0, false, false, true);
            }
        }
    }
}

void Memory_AddEvent(Event *event, long currentTime, double priority, long occurrenceTimeOffset, bool input, bool derived, bool revised)
{
    if(!revised && !input) //derivations get penalized by complexity as well, but revised ones not as they already come from an input or derivation
    {
        double complexity = Term_Complexity(&event->term);
        priority *= 1.0 / log2(1.0 + complexity);
    }
    if(event->truth.confidence < MIN_CONFIDENCE || priority <= MIN_PRIORITY)
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
        bool isImplication = Narsese_copulaEquals(event->term.atoms[0], '$');
        if(isImplication)
        {
            Memory_ProcessBeliefEvent(event, currentTime, priority, occurrenceTimeOffset, input, derived, revised, isImplication);
            return;
        }
        Memory_addCyclingEvent(event, priority, currentTime);
    }
    if(event->type == EVENT_TYPE_GOAL)
    {
        assert(event->occurrenceTime != OCCURRENCE_ETERNAL, "Eternal goals are not supported");
        Memory_addCyclingEvent(event, priority, currentTime);
    }
    Memory_printAddedEvent(event, priority, input, derived, revised);
    assert(event->type == EVENT_TYPE_BELIEF || event->type == EVENT_TYPE_GOAL, "Errornous event type");
}

void Memory_AddInputEvent(Event *event, long currentTime)
{
    Memory_AddEvent(event, currentTime, 1, 0, true, false, false);
}

void Memory_AddOperation(int id, Operation op)
{
    operations[id - 1] = op;
}

bool Memory_ImplicationValid(Implication *imp)
{
    return imp->sourceConceptId == ((Concept*) imp->sourceConcept)->id;
}
