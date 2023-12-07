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
PriorityQueue cycling_goal_events[CYCLING_GOAL_EVENTS_LAYERS];
//Hashtable of concepts used for fast retrieval of concepts via term:
HashTable HTconcepts;
//OccurrenceTimeIndex for accelerating temporal induction
OccurrenceTimeIndex occurrenceTimeIndex;
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
Event cycling_goal_event_storage[CYCLING_GOAL_EVENTS_LAYERS][CYCLING_GOAL_EVENTS_MAX];
Item cycling_goal_event_items_storage[CYCLING_GOAL_EVENTS_LAYERS][CYCLING_GOAL_EVENTS_MAX];
//Dynamic concept firing threshold
double conceptPriorityThreshold = 0.0;
//Priority threshold for printing derivations
double PRINT_EVENTS_PRIORITY_THRESHOLD = PRINT_EVENTS_PRIORITY_THRESHOLD_INITIAL;

static void Memory_ResetEvents()
{
    PriorityQueue_INIT(&cycling_belief_events, cycling_belief_event_items_storage, CYCLING_BELIEF_EVENTS_MAX);
    for(int i=0; i<CYCLING_BELIEF_EVENTS_MAX; i++)
    {
        cycling_belief_event_storage[i] = (Event) {0};
        cycling_belief_events.items[i] = (Item) { .address = &(cycling_belief_event_storage[i]) };
    }
    for(int layer=0; layer<CYCLING_GOAL_EVENTS_LAYERS; layer++)
    {
        PriorityQueue_INIT(&cycling_goal_events[layer], cycling_goal_event_items_storage[layer], CYCLING_GOAL_EVENTS_MAX);
        for(int i=0; i<CYCLING_GOAL_EVENTS_MAX; i++)
        {
            cycling_goal_event_storage[layer][i] = (Event) {0};
            cycling_goal_events[layer].items[i] = (Item) { .address = &(cycling_goal_event_storage[layer][i]) };
        }
    }
}

static void Memory_ResetConcepts()
{
    PriorityQueue_INIT(&concepts, concept_items_storage, CONCEPTS_MAX);
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
    occurrenceTimeIndex = (OccurrenceTimeIndex) {0};
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

bool Memory_containsBeliefOrGoal(Event *e)
{
    Concept *c = Memory_FindConceptByTerm(&e->term);
    if(c != NULL)
    {
        if(e->type == EVENT_TYPE_BELIEF)
        {
            if(e->occurrenceTime == OCCURRENCE_ETERNAL)
            {
                if(c->belief.type != EVENT_TYPE_DELETED && Event_Equal(e, &c->belief))
                {
                    return true;
                }
            }
            else
            if(c->belief_spike.type != EVENT_TYPE_DELETED && Event_EqualTermEqualStampLessConfidentThan(e, &c->belief_spike))
            {
                return true;
            }
        }
        else
        if(e->type == EVENT_TYPE_GOAL && c->goal_spike.type != EVENT_TYPE_DELETED && Event_Equal(&c->goal_spike, e))
        {
            return true;
        }
    }
    return false;
}

//Add event for cycling through the system (inference and context)
//called by addEvent for eternal knowledge
bool Memory_addCyclingEvent(Event *e, double priority, long currentTime, int layer)
{
    assert(e->type == EVENT_TYPE_BELIEF || e->type == EVENT_TYPE_GOAL, "Only belief and goals events can be added to cycling events queue!");
    if((e->type == EVENT_TYPE_BELIEF && Memory_containsEvent(&cycling_belief_events, e)) || Memory_containsBeliefOrGoal(e)) //avoid duplicate derivations
    {
        return false;
    }
    if(e->type == EVENT_TYPE_GOAL) //avoid duplicate derivations
    {
        for(int layer=0; layer<CYCLING_GOAL_EVENTS_LAYERS; layer++)
        {
            if(Memory_containsEvent(&cycling_goal_events[layer], e))
            {
                return false;
            }
        }
    }
    Concept *c = Memory_FindConceptByTerm(&e->term);
    if(c != NULL)
    {
        if(e->type == EVENT_TYPE_BELIEF && c->belief.type != EVENT_TYPE_DELETED && e->occurrenceTime == OCCURRENCE_ETERNAL && c->belief.truth.confidence > e->truth.confidence)
        {
            return false; //the belief has a higher confidence and was already revised up (or a cyclic transformation happened!), get rid of the event!
        }   //more radical than OpenNARS!
    }
    PriorityQueue *priority_queue = e->type == EVENT_TYPE_BELIEF ? &cycling_belief_events : &cycling_goal_events[MIN(CYCLING_GOAL_EVENTS_LAYERS-1, layer+1)];
    PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(priority_queue, priority);
    if(feedback.added)
    {
        Event *toRecyle = feedback.addedItem.address;
        *toRecyle = *e;
        return true;
    }
    return false;
}

static void Memory_printAddedKnowledge(Stamp *stamp, Term *term, char type, Truth *truth, long occurrenceTime, double occurrenceTimeOffset, double priority, bool input, bool derived, bool revised, bool controlInfo, bool selected)
{
    if((input && PRINT_INPUT) || (!input && PRINT_DERIVATIONS && priority > PRINT_EVENTS_PRIORITY_THRESHOLD))
    {
        if(controlInfo)
            fputs(selected ? "Selected: " : (revised ? "Revised: " : (input ? "Input: " : "Derived: ")), stdout);
        if(Narsese_copulaEquals(term->atoms[0], TEMPORAL_IMPLICATION))
            printf("dt=%f ", occurrenceTimeOffset);
        Narsese_PrintTerm(term);
        fputs((type == EVENT_TYPE_BELIEF ? ". " : "! "), stdout);
        if(occurrenceTime != OCCURRENCE_ETERNAL)
        {
            printf(":|: occurrenceTime=%ld ", occurrenceTime);
        }
        if(controlInfo)
        {
            printf("Priority=%f ", priority);
            Stamp_print(stamp);
            fputs(" ", stdout);
            Truth_Print(truth);
        }
        else
        {
            Truth_Print2(truth);
        }
        fflush(stdout);
    }
}

void Memory_printAddedEvent(Stamp *stamp, Event *event, double priority, bool input, bool derived, bool revised, bool controlInfo, bool selected)
{
    Memory_printAddedKnowledge(stamp, &event->term, event->type, &event->truth, event->occurrenceTime, event->occurrenceTimeOffset, priority, input, derived, revised, controlInfo, selected);
}

void Memory_printAddedImplication(Stamp *stamp, Term *implication, Truth *truth, double occurrenceTimeOffset, double priority, bool input, bool revised, bool controlInfo)
{
    Memory_printAddedKnowledge(stamp, implication, EVENT_TYPE_BELIEF, truth, OCCURRENCE_ETERNAL, occurrenceTimeOffset, priority, input, true, revised, controlInfo, false);
}

void Memory_ProcessNewBeliefEvent(Event *event, long currentTime, double priority, bool input)
{
    bool eternalInput = input && event->occurrenceTime == OCCURRENCE_ETERNAL;
    Event eternal_event = Event_Eternalized(event);
    if(Narsese_copulaEquals(event->term.atoms[0], TEMPORAL_IMPLICATION))
    {
        //get predicate and add the subject to precondition table as an implication
        Term subject = Term_ExtractSubterm(&event->term, 1);
        Term predicate = Term_ExtractSubterm(&event->term, 2);
        Concept *target_concept = Memory_Conceptualize(&predicate, currentTime);
        if(target_concept != NULL)
        {
            target_concept->usage = Usage_use(target_concept->usage, currentTime, eternalInput);
            Implication imp = { .truth = eternal_event.truth,
                                .stamp = eternal_event.stamp,
                                .occurrenceTimeOffset = event->occurrenceTimeOffset,
                                .creationTime = currentTime };
            Term sourceConceptTerm = subject;
            //now extract operation id
            int opi = 0;
            if(Narsese_copulaEquals(subject.atoms[0], SEQUENCE)) //sequence
            {
                Term potential_op = Term_ExtractSubterm(&subject, 2);
                if(Narsese_isOperation(&potential_op)) //necessary to be an executable operator
                {
                    if(!Narsese_isExecutableOperation(&potential_op))
                    {
                        return; //we can't store proc. knowledge of other agents
                    }
                    opi = Memory_getOperationID(&potential_op); //"<(a * b) --> ^op>" to ^op index
                    sourceConceptTerm = Narsese_GetPreconditionWithoutOp(&subject); //gets rid of op as MSC links cannot use it
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
                source_concept->usage = Usage_use(source_concept->usage, currentTime, eternalInput);
                imp.sourceConceptId = source_concept->id;
                imp.sourceConcept = source_concept;
                imp.term = event->term;
                Implication *revised = Table_AddAndRevise(&target_concept->precondition_beliefs[opi], &imp);
                if(revised != NULL)
                {
                    bool wasRevised = revised->truth.confidence > event->truth.confidence || revised->truth.confidence == MAX_CONFIDENCE;
                    Memory_printAddedImplication(&event->stamp, &event->term, &imp.truth, event->occurrenceTimeOffset, priority, input, false, true);
                    if(wasRevised)
                        Memory_printAddedImplication(&revised->stamp, &revised->term, &revised->truth, revised->occurrenceTimeOffset, priority, input, true, true);
                }
            }
        }
    }
    else
    {
        Concept *c = Memory_Conceptualize(&event->term, currentTime);
        if(c != NULL)
        {
            if(event->occurrenceTime != OCCURRENCE_ETERNAL && !Narsese_copulaEquals(event->term.atoms[0], HAS_CONTINUOUS_PROPERTY))
            {
                OccurrenceTimeIndex_Add(c, &occurrenceTimeIndex);
            }
            c->usage = Usage_use(c->usage, currentTime, eternalInput);
            c->priority = MAX(c->priority, priority);
            if(event->occurrenceTime != OCCURRENCE_ETERNAL && event->occurrenceTime <= currentTime)
            {
                if(ALLOW_NOT_SELECTED_PRECONDITIONS_CONDITIONING)
                {
                    c->lastSelectionTime = currentTime;
                }
                c->belief_spike = Inference_RevisionAndChoice(&c->belief_spike, event, currentTime, NULL);
                c->belief_spike.creationTime = currentTime; //for metrics
                if(PRINT_SURPRISE && input)
                {
                    double surprise = 1.0;
                    if(c->predicted_belief.type != EVENT_TYPE_DELETED)
                    {
                        float expectation = Truth_Expectation(Truth_Projection(c->predicted_belief.truth, c->predicted_belief.occurrenceTime, c->belief_spike.occurrenceTime));
                        surprise = fabs(expectation - Truth_Expectation(c->belief_spike.truth));
                    }
                    printf("//SURPRISE %f\n", surprise);
                }
            }
            if(event->occurrenceTime != OCCURRENCE_ETERNAL && event->occurrenceTime > currentTime)
            {
                c->predicted_belief = Inference_RevisionAndChoice(&c->predicted_belief, event, currentTime, NULL);
                c->predicted_belief.creationTime = currentTime;
            }
            bool revision_happened = false;
            c->belief = Inference_RevisionAndChoice(&c->belief, &eternal_event, currentTime, &revision_happened);
            c->belief.creationTime = currentTime; //for metrics
            if(input)
            {
                Memory_printAddedEvent(&event->stamp, event, priority, input, false, false, true, false);
            }
            if(revision_happened)
            {
                Memory_AddEvent(&c->belief, currentTime, priority, false, true, true, 0);
                if(event->occurrenceTime == OCCURRENCE_ETERNAL)
                {
                    Memory_printAddedEvent(&c->belief.stamp, &c->belief, priority, false, false, true, true, false);
                }
            }
        }
    }
}

void Memory_AddEvent(Event *event, long currentTime, double priority, bool input, bool derived, bool revised, int layer)
{
    if(!revised && !input) //derivations get penalized by complexity as well, but revised ones not as they already come from an input or derivation
    {
        double complexity = Term_Complexity(&event->term);
        priority *= 1.0 / log2(1.0 + complexity);
    }
    if(event->truth.confidence < MIN_CONFIDENCE || priority <= MIN_PRIORITY || priority == 0.0)
    {
        return;
    }
    if(input && event->type == EVENT_TYPE_GOAL)
    {
        Memory_printAddedEvent(&event->stamp, event, priority, input, false, false, true, false);
    }
    bool addedToCyclingEventsQueue = false;
    if(event->type == EVENT_TYPE_BELIEF)
    {
        if(!Narsese_copulaEquals(event->term.atoms[0], TEMPORAL_IMPLICATION))
        {
            addedToCyclingEventsQueue = Memory_addCyclingEvent(event, priority, currentTime, layer);
        }
        Memory_ProcessNewBeliefEvent(event, currentTime, priority, input);
    }
    if(event->type == EVENT_TYPE_GOAL)
    {
        addedToCyclingEventsQueue = Memory_addCyclingEvent(event, priority, currentTime, layer);
        assert(event->occurrenceTime != OCCURRENCE_ETERNAL, "Eternal goals are not supported");
    }
    if(addedToCyclingEventsQueue && !input) //print new tasks
    {
        Memory_printAddedEvent(&event->stamp, event, priority, input, derived, revised, true, false);
    }
    assert(event->type == EVENT_TYPE_BELIEF || event->type == EVENT_TYPE_GOAL, "Errornous event type");
}

void Memory_AddInputEvent(Event *event, long currentTime)
{
    Memory_AddEvent(event, currentTime, 1, true, false, false, 0);
}

bool Memory_ImplicationValid(Implication *imp)
{
    return imp->sourceConceptId == ((Concept*) imp->sourceConcept)->id;
}

int Memory_getOperationID(Term *term)
{
    Atom op_atom = Narsese_getOperationAtom(term);
    if(op_atom)
    {
        for(int k=1; k<=OPERATIONS_MAX; k++)
        {
            if(operations[k-1].term.atoms[0] == op_atom)
            {
                return k;
            }
        }
    }
    return 0;
}
