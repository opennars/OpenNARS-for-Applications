#include "Cycle.h"

int eventsSelected = 0, eventsDerived = 0;
Event selectedEvents[EVENT_SELECTIONS]; //better to be global
Concept selectedConcepts[CONCEPT_SELECTIONS]; //too large to be a local array
Event derivations[MAX_DERIVATIONS];

bool popAndForgetConcept(long currentTime, Concept **returnConcept)
{
    Concept *popped = NULL;
    if(PriorityQueue_PopMax(&concepts, (void*) &popped))
    {
        popped->attention = Attention_forgetConcept(&popped->attention, &popped->usage, currentTime);
        long popped_id = popped->id; //because popped ptr will be invalid to use after adding another concept
        if(Memory_addConcept(popped))
        {
            Concept *popped2 = NULL;
            if(PriorityQueue_PopMax(&concepts, (void*) &popped2))
            {
                if(popped_id == popped2->id)
                {
                    *returnConcept = popped2;
                    return true;
                }
                else
                {
                    Memory_addConcept(popped2);
                }
            }
        }
    }
    return false;
}

int popConcepts(long currentTime, int B_id, Concept** B) 
{
    int conceptsSelected = 0;
    for(int j=0; j<CONCEPT_SELECTIONS; j++)
    {
        Concept *A = NULL;
        if(!popAndForgetConcept(currentTime, &A))
        {
            IN_DEBUG( printf("Selecting concept failed, maybe item order changed.\n"); )
            continue;
        }
        selectedConcepts[conceptsSelected] = *A;
        //in case that we took the matched concept out, validate B pointer again:
        if(A->id == B_id)
        {
            *B = &selectedConcepts[conceptsSelected];
        }
        conceptsSelected++;
    }
    return conceptsSelected;
}

void pushConcepts(int conceptsSelected, Concept *B)
{
    for(int j=0; j<conceptsSelected; j++)
    {   //push again what we popped
        if(!Memory_addConcept(&selectedConcepts[j]))
        {
            IN_DEBUG( printf("Selected concept was not added anymore:\n"); Concept_Print(B); )
        }
    }
}

//doing inference with the highest priority concepts
void temporalInference(Concept *B, Event *b, long currentTime)
{
    //1. pop concepts
    int B_id = B->id;
    int conceptsSelected = popConcepts(currentTime, B_id, &B);
    //2. apply composition rules
    for(int j=0; j<conceptsSelected; j++)
    {   
        Concept *A = &selectedConcepts[j];
        IN_DEBUG( printf("Concept was chosen as temporal inference partner (equals matched? %d):\n", A->id == B->id); Concept_Print(A); )
        RuleTable_Composition(A, B, b, currentTime); // deriving a =/> b
    }
    //3. push concepts again
    pushConcepts(conceptsSelected, B);
}

//retrieve k events from event queue (attention buffer)
void popEvents(long currentTime)
{
    for(int i=0; i<EVENT_SELECTIONS; i++)
    {
        Event *e;
        if(!PriorityQueue_PopMax(&events, (void**) &e))
        {
            assert(events.itemsAmount == 0, "No item was popped, only acceptable reason is when it's empty");
            IN_DEBUG( printf("Selecting event failed, maybe there is no event left.\n"); )
            break;
        }
        selectedEvents[eventsSelected] = *e; //needs to be copied because will be added in a batch
        eventsSelected++; //that while processing, would make recycled pointers invalid to use
    }
}

//put the processed ones back but events that have lost the resource competition die
void pushEvents(long currentTime)
{
    Memory_ResetEvents();
    for(int i=0; i<eventsSelected; i++)
    {
        Event *e = &selectedEvents[i];
        e->attention = Attention_forgetEvent(&e->attention, currentTime);
        if(e->attention.priority > MIN_PRIORITY && e->truth.confidence > MIN_CONFIDENCE)
        {
            Memory_addEvent(e);
        }
    }
    for(int i=0; i<eventsDerived; i++)
    {
        Event *e = &derivations[i];
        if(e->attention.priority > MIN_PRIORITY && e->truth.confidence > MIN_CONFIDENCE)
        {
            Memory_addEvent(e);
        }
    }    
}

void cycle(long currentTime)
{
    eventsSelected = eventsDerived = 0;
    popEvents(currentTime);
    for(int i=0; i<eventsSelected; i++)
    {
        Event *e = &selectedEvents[i];
        Event_SetSDR(e, e->sdr); // TODO make sure that hash needs to be calculated once instead already
        IN_DEBUG( printf("Event was selected:\n"); Event_Print(e); )
        //determine the concept it is related to
        int closest_concept_i;
        Decision decision = (Decision) {0};
        if(Memory_getClosestConcept(e, &closest_concept_i))
        {
            Concept *c = concepts.items[closest_concept_i].address;
            //Matched event, see https://github.com/patham9/ANSNA/wiki/SDR:-SDRInheritance-for-matching,-and-its-truth-value
            Event eMatch = *e;
            eMatch.sdr = c->sdr;
            eMatch.truth = Truth_Deduction(SDR_Inheritance(&e->sdr, &c->sdr), e->truth);
            if(eMatch.truth.confidence > MIN_CONFIDENCE)
            {
                Concept_SDRInterpolation(c, &e->sdr, eMatch.truth); 
                //apply decomposition-based inference: prediction/explanation
                RuleTable_Decomposition(c, &eMatch, currentTime);
                c->usage = Usage_use(&c->usage, currentTime);
                //add event to the FIFO of the concept
                FIFO *fifo =  e->type == EVENT_TYPE_BELIEF ? &c->event_beliefs : &c->event_goals;
                Event revised = FIFO_AddAndRevise(&eMatch, fifo);
                Event *goal = e->type == EVENT_TYPE_GOAL ? &eMatch : NULL;
                if(revised.type != EVENT_TYPE_DELETED && revised.truth.confidence >= MIN_CONFIDENCE && revised.attention.priority >= MIN_PRIORITY)
                {
                    goal = revised.type == EVENT_TYPE_GOAL ? &revised : NULL;
                    IN_OUTPUT( printf("REVISED EVENT: "); Event_Print(&revised); )
                }
                if(goal != NULL)
                {
                    decision = Decision_PotentiallyExecute(c, goal, currentTime);
                    //if no operation matched, try motor babbling with a certain chance
                    if(!decision.matched && !decision.executed)
                    {
                        if(rand() % 1000000 < (int)(MOTOR_BABBLING_CHANCE*1000000.0))
                        {
                            decision = Decision_MotorBabbling();
                        } 
                    }
                    if(decision.executed)
                    {
                        Decision_MotorTagging(c, decision.operationID);
                    }
                }
                //activate concepts attention with the event's attention, but penalize for mismatch to concept
                //eMatch.attention.priority *= Truth_Expectation(eMatch.truth);
                c->attention = Attention_activateConcept(&c->attention, &eMatch.attention); 
                PriorityQueue_IncreasePriority(&concepts, closest_concept_i, c->attention.priority); //priority was increased
            }
            //send event to the highest prioriry concepts
            temporalInference(c, e, currentTime); //c pointer should not be used after this position, as it involves push operation to concepts
        }
        else
        {
            assert(concepts.itemsAmount == 0, "No matching concept is only allowed to happen if memory is empty.");
        }
        if(!decision.matched && !Memory_FindConceptBySDR(&e->sdr, e->sdr_hash, NULL)) //not conceptualizing (&/,a,op())
        {   
            //add a new concept for e too at the end, as it does not exist already
            Concept *eNativeConcept = Memory_Conceptualize(&e->sdr, e->attention);
            if(eNativeConcept != NULL)
            {
                IN_DEBUG( printf("ADDED CONCEPT \n"); )
                FIFO_Add(e, (e->type == EVENT_TYPE_BELIEF ? &eNativeConcept->event_beliefs : &eNativeConcept->event_goals));
            }
        }
    }
    pushEvents(currentTime);
}
