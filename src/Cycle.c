#include "Cycle.h"

void composition(Concept *B, Concept *A, Event *b, long currentTime)
{
    //temporal induction and intersection
    if(b->type == EVENT_TYPE_BELIEF && A->event_beliefs.itemsAmount > 0)
    {
        Event *a = &A->event_beliefs.array[0]; //most recent, highest revised
        if(!Stamp_checkOverlap(&a->stamp, &b->stamp))
        {
            Implication implication = Inference_BeliefInduction(a, b);
            if(implication.truth.confidence >= MIN_CONFIDENCE)
            {
                IN_OUTPUT( printf("Formed implication: "); Implication_Print(&implication); )
                Implication revised_precon = Table_AddAndRevise(&B->precondition_beliefs, &implication);
                IN_OUTPUT( if(revised_precon.sdr_hash != 0) { printf("Revised pre-condition implication: "); Implication_Print(&revised_precon); } )
                Implication revised_postcon = Table_AddAndRevise(&A->postcondition_beliefs, &implication);
                IN_OUTPUT( if(revised_postcon.sdr_hash != 0) { printf("Revised post-condition implication: "); Implication_Print(&revised_postcon); } )
            }
            Event sequence = b->occurrenceTime > a->occurrenceTime ? Inference_BeliefIntersection(a, b) : Inference_BeliefIntersection(b, a);
            sequence.attention = Attention_deriveEvent(&B->attention, &a->truth, currentTime);
            if(sequence.truth.confidence < MIN_CONFIDENCE || sequence.attention.priority < MIN_PRIORITY)
            {
                return;
            }
            Memory_addEvent(&sequence);
            IN_OUTPUT( printf("COMPOSED SEQUENCE EVENT: "); Event_Print(&sequence); )
        }
        else
        {
            IN_DEBUG( Stamp_print(&a->stamp); Stamp_print(&b->stamp); )
        }
    }
}

#define MIN(a,b) (((a)<(b))?(a):(b))

void decomposition(Concept *c, Event *e, long currentTime)
{
    //detachment
    if(c->postcondition_beliefs.itemsAmount>0)
    {
        int k=0;
        for(int i=0; i<c->postcondition_beliefs.itemsAmount; i++)
        {
            Implication postcon = c->postcondition_beliefs.array[i];
            if(!Stamp_checkOverlap(&e->stamp, &postcon.stamp))
            {
                Event res = e->type == EVENT_TYPE_BELIEF ? Inference_BeliefDeduction(e, &postcon) : Inference_GoalAbduction(e, &postcon);
                res.attention = Attention_deriveEvent(&c->attention, &postcon.truth, currentTime);
                if(res.truth.confidence < MIN_CONFIDENCE || res.attention.priority < MIN_PRIORITY)
                {
                    continue;
                }
                Memory_addEvent(&res);
                //add negative evidence to the used predictive hypothesis (assumption of failure, for extinction)
                Table_PopHighestTruthExpectationElement(&c->postcondition_beliefs);
                Implication updated = Inference_AssumptionOfFailure(&postcon);
                Table_Add(&c->postcondition_beliefs, &updated);
                k++;
                IN_OUTPUT( printf("DETACHED FORWARD EVENT: "); Event_Print(&res); )
                if(k>MAX_PREDICTIONS)
                {
                    break;
                }
            }
        }
    }
    if(c->precondition_beliefs.itemsAmount>0)
    {
        int k=0;
        for(int i=0; i<c->precondition_beliefs.itemsAmount; i++)
        {
            Implication precon = c->precondition_beliefs.array[i];
            if(!Stamp_checkOverlap(&e->stamp, &precon.stamp))
            {
                Event res = e->type == EVENT_TYPE_BELIEF ? Inference_BeliefAbduction(e, &precon) : Inference_GoalDeduction(e, &precon);
                res.attention = Attention_deriveEvent(&c->attention, &precon.truth, currentTime);
                if(res.truth.confidence < MIN_CONFIDENCE || res.attention.priority < MIN_PRIORITY)
                {
                    continue;
                }
                Memory_addEvent(&res);
                k++;
                IN_OUTPUT( printf("DETACHED BACKWARD EVENT: "); Event_Print(&res); )
                if(k>MAX_PREDICTIONS)
                {
                    break;
                }
            }
        }
    }
}

//Lazy relative forgetting, making sure the first element also deserves to be the first
Item popAndForget(PriorityQueue *queue, bool isConcept, long currentTime)
{
    //1. pop an item
    Item item = PriorityQueue_PopMax(queue);
    if(item.address == 0)
    {
        return item;
    }
    //2. forget item the more the longer it wasn't last forgotten
    double priority = 0;
    if(isConcept)
    {
        Concept *c = (Concept*) item.address;
        Attention ret = Attention_forgetConcept(&c->attention, &c->usage, currentTime);
        priority = ret.priority;
    }
    else
    {
        Event *e = (Event*) item.address;
        Attention ret = Attention_forgetEvent(&e->attention, currentTime);
        priority = ret.priority;
    }
    //3. push it again
    PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(queue, priority);
    if(!feedback.added)
    {
        return (Item) {0};
    }
    else
    {
        feedback.addedItem.address = item.address;
    }
    //4. check if the pushed item is still considered the Max
    Item item2 = PriorityQueue_PopMax(queue);
    if(item2.address != item.address)
    { //item did not deserve to be the highest, it just wasn't selected
        //PriorityQueue_Push(queue, priority);
        //return (Item) {0}; //to be relative forgotten for some time
    }
    return item;
}

typedef struct
{
    bool matched;
    bool executed;
    Operation op;
}PotentialExecutionResult;

//it returns whether and which operation matched and whether it was executed
PotentialExecutionResult potentialExecution(Concept *c, Event *goal, long currentTime)
{
    PotentialExecutionResult result = (PotentialExecutionResult) {0};
    for(int i=0; i<OPERATIONS_MAX; i++)
    {
        Event G = *goal;
        Operation op = operations[i];
        if(op.action == 0)
        {
            break;
        }
        result.matched = SDR_Subset(&op.sdr, &G.sdr);
        if(result.matched)
        {
            result.op = op;
            G.truth = Truth_Projection(G.truth, G.occurrenceTime, currentTime);
            G.occurrenceTime = currentTime;
            SDR a_rest = SDR_Minus(&goal->sdr, &op.sdr);
            Event b = FIFO_GetHighestConfidentProjectedTo(&c->event_beliefs, currentTime, &a_rest).projectedEvent;
            if(Truth_Expectation(Truth_Deduction(G.truth, b.truth)) > DECISION_THRESHOLD)
            {
                result.executed = true;
                (*op.action)();
            }
            return result;
        }
        
    }
    return result;
}

PotentialExecutionResult motorBabbling()
{
    PotentialExecutionResult result = (PotentialExecutionResult) {0};
    int n_ops = OPERATIONS_MAX;
    for(int i=0; i<OPERATIONS_MAX; i++)
    {
        if(operations[i].action == 0)
        {
            n_ops = i;
            break;
        }
    }
    if(n_ops > 0)
    {
        int chosen = random() % n_ops;
        result.op = operations[chosen];
        (*result.op.action)();
        result.executed = true;
    }
    return result;
}

void motorTagging(Concept *c, Operation op)
{
    Event ev = FIFO_GetNewestElement(&c->event_beliefs);
    ev.sdr = SDR_Union(&ev.sdr, &op.sdr);
    FIFO_AddAndRevise(&ev, &c->event_beliefs);
}

void temporalInference(Concept *c, Event *e, PotentialExecutionResult *consideredOperation, long currentTime)
{
    //trigger composition-based inference hypothesis formation
    Item selectedItem[CONCEPT_SELECTIONS];
    int conceptsSelected = 0;
    for(int j=0; j<CONCEPT_SELECTIONS; j++)
    {   //by doing inference with the highest priority concepts
        selectedItem[j] = popAndForget(&concepts, true, currentTime);
        if(selectedItem[j].address == 0)
        {
            IN_DEBUG
            ( 
                printf("Selecting concept failed, maybe item order changed.\n"); 
                if(selectedItem[j].address != 0)
                {
                    SDR_PrintWhereTrue(&((Concept*) selectedItem[j].address)->sdr);
                }
            )
            continue;
        }
        if(selectedItem[j].address == c)
        {
            IN_DEBUG( printf("Selected concept is the same as matched one.\n"); )
        }
        IN_DEBUG( printf("Concept was chosen as temporal inference partner:\n"); Concept_Print(selectedItem[j].address); )
        if(consideredOperation->executed)
        {
            motorTagging(selectedItem[j].address, consideredOperation->op);
        }
        composition(c, selectedItem[j].address, e, currentTime); // deriving a =/> b
        conceptsSelected++;
    }
    for(int j=0; j<conceptsSelected; j++)
    {   //push again what we popped
        PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&concepts, selectedItem[j].priority);
        if(feedback.added)
        {
            *((Concept*) feedback.addedItem.address) = *((Concept*) selectedItem[j].address);
        }
    }
}

void cycle(long currentTime)
{
    Item selectedItem[EVENT_SELECTIONS];
    int eventsSelected = 0;
    for(int i=0; i<EVENT_SELECTIONS; i++)
    {
        //1. get an event from the event queue
        Item item = popAndForget(&events, false, currentTime);
        if(item.address == 0)
        {
            IN_DEBUG( printf("Selecting event failed, maybe the item order changed.\n"); )
            continue;
        }
        selectedItem[eventsSelected] = item;
        eventsSelected++;
        Event *e = item.address;
        Event_SetSDR(e, e->sdr); // TODO make sure that hash needs to be calculated once instead already
        IN_DEBUG( printf("Event was selected:\n"); Event_Print(e); )
        //TODO better handling
        //end pop forget push pop strategu
        //determine the concept it is related to
        int closest_concept_i = Memory_getClosestConcept(e);
        PotentialExecutionResult consideredOperation = (PotentialExecutionResult) {0};
        if(closest_concept_i != MEMORY_MATCH_NO_CONCEPT)
        {
            Concept *c = concepts.items[closest_concept_i].address;
            //Matched event, see https://github.com/patham9/ANSNA/wiki/SDR:-SDRInheritance-for-matching,-and-its-truth-value
            Event eMatch = *e;
            eMatch.truth = Truth_Deduction(SDR_Inheritance(&e->sdr, &c->sdr), e->truth);
            //apply decomposition-based inference: prediction/explanation
            if(currentTime - c->usage.lastUsed > CONCEPT_WAIT_TIME)
            {
                decomposition(c, &eMatch, currentTime);
            }
            c->usage = Usage_use(&c->usage, currentTime);
            //send event to the highest prioriry concepts
            temporalInference(c, e, &consideredOperation, currentTime);
            //add event to the FIFO of the concept
            FIFO *fifo =  e->type == EVENT_TYPE_BELIEF ? &c->event_beliefs : &c->event_goals;
            Event revised = FIFO_AddAndRevise(&eMatch, fifo);
            Event *goal = e->type == EVENT_TYPE_GOAL ? e : NULL;
            if(revised.type != EVENT_TYPE_DELETED && revised.truth.confidence >= MIN_CONFIDENCE && revised.attention.priority >= MIN_PRIORITY)
            {
                goal = revised.type == EVENT_TYPE_GOAL ? &revised : NULL;
                Memory_addEvent(&revised);
                IN_OUTPUT( printf("REVISED EVENT: "); Event_Print(&revised); )
            }
            if(goal != NULL)
            {
                consideredOperation = potentialExecution(c, goal, currentTime);
                //if no operation matched, try motor babbling with a certain chance
                if(!consideredOperation.matched && !consideredOperation.executed)
                {
                    if(random() % 1000000 < (int)(MOTOR_BABBLING_CHANCE*1000000.0))
                    {
                        consideredOperation = motorBabbling();
                    } 
                }
                if(consideredOperation.executed)
                {
                    motorTagging(c, consideredOperation.op);
                }
            }
            //relatively forget the event, as it was used, and add back to events
            e->attention = Attention_forgetEvent(&e->attention, currentTime);
            Memory_addEvent(e);
            //activate concepts attention with the event's attention
            c->attention = Attention_activateConcept(&c->attention, &e->attention); 
            PriorityQueue_IncreasePriority(&concepts,closest_concept_i, c->attention.priority); //priority was increased
        }
        else
        {
            assert(concepts.itemsAmount == 0, "No matching concept is only allowed to happen if memory is empty.");
        }
        if(!consideredOperation.matched) //(&/,a,op()) events don't form concepts, they are revised in a
        {
            //add a new concept for e too at the end, just before it needs to be identified with something existing
            bool duplicate = false; //todo improve
            for(int i=0; i<concepts.itemsAmount; i++)
            {
                Concept *existing = concepts.items[i].address;
                //SDR_PrintWhereTrue(&existing->sdr);
                if(existing->sdr_hash == e->sdr_hash)
                {
                    if(SDR_Equal(&existing->sdr, &e->sdr))
                    {
                        IN_DEBUG( printf("EXISTING CONCEPT \n"); )
                        duplicate = true;
                        break;
                    }
                }
            }
            if(!duplicate)
            {
                Concept *eNativeConcept = Memory_addConcept(&e->sdr, e->attention);
                if(eNativeConcept == NULL)
                {
                    IN_DEBUG( printf("ADDING CONCEPT FAILED"); )
                }
                else
                {
                    IN_DEBUG( printf("ADDED CONCEPT \n"); )
                    FIFO_Add(e, (e->type == EVENT_TYPE_BELIEF ? &eNativeConcept->event_beliefs : &eNativeConcept->event_goals));
                }
                
            }
        }
        PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&events, e->attention.priority);
        if(feedback.added)
        {
            Event *added = feedback.addedItem.address;
            *added = *e;
        }
    }
    /*for(int j=0; j<eventsSelected; j++)
    {   //push again what we popped
        PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&events, selectedItem[j].priority);
        if(feedback.added)
        {
            *((Concept*) feedback.addedItem.address) = *((Concept*) selectedItem[j].address);
        }
    }*/
}
