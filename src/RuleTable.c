#include "RuleTable.h"

void RuleTable_Composition(long currentTime, Event *a, Event *b, int operationID)
{   
    if(a->type != EVENT_TYPE_BELIEF || b->type != EVENT_TYPE_BELIEF)
    {
        return;
    }
    int AConceptIndex;
    int BConceptIndex;
    if(Memory_getClosestConcept(&a->sdr, a->sdr_hash, &AConceptIndex) &&
       Memory_getClosestConcept(&b->sdr, b->sdr_hash, &BConceptIndex))
    {
        Concept *A = concepts.items[AConceptIndex].address;
        Concept *B = concepts.items[BConceptIndex].address;
        if(A != B)
        {
            //temporal induction and intersection
            if(!Stamp_checkOverlap(&a->stamp, &b->stamp))
            {
                Implication precondition_implication =   b->occurrenceTime > a->occurrenceTime ? Inference_BeliefInduction(a, b, false) : Inference_BeliefInduction(b, a, false);
                Implication postcondition_implication =  b->occurrenceTime > a->occurrenceTime ? Inference_BeliefInduction(a, b, true)  : Inference_BeliefInduction(b, a, true);
                if(precondition_implication.truth.confidence >= MIN_CONFIDENCE) //has same truth as postcon, just different SDR
                {
                    char debug[50];
                    sprintf(debug, "<(&/,%s,%d) =/> %s>.",a->debug, operationID, b->debug); //++
                    sprintf(debug, "<(&/,%s,%d) =/> %s>.",a->debug, operationID, b->debug); //++
                    IN_DEBUG ( if(operationID != 0) { puts(debug); Truth_Print(&precondition_implication.truth); puts("\n"); getchar(); } )
                    IN_OUTPUT( printf("Formed (pre- and post-condition) implication: "); Implication_Print(&postcondition_implication); Implication_Print(&precondition_implication); )
                    Implication revised_precon = Table_AddAndRevise(&B->precondition_beliefs[operationID], &precondition_implication, debug);
                    IN_OUTPUT( if(revised_precon.sdr_hash != 0) { printf("REVISED pre-condition implication: "); Implication_Print(&revised_precon); } )
                    Implication revised_postcon = Table_AddAndRevise(&A->postcondition_beliefs[operationID], &postcondition_implication, debug);
                    IN_OUTPUT( if(revised_postcon.sdr_hash != 0) { printf("REVISED post-condition implication: "); Implication_Print(&revised_postcon); } )
                }
                Event sequence = b->occurrenceTime > a->occurrenceTime ? Inference_BeliefIntersection(a, b) : Inference_BeliefIntersection(b, a);
                sequence.attention = Attention_deriveEvent(&B->attention, &a->truth, currentTime);
                if(sequence.truth.confidence < MIN_CONFIDENCE || sequence.attention.priority < MIN_PRIORITY)
                {
                    return;
                }
                //derivations[eventsDerived++] = sequence; //not yet
                IN_OUTPUT( printf("COMPOSED SEQUENCE EVENT: "); Event_Print(&sequence); )
            }
        }
    }
}

/*void RuleTable_Decomposition(Concept *c, Event *e, long currentTime)
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
                if(res.type == EVENT_TYPE_GOAL && !ALLOW_ABDUCTION)
                {
                    continue;
                }
                res.attention = Attention_deriveEvent(&c->attention, &postcon.truth, currentTime);
                if(res.truth.confidence < MIN_CONFIDENCE || res.attention.priority < MIN_PRIORITY)
                {
                    continue;
                }
                derivations[eventsDerived++] = res;
                //add negative evidence to the used predictive hypothesis (assumption of failure, for extinction)
                Table_PopHighestTruthExpectationElement(&c->postcondition_beliefs);
                Implication updated = Inference_AssumptionOfFailure(&postcon);
                Table_Add(&c->postcondition_beliefs, &updated);
                k++;
                IN_OUTPUT( printf("DETACHED FORWARD EVENT: "); Event_Print(&res); )
                if(k>MAX_FORWARD)
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
                if(res.type == EVENT_TYPE_BELIEF && !ALLOW_ABDUCTION)
                {
                    continue;
                }
                res.attention = Attention_deriveEvent(&c->attention, &precon.truth, currentTime);
                if(res.truth.confidence < MIN_CONFIDENCE || res.attention.priority < MIN_PRIORITY)
                {
                    continue;
                }
                derivations[eventsDerived++] = res;
                k++;
                IN_OUTPUT( printf("DETACHED BACKWARD EVENT: "); Event_Print(&res); )
                if(k>MAX_BACKWARD)
                {
                    break;
                }
            }
        }
    }
}*/
