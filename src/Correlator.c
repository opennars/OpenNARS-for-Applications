#include "Correlator.h"

static int stampID = -1;
static void Correlator_PenalizeLinks(int operationID, long currentTime)
{
    assert(operationID >= 0 && operationID <= OPERATIONS_MAX, "Wrong operation id, did you inject an event manually?");
    for(int j=0; j<concepts.itemsAmount; j++)
    {
        Concept *postc = concepts.items[j].address;
        for(int  h=0; h<postc->precondition_beliefs[operationID].itemsAmount; h++)
        {
            if(!Memory_ImplicationValid(&postc->precondition_beliefs[operationID].array[h]))
            {
                Table_Remove(&postc->precondition_beliefs[operationID], h);
                h--;
                continue;
            }
            Implication imp = postc->precondition_beliefs[operationID].array[h]; //(&/,a,op) =/> b.
            Concept *current_prec = imp.sourceConcept;
            Event *precondition = &current_prec->belief_spike; //a. :|:
            if(precondition != NULL && precondition->type != EVENT_TYPE_DELETED)
            {
                assert(precondition->occurrenceTime != OCCURRENCE_ETERNAL, "Precondition should not be eternal!");
                Event updated_precondition = Inference_EventUpdate(precondition, currentTime);
                Event op = { .type = EVENT_TYPE_BELIEF,
                             .truth = (Truth) { .frequency = 1.0, .confidence = 0.9 },
                             .occurrenceTime = currentTime };
                bool success;
                Event seqop = Inference_BeliefIntersection(&updated_precondition, &op, &success); //(&/,a,op). :|:
                if(success)
                {
                    Event result = Inference_BeliefDeduction(&seqop, &imp); //b. :/:
                    if(Truth_Expectation(result.truth) > ANTICIPATION_THRESHOLD)
                    {
                        Implication negative_confirmation = imp;
                        Truth TNew = { .frequency = 0.0, .confidence = ANTICIPATION_CONFIDENCE };
                        Truth TPast = Truth_Projection(precondition->truth, 0, imp.occurrenceTimeOffset);
                        negative_confirmation.truth = Truth_Eternalize(Truth_Induction(TNew, TPast));
                        negative_confirmation.stamp = (Stamp) { .evidentalBase = { -stampID } };
                        assert(negative_confirmation.truth.confidence >= 0.0 && negative_confirmation.truth.confidence <= 1.0, "(666) confidence out of bounds");
                        Implication *added = Table_AddAndRevise(&postc->precondition_beliefs[operationID], &negative_confirmation);
                        if(added != NULL)
                        {
                            added->sourceConcept = negative_confirmation.sourceConcept;
                            added->sourceConceptId = negative_confirmation.sourceConceptId;
                        }                                
                        stampID--;
                    }
                }
            }
        }
    }
}

//Reinforce link between concept a and b (creating it if non-existent)
static void Correlator_ReinforceLink(Event *a, Event *b, long currentTime)
{
    assert(a->type == EVENT_TYPE_BELIEF && b->type == EVENT_TYPE_BELIEF, "Correlator_ReinforceLink called on wrong event type");
    Term a_term_nop = Narsese_GetPreconditionWithoutOp(&a->term);
    if(!Stamp_checkOverlap(&a->stamp, &b->stamp))
    {
        bool success;
        Implication precondition_implication = Inference_BeliefInduction(a, b, &success); //temporal induction
        if(success)
        {
            if(precondition_implication.truth.confidence >= MIN_CONFIDENCE)
            {
                bool success2;
                Term general_implication_term = IntroduceImplicationVariables(precondition_implication.term, &success2);
                if(success2 && Variable_hasVariable(&general_implication_term, true, true, false))
                {
                    NAL_DerivedEvent(general_implication_term, OCCURRENCE_ETERNAL, precondition_implication.truth, precondition_implication.stamp, currentTime, 1, 1, precondition_implication.occurrenceTimeOffset, NULL, 0);
                }
                NAL_DerivedEvent(precondition_implication.term, OCCURRENCE_ETERNAL, precondition_implication.truth, precondition_implication.stamp, currentTime, 1, 1, precondition_implication.occurrenceTimeOffset, NULL, 0);
            }
        }
    }
}

void Correlator_CorrelateEvents(long currentTime)
{
    //1. process newest event
    if(belief_events.itemsAmount > 0)
    {
        //form concepts for the sequences of different length
        for(int len=0; len<MAX_SEQUENCE_LEN; len++)
        {
            Event *toProcess = FIFO_GetNewestSequence(&belief_events, len);
            if(toProcess != NULL && !toProcess->processed && toProcess->type != EVENT_TYPE_DELETED)
            {
                toProcess->processed = true;
                assert(toProcess->type == EVENT_TYPE_BELIEF, "A different event type made it into belief events!");
                Memory_ProcessBeliefEvent(toProcess, currentTime, 1.0, 0, false, true, false, false);
                Event postcondition = *toProcess;
                //Mine for <(&/,precondition,operation) =/> postcondition> patterns in the FIFO:
                if(len == 0) //postcondition always len1
                {
                    int op_id = Narsese_getOperationID(&postcondition.term);
                    Correlator_PenalizeLinks(op_id, currentTime);
                    //build link between internal derivations and external event to explain it:
                    for(int k=0; k<beliefsSelectedCnt; k++)
                    {
                        if(selectedBeliefs[k].occurrenceTime < postcondition.occurrenceTime)
                        {
                            Correlator_ReinforceLink(&selectedBeliefs[k], &postcondition, currentTime);
                        }
                    }
                    for(int k=1; k<belief_events.itemsAmount; k++)
                    {
                        for(int len2=0; len2<MAX_SEQUENCE_LEN; len2++)
                        {
                            Event *precondition = FIFO_GetKthNewestSequence(&belief_events, k, len2);
                            if(len2 > 0)
                            {
                                Event *potential_op = FIFO_GetKthNewestSequence(&belief_events, k+len2, 0);
                                if(potential_op != NULL && potential_op->type != EVENT_TYPE_DELETED && Narsese_isOperation(&potential_op->term))
                                {
                                    break;
                                }
                            }
                            if(precondition != NULL && precondition->type != EVENT_TYPE_DELETED)
                            {
                                Correlator_ReinforceLink(precondition, &postcondition, currentTime);
                            }
                        }
                    }
                }
            }
        }
    }
}
