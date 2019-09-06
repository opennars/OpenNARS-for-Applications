#include "Decision.h"

double MOTOR_BABBLING_CHANCE = MOTOR_BABBLING_CHANCE_INITIAL;
//Inject action event after execution or babbling
static void Decision_InjectActionEvent(Decision *decision)
{
    assert(decision->operationID > 0, "Operation 0 is reserved for no action");
    decision->op = operations[decision->operationID-1]; //TODO put into InjectActionEvent
    (*decision->op.action)();
    //and add operator feedback
    ANSNA_AddInputBelief(decision->op.sdr);
}

//"reflexes" to try different operations, especially important in the beginning
static Decision Decision_MotorBabbling()
{
    Decision decision = (Decision) {0};
    int n_ops = 0;
    for(int i=0; i<OPERATIONS_MAX && operations[i].action != 0; i++)
    {
        n_ops = i+1;
    }
    if(n_ops > 0)
    {
        decision.operationID = 1+(rand() % (n_ops));
        IN_DEBUG (
            printf(" ANSNA BABBLE %d\n", decision.operationID);
        )
        decision.execute = true;
    }
    return decision;
}

void Relink_Implication(int layer, Implication *imp)
{
    if(imp->sourceConceptSDRHash != ((Concept*) &imp->sourceConcept)->sdr_hash && !SDR_Equal(&imp->sourceConceptSDR, &((Concept*) &imp->sourceConcept)->sdr))
    {
        int closest_concept_i;
        if(Memory_getClosestConcept(layer, &imp->sourceConceptSDR, SDR_Hash(&imp->sourceConceptSDR), &closest_concept_i))
        {
            imp->sourceConcept = concepts[layer].items[closest_concept_i].address;
            imp->sourceConceptSDR = ((Concept*) imp->sourceConcept)->sdr;
            imp->sourceConceptSDRHash = SDR_Hash(&imp->sourceConceptSDR);
        }
        else
        {
            assert(false, "No concept to re-link to, call the ghostbusters!\n");
        }
    }
}

int stampID = -1;
Decision Decision_RealizeGoal(int layer, Event *goal, long currentTime)
{
    Decision decision = (Decision) {0};
    int closest_postc_i;
    if(Memory_getClosestConcept(layer, &goal->sdr, goal->sdr_hash, &closest_postc_i))
    {
        Concept *postc = concepts[layer].items[closest_postc_i].address;
        double bestTruthExpectation = 0;
        Implication bestImp = {0};
        Concept *prec;
        for(int opi=1; opi<OPERATIONS_MAX; opi++)
        {
            if(operations[opi-1].action == 0)
            {
                break;
            }
            for(int j=0; j<postc->precondition_beliefs[opi].itemsAmount; j++)
            {
                Relink_Implication(layer, &postc->precondition_beliefs[opi].array[j]);
                Implication imp = postc->precondition_beliefs[opi].array[j];
                IN_DEBUG
                (
                    printf("CONSIDERED IMPLICATION: %s\n", imp.debug);
                    SDR_Print(&imp.sdr);
                )
                //now look at how much the precondition is fulfilled
                Concept *current_prec = imp.sourceConcept;
                Event *precondition = &current_prec->belief_spike; //a. :|:
                if(precondition != NULL)
                {
                    Event ContextualOperation = Inference_GoalDeduction(goal, &imp); //(&/,a,op())! :\:
                    double operationGoalTruthExpectation = Truth_Expectation(Inference_OperationDeduction(&ContextualOperation, precondition, currentTime).truth); //op()! :|:
                    if(operationGoalTruthExpectation > bestTruthExpectation)
                    {
                        IN_DEBUG
                        (
                            printf("CONSIDERED PRECON: %s\n", current_prec->debug);
                            fputs("CONSIDERED PRECON truth ", stdout);
                            Truth_Print(&precondition->truth);
                            fputs("CONSIDERED goal truth ", stdout);
                            Truth_Print(&goal->truth);
                            fputs("CONSIDERED imp truth ", stdout);
                            Truth_Print(&imp.truth);
                            printf("CONSIDERED time %ld\n", precondition->occurrenceTime);
                            SDR_Print(&current_prec->sdr);
                            SDR_Print(&precondition->sdr);
                        )
                        prec = current_prec;
                        bestImp = imp;
                        decision.operationID = opi;
                        bestTruthExpectation = operationGoalTruthExpectation;
                    }
                }
            }
        }
        if(bestTruthExpectation < DECISION_THRESHOLD)
        {
            return decision;
        }
        printf("decision layer%d expectation %f impTruth=(%f, %f): %s future=%ld maxFuture=%ld\n", layer, bestTruthExpectation, bestImp.truth.frequency, bestImp.truth.confidence, bestImp.debug, bestImp.occurrenceTimeOffset, bestImp.maxOccurrenceTimeOffset);
        IN_DEBUG
        (
            printf("%s %f,%f",bestImp.debug, bestImp.truth.frequency, bestImp.truth.confidence);
            puts("");
            printf("SELECTED PRECON: %s\n", prec->debug);
            puts(bestImp.debug); //++
            printf(" ANSNA TAKING ACTIVE CONTROL %d\n", decision.operationID);
        )
        decision.execute = true;

    }
    return decision;
}

void Decision_Anticipate(Decision *decision)
{
    //ANTICIPATON (neg. evidence numbers for now)
    for(int l=0; l<CONCEPT_LAYERS; l++)
    {
        for(int j=0; j<concepts[l].itemsAmount; j++)
        {
            Concept *postc = concepts[l].items[j].address;
            for(int  h=0; h<postc->precondition_beliefs[decision->operationID].itemsAmount; h++)
            {
                Relink_Implication(l, &postc->precondition_beliefs[decision->operationID].array[h]);
                Implication imp = postc->precondition_beliefs[decision->operationID].array[h]; //(&/,a,op) =/> b.
                Concept *current_prec = imp.sourceConcept;
                Event *precondition = &current_prec->belief_spike; //a. :|:
                if(precondition != NULL)
                {
                    Event op = (Event) { .sdr = operations[decision->operationID-1].sdr,
                                         .type = EVENT_TYPE_BELIEF,
                                         .truth = { .frequency = 1.0, .confidence = 0.9 },
                                         .occurrenceTime = currentTime,
                                         .operationID = decision->operationID };
                    Event seqop = Inference_BeliefIntersection(precondition, &op); //(&/,a,op). :|:
                    Event result = Inference_BeliefDeduction(&seqop, &imp); //b. :/:
                    if(Truth_Expectation(result.truth) > ANTICIPATION_THRESHOLD)
                    {
                        for(int i=0; i<ANTICIPATIONS_MAX; i++)
                        {
                            if(postc->anticipation_deadline[i] == 0)
                            {
                                //"compute amount of negative evidence based on current evidence" (Robert's estimation)
                                //"we just take the counter and don't add one because we want to compute a w "unit" which will be revised"
                                long countWithNegativeEvidence = imp.revisions;
                                double negativeEvidenceRatio = 1.0 / ((double) countWithNegativeEvidence);
                                //compute confidence by negative evidence
                                double w = Truth_c2w(imp.truth.confidence) * negativeEvidenceRatio;
                                double c = Truth_w2c(w);
                                //deadline: predicted time + tolerance
                                postc->anticipation_deadline[i] = currentTime + imp.maxOccurrenceTimeOffset;
                                postc->anticipation_negative_confirmation[i] = imp;
                                //assert(c > 0, "hmm conf should be >0");
                                postc->anticipation_negative_confirmation[i].truth = (Truth) { .frequency = 0.0, .confidence = c };
                                postc->anticipation_negative_confirmation[i].stamp = (Stamp) { .evidentalBase = { -stampID } };
                                postc->anticipation_negative_confirmation[i].occurrenceTimeOffset = imp.occurrenceTimeOffset;
                                postc->anticipation_negative_confirmation[i].maxOccurrenceTimeOffset = imp.maxOccurrenceTimeOffset;
                                postc->anticipation_operation_id[i] = decision->operationID;
                                IN_DEBUG ( printf("ANTICIPATE %s, future=%ld maxfuture=%ld layer=%d\n", imp.debug, imp.occurrenceTimeOffset, imp.maxOccurrenceTimeOffset, l); )
                                //puts(postc->debug);
                                //puts("");
                                //getchar();
                                stampID--;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    //EMD anticipation
}

bool Decision_Making(int layer, Event *goal, long currentTime)
{
    Decision decision = {0};
    //try motor babbling with a certain chance
    if(layer==0 && !decision.execute && rand() % 1000000 < (int)(MOTOR_BABBLING_CHANCE*1000000.0))
    {
        decision = Decision_MotorBabbling();
    }
    //try matching op if didn't motor babble
    if(!decision.execute)
    {
        decision = Decision_RealizeGoal(layer, goal, currentTime);
    }
    if(decision.execute && decision.operationID)
    {
        Decision_InjectActionEvent(&decision);
    }
    if(decision.execute)
    {
        Decision_Anticipate(&decision);
    }
    return decision.execute;
}
