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

int stampID = -1;
Decision Decision_RealizeGoal(Event *goal, long currentTime)
{
    Decision decision = (Decision) {0};
    int closest_postc_i;
    if(Memory_getClosestConcept(&goal->sdr, goal->sdr_hash, &closest_postc_i))
    {
        Concept *postc = concepts.items[closest_postc_i].address;
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
                Relink_Implication(&postc->precondition_beliefs[opi].array[j]);
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
        printf("decision expectation %f impTruth=(%f, %f): %s future=%ld\n", bestTruthExpectation, bestImp.truth.frequency, bestImp.truth.confidence, bestImp.debug, bestImp.occurrenceTimeOffset);
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

void Decision_AssumptionOfFailure(Decision *decision)
{
    for(int j=0; j<concepts.itemsAmount; j++)
    {
        Concept *postc = concepts.items[j].address;
        for(int  h=0; h<postc->precondition_beliefs[decision->operationID].itemsAmount; h++)
        {
            Relink_Implication(&postc->precondition_beliefs[decision->operationID].array[h]);
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
                    //"compute amount of negative evidence based on current evidence" (Robert's estimation)
                    //"we just take the counter and don't add one because we want to compute a w "unit" which will be revised"
                    long countWithNegativeEvidence = imp.revisions;
                    double negativeEvidenceRatio = 1.0 / ((double) countWithNegativeEvidence);
                    //compute confidence by negative evidence
                    double w = Truth_c2w(imp.truth.confidence) * negativeEvidenceRatio;
                    double c = Truth_w2c(w);
                    Implication negative_confirmation = imp;
                    negative_confirmation.truth = (Truth) { .frequency = 0.0, .confidence = c };
                    negative_confirmation.stamp = (Stamp) { .evidentalBase = { -stampID } };
                    IN_DEBUG ( printf("ANTICIPATE %s, future=%ld \n", imp.debug, imp.occurrenceTimeOffset); )
                    //assumption of failure
                    Implication *added = Table_AddAndRevise(&postc->precondition_beliefs[decision->operationID], &negative_confirmation, negative_confirmation.debug);
                    if(added != NULL)
                    {
                        added->sourceConcept = negative_confirmation.sourceConcept;
                        added->sourceConceptSDR = negative_confirmation.sourceConceptSDR;
                    }                                
                    stampID--;
                }
            }
        }
    }
}

bool Decision_Making(Event *goal, long currentTime)
{
    Decision decision = {0};
    //try motor babbling with a certain chance
    if(!decision.execute && rand() % 1000000 < (int)(MOTOR_BABBLING_CHANCE*1000000.0))
    {
        decision = Decision_MotorBabbling();
    }
    //try matching op if didn't motor babble
    if(!decision.execute)
    {
        decision = Decision_RealizeGoal(goal, currentTime);
    }
    if(decision.execute && decision.operationID)
    {
        Decision_InjectActionEvent(&decision);
    }
    if(decision.execute)
    {
        Decision_AssumptionOfFailure(&decision);
    }
    return decision.execute;
}
