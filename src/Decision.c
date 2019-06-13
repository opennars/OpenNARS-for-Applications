#include "Decision.h"

//Inject action event after execution or babbling
void InjectActionEvent(Decision *decision)
{
    assert(decision->operationID > 0, "Operation 0 is reserved for no action");
    decision->op = operations[decision->operationID-1]; //TODO put into InjectActionEvent
    (*decision->op.action)();
    decision->executed = true;
    decision->matched = true;
    Operation op = operations[decision->operationID];
    ANSNA_AddInputBelief(op.sdr);
}

//"reflexes" to try different operations, especially important in the beginning
Decision MotorBabbling()
{
    Decision decision = (Decision) {0};
    int n_ops = 0;
    for(int i=0; i<OPERATIONS_MAX && operations[i].action != 0; i++)
    {
        n_ops = i;
    }
    if(n_ops > 0)
    {
        decision.operationID = 1+(rand() % (n_ops+1));
        InjectActionEvent(&decision); //end TODO
    }
    return decision;
}

Decision RealizeGoal(Event *goal, long currentTime)
{
    Decision decision = (Decision) {0};
    int closest_postcon_concept_i;
    if(Memory_getClosestConcept(&goal->sdr, goal->sdr_hash, &closest_postcon_concept_i))
    {
        Concept *postcon_c = concepts.items[closest_postcon_concept_i].address;
        double bestTruthExpectation = 0;
        printf("Realize goal called 1\n"); //++
        for(int i=1; i<OPERATIONS_MAX; i++)
        {
            
            if(operations[i-1].action == 0)
            {
                printf("break i=%d\n",i); //++
                break;
            }
            //printf("i=%d\n",i); //++
            for(int j=0; j<postcon_c->precondition_beliefs[i].itemsAmount; j++)
            {
                //printf("BREAKP\n");exit(0); //++
                Implication imp = postcon_c->precondition_beliefs[i].array[j];
                //now look at how much the precondition is fulfilled
                int closest_precon_concept_i;
                if(Memory_getClosestConcept(&imp.sdr, imp.sdr_hash, &closest_precon_concept_i))
                {
                    Concept *precon_c = concepts.items[closest_precon_concept_i].address;
                    FIFO_Query_Result result = FIFO_GetHighestConfidentProjectedTo(&precon_c->event_beliefs, goal->occurrenceTime-imp.occurrenceTimeOffset, &goal->sdr); //a. :|:
                    Event ContextualOperation = Inference_GoalDeduction(goal, &imp); //(&/,a,op())!
                    double operationGoalTruthExpectation = Truth_Expectation(Truth_Deduction(ContextualOperation.truth, result.projectedEvent.truth)); //op()!
                    if(operationGoalTruthExpectation > bestTruthExpectation)
                    {
                        decision.operationID = i;
                        //assert(imp.operationID == i, "Index must agree to what the implication states");
                        bestTruthExpectation = operationGoalTruthExpectation;
                        printf("TEST %d\n", decision.operationID); //++
                    }
                }
            }
            
            
            /*FIX:
            //printf("Having knowledge for action %d\n", i);
            Implication imp = Table_PopHighestTruthExpectationElement(&c->precondition_beliefs[i]);
            int closest_concept_i;
            if(Memory_getClosestConcept(goal, &closest_concept_i))
            
            
            
            
            double currentTruthExpectation = Truth_Expectation(imp.truth);
            if(currentTruthExpectation > bestTruthExpectation)
            {
                decision.operationID = imp.operationID;
                bestTruthExpectation = currentTruthExpectation;
                printf("TEST");
            }*/
        }
        if(decision.operationID == 0)
        {
            printf("Best operation ID 0\n"); //++
            return decision;
        }
        InjectActionEvent(&decision); //end TODO */
    }
    return decision;
}

bool Decision_Making(Event *goal, long currentTime)
{
    Decision decision = {0};
    decision = RealizeGoal(goal, currentTime);
    //if no operation matched, try motor babbling with a certain chance
    if(!decision.matched && !decision.executed && rand() % 1000000 < (int)(MOTOR_BABBLING_CHANCE*1000000.0))
    {
        decision = MotorBabbling();
    }
    if(decision.executed)
    {
        InjectActionEvent(&decision);
    }
    return decision.matched;
}
