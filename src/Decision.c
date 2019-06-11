#include "Decision.h"

//Inject action event after execution or babbling
void InjectActionEvent(int opID)
{
    assert(opID != 0, "Operation 0 is reserved for no action");
    Operation op = operations[opID];
    ANSNA_AddInputBelief(op.sdr);
}

//"reflexes" to try different operations, especially important in the beginning
Decision MotorBabbling()
{
    Decision result = (Decision) {0};
    int n_ops = 0;
    for(int i=0; i<OPERATIONS_MAX && operations[i].action != 0; i++)
    {
        n_ops = i;
    }
    if(n_ops++ > 0)
    {
        int chosen = (rand() % n_ops);
        result.op = operations[chosen];
        (*result.op.action)();
        result.executed = true;
        result.matched = true;
        result.operationID = chosen+1;
        InjectActionEvent(chosen+1);
    }
    return result;
}

Decision RealizeGoal(Event *goal, long currentTime)
{
    Decision result = (Decision) {0};
    //TODO goal realization
    //Plan through knowledge
    //Select an action realizing goal
    //invoke InjectActionEvent
    return result;
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
        InjectActionEvent(decision.operationID);
    }
    return decision.matched;
}
