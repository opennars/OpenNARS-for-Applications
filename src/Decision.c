#include "Decision.h"

Decision Decision_PotentiallyExecute(Concept *c, Event *goal, long currentTime)
{
    Decision result = (Decision) {0};
    Event G = *goal;
    if(goal->operationID > 0)
    {
        Operation op = operations[goal->operationID-1];
        if(op.action == 0)
        {
            return result;
        }
        result.matched = true;
        result.op = op;
        G.truth = Truth_Projection(G.truth, G.occurrenceTime, currentTime);
        G.occurrenceTime = currentTime;
        Event b = FIFO_GetHighestConfidentProjectedTo(&c->event_beliefs, currentTime, &goal->sdr).projectedEvent;
        if(Truth_Expectation(Truth_Deduction(G.truth, b.truth)) > DECISION_THRESHOLD)
        {
            result.executed = true;
            printf("!!!!!!!!!!!!!!!ANSNA TAKING ACTIVE CONTROL!!!!!!!!!!!!!!!!!!!!\n");
            (*op.action)();
            result.operationID = goal->operationID;
        }
    }
    return result;
}

Decision Decision_MotorBabbling()
{
    Decision result = (Decision) {0};
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
        int chosen = rand() % n_ops;
        result.op = operations[chosen];
        (*result.op.action)();
        result.executed = true;
        result.matched = true;
        result.operationID = chosen+1;
    }
    return result;
}

void Decision_MotorTagging(Concept *c, int opID)
{
    Event ev = FIFO_GetNewestElement(&c->event_beliefs);
    ev.operationID = opID;
    FIFO_AddAndRevise(&ev, &c->event_beliefs);
}
