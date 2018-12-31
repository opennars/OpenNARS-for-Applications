#include "Decision.h"

Decision Decision_PotentiallyExecute(Concept *c, Event *goal, long currentTime)
{
    Decision result = (Decision) {0};
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
    }
    return result;
}

void Decision_MotorTagging(Concept *c, Operation op)
{
    Event ev = FIFO_GetNewestElement(&c->event_beliefs);
    ev.sdr = SDR_Union(&ev.sdr, &op.sdr);
    FIFO_AddAndRevise(&ev, &c->event_beliefs);
}
