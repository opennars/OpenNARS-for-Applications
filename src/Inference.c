#include "Inference.h"
#include "SDR.h"

//{Event task a., Event belief b.} |- Derived event task (&/,a,b).
Task Inference_BeliefEventIntersection(Task *a, Task *b)
{
    TruthValue conclusionTruth = intersection(a->truth, b->truth);
    SDR sdr = SDR_Tuple(&a->sdr,&b->sdr);
    Stamp stamp = {0};
    Task dummy = { .sdr = sdr, .type = JUDGMENT, .truth = conclusionTruth, .stamp = stamp, .priority = 0 };
    return dummy;
}

//{Event task a., Event belief b.} |- Precondition and Postcondition belief <a =/> c>.
Task Inference_BeliefInduction(Task *subject, Task *predicate)
{
    TruthValue conclusionTruth = induction(subject->truth, predicate->truth);
    SDR sdr = SDR_Tuple(&subject->sdr, &predicate->sdr);
    Stamp stamp = {0};
    Task dummy = { .sdr = sdr, .type = JUDGMENT, .truth = conclusionTruth, .stamp = stamp, .priority = 0 };
    return dummy;
}

//{Precondition or Postcondition belief a., Precondition or Postcondition belief a.} |- 
// Precondition or Postcondition belief a.
Task Inference_BeliefRevision(Task *a, Task *b)
{
    TruthValue conclusionTruth = revision(a->truth, b->truth);
    SDR sdr = a->sdr;
    Stamp stamp = {0};
    Task dummy = { .sdr = sdr, .type = JUDGMENT, .truth = conclusionTruth, .stamp = stamp, .priority = 0 };
    return dummy;
}

//{Event task a., Postcondition belief <a =/> b>.} |- Derived event task b.
Task Inference_BeliefEventDeduction(Task *component, Task *compound)
{
    TruthValue conclusionTruth = deduction(compound->truth, component->truth);
    SDR sdr = SDR_TupleGetSecondElement(&compound->sdr,&component->sdr);
    Stamp stamp = {0};
    Task dummy = { .sdr = sdr, .type = JUDGMENT, .truth = conclusionTruth, .stamp = stamp, .priority = 0 };
    return dummy;
}

//{Event task b!, Postcondition belief <a =/> b>.} |- Derived event task a!
Task Inference_GoalEventDeduction(Task *component, Task *compound)
{
    TruthValue conclusionTruth = deduction(compound->truth, component->truth);
    SDR sdr = SDR_TupleGetFirstElement(&compound->sdr,&component->sdr);
    Stamp stamp = {0};
    Task dummy = { .sdr = sdr, .type = JUDGMENT, .truth = conclusionTruth, .stamp = stamp, .priority = 0 };
    return dummy;
}

//{Event task b., Postcondition belief <a =/> b>.} |- Derived event task a.
Task Inference_BeliefEventAbduction(Task *component, Task *compound)
{
    TruthValue conclusionTruth = abduction(compound->truth, component->truth);
    SDR sdr = SDR_TupleGetFirstElement(&compound->sdr,&component->sdr);
    Stamp stamp = {0};
    Task dummy = { .sdr = sdr, .type = JUDGMENT, .truth = conclusionTruth, .stamp = stamp, .priority = 0 };
    return dummy;
}

//{Event task a!, Precondition belief <a =/> b>.} |- Derived event task b!
Task Inference_GoalEventAbduction(Task *component, Task *compound)
{
    TruthValue conclusionTruth = abduction(compound->truth, component->truth);
    SDR sdr = SDR_TupleGetSecondElement(&compound->sdr,&component->sdr);
    Stamp stamp = {0};
    Task dummy = { .sdr = sdr, .type = GOAL, .truth = conclusionTruth, .stamp = stamp, .priority = 0 };
    return dummy;
}
