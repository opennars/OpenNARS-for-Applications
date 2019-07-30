#include "Inference.h"
#include "SDR.h"

#define DERIVATION_STAMP(a,b) Stamp conclusionStamp = Stamp_make(&a->stamp, &b->stamp);
#define DERIVATION_STAMP_AND_TIME(a,b) DERIVATION_STAMP(a,b) \
                long conclusionTime = (a->occurrenceTime + b->occurrenceTime)/2.0; \
                Truth truthA = Truth_Projection(a->truth, a->occurrenceTime, conclusionTime); \
                Truth truthB = Truth_Projection(b->truth, b->occurrenceTime, conclusionTime);
                
static double weighted_average(double a1, double a2, double w1, double w2)
{
    return (a1*w1+a2*w2)/(w1+w2);
}
                
//{Event a., Event b.} |- Event (&/,a,b).
Event Inference_BeliefIntersection(Event *a, Event *b)
{
    DERIVATION_STAMP_AND_TIME(a,b)
    return (Event) { .sdr = SDR_Tuple(&a->sdr,&b->sdr),
                     .type = EVENT_TYPE_BELIEF,
                     .truth = Truth_Intersection(truthA, truthB),
                     .stamp = conclusionStamp, 
                     .occurrenceTime = conclusionTime };
}

//{Event a., Event b., after(b,a)} |- Implication <a =/> b>.
Implication Inference_BeliefInduction(Event *a, Event *b)
{
    assert(b->occurrenceTime > a->occurrenceTime, "after(b,a) violated in Inference_BeliefInduction");
    DERIVATION_STAMP_AND_TIME(a,b)
    return  (Implication) { .sdr = a->sdr, 
                            .truth = Truth_Eternalize(Truth_Induction(truthA, truthB)),
                            .stamp = conclusionStamp,
                            .revisions = 1,
                            .occurrenceTimeOffset = b->occurrenceTime - a->occurrenceTime,
                            .minOccurrenceTimeOffset = b->occurrenceTime - a->occurrenceTime,
                            .maxOccurrenceTimeOffset = b->occurrenceTime - a->occurrenceTime };
}

//{Event a., Event a.} |- Event a.
//{Event a!, Event a!} |- Event a!
Event Inference_EventRevision(Event *a, Event *b)
{
    DERIVATION_STAMP_AND_TIME(a,b)
    return (Event) { .sdr = a->sdr, 
                     .type = a->type,
                     .truth = Truth_Revision(truthA, truthB),
                     .stamp = conclusionStamp, 
                     .occurrenceTime = conclusionTime };
}

//{Implication <a =/> b>., <a =/> b>.} |- Implication <a =/> b>.
Implication Inference_ImplicationRevision(Implication *a, Implication *b)
{
    DERIVATION_STAMP(a,b)
    double occurrenceTimeOffsetAvg = weighted_average(a->occurrenceTimeOffset, b->occurrenceTimeOffset, Truth_c2w(a->truth.confidence), Truth_c2w(b->truth.confidence));
    Implication ret = (Implication) { .sdr = a->sdr,
                                      .truth = Truth_Projection(Truth_Revision(a->truth, b->truth), a->occurrenceTimeOffset, b->occurrenceTimeOffset),
                                      .stamp = conclusionStamp, 
                                      .revisions = a->revisions + b->revisions,
                                      .occurrenceTimeOffset = occurrenceTimeOffsetAvg,
                                      .minOccurrenceTimeOffset = MIN(a->minOccurrenceTimeOffset, b->minOccurrenceTimeOffset),
                                      .maxOccurrenceTimeOffset = MAX(a->maxOccurrenceTimeOffset, b->maxOccurrenceTimeOffset) };
    strcpy(ret.debug, a->debug);
    return ret;
}

//{Event b!, Implication <a =/> b>.} |- Event a!
Event Inference_GoalDeduction(Event *component, Implication *compound)
{
    DERIVATION_STAMP(component,compound)
    return (Event) { .sdr = compound->sdr, 
                     .type = EVENT_TYPE_GOAL, 
                     .truth = Truth_Deduction(compound->truth, component->truth),
                     .stamp = conclusionStamp, 
                     .occurrenceTime = component->occurrenceTime /*"to be realized ASAP, so not - compound->occurrenceTimeOffset*/ };
}

//{Event a.} |- Event a. updated to currentTime
Event Inference_EventUpdate(Event *ev, long currentTime)
{
    Event ret = *ev;
    ret.truth = Truth_Projection(ret.truth, ret.occurrenceTime, currentTime);
    return ret;
}

//{Event (&/,a,op())!, Event a.} |- Event op()!
Event Inference_OperationDeduction(Event *compound, Event *component, long currentTime)
{
    DERIVATION_STAMP(component,compound)
    Event compoundUpdated = Inference_EventUpdate(compound, currentTime);
    Event componentUpdated = Inference_EventUpdate(component, currentTime);
    return (Event) { .sdr = compound->sdr, 
                     .type = EVENT_TYPE_GOAL, 
                     .truth = Truth_Deduction(compoundUpdated.truth, componentUpdated.truth),
                     .stamp = conclusionStamp, 
                     .occurrenceTime = compound->occurrenceTime };
}
