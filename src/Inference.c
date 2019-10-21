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
    assert(b->occurrenceTime >= a->occurrenceTime, "after(b,a) violated in Inference_BeliefIntersection");
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
                            .occurrenceTimeOffset = b->occurrenceTime - a->occurrenceTime };
}

//{Event a., Event a.} |- Event a.
//{Event a!, Event a!} |- Event a!
static Event Inference_EventRevision(Event *a, Event *b)
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
                                      .truth = Truth_Revision(a->truth, b->truth),
                                      .stamp = conclusionStamp, 
                                      .occurrenceTimeOffset = occurrenceTimeOffsetAvg };
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
                     .occurrenceTime = component->occurrenceTime - compound->occurrenceTimeOffset };
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

//{Event a!, Event a!} |- Event a! (revision and choice)
Event Inference_IncreasedActionPotential(Event *existing_potential, Event *incoming_spike, long currentTime)
{
    if(existing_potential->type == EVENT_TYPE_DELETED)
    {
        return *incoming_spike;
    }
    else
    {
        double expExisting = Truth_Expectation(Inference_EventUpdate(existing_potential, currentTime).truth);
        double expIncoming = Truth_Expectation(Inference_EventUpdate(incoming_spike, currentTime).truth);
        //check if there is evidental overlap
        bool overlap = Stamp_checkOverlap(&incoming_spike->stamp, &existing_potential->stamp);
        //if there is, apply choice, keeping the stronger one:
        if(overlap)
        {
            if(expIncoming > expExisting)
            {
                return *incoming_spike;
            }
        }
        else
        //and else revise, increasing the "activation potential"
        {
            Event revised_spike = Inference_EventRevision(existing_potential, incoming_spike);
            if(revised_spike.truth.confidence >= existing_potential->truth.confidence)
            {
                return revised_spike;
            }
            //lower, also use choice
            if(expIncoming > expExisting)
            {
                return *incoming_spike;
            }
        }
    }
    return *existing_potential;
}

//{Event a., Implication <a =/> b>.} |- Event b.
Event Inference_BeliefDeduction(Event *component, Implication *compound)
{
    DERIVATION_STAMP(component,compound)
    return (Event) { .sdr = compound->sdr, 
                     .type = EVENT_TYPE_BELIEF, 
                     .truth = Truth_Deduction(compound->truth, component->truth),
                     .stamp = conclusionStamp, 
                     .occurrenceTime = component->occurrenceTime + compound->occurrenceTimeOffset };
}
