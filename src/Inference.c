#include "Inference.h"
#include "SDR.h"

#define DERIVATION_STAMP(a,b) Stamp conclusionStamp = Stamp_make(&a->stamp, &b->stamp);
#define DERIVATION_STAMP_AND_TIME(a,b) DERIVATION_STAMP(a,b) \
                long conclusionTime = (a->occurrenceTime + b->occurrenceTime)/2.0; \
                Truth truthA = Truth_Projection(a->truth, a->occurrenceTime, conclusionTime); \
                Truth truthB = Truth_Projection(b->truth, b->occurrenceTime, conclusionTime);
                
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

//{Event a., Event b.} |- Implication <a =/> c>.
Implication Inference_BeliefInduction(Event *a, Event *b)
{
    DERIVATION_STAMP_AND_TIME(a,b)
    return  (Implication) { .sdr = SDR_Tuple(&a->sdr, &b->sdr), 
                            .truth = Truth_Eternalize(Truth_Induction(truthA, truthB)), 
                            .occurrenceTimeOffset = b->occurrenceTime - a->occurrenceTime };
}

//{Event a., Event a.} |- Event a.
//{Event a!, Event a!} |- Event a!
Event Inference_EventRevision(Event *a, Event *b)
{
    DERIVATION_STAMP_AND_TIME(a,b)
    Event B = *b;
    //symmetric case of https://github.com/patham9/ANSNA/wiki/SDR:-SDRInheritance-for-matching,-and-its-truth-value
    B.truth = Truth_Analogy(SDR_Similarity(&b->sdr, &a->sdr), truthB);
    return (Event) { .sdr = a->sdr, 
                     .type = a->type,
                     .truth = Truth_Revision(truthA, B.truth),
                     .stamp = conclusionStamp, 
                     .occurrenceTime = conclusionTime };
}

//{Implication <a =/> b>., <a =/> b>.} |- Implication <a =/> b>.
Implication Inference_ImplicationRevision(Implication *a, Implication *b)
{
    DERIVATION_STAMP(a,b)
    Implication B = *b;
    //symmetric case of https://github.com/patham9/ANSNA/wiki/SDR:-SDRInheritance-for-matching,-and-its-truth-value
    B.truth = Truth_Analogy(SDR_Similarity(&b->sdr, &a->sdr), b->truth);
    return (Implication) { .sdr = a->sdr, 
                           .truth = Truth_Projection(Truth_Revision(a->truth, B.truth), a->occurrenceTimeOffset, B.occurrenceTimeOffset),
                           .stamp = conclusionStamp, 
                           .occurrenceTimeOffset = (a->occurrenceTimeOffset + B.occurrenceTimeOffset)/2.0 };
}

//{Event a., Implication <a =/> b>.} |- Event b.
Event Inference_BeliefDeduction(Event *component, Implication *compound)
{
    DERIVATION_STAMP(component,compound)
    return (Event) { .sdr = SDR_TupleGetSecondElement(&compound->sdr,&component->sdr), 
                     .type = EVENT_TYPE_BELIEF, 
                     .truth = Truth_Deduction(compound->truth, component->truth),
                     .stamp = conclusionStamp, 
                     .occurrenceTime = component->occurrenceTime + compound->occurrenceTimeOffset };
}

//{Event b!, Implication <a =/> b>.} |- Event a!
Event Inference_GoalDeduction(Event *component, Implication *compound)
{
    DERIVATION_STAMP(component,compound)
    return (Event) { .sdr = SDR_TupleGetFirstElement(&compound->sdr,&component->sdr), 
                     .type = EVENT_TYPE_GOAL, 
                     .truth = Truth_Deduction(compound->truth, component->truth),
                     .stamp = conclusionStamp, 
                     .occurrenceTime = component->occurrenceTime - compound->occurrenceTimeOffset };
}

//{Event b., Implication <a =/> b>.} |- Event a.
Event Inference_BeliefAbduction(Event *component, Implication *compound)
{
    DERIVATION_STAMP(component,compound)
    return (Event) { .sdr = SDR_TupleGetFirstElement(&compound->sdr,&component->sdr), 
                     .type = EVENT_TYPE_BELIEF, 
                     .truth = Truth_Abduction(compound->truth, component->truth), 
                     .stamp = conclusionStamp, 
                     .occurrenceTime = component->occurrenceTime - compound->occurrenceTimeOffset };
}

//{Event task a!, Implication <a =/> b>.} |- Event b!
Event Inference_GoalAbduction(Event *component, Implication *compound)
{
    DERIVATION_STAMP(component,compound)
    return (Event) { .sdr = SDR_TupleGetSecondElement(&compound->sdr,&component->sdr), 
                     .type = EVENT_TYPE_GOAL,
                     .truth = Truth_Abduction(compound->truth, component->truth), 
                     .stamp = conclusionStamp, 
                     .occurrenceTime = component->occurrenceTime + compound->occurrenceTimeOffset };
}

Implication Inference_AssumptionOfFailure(Implication *compound)
{
    Implication imp = *compound;
    imp.truth = Truth_Revision(imp.truth, (Truth) {.frequency=0.0, .confidence=ASSUMPTION_OF_FAILURE_CONFIDENCE});
    return imp;
}
