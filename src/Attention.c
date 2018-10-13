#include "Attention.h"

Attention Attention_forgetEvent(Attention *eventAttention, long currentTime)
{
    long dt = currentTime - eventAttention->lastForgotten;
    return (Attention) { .priority   = eventAttention->priority * pow(eventAttention->durability, dt),
                         .durability = EVENT_DURABILITY,
                         .lastForgotten = currentTime };
}

#define MAX(a, b) ((a) > (b) ? (a) : (b))
Attention Attention_forgetConcept(Attention *conceptAttention, Usage *conceptUsage, long currentTime)
{
    long dt = currentTime - conceptAttention->lastForgotten;
    double usefulness = Usage_usefulness(conceptUsage, currentTime);
    double lowerPriorityBarrier = usefulness * USEFULNESS_MAX_PRIORITY_BARRIER;
    return (Attention) { .priority = MAX(lowerPriorityBarrier, conceptAttention->priority * pow(conceptAttention->durability, dt)),
                         .durability = CONCEPT_DURABILITY,
                         .lastForgotten = currentTime };
}

Attention Attention_activateConcept(Attention *conceptAttention, Attention *eventAttention)
{
    return (Attention) { .priority = or(conceptAttention->priority, eventAttention->priority),
                         .durability = CONCEPT_DURABILITY,
                         .lastForgotten = conceptAttention->lastForgotten };
}

Attention Attention_deriveEvent(Attention *conceptAttention, Truth *truth, long currentTime)
{
    return (Attention) { .priority = conceptAttention->priority * Truth_Expectation(*truth),
                         .durability = EVENT_DURABILITY,
                         .lastForgotten = currentTime };
}

Attention Attention_inputEvent(Truth *truth, long currentTime)
{
    return (Attention) { .priority = Truth_Expectation(*truth),
                         .durability = EVENT_DURABILITY,
                         .lastForgotten = currentTime };
}
