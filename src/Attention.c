#include "Attention.h"

Attention Attention_forgetEvent(Attention *taskAttention)
{
    return (Attention) { .priority   = taskAttention->priority * taskAttention->durability,
                         .durability = taskAttention->durability };
}

#define MAX(a, b) ((a) > (b) ? (a) : (b))
Attention Attention_forgetConcept(Attention *conceptAttention, Usage *conceptUsage, long currentTime)
{
    double usefulness = Usage_usefulness(conceptUsage, currentTime);
    double lowerPriorityBarrier = usefulness * USEFULNESS_MAX_PRIORITY_BARRIER;
    return (Attention) { .priority = MAX(lowerPriorityBarrier, conceptAttention->priority * conceptAttention->durability),
                         .durability = conceptAttention->durability };
}

Attention Attention_activateConcept(Attention *conceptAttention, Attention *taskAttention)
{
    return (Attention) { .priority = or(conceptAttention->priority, taskAttention->priority),
                         .durability = conceptAttention->durability };
}

Attention Attention_deriveEvent(Attention *conceptAttention, Truth *beliefTruth)
{
    return (Attention) { .priority = conceptAttention->priority * Truth_Expectation(*beliefTruth),
                         .durability = conceptAttention->durability };
}
