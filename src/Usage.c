#include "Usage.h"

double Usage_usefulness(Usage usage, long currentTime)
{
    double recency = currentTime - usage.lastUsed;
    double usefulnessToNormalize = ((double) usage.useCount) / (recency + 1.0);
    return usefulnessToNormalize / (usefulnessToNormalize + 1.0);
}

Usage Usage_use(Usage usage, long currentTime)
{
    return (Usage) { .useCount = usage.useCount+1,
                     .lastUsed = currentTime };
}

void Usage_Print(Usage *usage)
{
    printf("Usage: useCount=%ld lastUsed=%ld\n", usage->useCount, usage->lastUsed);
}
