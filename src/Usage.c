#include "Usage.h"

double Usage_usefulness(Usage *usage, long currentTime)
{
    double age = currentTime - usage->lastUsed;
    double usefulnessToNormalize = ((double) usage->useCount) / age;
    return usefulnessToNormalize / (usefulnessToNormalize + 1.0);
}

Usage Usage_use(Usage *usage, long currentTime)
{
    return (Usage) { .useCount = usage->useCount+1,
                     .lastUsed = currentTime };
}
