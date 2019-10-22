#include "Usage.h"

double Usage_usefulness(Usage *usage, long currentTime)
{
    double age = currentTime - usage->lastUsed;
    double usefulnessToNormalize = ((double) usage->useCount) / (age + 1.0);
    return usefulnessToNormalize / (usefulnessToNormalize + 1.0);
}

void Usage_use(Usage *usage, long currentTime)
{
    usage->useCount++;
}

void Usage_Print(Usage *usage)
{
    printf("Usage: useCount=%d lastUsed=%d\n", usage->useCount, usage->lastUsed);
}
