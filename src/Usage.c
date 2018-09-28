#include "Usage.h"

double Usage_usefulness(Usage *usage, long currentTime)
{
    double age = currentTime - usage->lastUsed;
    double usefulnessToNormalize = ((double) usage->useCount) / age;
    return usefulnessToNormalize / (usefulnessToNormalize + 1.0);
}
