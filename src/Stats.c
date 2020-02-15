#include "Stats.h"

long Stats_countConceptsMatchedTotal = 0;
long Stats_countConceptsMatchedMax = 0;

void Stats_Print(long currentTime)
{
    puts("Statistics:");
    printf("countConceptsMatchedTotal:\t%ld\n", Stats_countConceptsMatchedTotal);
    printf("countConceptsMatchedMax:\t%ld\n", Stats_countConceptsMatchedMax);
    long countConceptsMatchedAverage = Stats_countConceptsMatchedTotal / currentTime;
    printf("countConceptsMatchedAverage:\t%ld\n", countConceptsMatchedAverage);
    printf("currentTime:\t\t\t%ld\n", currentTime);
    printf("total concepts:\t\t\t%d\n", concepts.itemsAmount);
}
