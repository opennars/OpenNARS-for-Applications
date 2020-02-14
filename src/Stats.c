long Stats_countConceptsMatchedTotal = 0;
long Stats_countConceptsMatchedMax = 0;

void Stats_Print(long currentTime)
{
    puts("Statistics:");
    printf("countConceptsMatchedTotal=%ld\n", Stats_countConceptsMatchedTotal);
    printf("countConceptsMatchedMax=%ld\n", Stats_countConceptsMatchedMax);
    long countConceptsMatchedAverage = Stats_countConceptsMatchedTotal / currentTime;
    printf("countConceptsMatchedAverage=%ld\n", countConceptsMatchedAverage);
    printf("currentTime=%ld\n", currentTime);
    //printf("total concepts=%d\n", concepts.itemsAmount);
}
