#include "Table.h"

void Table_Add(Table *table, Implication *imp)
{
    double impTruthExp = Truth_Expectation(imp->truth);
    for(int i=0; i<TABLE_SIZE; i++)
    {
        if(i==table->itemsAmount || impTruthExp > Truth_Expectation(table->array[i].truth))
        {
            //ok here it has to go, move down the rest, evicting the last element if we hit TABLE_SIZE-1.
            for(int j=MIN(table->itemsAmount, TABLE_SIZE-1); j>i; j--)
            {
                table->array[j] = table->array[j-1];
            }
            table->array[i] = *imp;
            table->itemsAmount = MIN(table->itemsAmount+1, TABLE_SIZE);
            return;
        }
    }
}

Implication Table_AddAndRevise(Table *table, Implication *imp)
{
    Implication RetRevised = (Implication) {0};
    //1. get closest item in the table
    int best_i = -1;
    double best_expectation = 0.0;
    for(int i=0; i<table->itemsAmount; i++)
    {
        double cur_expectation = Truth_Expectation(SDR_Similarity(&imp->sdr, &table->array[i].sdr));
        if(cur_expectation > best_expectation && !Stamp_checkOverlap(&imp->stamp, &table->array[i].stamp))
        {
            best_i = i;
            best_expectation = cur_expectation;
        }
    }
    //2. if there was one, revise with closest, and add the revised element
    if(best_i != -1)
    {
        Implication* closest = &table->array[best_i];
        Implication revised = Inference_ImplicationRevision(closest, imp);
        Implication_SetSDR(&revised, imp->sdr); //update sdr hash
        if(revised.truth.confidence > closest->truth.confidence)
        {
            Table_Add(table, &revised);
            RetRevised = revised;
        }
    }
    //3. add imp too:
    Table_Add(table, imp);
    return RetRevised;
}

Implication Table_PopHighestTruthExpectationElement(Table *table)
{
    Implication result = table->array[0];
    for(int i=1; i<table->itemsAmount; i++)
    {
        table->array[i-1] = table->array[i];
    }
    table->itemsAmount--;
    return result;
}
