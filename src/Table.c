#include "Table.h"

void Table_Add(Table *table, Implication *imp)
{
    double impTruthExp = Truth_Expectation(imp->truth);
    for(int i=0; i<TABLE_SIZE; i++)
    {
        bool same_term = (/*table->array[i].sdr_hash == imp->sdr_hash &&*/ SDR_Equal(&table->array[i].sdr,&imp->sdr));
        //either it's not yet full and we reached a new space,
        //or the term is different and the truth expectation is higher
        //or the term is the same and the confidence is higher
        if(i==table->itemsAmount || (!same_term && impTruthExp > Truth_Expectation(table->array[i].truth)) || (same_term && imp->truth.confidence > table->array[i].truth.confidence))
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

void Table_Remove(Table *table, int index)
{
    //move up the rest beginning at index
    for(int j=index; j<table->itemsAmount-1; j++)
    {
        table->array[j] = table->array[j+1];
    }
    table->itemsAmount = MAX(0, table->itemsAmount-1);
}

Implication Table_AddAndRevise(Table *table, Implication *imp, char *debug)
{
    Implication RetRevised = (Implication) {0};
    //1. find element with same SDR
    int same_i = -1;
    for(int i=0; i<table->itemsAmount; i++)
    {
        if(/*imp->sdr_hash == table->array[i].sdr_hash &&*/ SDR_Equal(&imp->sdr, &table->array[i].sdr))
        {
            same_i = i;
            break;
        }
    }
    //2. if there was one, revise with it or apply choice if overlap
    if(same_i != -1)
    {
        //choice replaces the existing with the new if the new has higher confidence
        if(Stamp_checkOverlap(&imp->stamp, &table->array[same_i].stamp))
        {
            if(imp->truth.confidence > table->array[same_i].truth.confidence)
            {
                Table_Remove(table, same_i);
                Table_Add(table, imp);
            }
        }
        //revision adds the revised element, removing the old implication from the table if it results in higher confidence than premises
        else
        {
            Implication* OldImp = &table->array[same_i];
            Implication revised = Inference_ImplicationRevision(OldImp, imp);
            if(revised.truth.confidence > OldImp->truth.confidence && revised.truth.confidence > imp->truth.confidence)
            {
                strcpy(revised.debug, debug);
                Implication_SetSDR(&revised, imp->sdr);
                //printf("AAA %s  %.02f,%.02f\n", revised.debug, revised.truth.frequency, revised.truth.confidence);
                Table_Remove(table, same_i);
                Table_Add(table, &revised);
                RetRevised = revised;
                IN_DEBUG
                (
                    puts("START\n\n");
                    for(int i=0; i<table->itemsAmount; i++)
                    {
                        puts(table->array[i].debug);
                        puts("\n");
                        Implication_Print(&table->array[i]);
                    }
                    puts("REVISION END\n");
                    getchar();
                )
            }
        }
    }
    else
    //else addition adds a new element that had no matching element in the table
    {
        //3. add imp too:
        strcpy(imp->debug, debug);
        Table_Add(table, imp);
        IN_DEBUG
        (
            puts("START");
            for(int i=0; i<table->itemsAmount; i++)
            {
                puts(table->array[i].debug);
                puts("");
                Implication_Print(&table->array[i]);
            }
            puts("ADDITION END");
            getchar();
        )
    }
    return RetRevised;
}

void Table_COPY(Table *src, Table *target)
{
    target->itemsAmount = src->itemsAmount;
    for(int i=0; i<src->itemsAmount; i++)
    {
        target->array[i] = src->array[i];
    }
}
