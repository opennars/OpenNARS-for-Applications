#include "Table.h"

Implication *Table_Add(Table *table, Implication *imp)
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
            return &table->array[i];
        }
    }
    return NULL;
}

static void Table_Remove(Table *table, int index)
{
    //move up the rest beginning at index
    for(int j=index; j<table->itemsAmount; j++)
    {
        table->array[j] = j == table->itemsAmount ? (Implication) {0} : table->array[j+1];
    }
    table->itemsAmount = MAX(0, table->itemsAmount-1);
}

static void Table_SantiyCheck(Table *table)
{
    for(int i=0; i<table->itemsAmount; i++)
    {
        for(int j=0; j<table->itemsAmount; j++)
        {
            if(i != j)
            {
                assert(!SDR_Equal(&table->array[i].sdr, &table->array[j].sdr), "THEY CANNOT BE THE SAME\n");
            }
        }
    }
}

Implication *Table_AddAndRevise(Table *table, Implication *imp, char *debug)
{
    assert(imp->revisions > 0, "Table_AddAndRevise: issue with revisions counter");
    IN_DEBUG ( Table_SantiyCheck(table); )
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
        //revision adds the revised element, removing the old implication from the table
        Implication OldImp = table->array[same_i];
        assert(OldImp.truth.frequency >= 0.0 && OldImp.truth.frequency <= 1.0, "(1) frequency out of bounds");
        assert(OldImp.truth.confidence >= 0.0 && OldImp.truth.confidence <= 1.0, "(1) confidence out of bounds");
        assert(imp->truth.frequency >= 0.0 && imp->truth.frequency <= 1.0, "(2) frequency out of bounds");
        assert(imp->truth.confidence >= 0.0 && imp->truth.confidence <= 1.0, "(2) confidence out of bounds");
        assert(imp->revisions > 0, "Table_AddAndRevise: [imp] issue with revisions counter");
        assert(OldImp.revisions > 0, "Table_AddAndRevise: [OldImp] issue with revisions counter");
        Implication revised = Inference_ImplicationRevision(&OldImp, imp);
        assert(revised.revisions > 0, "Table_AddAndRevise: revision issue with revisions counter");
        assert(revised.truth.frequency >= 0.0 && revised.truth.frequency <= 1.0, "(3) frequency out of bounds");
        assert(revised.truth.confidence >= 0.0 && revised.truth.confidence <= 1.0, "(3) confidence out of bounds");
        strcpy(revised.debug, debug);
        Implication_SetSDR(&revised, imp->sdr);
        //printf("AAA %s  %.02f,%.02f\n", revised.debug, revised.truth.frequency, revised.truth.confidence);
        Table_Remove(table, same_i);
        //printf("REVISED\n");
        Implication *ret = Table_Add(table, &revised);
        assert(ret != NULL, "Deletion and re-addition should have succeeded");
        return ret;
    }
    else
    {
        //3. add imp too:
        strcpy(imp->debug, debug);
        //printf("ADDED\n");
        return Table_Add(table, imp);
    }
    return NULL;
}
