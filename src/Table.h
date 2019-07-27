#ifndef TABLE_H
#define TABLE_H

//References//
//----------//
#include "Inference.h"
#include "Globals.h"
#include <string.h>

//Parameters//
//----------//
#define TABLE_SIZE 50

//Data structure//
//--------------//
//A truth-expectation-ranked table for Implications, similar as pre- and post-condition table in OpenNARS,
//except that this table supports revision by itself (as in ANSNA implications don't form concepts).
typedef struct {
    Implication array[TABLE_SIZE];
    int itemsAmount;
} Table;

//Methods//
//-------//
//Add implication to table
void Table_Add(Table *table, Implication *imp);
//Add implication to table while allowing revision
Implication Table_AddAndRevise(Table *table, Implication *imp, char *debug);
//Copy all table entries
void Table_COPY(Table *src, Table *target);

#endif
