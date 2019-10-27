#ifndef TABLE_H
#define TABLE_H

//////////////////////
//  Table           //
//////////////////////
//A table of implications, ranked by truth expectation

//References//
//----------//
#include "Inference.h"
#include "Globals.h"
#include <string.h>

//Parameters//
//----------//
#define TABLE_SIZE 20

//Data structure//
//--------------//
//A truth-expectation-ranked table for Implications, similar as pre- and post-condition table in OpenNARS,
//except that this table supports revision by itself (as in YAN implications don't form concepts).
typedef struct {
    Implication array[TABLE_SIZE];
    int itemsAmount;
} Table;

//Methods//
//-------//
//Add implication to table
Implication *Table_Add(Table *table, Implication *imp);
//Add element at index from table
void Table_Remove(Table *table, int index);
//Add implication to table while allowing revision
Implication* Table_AddAndRevise(Table *table, Implication *imp, char *debug);

#endif
