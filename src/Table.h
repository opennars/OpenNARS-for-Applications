#ifndef TABLE_H
#define TABLE_H

//References//
//----------//
#include "Implication.h"

//Parameters//
//----------//
#define TABLE_SIZE 1000

/** A truth-expectation-ranked table for Implications, similar as pre- and post-condition table in OpenNARS,
 * except that this table supports revision by itself (as in ANSNA implications don't form concepts). */
typedef struct {
    Implication array[TABLE_SIZE];
    int itemsAmount;
} Table;

//Methods//
//-------//
void Table_Add(Table *table, Implication *imp);
void Table_AddAndRevise(Table *table, Implication *imp);

#endif
