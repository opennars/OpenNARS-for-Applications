/* 
 * The MIT License
 *
 * Copyright 2020 The OpenNARS authors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef H_TABLE
#define H_TABLE

///////////////
//   Table   //
///////////////
//A bounded table of temporal implications, ranked by truth expectation
//Also revision and choice is supported in this structure
//Please note: when a new item has lower truth expectation
//than the lowest in the table, it will still replace the lowest.
//This makes sure the system can still adapt when tables are full,
//by giving the new link a place to grow.
//As this isn't ideal yet, later version this might either be extended
//to multiple places at the bottom of the table,
//or the ranking will take the creation time of links into account

//References//
//----------//
#include "Inference.h"
#include "Globals.h"
#include <string.h>
#include "Config.h"

//Data structure//
//--------------//
//A truth-expectation-ranked table for Implications, similar as pre- and post-condition table in OpenNARS,
//except that this table supports revision by itself (as in NAR implications don't form concepts).
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
Implication* Table_AddAndRevise(Table *table, Implication *imp);

#endif
