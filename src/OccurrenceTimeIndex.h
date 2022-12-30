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

#ifndef H_OCCURRENCETIMEINDEX
#define H_OCCURRENCETIMEINDEX

/////////////////////////////
//  Occurrence time index  //
/////////////////////////////
//The occurrence time OccurrenceTimeIndex for efficient query of to an event temporally related concepts
//used for event sequencing, and overrides the oldest concept reference when full on Add

//References//
//----------//
#include "Concept.h"
#include "Globals.h"
#include "Config.h"

//Data structure//
//--------------//
typedef struct
{
    int itemsAmount;
    int currentIndex;
    Concept* array[OCCURRENCE_TIME_INDEX_SIZE];
} OccurrenceTimeIndex;
//Methods//
//-------//
//Add an event to the OccurrenceTimeIndex
void OccurrenceTimeIndex_Add(Concept *concept, OccurrenceTimeIndex *fifo);
//Get the k-th newest OccurrenceTimeIndex element
Concept* OccurrenceTimeIndex_GetKthNewestElement(OccurrenceTimeIndex *fifo, int k);

#endif
