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

#ifndef H_FIFO
#define H_FIFO

/////////////////////////////////////
//  First in first out (forgotten) //
/////////////////////////////////////
//A FIFO-like structure, that only supports put in and overwrites
//the oldest task when full

//References//
//-----------//
#include "Inference.h"
#include "Globals.h"
#include "Narsese.h"
#include "Config.h"

//Data structure//
//--------------//
typedef struct
{
    int32_t itemsAmount;
    int32_t currentIndex;
    Event array[MAX_SEQUENCE_LEN][FIFO_SIZE];
} FIFO;
typedef struct
{
    Event *originalEvent;
    Event projectedEvent;
} FIFO_Query_Result;

//Methods//
//-------//
//Resets the FIFO
void FIFO_RESET(FIFO *fifo);
//Add an event to the FIFO
void FIFO_Add(Event *event, FIFO *fifo);
//Get the newest element
Event* FIFO_GetNewestSequence(FIFO *fifo, int32_t len);
//Get the k-th newest FIFO element
Event* FIFO_GetKthNewestSequence(FIFO *fifo, int32_t k, int32_t len);

#endif
