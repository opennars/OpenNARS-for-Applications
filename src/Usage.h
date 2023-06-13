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

#ifndef H_USAGE
#define H_USAGE

////////////////////////////////
//  concept usefulness value  //
////////////////////////////////
//Usefulness value consists of both use count and last used
//since new concepts need a grace period where they
//can prove themselves to be useful
//This is a solution of stability-plasticity dilemma.

//References//
//----------//
#include <stdio.h>
#include <stdbool.h>
#include "Config.h"
#include "Globals.h"

//Data structure//
//--------------//
typedef struct {
    //use_count, how often it was used in total
    long useCount;
    //age, how many cycles ago it was last used
    long lastUsed;
} Usage;

//Methods//
//-------//
//how useful it is in respect to the current moment
double Usage_usefulness(Usage usage, long currentTime);
//use the item
Usage Usage_use(Usage usage, long currentTime, bool eternalInput);
//print it
void Usage_Print(Usage *usage);

#endif
