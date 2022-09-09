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

#ifndef H_STAMP
#define H_STAMP

/////////////
//  Stamp  //
/////////////
//Keeps track of evidental bases
//This ensures that evidence is only counted once in the conclusions the system makes
//Design decisions:
//- Stamps are merged by zipping two existing stamps
//- Stamps have a max. capacity which controls how far the zipping goes
//- Since new inputs have a single stamp entry, new stamp entries will be in the beginning (1. or 2. element) of the stamp after revision

//References//
//----------//
#include <stdbool.h>
#include <stdio.h>
#include "Config.h"

//Data structure//
//--------------//
//Stamp as implemented by all NARS implementations
#define STAMP_FREE 0
typedef struct {
    //EvidentalBase of stamp
    long evidentalBase[STAMP_SIZE];
} Stamp;

//Methods//
//-------//
//zip stamp1 and stamp2 into a stamp
Stamp Stamp_make(Stamp *stamp1, Stamp *stamp2);
//true iff there is evidental base overlap between a and b
bool Stamp_checkOverlap(Stamp *a, Stamp *b);
//Whether two stamps are equal
bool Stamp_Equal(Stamp *a, Stamp *b);
//print stamp
void Stamp_print(Stamp *stamp);

#endif
