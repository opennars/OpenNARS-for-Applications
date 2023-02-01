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

#include "Stamp_Test.h"
#include "PriorityQueue_Test.h"
#include "Memory_Test.h"
#include "OccurrenceTimeIndex_Test.h"
#include "InvertedAtomIndex_Test.h"
#include "Narsese_Test.h"
#include "RuleTable_Test.h"
#include "Stack_Test.h"
#include "Table_Test.h"
#include "HashTable_Test.h"
#include "UDP_Test.h"

void Run_Unit_Tests()
{
    Stamp_Test();
    PriorityQueue_Test();
    Table_Test();
    Memory_Test();
    OccurrenceTimeIndex_Test();
    InvertedAtomIndex_Test();
    Narsese_Test();
    RuleTable_Test();
    Stack_Test();
    HashTable_Test();
    UDP_Test();
}
