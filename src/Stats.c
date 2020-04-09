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

#include "Stats.h"

long Stats_countConceptsMatchedTotal = 0;
long Stats_countConceptsMatchedMax = 0;

void Stats_Print(long currentTime)
{
    puts("Statistics:");
    printf("countConceptsMatchedTotal:\t%ld\n", Stats_countConceptsMatchedTotal);
    printf("countConceptsMatchedMax:\t%ld\n", Stats_countConceptsMatchedMax);
    long countConceptsMatchedAverage = Stats_countConceptsMatchedTotal / currentTime;
    printf("countConceptsMatchedAverage:\t%ld\n", countConceptsMatchedAverage);
    printf("currentTime:\t\t\t%ld\n", currentTime);
    printf("total concepts:\t\t\t%d\n", concepts.itemsAmount);
    printf("Maximum chain length in concept hashtable = %d\n", HashTable_MaximumChainLength(&HTconcepts));
    printf("Maximum chain length in atoms hashtable = %d\n", HashTable_MaximumChainLength(&HTatoms));
    fflush(stdout);
}
