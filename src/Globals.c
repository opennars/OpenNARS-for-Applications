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

#include "Globals.h"
#include <stdio.h>
#include <stdlib.h>

void assert(bool b, char* message)
{
    if(!b)
    {
        puts(message);
        puts("Test failed.");
        exit(1);
    }
}

HASH_TYPE Globals_Hash(HASH_TYPE *data, int pieces)
{
    HASH_TYPE hash = 0;
    for(int i=0; i<pieces; i++, data++)
    {
        hash ^= *data;
        hash += ~(hash << 15);
        hash ^=  (hash >> 10);
        hash +=  (hash << 3);
        hash ^=  (hash >> 6);
        hash += ~(hash << 11);
        hash ^=  (hash >> 16);
    }
    return hash;
}
