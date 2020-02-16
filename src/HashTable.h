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

#ifndef HASHTABLE_H
#define HASHTABLE_H

/////////////////
//  HashTable  //
/////////////////
//The hashtable HT[Term] -> Concept*

//References//
//-----------//
#include <stdlib.h>
#include <stdbool.h>
#include "Stack.h"

//Data structure//
//--------------//
typedef struct
{
    VMItem storage[CONCEPTS_MAX];
    Stack VMStack; //"Virtual memory" stack
    VMItem* HT[CONCEPTS_MAX]; //the hash of the concept term is the index
} HashTable;

//Methods//
//-------//
//Get a concept from the hashtable via term
Concept *HashTable_Get(HashTable *hashtable, Term *term);
//Add a concept to the hashtable using the concept term
void HashTable_Set(HashTable *hashtable, Concept *c);
//Delete a concept from hashtable (the concept's term is the key)
void HashTable_Delete(HashTable *hashtable, Concept *c);
//Initialize hashtable "virtual memory" stack and HT array
void HashTable_Init(HashTable *hashtable);

#endif

