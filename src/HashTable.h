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

#ifndef H_HASHTABLE
#define H_HASHTABLE

/////////////////
//  HashTable  //
/////////////////
//A generic bounded hashtable
//Also used for other purposes, such as:
//- The concept hashtable HT[Term] -> Concept*
//- Mapping atoms to atom names
//- Counting how often the atoms in a term occur in the same term

//References//
//----------//
#include <stdlib.h>
#include <stdbool.h>
#include "Stack.h"

//Data structure//
//--------------//
typedef bool (*Equal)(void*, void*);
typedef HASH_TYPE (*Hash)(void*);
typedef struct
{
    void *key;
    void *value;
    void *next;
} VMItem;
typedef struct
{
    VMItem** storageptrs;
    VMItem* storage;
    VMItem** HT; //the hash of the concept term is the index
    Stack VMStack; //"Virtual memory" stack
    int buckets;
    Equal equal;
    Hash hash;
} HashTable;

//Methods//
//-------//
//Get a concept from the hashtable via term
void* HashTable_Get(HashTable *hashtable, void *key);
//Add a concept to the hashtable using the concept term
void HashTable_Set(HashTable *hashtable, void *key, void *value);
//Delete a concept from hashtable (the concept's term is the key)
void HashTable_Delete(HashTable *hashtable, void *key);
//Initialize hashtable "virtual memory" stack and HT array
void HashTable_INIT(HashTable *hashtable, VMItem* storage, VMItem** storageptrs, VMItem** HT, int buckets, int maxElements, Equal equal, Hash hash);
//Maximum chain length in hashtable
int HashTable_MaximumChainLength(HashTable *hashtable);

#endif

