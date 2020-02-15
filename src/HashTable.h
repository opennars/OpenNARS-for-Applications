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

