#ifndef STACK_H
#define STACK_H

/////////////
//  Stack  //
/////////////
//The stack for use by the hashtable

//References//
//-----------//
#include <stdlib.h>
#include <stdbool.h>
#include "Concept.h"

//Data structure//
//--------------//
typedef struct
{
    Concept *value; //the key is value->term
    void *next;
} VMItem;

typedef struct
{
    VMItem* items[CONCEPTS_MAX];
    int stackpointer;
} Stack;

//Methods//
//-------//
//Add a VMItem on the top of the stack
void Stack_Push(Stack *stack, VMItem *item);
//Remove a VMItem from the top of the stack
VMItem *Stack_Pop(Stack *stack);
//Check if there aren't VMItems left on the stack
bool Stack_IsEmpty(Stack *stack);

#endif

