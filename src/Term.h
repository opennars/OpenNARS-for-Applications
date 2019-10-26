#ifndef Term_H
#define Term_H

///////////////////
//   Term        //
///////////////////

//Parameters//
//----------//
#define MAX_SEQUENCE_LEN 2

//Description//
//-----------//
//An Term is an blocksay of a specific number of 128 bit blocks
//(that way no Hash ops are necessary, it's faster for this Term size)

//References//
//-----------//
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "Truth.h"

//Data structure//
//--------------//
typedef struct
{
    char terms[MAX_SEQUENCE_LEN];
}Term;

//Methods//
//-------//
// print indices of true bits
void Term_Print(Term *term);
//Tuple on the other hand:
Term Term_Tuple(Term *a, Term *b);
//Whether two Term's are equal completely
bool Term_Equal(Term *a, Term *b);

#endif
