#ifndef Term_H
#define Term_H

///////////////////
//   Term        //
///////////////////

//Parameters//
//----------//
#define Atom char
#define MAX_SEQUENCE_LEN 2
#define COMPOUND_TERM_SIZE_MAX 100

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
    Atom atoms[COMPOUND_TERM_SIZE_MAX];
}Term;

//Methods//
//-------//
// print indices of true bits
void Term_Print(Term *term);
//Tuple on the other hand:
Term Term_Sequence(Term *a, Term *b);
//Whether two Term's are equal completely
bool Term_Equal(Term *a, Term *b);

#endif
