#ifndef Term_H
#define Term_H

///////////////////
//   Term        //
///////////////////

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
#include "Config.h"
#include <string.h>

//Data structure//
//--------------//
#define TERM_HASH_TYPE long
#define TERM_HASH_TYPE_SIZE sizeof(TERM_HASH_TYPE)
#define TERM_ATOMS_SIZE (sizeof(Atom)*COMPOUND_TERM_SIZE_MAX)
typedef struct
{
    Atom atoms[COMPOUND_TERM_SIZE_MAX];
}Term;

//Methods//
//-------//
// print indices of true bits
void Term_Print(Term *term);
//Whether two Term's are equal completely
bool Term_Equal(Term *a, Term *b);
//Overwrites a subterm
bool Term_OverrideSubterm(Term *term, int i, Term *subterm);
//Extract a subterm as a term
Term Term_ExtractSubterm(Term *term, int j);
//The complexity of a term
int Term_Complexity(Term *term);
//Hash of a term (needed by the term->concept HashTable)
TERM_HASH_TYPE Term_Hash(Term *term);

#endif
