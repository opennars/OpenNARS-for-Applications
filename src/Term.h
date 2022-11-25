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

#ifndef H_TERM
#define H_TERM

//////////////
//   Term   //
//////////////

//Description//
//-----------//
//A term is a hashed array of atoms (which also includes atoms for copulas)
//The encoding used is that of a binary heap.
//Please note: this encoding is relative space-wasteful for terms of low complexity
//Future versions of this module might utilize an index-sorted array of (index, atom) tuples
//however this will be a major change.

//References//
//----------//
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "Truth.h"
#include "Config.h"
#include <string.h>

//Data structure//
//--------------//
#define HASH_TYPE_SIZE sizeof(HASH_TYPE)
#define TERM_ATOMS_SIZE (sizeof(Atom)*COMPOUND_TERM_SIZE_MAX)
typedef struct
{
    bool hashed;
    HASH_TYPE hash;
    Atom atoms[COMPOUND_TERM_SIZE_MAX];
}Term;

//Methods//
//-------//
//Whether two Term's are equal completely
bool Term_Equal(Term *a, Term *b);
//Overwrites a subterm
bool Term_OverrideSubterm(Term *term, int i, Term *subterm);
//Extract a subterm as a term
Term Term_ExtractSubterm(Term *term, int j);
//The complexity of a term
int Term_Complexity(Term *term);
//Hash of a term (needed by the term->concept HashTable)
HASH_TYPE Term_Hash(Term *term);
//Whether the term has the atom
bool Term_HasAtom(Term *term, Atom atom);

#endif
