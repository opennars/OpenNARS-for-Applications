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

#ifndef H_NARSESE
#define H_NARSESE

/////////////////////
// Narsese encoder //
/////////////////////
//Supports converting Narsese strings to compound terms
//and dictates the format of the internal compound term encoding
//TODO add grammar

//References//
//----------//
#include <string.h>
#include <stdio.h>
#include "Term.h"
#include "Globals.h"
#include "Config.h"

//Data structure//
//--------------//
//Atomic term names:
extern char Narsese_atomNames[ATOMS_MAX][ATOMIC_TERM_LEN_MAX];
extern char Narsese_operatorNames[OPERATIONS_MAX][ATOMIC_TERM_LEN_MAX];
extern Atom SELF;
#define Narsese_RuleTableVars "ABCMRSPXYZ"
#define Naresese_CanonicalCopulas "@*&|;:=$'\"/\\.-%#~+!?^_,"
#define PRODUCT '*'
#define EXT_INTERSECTION '&'
#define INT_INTERSECTION '|'
#define CONJUNCTION ';'
#define INHERITANCE ':'
#define SIMILARITY '='
#define TEMPORAL_IMPLICATION '$'
#define INT_SET '\''
#define EXT_SET '"'
#define EXT_IMAGE1 '/'
#define INT_IMAGE1 '\\'
#define SET_ELEMT '.'
#define EXT_DIFFERENCE '-'
#define EXT_IMAGE2 '%'
#define INT_IMAGE2 '#'
#define INT_DIFFERENCE '~'
#define SEQUENCE '+'
#define NEGATION '!'
#define IMPLICATION '?'
#define EQUIVALENCE '^'
#define DISJUNCTION '_'
#define HAS_CONTINUOUS_PROPERTY ','
#define SET_TERMINATOR '@'

//Methods//
//-------//
//Initializes encoder
void Narsese_INIT();
//Expands Narsese into by strtok(str," ") tokenizable string with canonical copulas
char* Narsese_Expand(char *narsese);
//Tokenize expanded Narsese in prefix copula order
char** Narsese_PrefixTransform(char* narsese_expanded);
//Parses a Narsese string to a compound term
Term Narsese_Term(char *narsese);
//Parses a Narsese string to a compound term and a tv, tv is default if not present
void Narsese_Sentence(char *narsese, Term *destTerm, char *punctuation, int *tense, Truth *destTv, double *occurrenceTimeOffset);
//Encodes a sequence
Term Narsese_Sequence(Term *a, Term *b, bool *success);
//Parses an atomic term string to a term
Term Narsese_AtomicTerm(char *name);
//Index of atomic term
int Narsese_AtomicTermIndex(char *name);
int Narsese_CopulaIndex(char name);
//Print an atom
void Narsese_PrintAtom(Atom atom);
//Print a term
void Narsese_PrintTerm(Term *term);
//Whether it is a certain copula:
bool Narsese_copulaEquals(Atom atom, char name);
//Whether it is an operator
bool Narsese_isOperator(Atom atom);
//Get operator id
Atom Narsese_getOperationAtom(Term *term);
//Get operation term
Term Narsese_getOperationTerm(Term *term);
//Is an operation
bool Narsese_isOperation(Term *term);
//Is an executable op (has {SELF} or variable as first arg)
bool Narsese_isExecutableOperation(Term *term);
//Get precondition without operation
Term Narsese_GetPreconditionWithoutOp(Term *precondition);
//Get whether something is a true atom, not a copula or variable
bool Narsese_IsSimpleAtom(Atom atom);
//Whether the term has a simple atom
bool Narsese_HasSimpleAtom(Term *term);
//Whether two Narsese strings are equal
bool Narsese_StringEqual(char *name1, char *name2);
//The hash code of a string
HASH_TYPE Narsese_StringHash(char *name);
//Whether the term has an operation
bool Term_HasOperation(Term *term);
//Append a sequence in left-nested way:
bool Narsese_OperationSequenceAppendLeftNested(Term *start, Term *sequence);

#endif
