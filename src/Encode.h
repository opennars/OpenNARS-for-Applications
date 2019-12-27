#ifndef H_ENCODE
#define H_ENCODE

/////////////////////
// Narsese encoder //
/////////////////////
//Supports converting Narsese strings to compound terms

//References//
//-----------//
#include <string.h>
#include "Term.h"
#include "Globals.h"

//Parameters//
//----------//
#define TERMS_MAX 255
#define ATOMIC_TERM_LEN_MAX 30
#define NARSESE_LEN_MAX 1000

//Data structure//
//--------------//
//Atomic term names:
Atom atom_names[TERMS_MAX][ATOMIC_TERM_LEN_MAX];

//Methods//
//-------//
//Initializes encoder
void Encode_INIT();
//Expands Narsese into by strtok(str," ") tokenizable string with canonical copulas
char* Encode_Expand(char *narsese);
//Tokenize expanded Narsese in prefix copula order
char** Encode_PrefixTransform(char* narsese_expanded);
//Parses a Narsese string to a compound term
Term Encode_Term(char *narsese);
//Parses an atomic term string to a term
Term Encode_AtomicTerm(char *name);
//Print an atom
void Encode_PrintAtom(Atom atom);
//Print a term
void Encode_PrintTerm(Term *term);

#endif
