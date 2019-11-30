#ifndef H_SCALARENCODER
#define H_SCALARENCODER

//////////////////////////////
//  Scalar and term encoder //
//////////////////////////////
//Supports to encode a value in HTM way:
//https://www.youtube.com/watch?v=V3Yqtpytif0&list=PL3yXMgtrZmDqhsFQzwUC9V8MeeVOQ7eZ9&index=6

//References//
//-----------//
#include <string.h>
#include <math.h>
#include "Term.h"
#include "Globals.h"

//Parameters//
//----------//
#define TERMS_MAX 255
#define TERMS_LEN 255
#define TERM_ONES 5
#define NARSESE_LEN_MAX 1000
#define ATOMIC_TERM_LEN_MAX 30

//Data structure//
//--------------//
//Atomic terms:
Atom atoms[TERMS_MAX][ATOMIC_TERM_LEN_MAX];

//Methods//
//-------//
//
void Encode_INIT();
//Convert Narsese to expanded minimal Narsese
char* Encode_Expand(char *narsese);
//Tokenize minimal Narsese in prefix format
char** Encode_PrefixTransform(char* narsese_expanded);
//Parses a term in Narsese
Term Encode_Term(char *narsese);
//Parses an atomic term in Narsese
Term Encode_AtomicTerm(char *name);
//Print a term to Narsese
void Encode_PrintTerm(Term *term);
#endif
