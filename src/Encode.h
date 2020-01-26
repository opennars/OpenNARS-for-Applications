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
#define OPERATIONS_MAX 10


//Data structure//
//--------------//
//Atomic term names:
Atom Encode_atomNames[TERMS_MAX][ATOMIC_TERM_LEN_MAX];
Atom Encode_operatorNames[OPERATIONS_MAX][ATOMIC_TERM_LEN_MAX];
extern Atom SELF;

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
//Encodes a sequence
Term Encode_Sequence(Term *a, Term *b);
//Parses an atomic term string to a term
Term Encode_AtomicTerm(char *name);
//Index of atomic term
int Encode_AtomicTermIndex(char *name);
//Index of operator
int Encode_OperatorIndex(char *name);
//Print an atom
void Encode_PrintAtom(Atom atom);
//Print a term
void Encode_PrintTerm(Term *term);
//Whether it is a certain copula:
bool Encode_copulaEquals(Atom atom, char name);
//Whether it is an operator
bool Encode_isOperator(Atom atom);
//Get operator id
int Encode_getOperationID(Term *atom);
//Is an operation
bool Encode_isOperation(Term *term);
//Get precondition without operation
Term Encode_GetPreconditionWithoutOp(Term *precondition);


#endif
