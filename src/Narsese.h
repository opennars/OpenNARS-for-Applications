#ifndef H_NARSESE
#define H_NARSESE

/////////////////////
// Narsese encoder //
/////////////////////
//Supports converting Narsese strings to compound terms
//and dictates the format of the internal compound term encoding

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
Atom Narsese_atomNames[TERMS_MAX][ATOMIC_TERM_LEN_MAX];
Atom Narsese_operatorNames[OPERATIONS_MAX][ATOMIC_TERM_LEN_MAX];
extern Atom SELF;

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
void Narsese_Sentence(char *narsese, Term *destTerm, char *punctuation, bool *isEvent, Truth *destTv);
//Encodes a sequence
Term Narsese_Sequence(Term *a, Term *b);
//Parses an atomic term string to a term
Term Narsese_AtomicTerm(char *name);
//Index of atomic term
int Narsese_AtomicTermIndex(char *name);
//Index of operator
int Narsese_OperatorIndex(char *name);
//Print an atom
void Narsese_PrintAtom(Atom atom);
//Print a term
void Narsese_PrintTerm(Term *term);
//Whether it is a certain copula:
bool Narsese_copulaEquals(Atom atom, char name);
//Whether it is an operator
bool Narsese_isOperator(Atom atom);
//Get operator id
int Narsese_getOperationID(Term *term);
//Is an operation
bool Narsese_isOperation(Term *term);
//Get precondition without operation
Term Narsese_GetPreconditionWithoutOp(Term *precondition);
//Get whether something is a true atom, not a copula
bool Narsese_IsNonCopulaAtom(Atom atom);

#endif
