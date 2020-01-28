#ifndef H_VARIABLE
#define H_VARIABLE

////////////////////////
// Variable utilities //
////////////////////////
//Support for NAL-6 variables

//References//
//-----------//
#include "Narsese.h"

//Data structure//
//--------------//
//Substitution, mapping variable atoms to terms
typedef struct {
    Term map[255];
    bool success;
} Substitution;

//Methods//
//-------//
bool Variable_isVariable(Atom atom);
bool Variable_hasVariable(Term *term);
Substitution Variable_Unify(Term *general, Term *specific);
Term Variable_ApplySubstitute(Term term, Substitution substitution);

#endif
