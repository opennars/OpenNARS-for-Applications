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
    Term map[TERMS_MAX];
    bool success;
} Substitution;

//Methods//
//-------//
bool Variable_isIndependentVariable(Atom atom);
bool Variable_isDependentVariable(Atom atom);
bool Variable_isQueryVariable(Atom atom);
bool Variable_isVariable(Atom atom);
bool Variable_hasVariable(Term *term, bool independent, bool dependent, bool query);
Substitution Variable_Unify(Term *general, Term *specific);
Term Variable_ApplySubstitute(Term term, Substitution substitution);
Term IntroduceImplicationVariables(Term implication);

#endif
