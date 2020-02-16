#ifndef CONCEPT_H
#define CONCEPT_H

///////////////////
//  Term Concept  //
///////////////////
//A concept named by a Term

//References//
//-----------//
#include "FIFO.h"
#include "Table.h"
#include "Usage.h"

//Data structure//
//--------------//
typedef struct {
    long id;
    Usage usage;
    Term term;
    TERM_HASH_TYPE term_hash;
    Event belief; //the highest confident eternal belief
    Event belief_spike;
    Event predicted_belief;
    Event incoming_goal_spike;
    Event goal_spike;
    Table precondition_beliefs[OPERATIONS_MAX];
    double priority;
} Concept;

//Methods//
//-------//
//Assign a new name to a concept
void Concept_SetTerm(Concept *concept, Term term);
//print a concept
void Concept_Print(Concept *concept);

#endif
