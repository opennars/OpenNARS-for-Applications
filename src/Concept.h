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

//Parameters//
//----------//
#define OPERATIONS_MAX 10
#define MIN_CONFIDENCE 0.01

//Data structure//
//--------------//
typedef struct {
    int id;
    Usage usage;
    Term sdr;
    //Term_HASH_TYPE sdr_hash;
    Event belief_spike;
    Event incoming_goal_spike;
    Event goal_spike;
    Table precondition_beliefs[OPERATIONS_MAX];
    //For debugging:
    char debug[50];
} Concept;

//Methods//
//-------//
//Assign a new name to a concept
void Concept_SetTerm(Concept *concept, Term sdr);
//print a concept
void Concept_Print(Concept *concept);

#endif
