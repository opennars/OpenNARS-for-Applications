#ifndef CONCEPT_H
#define CONCEPT_H

///////////////////
//  SDR Concept  //
///////////////////
//A concept named by a SDR

//References//
//-----------//
#include "SDR.h"
#include "Task.h"
#include "AttentionValue.h"

//Parameters//
//----------//
#define PRECONDITION_BELIEFS_MAX 512
#define POSTCONDITION_BELIEFS_MAX 512
#define EVENT_BELIEFS_MAX 512
#define CONCEPT_HASH_TYPE uint64_t

//Data structure//
//--------------//
typedef struct {
	AttentionValue attention;
    /** name of the concept like in OpenNARS */
    SDR name;
    CONCEPT_HASH_TYPE name_hash;
    Task event_beliefs[EVENT_BELIEFS_MAX];
    int event_beliefs_amount;
    Task precondition_beliefs[PRECONDITION_BELIEFS_MAX];
    int precondition_beliefs_amount;
    Task postcondition_beliefs[POSTCONDITION_BELIEFS_MAX];
    int postcondition_beliefs_amount;
} Concept;

//Methods//
//-------//
//Reset the concept and assign a new name to it
void Concept_RESET(Concept *concept, SDR name);

#endif
