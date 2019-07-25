#ifndef CONCEPT_H
#define CONCEPT_H

///////////////////
//  SDR Concept  //
///////////////////
//A concept named by a SDR

//References//
//-----------//
#include "FIFO.h"
#include "Table.h"
#include "Usage.h"

//Parameters//
//----------//
#define OPERATIONS_MAX 10
#define ANTICIPATIONS_MAX 10

//Data structure//
//--------------//
typedef struct {
    Usage usage;
    SDR sdr;
    SDR_HASH_TYPE sdr_hash;
    Event belief_spike;
    Event goal_spike;
    Table precondition_beliefs[OPERATIONS_MAX];
    Table postcondition_beliefs[OPERATIONS_MAX];
    //Anticipation:
    Implication anticipation_negative_confirmation[ANTICIPATIONS_MAX];
    long anticipation_deadline[ANTICIPATIONS_MAX];
    int anticipation_operation_id[ANTICIPATIONS_MAX];
    //For ebugging:
    char debug[50];
} Concept;

//Methods//
//-------//
//Assign a new name to a concept
void Concept_SetSDR(Concept *concept, SDR sdr);
//print a concept
void Concept_Print(Concept *concept);
//Check anticipation disappointment
void Concept_CheckAnticipationDisappointment(Concept *c, long currentTime);
//Confirm anticipation
void Concept_ConfirmAnticipation(Concept *c, Event *e);

#endif
