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

//Parameters//
//----------//
#define CONCEPT_INTERPOLATION_STRENGTH 1.0
#define CONCEPT_INTERPOLATION_INIT_STRENGTH 1.0
#define OPERATIONS_MAX 10
#define ANTICIPATIONS_MAX 10

//Data structure//
//--------------//
typedef struct {
    Attention attention;
    Usage usage;
    /** name of the concept like in OpenNARS */
    long id; //ID assigned to the concept on conceptualization, cleaner than using its address
    SDR sdr;
    SDR_HASH_TYPE sdr_hash;
    Event incoming_belief_spike;
    Event incoming_goal_spike;
    Event belief_spike;
    Event goal_spike;
    Table precondition_beliefs[OPERATIONS_MAX];
    Table postcondition_beliefs[OPERATIONS_MAX];
    //For concept interpolation:
    double sdr_bit_counter[SDR_SIZE];
    char debug[50];
    //Anticipation:
    Implication anticipation_negative_confirmation[ANTICIPATIONS_MAX];
    long anticipation_deadline[ANTICIPATIONS_MAX];
    int anticipation_operation_id[ANTICIPATIONS_MAX]; //the operation ID that was used
} Concept;

//Methods//
//-------//
//Assign a new name to a concept
void Concept_SetSDR(Concept *concept, SDR sdr);
//print a concept
void Concept_Print(Concept *concept);
//Check anticipation disappointment
void CheckAnticipationDisappointment(Concept *c, long currentTime);
//Confirm anticipation
void ConfirmAnticipation(Concept *c, Event *e);

#endif
