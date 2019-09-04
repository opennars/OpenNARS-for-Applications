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
#define ANTICIPATIONS_MAX 100
#define MIN_CONFIDENCE 0.01
#define CONCEPT_INTERPOLATION_STRENGTH 1.0
#define CONCEPT_INTERPOLATION_INIT_STRENGTH 1000000.0

//Data structure//
//--------------//
typedef struct {
    Usage usage;
    SDR sdr;
    SDR_HASH_TYPE sdr_hash;
    Event belief_spike;
    Event incoming_goal_spike;
    Event goal_spike;
    Table precondition_beliefs[OPERATIONS_MAX];
    //Concept interpolation:
    double sdr_bit_counter[SDR_SIZE];
    //Anticipation:
    Implication anticipation_negative_confirmation[ANTICIPATIONS_MAX];
    long anticipation_deadline[ANTICIPATIONS_MAX];
    int anticipation_operation_id[ANTICIPATIONS_MAX];
    //For debugging:
    char debug[50];
} Concept;

//Methods//
//-------//
//Assign a new name to a concept
void Concept_SetSDR(Concept *concept, SDR sdr);
//print a concept
void Concept_Print(Concept *concept);
//Interpolate concepts, see https://github.com/patham9/ANSNA/wiki/Concept:-Conceptual-Interpolation
void Concept_SDRInterpolation(Concept *concept, SDR *eventSDR, Truth matchTruth);
//Check anticipation disappointment
void Concept_CheckAnticipationDisappointment(int layer, Concept *c, long currentTime);
//Confirm anticipation
void Concept_ConfirmAnticipation(int layer, Concept *c, Event *e);
//Local inference: confirming anticipations, firing spikes, matching event, adjusting Usage
Event Concept_LocalInference(int layer, Concept *c, Event *e, long currentTime);

#endif
