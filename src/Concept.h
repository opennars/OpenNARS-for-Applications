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
#define MIN_CONFIDENCE 0.01
#define CONCEPT_INTERPOLATION_STRENGTH 0.5
#define CONCEPT_INTERPOLATION_INIT_STRENGTH 1.0

//Data structure//
//--------------//
typedef struct {
    int id;
    Usage usage;
    SDR sdr;
    SDR_HASH_TYPE sdr_hash;
    Event belief_spike;
    Event incoming_goal_spike;
    Event goal_spike;
    Table precondition_beliefs[OPERATIONS_MAX];
    //Concept interpolation:
    double sdr_bit_counter[SDR_SIZE];
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
//Local inference: confirming anticipations, firing spikes, matching event, adjusting Usage
Event Concept_LocalInference(Concept *c, Event *e, long currentTime);

#endif
