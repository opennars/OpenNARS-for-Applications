#ifndef BELIEF_H
#define BELIEF_H

//References//
//-----------//
#include "SDR.h"
#include "Stamp.h"

//Data structure//
//--------------//
typedef struct {
    SDR sdr;
    SDR_HASH_TYPE sdr_hash;
    Truth truth;
    Stamp stamp;
    long occurrenceTimeOffset;
    long variance;
    long revisions;
    char debug[100]; //++ DEBUG
} Implication;

//Methods//
//-------//
//Assign a new name to an implication
void Implication_SetSDR(Implication *implication, SDR sdr);
void Implication_Print(Implication *implication);

#endif


