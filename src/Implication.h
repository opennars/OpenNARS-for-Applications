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
} Implication;

//Methods//
//-------//
//Assign a new name to an implication
void Implication_SetSDR(Implication *implication, SDR sdr);

#endif


