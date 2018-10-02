#ifndef BELIEF_H
#define BELIEF_H

//References//
//-----------//
#include "SDR.h"
#include "Stamp.h"
#include "Truth.h"

//Data structure//
//--------------//
typedef struct {
    SDR sdr;
    Truth truth;
    Stamp stamp;
    long occurrenceTimeOffset;
} Implication;

#endif


