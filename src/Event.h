#ifndef EVENT_H
#define EVENT_H

//References//
//-----------//
#include "SDR.h"
#include "Stamp.h"
#include "Truth.h"
#include "Attention.h"

//Data structure//
//--------------//
#define EVENT_TYPE_GOAL 1
#define EVENT_TYPE_BELIEF 2
typedef struct {
    Attention attention;
    SDR sdr;
    char type; //either JUDGMENT or GOAL
    Truth truth;
    Stamp stamp;
    long occurrenceTime; 
} Event;

#endif
